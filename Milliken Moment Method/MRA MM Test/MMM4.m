clc
clear
close all
delta = -130:5:130; %Change indexing back to normal  % Steer Angle [Deg]
Beta = -10:1:10; %change indexing back to normal    % Body Slip Angle [Deg]
Lat_Acc_Hold = ones(1,length(Beta));
Yaw_Acc_Hold = ones(1,length(Beta));
Lat_Acc_All = [];
Yaw_Acc_All = [];
tol = 1e-12;
res = Inf;

hold on
for i = 1:length(delta)
    for j = 1:length(Beta)
        tic
        
            %Parameters
            Percent_Front = 0.5;                         % Percent Front [%]
            WheelBase = 1.525;                           % Wheelbase [m]
            Front_Toe = -0.5;                            % Front Toe Angle [Deg]
            Rear_Toe = 0.5;                              % Rear Toe Angle  [Deg]
            Front_Ackerman = 0;                          % Front Ackerman
            Rear_Ackerman = 0;                           % Rear Ackerman
            rearsteer = 0;                               % Rear Steer  [Deg]
            Track_Width_Front = 1.220;                   % Front Track Width [m]
            Track_Width_Rear = 1.220;                    % Rear Track Width [m]
            Long_Vel = 10;                               % Longitudinal Velocity [m] 
            NSM = 77;                                    % Non Suspended Mass [kg]
            NSM_CG = 0.220;                              % Non Suspended Mass CoG Height [m]
            SM = 191;                                    % Suspended Mass [kg]
            Roll_Center_Height_Front = 0.0254;           % Front Roll Center Height [m]
            Roll_Center_Height_Rear = 0.0762;            % Rear Roll Center Height [m]
            SM_CG = 0.245;                               % Suspended Mass CoG Height [m]
            Percent_Roll_Stiffness = 0.60;               % Percent Front Roll Stiffness
            rho = 1.225;                                 % Air Density
            Cross_Area = 1.1;                            % Cross Sectional Area
            Coef_Downforce = 1.5;                        % Downforce Coefficient
            m = 268;                                     % Mass  [kg]
            g = 9.81;                                    % Acceleration Due to Gravity [m/s^2]
            Front_Aero_Distribution = 0.4;               % Percent Front Aero Distrobution [%]
            I_CG = m .* 0.7^2;                           % Yaw Moment of Inertia
            
            % Tire Parameters
            load('Hoosier_R25B_16x75-10x7.mat');           % Loading Tire Data (Tire Repo);)
            SlipRatio = 0;                   % Slip Ratio (degree)
            Pressure = 80;                      % Tire Pressure (KPa)
            Inclination = 0;                    % Tire Inclination (degrees)
            Idx = 1;
            Fidelity = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
            
            
            % Weight Distribution
            a = WheelBase * (1 - Percent_Front);
            b = WheelBase * (Percent_Front);

        R_CG_Old = 100; %Initial Guess
         while res > tol
            % Converting steer angle to individual tire steer angle
            %Fronts
            Tire_Angle(1) = -Front_Toe + (delta(i) + Front_Ackerman * delta(i)^2);
            Tire_Angle(2) = Front_Toe + (delta(i) - Front_Ackerman * delta(i)^2);   %Different toe because different left and right side headings
            
            %Rears
            Tire_Angle(3) = -Rear_Toe  - rearsteer * (delta(i) - Rear_Ackerman * delta(i)^2);
            Tire_Angle(4) = Rear_Toe - rearsteer * (delta(i) + Rear_Ackerman * delta(i)^2);
            
            %Contact Patch Locations
            Contact_Patch{1} = [a;Track_Width_Front/2;0];
            Contact_Patch{2} = [a;-Track_Width_Front/2;0];
            Contact_Patch{3} = [-b;Track_Width_Rear/2;0];
            Contact_Patch{4} = [-b;-Track_Width_Rear/2;0];
            
            %Velocity Calculations For Tire Velocities
            Lat_Vel = tand(Beta(j))*Long_Vel;
            v_CG = [Long_Vel;Lat_Vel;0];
            Yaw_Rate = ((Long_Vel^2 + Lat_Vel^2)^(1/2))/(R_CG_Old);
            Omega = [0;0;Yaw_Rate];
            
            %Tire Velocity Calculations
            V{1} = v_CG + cross(Omega,Contact_Patch{1});
            V{2} = v_CG + cross(Omega,Contact_Patch{2});
            V{3} = v_CG + cross(Omega,Contact_Patch{3});
            V{4} = v_CG + cross(Omega,Contact_Patch{4});
            
            %Slip Angle Calculations
            SA(1) = Tire_Angle(1)-atand(V{1}(2)/V{1}(1));
            SA(2) = Tire_Angle(2)-atand(V{2}(2)/V{2}(1));
            SA(3) = Tire_Angle(3)-atand(V{3}(2)/V{3}(1));
            SA(4) = Tire_Angle(4)-atand(V{4}(2)/V{4}(1));
            
            Lat_Acc = (Long_Vel^2+Lat_Vel^2)/R_CG_Old;
            
            %Weight Transfer Intermediates
            Front_NSM_LT = (2 * NSM * Lat_Acc * NSM_CG) / Track_Width_Front;
            Rear_NSM_LT = (2 * NSM * Lat_Acc * NSM_CG) / Track_Width_Rear;
            
            Front_SM_Geo_LT = (SM * Percent_Front * Lat_Acc * Roll_Center_Height_Front)/Track_Width_Front;
            Rear_SM_Geo_LT = (SM * (1-Percent_Front) * Lat_Acc * Roll_Center_Height_Rear)/Track_Width_Rear;
            
            Front_SM_Elastic_LT = ((SM * Lat_Acc * (SM_CG-Roll_Center_Height_Front))/Track_Width_Front) * (Percent_Roll_Stiffness);
            Rear_SM_Elastic_LT = ((SM * Lat_Acc * (SM_CG-Roll_Center_Height_Rear))/Track_Width_Rear) * (1-Percent_Roll_Stiffness);
            
            F_LLT = Front_NSM_LT + Front_SM_Geo_LT + Front_SM_Elastic_LT;
            R_LLT = Rear_NSM_LT + Rear_SM_Geo_LT + Rear_SM_Elastic_LT;
            F_Aero = 0.5 * rho * Cross_Area * Long_Vel^2 * Coef_Downforce;
            
            F_Vert(1) = 0.5 * m * g * Percent_Front - F_LLT + 0.5 * F_Aero * Front_Aero_Distribution;
            F_Vert(2) = 0.5 * m * g * Percent_Front + F_LLT + 0.5 * F_Aero * Front_Aero_Distribution;
            F_Vert(3) = 0.5 * m * g * (1-Percent_Front) - R_LLT + 0.5 * F_Aero * (1-Front_Aero_Distribution);
            F_Vert(4) = 0.5 * m * g * (1-Percent_Front) + R_LLT + 0.5 * F_Aero * (1-Front_Aero_Distribution);
            
            [~,Fy(1),~,~,~] = ContactPatchLoads(Tire,SA(1),SlipRatio,F_Vert(1),Pressure,Inclination,0,Idx,Fidelity);
            [~,Fy(2),~,~,~] = ContactPatchLoads(Tire,SA(2),SlipRatio,F_Vert(2),Pressure,Inclination,0,Idx,Fidelity);
            [~,Fy(3),~,~,~] = ContactPatchLoads(Tire,SA(3),SlipRatio,F_Vert(3),Pressure,Inclination,0,Idx,Fidelity);
            [~,Fy(4),~,~,~] = ContactPatchLoads(Tire,SA(4),SlipRatio,F_Vert(4),Pressure,Inclination,0,Idx,Fidelity);
            
            Lat_Acc_1 = (Fy(1)*cosd(Tire_Angle(1))+Fy(2)*cosd(Tire_Angle(2))+Fy(3)*cosd(Tire_Angle(3))+Fy(4)*cosd(Tire_Angle(4)))/m;
            R_CG_New = (Long_Vel^2+Lat_Vel^2)/Lat_Acc_1;
            res = (R_CG_New - R_CG_Old);
            R_CG_Old = R_CG_New;
         end
            res = Inf;
            Lat_Acc_Hold(j) = Lat_Acc_1;
            Lat_Acc_All(i,j) = Lat_Acc_1;
        
            %Tire Forces (Only Lateral) (Degrees or radians?)
            F_Matrix{1} = [-Fy(1) * sind(Tire_Angle(1)); Fy(1) * cosd(Tire_Angle(1)); F_Vert(1) ];
            F_Matrix{2} = [-Fy(2) * sind(Tire_Angle(2)); Fy(2) * cosd(Tire_Angle(2)); F_Vert(2) ];
            F_Matrix{3} = [-Fy(3) * sind(Tire_Angle(3)); Fy(3) * cosd(Tire_Angle(3)); F_Vert(3) ];
            F_Matrix{4} = [-Fy(4) * sind(Tire_Angle(4)); Fy(4) * cosd(Tire_Angle(4)); F_Vert(4) ];
            Yaw_Acc = cross(Contact_Patch{1},F_Matrix{1}) + cross(Contact_Patch{2},F_Matrix{2}) + cross(Contact_Patch{3},F_Matrix{3}) + cross(Contact_Patch{4},F_Matrix{4});
            Yaw_Acc_Hold(j) = (Yaw_Acc(3)/I_CG);
            Yaw_Acc_All(i,j) = Yaw_Acc_Hold(j);
            toc
    end
    plot(Lat_Acc_Hold./9.8,Yaw_Acc_Hold./9.8, 'b')
    clear Yaw_Acc_Hold Lat_Acc_Hold
end

% title('Milliken Moment Diagram with Planar Chassis Dynamics','FontSize',15)
grid on;
line([0,0], ylim, 'Color', 'k', 'LineWidth', 1); % Draw line for Y axis.
line(xlim, [0,0], 'Color', 'k', 'LineWidth', 1); % Draw line for X axis.
xlabel("Lateral Acceleration $[g]$",'interpreter','latex','fontsize',16)
ylabel("Yaw Acceleration $[g]$",'interpreter','latex','fontsize',16)
clearvars -except Lat_Acc_Hold Lat_Acc_All Yaw_Acc_Hold Yaw_Acc_All