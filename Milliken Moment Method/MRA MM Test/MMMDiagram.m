clc ; clear;close all;
format long g

%% Notation
% 1,2 = front L & R wheels
% 3,4 = rear L & R wheels
% delta_1,2,3,4 = Steering angle of each individual wheel

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NEED TO FIX FOR NEXT TIME:
% alat is not changing according to steering angle change - investigate
%   this is causing Rfinal to not become smaller as fast as preferred
%   change of Rfinal in relation to SA_CG is significant - investigate
%       Problem: It uses the same R = 20 for every acceleration calculation -
%       FIXED, but now stays same because radius is decreasing with Vlat

% 6/19
% Mass problem - fixed
% Variables actual values problem - fixed
% Added lots of logging matrixes at the end
% why dFz is so high prolly due to NSM and SM elastic(roll stiffness?) -
% INVESTIGATE
% Problems: still Rfinal and alat_final are unchanging - INVESTIGATE
% 7/4
% Yay we did it 
% No we didn't do it
% Fixed dSteering and SA_CG conversion factor (pi/360) to (pi/180)
% Graphing only shows half the graph - INVESTIGATE


%% Constants
perc_front = 0.53;                          %percent front [%]
W = 1.567;                                   %Wheelbase [m]
lf = W * (1-perc_front);                    %front length to cg [m]
lr = W * perc_front;                        %rear length to cg [m]
twidth.front = 1.183;                       %Track Width [m]
twidth.rear = 1.183;
dtoe.front = -0.5 * (pi/180);               %Toe Angles [rads]
dtoe.rear = 0.5 * (pi/180);
acker.front = 0.4;                          %Ackerman Steering Intensity [-] {0-1}
acker.rear = 0.0;
rearsteer = 0;                              %Rearsteering Intensity [-] {0-1}

%More Constants in the iteration section

% Variables
dSteering = linspace(30,0,25)' .* (pi/180);          %Steering Angle [rads] - left turning is positive
SA_CG = linspace(10,0,25)' .* (pi/180);            %Body Slip Angle [rads]


%% Wheel Steering Angle and Coordinate
delta_1 = -dtoe.front + (dSteering + acker.front .* dSteering.^2);
delta_2 = dtoe.front + (dSteering - acker.front .* dSteering.^2);
delta_3 = -dtoe.rear - rearsteer.*(dSteering - acker.rear.* dSteering.^2);
delta_4 = dtoe.rear - rearsteer.*(dSteering + acker.rear.*dSteering.^2);

dSteering_all = [delta_1 delta_2 delta_3 delta_4];

% Confirming Graph
dSteering_all_deg = dSteering_all .* (180/pi);
dSteering_deg = dSteering .* (180/pi);

plot(dSteering_deg, dSteering_all_deg);
grid on;
ylabel("Wheel Steering Angle");
xlabel("Input Steering Angle");
title("Overall Steering Angle vs. Wheel Steering Angle");
legend(["1", "2", "3", "4"],"location", "best");

% Point Coordinate for each wheel
r1 = [lf twidth.front/2 0]';
r2 = [lf -twidth.front/2 0]';
r3 = [-lr twidth.rear/2 0]';
r4 = [-lr -twidth.rear/2 0]';
r_wheel = [r1 r2 r3 r4];

%% Iteration for R_CG Through Weight Transfer and Wheel Slip Angle Models

%%% Constants for Weight Transfer and Wheel Slip Angle
Vlong = 15;                 %Longitudinal Body Velocity [m/s] - Stays constant for one graph(can be changed to observe changes through each velocity)
Vlat = 1;                   %Lateral Body Velocity [m/s] - Just Initialization, actual Vlat is a function of Vlong and SA_CG further down

%   Side note: Vlong is a body velocity, which is kept constant, meaning the
%              resultant total velocity as it appears to outside observer will change
%              due to Body Slip Angle.

m.NSM = 51.437375;               %Non-Suspended Mass [kg] - Unsprung: Everything touching the ground
m.NSM = 75;
m.SM = 275-m.NSM;               %Suspended Mass [kg] - Sprung: Everything supported by the springs & stuff
m.SM = 165;
m.tot = (m.SM + m.NSM);     %Total Mass [kg]
g = 9.81;                   %Gravitational Acceleration [m/s^2]
hCG.NSM = 0.204724;              %CG Height of NSM [m]
hCG.NSM = 0.22;
hCG.SM = 0.25;               %CG Height of SM [m]
hRC.front = 0.040894;            %Front Roll Center [m]
hRC.front = 0.0254;            
hRC.rear = 0.04191;             %Rear Roll Center [m]
hRC.rear = 0.0762;            
ARStfDistr = 0.5;           %Roll Stiffness Distribution Front [-]
rho = 1.17;                 %Density of Air [kg/m^3]
A = 0.96;                    %Frontal Area of the vehicle [m^2]
Cf = 0.67;                   %Downforce Coefficient (+ is downwards)
aeroDistr = 0.5;            %Aerodynamic Downforce Percent Front [-]
ICG = m.tot * 0.7^2;        %Yaw moment of inertia

%Tire Model Parameters
Idx = 1;                    %Moment of Inertia in x for wheel
TirePressure = 70; %psi
TireInclination = 1;        %deg 
TireSR = 0; % -
Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
load('Hoosier_R25B_16x75-10x7.mat');

%%% Setup Functions and Matrix Initialization
F_aero = 0.5 * rho * A * Vlong^2 * Cf ;             %Downforce 
func_alat = @(radius, V_lat) ((Vlong^2) + (V_lat^2))./radius;   %Lateral Acceleration using R

% Iteration Constants
res = 1;
c = 1;
tol = 1e-6;

% Preallocating Matrixes for Speed
% V_wheel_input = zeros(1,4);
% 
% 
% V_wheel_input(p)
% Fx(p), Fym(p), Mz(p), Mx(p), My(p)
% F(:,p)
% R_final(i,j)
%  alat_final(i,j)
%  temp_alat_tot(i)
%   SA_wheel_deg_tot(i,:)
%   yaw_calc(:,p)
%   yaw_final(i,j)



for j = 1:length(SA_CG)

    Vlat = tan(SA_CG(j)) .* Vlong;
    V_CG = [Vlong Vlat 0]';

    for i = 1:length(dSteering)
        R = [50];       
        Fz = [];
        V_wheel = [];
        Omega_v = [0 0 0]';
        c = 1;
        while abs(res)>tol & c<1000
            %%% Weight Transfer
            temp_alat = func_alat(R(c), Vlat);
            dFz.front.NSM = (2 .* m.NSM .* temp_alat .* hCG.NSM)./twidth.front;
            dFz.rear.NSM = (2 .* m.NSM .* temp_alat .* hCG.NSM)./twidth.rear;
            dFz.front.SM_geo = m.SM .* perc_front .* temp_alat .* hRC.front ./ twidth.front;
            dFz.rear.SM_geo = m.SM .* (1-perc_front) .* temp_alat .* hRC.rear ./ twidth.rear;
            dFz.front.SM_elas = m.SM .* temp_alat .* (hCG.SM - hRC.front) .* ARStfDistr ./ twidth.front;
            dFz.rear.SM_elas = m.SM .* temp_alat .* (hCG.SM - hRC.rear) .* (1-ARStfDistr) ./ twidth.rear;
            
            dFz.front.tot = dFz.front.NSM + dFz.front.SM_geo + dFz.front.SM_elas;
            dFz.rear.tot = dFz.rear.NSM + dFz.rear.SM_geo + dFz.rear.SM_elas;

            % dFz.front.tot = dFz.front.SM_geo;
            % dFz.rear.tot = dFz.rear.SM_geo;
            
            F_z_massfront = (0.5 .* m.tot .* g .* perc_front);
            F_z_massrear = (0.5 .* m.tot .* g .* (1-perc_front));
            F_z_aerofront = (0.5 .* F_aero* aeroDistr);
            F_z_aerorear = (0.5 .* F_aero* (1-aeroDistr));

            Fz(1) = F_z_massfront - dFz.front.tot + F_z_aerofront;
            Fz(2) = F_z_massfront + dFz.front.tot + F_z_aerofront;
            Fz(3) = F_z_massrear - dFz.rear.tot + F_z_aerorear;
            Fz(4) = F_z_massrear + dFz.rear.tot + F_z_aerorear;
            
            %%% Wheel Slip Angle and Velocity
            om = ((Vlong^2 + Vlat^2)^(1/2)) /(R(c));
            Omega_v(3) = om;
            V_wheel(:,1) = V_CG + cross(Omega_v,r_wheel(:,1));
            V_wheel(:,2) = V_CG + cross(Omega_v,r_wheel(:,2));
            V_wheel(:,3) = V_CG + cross(Omega_v,r_wheel(:,3));
            V_wheel(:,4) = V_CG + cross(Omega_v,r_wheel(:,4));
            
            SA_wheel(1) = -atan(V_wheel(2,1)/V_wheel(1,1)) + dSteering_all(i,1);
            SA_wheel(2) = -atan(V_wheel(2,2)/V_wheel(1,2)) + dSteering_all(i,2);
            SA_wheel(3) = -atan(V_wheel(2,3)/V_wheel(1,3)) + dSteering_all(i,3);
            SA_wheel(4) = -atan(V_wheel(2,4)/V_wheel(1,4)) + dSteering_all(i,4);

            SA_wheel_deg = rad2deg(SA_wheel);
            
            for p = 1:4
                V_wheel_input(p) = sqrt(V_wheel(1,p)^2 + V_wheel(2,p)^2 + V_wheel(3,p)^2);
            end

            %%% Tire Model
            for p = 1:4
                [Fx(p), Fym(p), Mz(p), Mx(p), My(p)] = ContactPatchLoads(Tire, rad2deg(SA_wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_wheel_input(p), Idx, Model);
            end

            Fy = Fym; %Fy is all we need for the MMM
            
            %%% Force Vectors of each wheel
            for p = 1:4
            F(:,p) = [-Fy(p) * sin(dSteering_all(i,p));
                       Fy(p) * cos(dSteering_all(i,p));
                       Fz(p)                          ];
            end
            
            alat_calc = sum(F(2,:))/m.tot;
            R(c+1,1) = (Vlong.^2 + Vlat.^2)./alat_calc;
            res = R(c+1) - R(c);
            c = c+1;
            
            %plot(R) %- Check oscillation
        end %While loop end
        
        R_final(i,j) = R(end);      %Record final iterated R
        alat_final(i,j) = alat_calc;
        res = 1;                    %Reset Residual so itll run again
        temp_alat_tot(i) = temp_alat;
        disp(['Run dSteering [',num2str(i), '] SA_CG [', num2str(j), '] completed'])
        SA_wheel_deg_tot(i,:) = SA_wheel_deg;
        Fy_tot(i,:) = Fy';
        Fz_tot(i,:) = Fz;
        
        for p = 1:4
            yaw_calc(:,p) = cross(r_wheel(:,p), F(:,p));
        end
        yaw_calc_sum = sum(yaw_calc(3,:))/ICG;
        yaw_final(i,j) = yaw_calc_sum;
    end %for dSteering end
    VCG_tot(:,j) = V_CG;
end %for SA_CG end

%% Calculating Lateral Acceleration using derived radius (R_final)
close all;


figure;
hold on
grid on

for u = 1:length(dSteering)
    plot(alat_final(u,:)/g, yaw_final(u,:), '-b');
end

for u = 1:length(SA_CG)
    plot(alat_final(:,u)/g, yaw_final(:,u), "-r");
end
ylabel("yaw acceleration ddpsi (rad/s^2)")
xlabel("lateral acceleration a_y (g)")
xline(0)
yline(0)
xline(max(max(alat_final))/9.81, "--")
yline(yaw_final(find(alat_final==max(max(alat_final)))), "--")
title("max a_y = [" + max(max(alat_final()))/9.81+ "] g    ddpsi = [" + ...
    yaw_final(find(alat_final==max(max(alat_final))))+ "] rad/s^2")



% 
% figure;
% hold on
% grid on
% index = find(dSteering == 0);
% stability = yaw_final(index, :)'./rad2deg(SA_CG);
% plot(SA_CG, stability)
% 
% figure;
% hold on
% grid on
% index = find(SA_CG == 0);
% control = yaw_final(:,index)./rad2deg(dSteering);
% plot(dSteering, control)
























