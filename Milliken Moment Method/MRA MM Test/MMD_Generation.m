clc ; clear;close all;
format long g

%% LOGGING
%
% 10/21/2024: Ended on line 90, trying to implement sum of tire forces into
% body forces, but getting confused on the direction of positive steer
% angle, it seems like weight transfer follows right turn convention, that
% is: WT onto W1, off of W2( right to left ), but need to look at tire
% forces to body forces convention to make sure dSteer is positive the way
% I want it.
%
% 10/24/2024: Finished implementing for loops and while loop, but solutions
% doesnt want to converge and is currently oscillating between two values,
% BUT I ADDED iteration limit so oh well, seemed to be struggling at
% SA_CG[3:14] --> Need to investigate
%
% 10/25/2024: Deriving and confirming beta(i) equations, but we need to
% find out which direction is positive turning, but then ran into troubles
% with defining correct yaw rate direction and body velocity heading --->
% need to work on overall coordinate system correction and confirmation
%% Constants
%
%
% RIGHT HAND SIDE IS POSITIVE Y - apparently the standard?
%
% Tires go 
% Forwards
% 1  2      Postive Y
% 3  4
% Backwards

g = 9.81; % Grav constant
m = 220; %kg
PFront = 52 /100;
WB = 1.5; %Wheelbase - m
TWf = 1.2; %Trackwidth - m
TWr = 1.2;
toe_f = 0.5 * (pi/180); %Toe Angles
toe_r = 0.5 * (pi/180);

hCG = 0.2; % CG height

%Tire Model Parameters
Idx = 1;                    %Moment of Inertia in x for wheel
TirePressure = 70; %psi
TireInclination = 0;        %deg 
TireSR = 0; % -
Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
load('Hoosier_R25B_16x75-10x7.mat');

dSteer = deg2rad(linspace(0,30,31))';
SA_CG = deg2rad(linspace(0,12,31))';

dSteer_W1 = toe_f + dSteer;
dSteer_W2 = -toe_f + dSteer;
dSteer_W3 = toe_r + dSteer.*0;
dSteer_W4 = -toe_r + dSteer.*0;
dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];

lf = WB * (1-PFront);
lr = WB * (PFront);
coord_W1 = [lf ; -TWf/2];
coord_W2 = [lf ; TWf/2];
coord_W3 = [-lr ; -TWr/2];
coord_W4 = [-lr ; TWr/2];
coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];


%% FREE ROLLING MMD SECTION
R = 15; %Radius of Turn - m

% Mat Initialization
SA_Wheel = zeros(4,1);
V_Wheel = zeros(4,1);
TM_Fx = zeros(4,1);
TM_Fy = zeros(4,1);
FxTire = zeros(4,1);
FyTire = zeros(4,1);
MzTire = zeros(4,1);
saveAyBody = zeros(length(dSteer), length(SA_CG));
saveAxBody = zeros(length(dSteer), length(SA_CG));
saveMzBody = zeros(length(dSteer), length(SA_CG));
saveItFz = [];


for i = 1:length(dSteer)

    for j = 1:length(SA_CG)

        AxGuess = 0;
        AyGuess = 0;
        res = 1;
        resAx = 1;
        tol = 1e-3;
        itAyBody = [];
        itAyBody(1,1) = AyGuess;
        itAxBody = [];
        itAxBody(1,1) = AxGuess;
        itFz = [];

        c = 1;
    
        while abs(res) > tol %&& c <500
            if c == 1
                AyCurr = itAyBody(c);
            else
                AyCurr = itAyBody(c);% + itAyBody(c-1).*0.01;
            end

            AxCurr = itAxBody(c);
            V = sqrt(abs(AyCurr).* R); %.* (abs(AyCurr)./AyCurr);
            Omega = V/R;
            
            % Free Rolling MMD Assumption
            Omega = 0;

            for p = 1:4
                
                % SA_Wheel(p,1) = atan( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
                %                     / (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
                % SA_Wheel(p,1) = atan( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) / (R.*cos(SA_CG(j)) - coord_AllW(2,p)) )...
                %                 - dSteer_AllW(i,p);
                SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
            
                V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
            end

            % Free Rolling MMD Assumption
            V_Wheel = V_Wheel .* 0;
            
            dFzf_dAx = (hCG .* m)./(2.* WB);
            dFzf_dAy = 1*(hCG .* m .* g .* PFront)/TWf;
            dFzr_dAy = 1*(hCG .* m .* g .* (1-PFront))/TWr;
            
            Fz_Wf = (m.*g.* PFront)/2;
            Fz_Wr = (m.*g.* (1-PFront))/2;
                
            Fz(1,1) = Fz_Wf + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
            Fz(2,1) = Fz_Wf + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
            Fz(3,1) = Fz_Wr - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
            Fz(4,1) = Fz_Wr - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
            
            for p = 1:4
                [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire, rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_Wheel(p), Idx, Model);
                
                % Free Rolling MMD Assumption
                TM_Fx = TM_Fx .* 0; 
                
                % Calspan TTC Data usual correction factor - 0.7
                TM_Fx = TM_Fx .* 0.7;
                TM_Fy = TM_Fy .* 0.7;
                FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                FyTire(p,1) = 1*TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
                
                % Made it a matrix sum so its not ugly :)
                MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                  - coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
                                    coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
                                    coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
            end
            
            FxBody = sum(FxTire);
            FyBody = sum(FyTire);
            MzBody = sum(MzTire);
            
            itFz(:,c) = Fz;
            itFyTire(:,c) =  FyTire;
            itAyBody(c+1,1) = FyBody/m;
            itAxBody(c+1,1) = FxBody/m;
            res = itAyBody(c+1) - itAyBody(c);
    
            %plot(1:c, itFz);
            plot(1:c+1, itAyBody);
            %legend(["FzW1" "FzW2" "FzW3" "FzW4"])
            
            c = c + 1;
    
            % if c > 499
            %     disp("balls");
            % end
                
        end % while loop end
        % if j == 13
        %     return
        % end
        %saveAyBody(i,j) = (itAyBody(end)+itAyBody(end-1))/2;
        
        saveAyBody(i,j) = itAyBody(end);
        saveAxBody(i,j) = itAxBody(end);
        saveMzBody(i,j) = MzBody/(m.*g.*WB);
        saveItFz(:,j) = Fz;
        saveItAyBody{j,1} = itAyBody;

        disp("Steering Angle [" + i + "] Slip Angle [" + j + "] finished, iterations: " +  c)

    end % SA_CG End

end % dSteer End

%%

saveAxVel = zeros(size(saveAyBody));
saveAyVel = zeros(size(saveAyBody));

for i = 1:length(SA_CG)

    saveAxVel(:,i) = saveAxBody(:,i) .* cos(SA_CG) + saveAyBody(:,i) .* sin(SA_CG);
    saveAyVel(:,i) = saveAyBody(:,i) .* cos(SA_CG) - saveAxBody(:,i) .* sin(SA_CG);

end

for i = 1:length(dSteer)
    plot(saveAyVel(i,:),saveMzBody(i,:), "Color", "blue")
end

for i = 1:length(SA_CG)
    plot(saveAyVel(:,i),saveMzBody(:,i), "Color", "red")
end





