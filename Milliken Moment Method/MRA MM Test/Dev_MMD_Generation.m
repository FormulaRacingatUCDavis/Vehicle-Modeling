clc ; clear; %close all;
format long g

%% LOGGING

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
% 
% 10/28/2024: Corrected track coordinates in accordance with right hand
% positive y, paper suspected to use coordinate system suggested in RCVD
% section 5, tried turning tire model output TM_Fy to negative, but didn't
% seem to affect anything? Model only seems to be struggling nearing the
% middle of each linspace for some reason(model doesn't like small angles?)
% could be the tire model as well. But added graphing and lowered weight
% transfer(by a multiplier) and seems to converge very fast with low weight
% transfer(kind of makes sense?)
% 
% 10/28/2024: Fixed the model, WT eq had an extra grav constant g for some
% reason, removed that and flipped TM_Fy sign by multiplying it by -1,
% since right hand turns are supposed to yield positive Ay, but TM outputs
% negative Ay for positive $alpha$, FreeRolling inf radius now works
% 
% 10/29/2024: Noticed a potential error, some forces from TM_Fy are
% negative for some reason, added logging for slip angles to try to
% evaluate problem, but WT seems to be working fine, SAWheel seems alright?
% Need to check again, and Vwheel is definitely 0, so something somewhere
% is going awry
% 
% 11/6/2024: PRoblem fixed, indexing problem that flipped the signage of
% TM_Fy everytime it output a wheel, problem was not putting the index of
% the value, so it multiplied the entire matrix everytime
% 
% 11/17/2024: Fixed Coordinate systen:  everything Y should be to the left,
% no longer needs negative signs on any Ay
% 
% 5/18/2025: Fixed the Ax graph, problem was a indexing problem in the
% coordinate transfer between the body frame and the velocity frame, it
% multiplied the steering sweep in the values by the SA_CG values, instead
% of the SA_CG sweep values by the SA_CG values

%% Constants
%
%
% RIGHT HAND SIDE IS POSITIVE Y - RCVD Standard (but lots of industry
% doesnt like to use this, but for the sake of the paper, we will use this,
% maybe change it later once I have this all figured out.
%
% Tires go 
%
% Forwards
% 2  1      Postive Y coord
% 4  3
% Backwards
%
% RIGHT HAND TURN RESULTS IN POSTIVE AY(yes you heard that right)

% %%% FE11 Constants
% g = 9.81;                   % Grav constant [m/s^2]
% m = 275;                    % Total Mass [kg]
% PFront = 51.1 /100;           % Percent Mass Front [0-1]
% WB = 1.595;                   % Wheelbase [m]
% TWf = 1.193;                  % Trackwidth [m]
% TWr = 1.193;
% toe_f = -0.5 * (pi/180);     % Toe Angles [radians] (positive is inwards)
% toe_r = 0 * (pi/180);
% hCG = 0.25346;                  % CG height [m]
% 
% Cl = -0.67; 
% Cd = 1.374; 
% CoP = 50/100;       % front downforce distribution (%)
% rho = 1.165;        % kg/m^3
% crossA = 0.92;      % m^2


%%% FE12 Constants
g = 9.81;                       % Grav constant [m/s^2]
m = 270;                        % Total Mass [kg]
PFront = 53.4/100;              % Percent Mass Front [0-1]
WB = 1.582;                     % Wheelbase [m]
TWf = 1.240;                    % Trackwidth [m]
TWr = 1.240;
toe_f = -0.5 * (pi/180);        % Toe Angles [radians] (positive is inwards)
toe_r = 0.5 * (pi/180);
hCG = 0.314;                    % CG height [m]

Cl = 3.215; % Real
% Cl = 4.2; % Fake: reaches 1.5g
Cd = 1.468; 
CoP = 45/100;       % front downforce distribution (%)
rho = 1.165;        % kg/m^3
crossA = 0.9237;      % m^2

B_FBB = 1.5;             % Front brake bias TODO: find the actual value



% TESTING PARAMETER CHANGES
WB = WB;
TWf = TWf;
TWr = TWr;
PFront = PFront;
hCG = hCG;

% Tire Model Parameters
Idx = 1;                     % Moment of Inertia in x for wheel
TirePressure = 70;           % kPa
TireInclinationFront = -1.3; % deg 
TireInclinationRear = -1; % deg   
Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
 % load('Hoosier_R25B_16x75-10x7.mat');
load('Hoosier_R20_16(18)x75(60)-10x8(7).mat');




%%% SELECT RANGES FOR BODY SLIP AND STEERING ANGLES
%%%
%%%
rangeSA = [-5,5];
rangeSteer = [-30,30];

if mod(sum(abs(rangeSA)), 2) == 0
    numSA = sum(abs(rangeSA)) + 1;
else
    numSA = sum(abs(rangeSA));
end

if mod(sum(abs(rangeSteer)), 2) == 0
    numST = sum(abs(rangeSteer)) + 1;
else
    numST = sum(abs(rangeSteer));
end

SA_CG = deg2rad(linspace(rangeSA(1), rangeSA(2),numSA))';
dSteer = deg2rad(linspace(rangeSteer(1), rangeSteer(2),numST))';




dSteer_W1 = toe_f + dSteer;
dSteer_W2 = -toe_f + dSteer;
dSteer_W3 = toe_r + dSteer.*0;
dSteer_W4 = -toe_r + dSteer.*0;
dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];

lf = WB * (1-PFront);
lr = WB * (PFront);
coord_W1 = [lf ; TWf/2];
coord_W2 = [lf ; -TWf/2];
coord_W3 = [-lr ; TWr/2];
coord_W4 = [-lr ; -TWr/2];
coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];


%% SECTION 1: FREE ROLLING MMD SECTION
% 
% R = 9; %Radius of Turn - m
% 
% % Mat Initialization
% SA_Wheel = zeros(4,1);
% V_Wheel = zeros(4,1);
% TM_Fx = zeros(4,1);
% TM_Fy = zeros(4,1);
% FxTire = zeros(4,1);
% FyTire = zeros(4,1);
% MzTire = zeros(4,1);
% saveAyBody = zeros(length(dSteer), length(SA_CG));
% saveAxBody = zeros(length(dSteer), length(SA_CG));
% saveMzBody = zeros(length(dSteer), length(SA_CG));
% saveItFz = [];
% saveTM_Fy = zeros(4,length(SA_CG));
% 
% pr =0; % Yaw Rate (Omega) iteration relaxation parameter for convergence
% 
% 
% for i = 1:length(dSteer)
% 
%     for j = 1:length(SA_CG)
% 
%         AxGuess = 0;
%         AyGuess = 0;
%         res = 1;
%         tol = 1e-3;
% 
%         itAyBody = [];
%         itAyBody(1,1) = AyGuess;
%         itAxBody = [];
%         itAxBody(1,1) = AxGuess;
%         itCAyBody = [];
%         itCAyBody(1,1) = AyGuess/g;
%         itFz = [];
%         itFyTire = [];
% 
%         %%% METHOD 2: Free Rolling MMD Assumption
%         itOmega = [];
%         itOmega(1,1) = 0;
% 
%         c = 1;
% 
%         while abs(res) > tol 
% 
%             AyCurr = -itAyBody(c);
%             AxCurr = itAxBody(c);
%             V = sqrt(abs(AyCurr).* R); 
% 
%             AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));
% 
%             VyCurr = V .* sin(SA_CG(j)); % Body coord velocity
%             VxCurr = V .* cos(SA_CG(j));
% 
%             %%%%% VEHICLE BODY YAW RATE
% 
%             % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%             % itOmega(c) = 0;
% 
%             %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             %%% Equation 1
%             itOmega(c) = VxCurr/R;
% 
%             % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             % %%% Equation 2
%             % if c == 1
%             %     % Nothing since relaxation parameter requires last index
%             % else
%             %     itOmega(c) = AyVelCurr/V * (1-pr) + itOmega(c-1)*pr;
%             % end
% 
%             Omega = itOmega(c);
% 
%             for p = 1:4
% 
%                 %%%%% WHEEL SLIP ANGLE EQUATIONS
% 
%                 %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
% 
%                 if c == 1
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 2
%                     SA_Wheel(p,1) = atan( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) / (R.*cos(SA_CG(j)) - coord_AllW(2,p)) )...
%                                     - dSteer_AllW(i,p);
%                 else
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 1
%                     SA_Wheel(p,1) = atan( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
%                                         / (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
% 
%                 end
% 
% 
%                %%%%% WHEEL SPEED EQUATIONS
% 
%                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                % V_Wheel(p,1) = 0;
% 
%                %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%                V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
% 
%             end
% 
%             dFzf_dAx = (hCG .* m)./(2.* WB);
%             dFzf_dAy = 1*(hCG .* m .* PFront)/TWf;
%             dFzr_dAy = 1*(hCG .* m .* (1-PFront))/TWr;
% 
%             Fz_Wf = (m.*g.* PFront)/2;
%             Fz_Wr = (m.*g.* (1-PFront))/2;
% 
%             Fz(1,1) = Fz_Wf + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
%             Fz(2,1) = Fz_Wf + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
%             Fz(3,1) = Fz_Wr - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
%             Fz(4,1) = Fz_Wr - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
% 
%             for p = 1:4
%                 [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire, rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_Wheel(p), Idx, Model);
% 
%                 %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 TM_Fx(p) = TM_Fx(p) .* 0; 
% 
%                 % Calspan TTC Data usual correction factor - 0.7
%                 TM_Fx(p) = TM_Fx(p) .* 0.7;
%                 % Tire Model outputs in opposite Y coordinates
%                 TM_Fy(p) = (1).* TM_Fy(p) .* 0.7;
% 
%                 FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
%                 FyTire(p,1) = 1*TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
% 
%                 % Made it a matrix sum so its not ugly :)
%                 MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
%                                    -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
%                                     coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
%                                     coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
%             end
% 
%             FxBody = sum(FxTire);
%             FyBody = sum(FyTire);
%             MzBody = sum(MzTire);
% 
%             itFz(:,c) = Fz;
%             itFyTire(:,c) =  FyTire;
%             itAyBody(c+1,1) = FyBody/m;
%             itAxBody(c+1,1) = FxBody/m;
% 
%             % Using dimensionless G's to iteration (apparently improves
%             % stability)?
%             itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
%             res = itCAyBody(c+1) - itCAyBody(c);
% 
%             % Tests the c 
%             plot(1:c+1, itAyBody);
% 
%             c = c + 1;
% 
%             if c > 1000
%                 iterationCtrl = itAyBody( (c-10) : (c-1) );
%                 itAyBody(c) = min(iterationCtrl);
%                 break
%             end
% 
%         end % while loop end
%         % if j == 13
%         %     return
%         % end
%         %saveAyBody(i,j) = (itAyBody(end)+itAyBody(end-1))/2;
%         saveTM_Fy(:,j) = TM_Fy;
%         saveSA_Wheel(:,j) = SA_Wheel;
%         saveFz(:,j) = Fz;
%         saveAyBody(i,j) = itAyBody(end);
%         saveAxBody(i,j) = itAxBody(end);
%         saveMzBody(i,j) = MzBody/(m.*g.*WB);
%         saveItFz(:,j) = Fz;
%         saveItAyBody{j,1} = itAyBody;
% 
%         disp("Steering Angle [" + i + "] Slip Angle [" + j + "] finished, iterations: " +  c)
% 
%     end % SA_CG End
% 
% end % dSteer End


%% SECTION 2: ITERATING WITH CONSTANT VELOCITY INSTEAD OF CONSTANT RADIUS
% Same code just swapped the radius constant with velocity constant,
% shouldnt be too different, but the output is still the same matrixes

% NOTES(11-16-2024): Might need to look into SA_CG vs dSteer ranges, they
% affect the diagram edges - Changes I made to this: removed the negative
% sign on the TM_Fy since the paper TM_Fy seems to give the same
% results?(maybe right side is still postitive even for AY?)(Also might be
% centrifugal acceleration is opposite to TM_Fy since TM_Fy is trying to
% keep you in and centriAccel is pulling you out, and evaluating at quasi
% steady state so technically each one is equal, but if theres a negative
% sign on TM_Fy the diagram loses its edges.

% V = 20; % Velocity [m/s]
% 
% % Mat Initialization
% SA_Wheel = zeros(4,1);
% V_Wheel = zeros(4,1);
% TM_Fx = zeros(4,1);
% TM_Fy = zeros(4,1);
% FxTire = zeros(4,1);
% FyTire = zeros(4,1);
% MzTire = zeros(4,1);
% saveAyBody = zeros(length(dSteer), length(SA_CG));
% saveAxBody = zeros(length(dSteer), length(SA_CG));
% saveMzBody = zeros(length(dSteer), length(SA_CG));
% saveItFz = [];
% saveTM_Fy = zeros(4,length(SA_CG));
% 
% pr = 0.1; % Yaw Rate (Omega) iteration relaxation parameter for convergence
% 
% 
% for i = 1:length(dSteer)
% 
%     for j = 1:length(SA_CG)
% 
%         AxGuess = 0;
%         AyGuess = 0.1;
%         res = 1;
%         tol = 1e-3;
% 
%         itAyBody = [];
%         itAyBody(1,1) = AyGuess;
%         itAxBody = [];
%         itAxBody(1,1) = AxGuess;
%         itCAyBody = [];
%         itCAyBody(1,1) = AxGuess/g;
%         itCAxBody = [];
%         itCAxBody(1,1) = AxGuess/g;
%         itFz = [];
%         itFyTire = [];
% 
%         %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%         itOmega = [];
%         itOmega(1,1) = 0;
% 
%         c = 1;
% 
%         while abs(res) > tol  
% 
%             AyCurr = itAyBody(c);
%             AxCurr = itAxBody(c);
% 
%             AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));
% 
%             VyCurr = V .* sin(SA_CG(j));
%             VxCurr = V .* cos(SA_CG(j));
% 
%             %%%%% VEHICLE BODY YAW RATE
% 
%             % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%             % Omega = 0;
% 
%             % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             % %%% Equation 1
%             % R = V.^2 / AyVelCurr;
%             % Omega = V/R;
% 
%             %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             %%% Equation 2
%             if c == 1
%                 % Nothing since relaxation parameter requires last index
%             else
%                 itOmega(c) = AyVelCurr/VxCurr * (1-pr) + itOmega(c-1)*pr;
%             end
%             R =  (VxCurr.^2 / AyVelCurr);
%             Omega = itOmega(c);
% 
%             for p = 1:4
% 
%                 %%%%% WHEEL SLIP ANGLE EQUATIONS
%                 % 
%                 % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 % SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
%                 % % 
%                 if c == 1
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 2
%                     SA_Wheel(p,1) = atan( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) / (R.*cos(SA_CG(j)) - coord_AllW(2,p)) )...
%                                     - dSteer_AllW(i,p);
% 
%                 else
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 1
%                     SA_Wheel(p,1) = atan( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
%                                         / (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
% 
%                 end
% 
% 
%                %%%%% WHEEL SPEED EQUATIONS
% 
%                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                % V_Wheel(p,1) = 0;
% 
%                %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%                V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
% 
%             end
% 
%             dFzf_dAx = (hCG .* m)./(2.* WB);
%             dFzf_dAy = 1*(hCG .* m .* PFront)/TWf;
%             dFzr_dAy = 1*(hCG .* m .* (1-PFront))/TWr;
% 
%             Fz_Wf = (m.*g.* PFront)/2;
%             Fz_Wr = (m.*g.* (1-PFront))/2;
% 
%             Fz(1,1) = Fz_Wf + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
%             Fz(2,1) = Fz_Wf + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
%             Fz(3,1) = Fz_Wr - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
%             Fz(4,1) = Fz_Wr - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
% 
%             for p = 1:4
%                 [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire, rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_Wheel(p), Idx, Model);
% 
%                 %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 TM_Fx(p) = TM_Fx(p) .* 1; 
% 
%                 % Calspan TTC Data usual correction factor - 0.7
%                 TM_Fx(p) = TM_Fx(p) .* 0.7;
%                 % Tire Model outputs in opposite Y coordinates
%                 TM_Fy(p) = (1).* TM_Fy(p) .* 0.7;
% 
%                 FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
%                 FyTire(p,1) = 1*TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
% 
%                 % Made it a matrix sum so its not ugly :)
%                 MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
%                                    -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
%                                     coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
%                                     coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
%             end
% 
%             FxBody = sum(FxTire);
%             FyBody = sum(FyTire);
%             MzBody = sum(MzTire);
% 
%             itFz(:,c) = Fz;
%             itFyTire(:,c) =  FyTire;
%             itAyBody(c+1,1) = FyBody/m;
%             itAxBody(c+1,1) = FxBody/m;
% 
%             % Using dimensionless G's to iteration (apparently improves
%             % stability)?
%             itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
%             itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
%             res = itCAyBody(c+1) - itCAyBody(c);
%             resAx = itCAxBody(c+1) - itCAxBody(c);
%             % Tests the c 
%             plot(1:c+1, itAyBody);
% 
%             c = c + 1;
% 
%             if c > 1000
%                 iterationCtrl = itAyBody( (c-10) : (c-1) );
%                 itAyBody(c) = min(iterationCtrl);
%                 break
%             end
% 
%         end % while loop end
%         % if j == 13
%         %     return
%         % end
%         %saveAyBody(i,j) = (itAyBody(end)+itAyBody(end-1))/2;
%         saveTM_Fy(:,j) = TM_Fy;
%         saveSA_Wheel(:,j) = SA_Wheel;
%         saveFz(:,j) = Fz;
%         saveAyBody(i,j) = itAyBody(end);
%         saveAxBody(i,j) = itAxBody(end);
%         saveMzBody(i,j) = MzBody/(m.*g.*WB);
%         saveItFz(:,j) = Fz;
%         saveItAyBody{j,1} = itAyBody;
% 
%         disp("Steering Angle [" + i + "] Slip Angle [" + j + "] finished, iterations: " +  c)
% 
%     end % SA_CG End
% 
% end % dSteer End


%% SECTION 3: TESTING FREE ROLLING LEVEL SURFACES - CONSTANT VELOCITY

% V = 15; % Velocity [m/s]
% 
% %%% SELECT RANGES FOR BODY SLIP AND STEERING ANGLES
% SA_CG = SA_CG;
% dSteer = dSteer;
% 
% % Mat Initialization
% SA_Wheel = zeros(4,1);
% V_Wheel = zeros(4,1);
% TM_Fx = zeros(4,1);
% TM_Fy = zeros(4,1);
% FxTire = zeros(4,1);
% FyTire = zeros(4,1);
% MzTire = zeros(4,1);
% saveAyBody = zeros(length(dSteer), length(SA_CG));
% saveAxBody = zeros(length(dSteer), length(SA_CG));
% saveMzBody = zeros(length(dSteer), length(SA_CG));
% saveItFz = [];
% saveTM_Fy = zeros(length(dSteer),length(SA_CG),4);
% 
% % Yaw Rate (Omega) iteration relaxation parameter for convergence:
% % Parameter is between [0-1], is dependent upon how fast the simulation is
% % ran, speeds below tangent speed pr<0.5, speeds above tangent speeds are
% % more stable and requires pr>0.6
% pr = 0.001; 
% 
% 
% f = waitbar(0, 'Starting');
% tic
% 
% 
% for i = 1:length(dSteer)
% 
%     for j = 1:length(SA_CG)
% 
%         AxGuess = 0;
%         AyGuess = 0;
%         res = 1;
%         tol = 1e-3;
% 
%         itAyBody = [];
%         itAyBody(1,1) = AyGuess;
%         itAxBody = [];
%         itAxBody(1,1) = AxGuess;
%         itCAyBody = [];
%         itCAyBody(1,1) = AxGuess/g;
%         itCAxBody = [];
%         itCAxBody(1,1) = AxGuess/g;
%         itFz = [];
%         itFyTire = [];
% 
%         %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%         itOmega = [];
%         itOmega(1,1) = 0;
% 
%         c = 1;
% 
%         while abs(res) > tol  
% 
%             AyCurr = itAyBody(c);
%             AxCurr = itAxBody(c);
% 
%             AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));
% 
%             VyCurr = V .* sin(SA_CG(j));
%             VxCurr = V .* cos(SA_CG(j));
% 
%             %%%%% VEHICLE BODY YAW RATE
% 
%             % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%             % Omega = 0;
% 
%             % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             % %%% Equation 1
%             % R = V.^2 / AyVelCurr;
%             % Omega = V/R;
% 
%             %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%             %%% Equation 2
%             if c == 1
%                 % Nothing since relaxation parameter requires last index
%             else
%                 itOmega(c) = AyVelCurr/VxCurr * (1-pr) + itOmega(c-1)*pr;
%             end
%             R =  VxCurr.^2 / AyVelCurr;
%             Omega = itOmega(c);
% 
%             for p = 1:4
% 
%                 %%%%% WHEEL SLIP ANGLE EQUATIONS
%                 % 
%                 % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 % SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
%                 % % 
%                 if  false
% 
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 2
%                     SA_Wheel(p,1) = atan2( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) , ...
%                         (R.*cos(SA_CG(j)) - coord_AllW(2,p)) ) - dSteer_AllW(i,p);
% 
%                 else
% 
%                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
%                     %%% Equation 1
%                     SA_Wheel(p,1) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
%                                         , (VxCurr - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
% 
%                 end
% 
% 
%                %%%%% WHEEL SPEED EQUATIONS
% 
%                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                % V_Wheel(p,1) = 0;
% 
%                %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
%                V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
% 
%             end
% 
% 
%             dFzf_dAx = (hCG .* m)./(2.* WB);
%             dFzf_dAy = (hCG .* m .* PFront)/TWf;
%             dFzr_dAy = (hCG .* m .* (1-PFront))/TWr;
% 
%             FzAero = (1/2)*rho*crossA*Cl*V^2;
%             Fz_WeightFront = (m.*g.* PFront + FzAero .* CoP)/2;
%             Fz_WeightRear = (m.*g.* (1-PFront) + FzAero .* (1 - CoP))/2;
% 
%             Fz(1,1) = Fz_WeightFront + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
%             Fz(2,1) = Fz_WeightFront + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
%             Fz(3,1) = Fz_WeightRear - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
%             Fz(4,1) = Fz_WeightRear - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
% 
%             for p = 1:4
%                 if p == 1 
%                     TireInclination = sign(SA_CG(j)) .* -TireInclinationFront;
%                 elseif p == 2
%                     TireInclination = TireInclinationFront;
%                 elseif p == 3
%                     TireInclination = sign(SA_CG(j)) .* -TireInclinationRear;
%                 else
%                     TireInclination = TireInclinationRear;
%                 end
% 
% 
%                 [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire,...
%                     rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure ,...
%                     TireInclination, V_Wheel(p), Idx, Model);
% 
%                 %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
%                 TM_Fx(p) = TM_Fx(p) .* 0; 
% 
%                 % Calspan TTC Data usual correction factor - 0.7
%                 TM_Fx(p) = TM_Fx(p) .* 0.7;
%                 % Tire Model outputs in opposite Y coordinates
%                 TM_Fy(p) = (1).* TM_Fy(p) .* 0.56;
% 
%                 FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
%                 FyTire(p,1) = TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
% 
%                 % Made it a matrix sum so its not ugly :)
%                 MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
%                                    -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
%                                     coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
%                                     coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
%             end
% 
%             FxDrag = (1/2)*rho*crossA*Cd*V^2;
% 
%             FxBody = sum(FxTire) - FxDrag;
%             FyBody = sum(FyTire);
%             MzBody = sum(MzTire);
% 
%             itFz(:,c) = Fz;
%             itFyTire(:,c) =  FyTire;
%             itAyBody(c+1,1) = FyBody/m;
%             itAxBody(c+1,1) = FxBody/m;
% 
%             % Using dimensionless G's to iteration (apparently improves
%             % stability)?
%             itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
%             itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
%             res = itCAyBody(c+1) - itCAyBody(c);
%             resAx = itCAxBody(c+1) - itCAxBody(c);
% 
%             % %%% CAN COMMENT IN AND WILL DISPLAY THE ITERATIONS THAT IT IS
%             % %%% STUCK ON BUT MASSIVELY IMPACTS PERFORMANCE - ONLY USE WHEN
%             % %%% THE SIMULATION IS SEEMINGLY STUCK
%             %
%             % % Tests the c 
%             % plot(1:c+1, itAyBody);
% 
%             c = c + 1;
% 
%             % If the iterations do not converge, take an average of last
%             % 100 values and set that
%             if c > 1000
%                 iterationCtrl = itAyBody( (c-100) : (c-1) );
%                 itAyBody(c) = min(iterationCtrl);
%                 break
%             end
% 
%         end % while loop end
% 
%         for p = 1:4
%         saveSA_WheelTerm(p,j) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
%                                         , (VxCurr - Omega .* coord_AllW(2,p)) );
%         end
% 
%         saveOmega(i,j) = Omega;
%         saveTM_Fy(:,j,i) = TM_Fy;
%         saveSA_Wheel(:,j,i) = SA_Wheel;
%         saveFzWheel(:,j,i) = Fz;
% 
%         saveAyBody(i,j) = itAyBody(end);
%         saveAxBody(i,j) = itAxBody(end);
%         saveMzBody(i,j) = MzBody/(m.*g.*WB);
% 
%         saveAxVel(i,j) = saveAxBody(i,j).* cos(SA_CG(j)) + saveAyBody(i,j) .* sin(SA_CG(j));
%         saveAyVel(i,j) = saveAyBody(i,j).* cos(SA_CG(j)) - saveAxBody(i,j) .* sin(SA_CG(j));
%         saveCAxVel(i,j) = saveAxVel(i,j)/g;
%         saveCAyVel(i,j) = saveAyVel(i,j)/g;
% 
% 
%         saveItFz(:,j) = Fz;
%         saveItAyBody{j,1} = itAyBody;
% 
% 
%     end % SA_CG End
% 
%     disp("Steering Angle [" + i + "] finished, iterations: " +  c)
%     waitbar(i/length(dSteer), f, sprintf('Progress: %d %%', floor(i/length(dSteer)*100)));
% 
% end % dSteer End
% 
% close(f)
% toc


%% SECTION 4: TESTING ACCELERATION LEVEL SURFACES - CONSTANT VELOCITY

V = 30; % Velocity [m/s]

% Mat Initialization
SA_Wheel = zeros(4,1);
V_Wheel = zeros(4,1);
TM_Fx = zeros(4,1);
TM_Fy = zeros(4,1);
FxTire = zeros(4,1);
FyTire = zeros(4,1);
TireFxTarget = zeros(4, 1);
MzTire = zeros(4,1);
saveAyBody = zeros(length(dSteer), length(SA_CG));
saveAxBody = zeros(length(dSteer), length(SA_CG));
saveMzBody = zeros(length(dSteer), length(SA_CG));


saveAxVel = zeros(size(saveAyBody));
saveAyVel = zeros(size(saveAyBody));
saveCAyVel = zeros(size(saveAyBody));
saveCAxVel = zeros(size(saveAyBody));

% Yaw Rate (Omega) iteration relaxation parameter for convergence:
% Parameter is between [0-1], is dependent upon how fast the simulation is
% ran, speeds below tangent speed pr<0.5, speeds above tangent speeds are
% more stable and requires pr>0.6
pr = 0.000; 


%%% CHOOSE RANGE FOR LONGITUDINAL ACCELERATION
%%%
%%%
targetCAx = 100; % G's
% f = waitbar(0, 'Starting');
tic


for i = 1:length(dSteer)
    
    sumc = 1;

    for j = 1:length(SA_CG)
        tireSR = zeros(4, 1);

        AxGuess = 0;
        AyGuess = 0;
        res = 1;
        resCAx = 1;
        Ferror = 1;
        tol = 1e-3;

        itAyBody = [];
        itAyBody(1,1) = AyGuess;
        itAxBody = [];
        itAxBody(1,1) = AxGuess;
        itCAyBody = [];
        itCAyBody(1,1) = AyGuess/g;
        itCAxBody = [];
        itCAxBody(1,1) = AxGuess/g;
        itFz = [];
        itFyTire = [];

        %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
        itOmega = [];
        itOmega(1,1) = 0;

        c = 1;

        while abs(res) > tol || abs(resCAx) > tol

            AyCurr = itAyBody(c);
            AxCurr = itAxBody(c);

            AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));

            VyCurr = V .* sin(SA_CG(j));
            VxCurr = V .* cos(SA_CG(j));

            %%%%% VEHICLE BODY YAW RATE

            % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
            % Omega = 0;

            % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
            % %%% Equation 1
            % R = V.^2 / AyVelCurr;
            % Omega = V/R;

            %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
            %%% Equation 2
            if c == 1
                % Nothing since relaxation parameter requires last index
            else
                itOmega(c) = AyVelCurr/VxCurr * (1-pr) + itOmega(c-1)*pr;
            end
            R =  VxCurr.^2 / AyVelCurr;
            Omega = itOmega(c);

            for p = 1:4

                %%%%% WHEEL SLIP ANGLE EQUATIONS
                % 
                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                % SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
                % % 
                if  false

                    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 2
                    SA_Wheel(p,1) = atan2( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) , ...
                        (R.*cos(SA_CG(j)) - coord_AllW(2,p)) ) - dSteer_AllW(i,p);

                else

                    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 1
                    SA_Wheel(p,1) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                        , (VxCurr - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);

                end


               %%%%% WHEEL SPEED EQUATIONS

               % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
               % V_Wheel(p,1) = 0;

               %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
               V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );

            end


            dFzf_dAx = (hCG .* m)./(2.* WB);
            dFzf_dAy = (hCG .* m .* PFront)/TWf;
            dFzr_dAy = (hCG .* m .* (1-PFront))/TWr;

            FzAero = (1/2)*rho*crossA*Cl*V^2;
            Fz_WeightFront = (m.*g.* PFront + FzAero .* CoP)/2;
            Fz_WeightRear = (m.*g.* (1-PFront) + FzAero .* (1 - CoP))/2;

            Fz(1,1) = Fz_WeightFront + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
            Fz(2,1) = Fz_WeightFront + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
            Fz(3,1) = Fz_WeightRear - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
            Fz(4,1) = Fz_WeightRear - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;

            for p = 1:4
                %%% Still need to figure out tire inclination
                %%% stuff--------- camber changes based on wheel SA, body
                %%% SA, and which wheel it is
                if p == 1 
                    TireInclination = -sign(SA_Wheel(p)) .* TireInclinationFront;
                elseif p == 2
                    TireInclination =  -sign(SA_Wheel(p)).* -TireInclinationFront;
                elseif p == 3
                    TireInclination = -sign(SA_Wheel(p)) .* TireInclinationRear;
                else
                    TireInclination =  -sign(SA_Wheel(p)) .* -TireInclinationRear;
                end


                [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire,...
                    rad2deg(SA_Wheel(p)), tireSR(p), Fz(p) , TirePressure ,...
                    TireInclination, V_Wheel(p), Idx, Model);

                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                % TM_Fx(p) = TM_Fx(p) .* 0; 

                % Calspan TTC Data usual correction factor - 0.7
                TM_Fx(p) = TM_Fx(p) .* 0.7;
                % Tire Model outputs in opposite Y coordinates
                TM_Fy(p) = (1).* TM_Fy(p) .* 0.53;

                FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                FyTire(p,1) = TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));

                % Made it a matrix sum so its not ugly :)
                MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                   -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
                                    coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
                                    coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
            end

            FxDrag = (1/2)*rho*crossA*Cd*V^2;

            FxBody = sum(FxTire) - FxDrag;
            FyBody = sum(FyTire);
            MzBody = sum(MzTire);

            itAyBody(c+1,1) = FyBody/m;
            itAxBody(c+1,1) = FxBody/m;
            itAxVel = itAxBody(c+1,1).* cos(SA_CG(j)) + itAyBody(c+1,1) .* sin(SA_CG(j));

            % Using dimensionless G's to iteration (apparently improves
            % stability)?
            itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
            itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
            res = itCAyBody(c+1) - itCAyBody(c);
            resCAx = itCAxBody(c+1) - itCAxBody(c);

            % %%% CAN COMMENT IN AND WILL DISPLAY THE ITERATIONS THAT IT IS
            % %%% STUCK ON BUT MASSIVELY IMPACTS PERFORMANCE - ONLY USE WHEN
            % %%% THE SIMULATION IS SEEMINGLY STUCK
            %
            % % Tests the c 
            % plot(1:c+1, itCAxBody);

            c = c + 1;
            
            % If the iterations do not converge, take an average of last
            % 100 values and set that
            if c > 1000
                iterationCtrl = itAyBody( (c-100) : (c-1) );
                itAyBody(c) = min(iterationCtrl);
                break
            end

        end % while loop end
    
        sumc = sumc + c;
        for p = 1:4
        saveSA_WheelTerm(p,j) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                        , (VxCurr - Omega .* coord_AllW(2,p)) );
        end

        saveOmega(i,j) = Omega;
        saveTM_Fy(:,j,i) = TM_Fy;
        saveSA_Wheel(:,j,i) = SA_Wheel;
        saveFzWheel(:,j,i) = Fz;

        saveAyBody(i,j) = itAyBody(end);
        saveAxBody(i,j) = itAxBody(end);
        saveCAxBody(i,j) = itCAxBody(end);
        saveMzBody(i,j) = MzBody/(m.*g.*WB);

        saveAxVel(i,j) = saveAxBody(i,j).* cos(SA_CG(j)) + saveAyBody(i,j) .* sin(SA_CG(j));
        saveAyVel(i,j) = saveAyBody(i,j).* cos(SA_CG(j)) - saveAxBody(i,j) .* sin(SA_CG(j));
        saveCAxVel(i,j) = saveAxVel(i,j)/g;
        saveCAyVel(i,j) = saveAyVel(i,j)/g;


        saveItFz(:,j) = Fz;
        saveItAyBody{j,1} = itAyBody;
    end % SA_CG End
    
    disp("Steering Angle [" + i + "] finished, Avg Iterations: " +  sumc/length(SA_CG))
    % waitbar(i/length(dSteer), f, sprintf('Progress: %d %%', floor(i/length(dSteer)*100)));
    
end % dSteer End

% close(f)
toc

%% SECTION 4 CONTINUED: ADJUSTING AX FOR EACH POINT

driveCondition = (ones(size(saveCAxVel)).* targetCAx - saveCAxVel) > 0;

for i = 1:length(dSteer)
    
    sumc = 1;

    for j = 1:length(SA_CG)
        tireSR = zeros(4, 1);

        res = 1;
        resCAx = 1;
        Ferror = 1;
        tol = 1e-3;

        itAyBody = [];
        itAyBody(1,1) = saveAyBody(i, j);
        itAxBody = [];
        itAxBody(1,1) = saveAxBody(i, j);
        itCAyBody = [];
        itCAyBody(1,1) = saveAyBody(i, j)/g;
        itCAxBody = [];
        itCAxBody(1,1) = saveAxBody(i, j)/g;
        itFz = [];
        itFyTire = [];

        %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
        itOmega = [];
        itOmega(1,1) = 0;

        c = 1;

        while abs(res) > tol || abs(resCAx) > tol

            % level surface stuff
            Ferror = m * g * (targetCAx - itAxVel/g);
            % determine drive/brake condition
            if abs(Ferror) > 0
                if driveCondition(i, j)
                    % drive
                    TireFxTarget = zeros(4, 1);
                    TireFxTarget([3, 4]) = (sum(TM_Fx([3, 4])) + Ferror) / 2;
                else
                    % brake
                    TireFxTarget([1, 2]) = B_FBB * (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                    TireFxTarget([3, 4]) = (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                end
                
                tireSR = calcSR(TireFxTarget, driveCondition(i, j), B_FBB, Tire, rad2deg(SA_Wheel), Fz , TirePressure ,...
                        TireInclination, V_Wheel, Idx, Model);
            end

            AyCurr = itAyBody(c);
            AxCurr = itAxBody(c);

            AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));

            VyCurr = V .* sin(SA_CG(j));
            VxCurr = V .* cos(SA_CG(j));

            %%%%% VEHICLE BODY YAW RATE

            % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
            % Omega = 0;

            % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
            % %%% Equation 1
            % R = V.^2 / AyVelCurr;
            % Omega = V/R;

            %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
            %%% Equation 2
            if c == 1
                % Nothing since relaxation parameter requires last index
            else
                itOmega(c) = AyVelCurr/VxCurr * (1-pr) + itOmega(c-1)*pr;
            end
            R =  VxCurr.^2 / AyVelCurr;
            Omega = itOmega(c);

            for p = 1:4

                %%%%% WHEEL SLIP ANGLE EQUATIONS
                % 
                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                % SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
                % % 
                if  false

                    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 2
                    SA_Wheel(p,1) = atan2( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) , ...
                        (R.*cos(SA_CG(j)) - coord_AllW(2,p)) ) - dSteer_AllW(i,p);

                else

                    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 1
                    SA_Wheel(p,1) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                        , (VxCurr - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);

                end


               %%%%% WHEEL SPEED EQUATIONS

               % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
               % V_Wheel(p,1) = 0;

               %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
               V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );

            end


            dFzf_dAx = (hCG .* m)./(2.* WB);
            dFzf_dAy = (hCG .* m .* PFront)/TWf;
            dFzr_dAy = (hCG .* m .* (1-PFront))/TWr;

            FzAero = (1/2)*rho*crossA*Cl*V^2;
            Fz_WeightFront = (m.*g.* PFront + FzAero .* CoP)/2;
            Fz_WeightRear = (m.*g.* (1-PFront) + FzAero .* (1 - CoP))/2;

            Fz(1,1) = Fz_WeightFront + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
            Fz(2,1) = Fz_WeightFront + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
            Fz(3,1) = Fz_WeightRear - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
            Fz(4,1) = Fz_WeightRear - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;

            for p = 1:4
                %%% Still need to figure out tire inclination
                %%% stuff--------- camber changes based on wheel SA, body
                %%% SA, and which wheel it is
                if p == 1 
                    TireInclination = -sign(SA_Wheel(p)) .* TireInclinationFront;
                elseif p == 2
                    TireInclination =  -sign(SA_Wheel(p)).* -TireInclinationFront;
                elseif p == 3
                    TireInclination = -sign(SA_Wheel(p)) .* TireInclinationRear;
                else
                    TireInclination =  -sign(SA_Wheel(p)) .* -TireInclinationRear;
                end


                [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire,...
                    rad2deg(SA_Wheel(p)), tireSR(p), Fz(p) , TirePressure ,...
                    TireInclination, V_Wheel(p), Idx, Model);

                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                % TM_Fx(p) = TM_Fx(p) .* 0; 

                % Calspan TTC Data usual correction factor - 0.7
                TM_Fx(p) = TM_Fx(p) .* 0.7;
                % Tire Model outputs in opposite Y coordinates
                TM_Fy(p) = (1).* TM_Fy(p) .* 0.53;

                FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                FyTire(p,1) = TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));

                % Made it a matrix sum so its not ugly :)
                MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                   -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
                                    coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
                                    coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
            end

            FxDrag = (1/2)*rho*crossA*Cd*V^2;

            FxBody = sum(FxTire) - FxDrag;
            FyBody = sum(FyTire);
            MzBody = sum(MzTire);

            itAyBody(c+1,1) = FyBody/m;
            itAxBody(c+1,1) = FxBody/m;
            itAxVel = itAxBody(c+1,1).* cos(SA_CG(j)) + itAyBody(c+1,1) .* sin(SA_CG(j));

            % Using dimensionless G's to iteration (apparently improves
            % stability)?
            itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
            itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
            res = itCAyBody(c+1) - itCAyBody(c);
            resCAx = itCAxBody(c+1) - itCAxBody(c);

            % %%% CAN COMMENT IN AND WILL DISPLAY THE ITERATIONS THAT IT IS
            % %%% STUCK ON BUT MASSIVELY IMPACTS PERFORMANCE - ONLY USE WHEN
            % %%% THE SIMULATION IS SEEMINGLY STUCK
            %
            % % Tests the c 
            % plot(1:c+1, itCAxBody);

            c = c + 1;
            
            % If the iterations do not converge, take an average of last
            % 100 values and set that
            if c > 1000
                iterationCtrl = itAyBody( (c-100) : (c-1) );
                itAyBody(c) = min(iterationCtrl);
                itCAxBody(c) = mean(itCAxBody(end-100:end));
                itCAxBody(c) = mean(itCAxBody(end-100:end));
                disp("over iter limit")
                break
            end

        end % while loop end
    
        sumc = sumc + c;
        for p = 1:4
        saveSA_WheelTerm(p,j) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                        , (VxCurr - Omega .* coord_AllW(2,p)) );
        end

        saveOmega(i,j) = Omega;
        saveTM_Fy(:,j,i) = TM_Fy;
        saveSA_Wheel(:,j,i) = SA_Wheel;
        saveFzWheel(:,j,i) = Fz;

        saveAyBody(i,j) = itAyBody(end);
        saveAxBody(i,j) = itAxBody(end);
        saveCAxBody(i,j) = itCAxBody(end);
        saveMzBody(i,j) = MzBody/(m.*g.*WB);

        saveAxVel(i,j) = saveAxBody(i,j).* cos(SA_CG(j)) + saveAyBody(i,j) .* sin(SA_CG(j));
        saveAyVel(i,j) = saveAyBody(i,j).* cos(SA_CG(j)) - saveAxBody(i,j) .* sin(SA_CG(j));
        saveCAxVel(i,j) = saveAxVel(i,j)/g;
        saveCAyVel(i,j) = saveAyVel(i,j)/g;


        saveItFz(:,j) = Fz;
        saveItAyBody{j,1} = itAyBody;
    end % SA_CG End
    
    disp("Steering Angle [" + i + "] finished, Avg Iterations: " +  sumc/length(SA_CG))
    % waitbar(i/length(dSteer), f, sprintf('Progress: %d %%', floor(i/length(dSteer)*100)));
    
end % dSteer End


%% FINDING HIGHEST STEADY STATE AY

zeroMz_CAy_SA = zeros(length(SA_CG),1);
zeroMz_CAy_ST = zeros(length(dSteer),1);
zeroMz_SteerDeg = zeros(length(SA_CG),1);
zeroMz_SADeg = zeros(length(dSteer),1);

% Go Through the Mz Data through SlipAngle(columns) to check the SA lines
% ---- CAUTIONS: This only finds the first point along the line that
% crosses the zero boundary
% Also interpolates for the other angle values not used (SA or Steer)
for j = 1:length(SA_CG)
    checkMat = saveMzBody(2:end,j).*saveMzBody(1:end-1,j);
    avgAx = (saveCAxVel(2:end, j) + saveCAxVel(1:end-1, j)) ./ 2;
    indexSS = abs(avgAx - targetCAx) < 5e-2 & checkMat < 0;
    indexSS = find(indexSS, 1, 'first');
    if isempty(indexSS)

        % The below creates the minimum value of when the line doesnt cross
        % the 0 value, but need to return 0 in case the minimum value is larger
        % than the actual legit maximum SS Ay
        %
        % [minMz,indexMin] = min(abs(saveMzBody(:,j)));
        % zeroMz_CAy_SA(j) = saveCAyVel(indexMin,j);
        % zeroMz_Steer(j) = dSteer(indexMin);

        zeroMz_CAy_SA(j) = 0;
        zeroMz_SteerDeg(j) = 0;
    else
        zeroMz_CAy_SA(j) = interp1( saveMzBody(indexSS:indexSS+1, j),...
                                    saveCAyVel(indexSS:indexSS+1, j), 0.0, 'linear');
        zeroMz_SteerDeg(j) = rad2deg(interp1( saveMzBody(indexSS:indexSS+1, j),...
                                   dSteer(indexSS:indexSS+1), 0.0, 'linear'));
    end
end

% Go Through the Mz Data through Steer Angle(rows) to check the steer lines
% ---- CAUTIONS: This only finds the first point along the line that
% crosses the zero boundary
for i = 1:length(dSteer)
    checkMat = saveMzBody(i, 2:end).*saveMzBody(i, 1:end-1);
    avgAx = (saveCAxVel(i, 2:end) + saveCAxVel(i, 1:end-1)) ./ 2;
    indexSS = abs(avgAx - targetCAx) < 5e-2 & checkMat < 0;
    indexSS = find(indexSS, 1, 'first');
    if isempty(indexSS)
        zeroMz_CAy_ST(i) = 0;
        zeroMz_SADeg(i) = 0;
    else
        zeroMz_CAy_ST(i) = interp1( saveMzBody(i, indexSS:indexSS+1),...
                                    saveCAyVel(i, indexSS:indexSS+1), 0.0, 'linear');
        zeroMz_SADeg(i) = rad2deg(interp1( saveMzBody(i, indexSS:indexSS+1),...
                                   SA_CG(indexSS:indexSS+1), 0.0, 'linear'));
    end
end

% Finds the maximum CAy value in both methods and compares and assigns the
% label values for SA and Steer
[maxCAy_SA, maxCAy_SA_ind] = max(zeroMz_CAy_SA);
[maxCAy_ST, maxCAy_ST_ind] = max(zeroMz_CAy_ST);

CAyMax = max(maxCAy_SA, maxCAy_ST);

if max(maxCAy_SA, maxCAy_ST) == maxCAy_SA
    SA_CAyMax = rad2deg(SA_CG(maxCAy_SA_ind));
    ST_CAyMax = zeroMz_SteerDeg(maxCAy_SA_ind);
else
    SA_CAyMax = zeroMz_SADeg(maxCAy_ST_ind);
    ST_CAyMax = rad2deg(dSteer(maxCAy_ST_ind));
end


%% Finding Stability and Control Derivatives

%%% Finds control derivative (Mz/steer slope on the SA line)
if sum(ismember(SA_CG, 0)) > 0
    deriv1 = gradient(saveMzBody(:, SA_CG == 0)) ./ rad2deg(gradient(dSteer));
    if sum(ismember(dSteer, 0)) > 0
        controlVal = deriv1((find(dSteer == 0))) .* (m.*g.*WB);
    else
        disp('Choose an odd number of array values for dSteer doofus');
    end
else
    disp('Choose an odd number of array values for SA_CG doofus');
end

%%% Finds stability derivative (Mz/SA slope on the steer line)
if sum(ismember(dSteer, 0)) > 0
    deriv2 = gradient(saveMzBody(dSteer == 0,:))' ./ rad2deg(gradient(SA_CG));
    if sum(ismember(SA_CG, 0)) > 0
        %%% This value is negative because the values for steering starts
        %%% on the right side of the map going to the left side
        %%% ------------- Maybe something wrong with the coordinates again?
        stabilityVal = -deriv2((find(SA_CG == 0))) .* (m.*g.*WB);
    else
        disp('Choose an odd number of array values for SA_CG doofus');
    end
else
    disp('Choose an odd number of array values for dSteer doofus');
end

fprintf('\n-----------------------------------------------------------------')
fprintf('\nControl Derivative Value: ' + string(controlVal)+ ' Nm/deg' +...
        '\nStability Derivative Value: ' + string(stabilityVal) + ' Nm/deg\n');
fprintf('-----------------------------------------------------------------\n')

%% PLOTTING
close all;

% %%% Shifting result numbers - can take out if wanted (Especially when there
% %%% isnt a 0 radians number for the SA_CG and dSteer
% %%% indexes---------------- Have not been implemented
% [dSteerMesh, SAMesh] = meshgrid(dSteer, SA_CG);
% 
% zeroIndex = find(dSteerMesh == 0);
% 
% saveCAxVel = saveCAxVel;


figure
hold on
grid on
for i = 1:length(dSteer)
    steer = plot(saveCAyVel(i,:),saveMzBody(i,:), "Color", "blue", 'LineStyle','--');
    if dSteer(i) == 0
        plot_zeroSteer = plot(saveCAyVel(i,:),saveMzBody(i,:), "Color", "blue", 'LineStyle','-');
    end
end

for i = 1:length(SA_CG)
    slip = plot(saveCAyVel(:,i),saveMzBody(:,i), "Color", "red", 'LineStyle', '--');
    if SA_CG(i) == 0
        plot_zeroSA = plot(saveCAyVel(:,i),saveMzBody(:,i), "Color", "red", "LineStyle", '-');
    end
end

xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
ylabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
xlim([-2,2])
ylim([-1,1])

title({['Free Rolling MMD: Constant Velocity'] ...
       ['Velocity = ', num2str(V),' m/s'] ...
       ['Minimum Radius = ' num2str(V.^2./(CAyMax.*9.81)) ' m']...
      },'Interpreter','latex')


%%% Maximum Ay Plotting Stuff

AyMaxSS = plot(CAyMax, 0, "Marker", ".", "MarkerSize", 20, "Color","g");
label = sprintf(['C0   = %.4g\n',...
                 'SA      = %.3g°\n',...
                 'Steer  = %.3g°'],...
                CAyMax, SA_CAyMax, ST_CAyMax);

text(CAyMax - 0.07, 0.20, label, "FontSize", 8, 'FontWeight','bold', 'Interpreter', 'latex');

legend([steer, slip, AyMaxSS], ...
        {"Constant Steer", ...
         "Constant Slip", ...
         "$C_{Ay_{Max SS}}$"},...
         "Location","northeast",'Interpreter','latex')

%% TESTING
% NOTES FOR FUTURE:12/6/2024: Theres smth wrong, cant get the corners of
% the 3D Ax graph to come down, tried changing track corrdinates, but may
% be something that lies in the problem of SA_Wheel, because non of them
% causes a positive steer angle of the front 2 wheels under positive
% dSteer, so something is wrong with calculating the SA_Wheel with the
% tangent equation, may need to do some hand calculations to see whether
% tangent values actually match up

figure
hold on
grid on
for i = 1:length(dSteer)
    steer = plot(saveCAyVel(i,:),saveCAxVel(i,:), "Color", "blue", 'LineStyle','--');
    % if mod(i,7) == 0
    %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
    %     % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
    % end

end

for i = 1:length(SA_CG)
    slip = plot(saveCAyVel(:,i),saveCAxVel(:,i), "Color", "red");
    % if mod(i,7) == 0
    %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
    %     % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
    % end
end

xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
ylabel("Normalized Longitudinal Accleration $(C_{Ax})$",'Interpreter','latex')

% Get sort the coordinate pairs
points = [saveCAyVel(1:end); saveCAxVel(1:end)];
[~, idx] = sort(points(1, :));
points = points(:, idx);

% scatter(points(1, :), points(2, :))

% split into left and right parts
[~, idx] = min(abs(points(1, :)));
leftPart = points(:, 1:idx);
rightPart = points(:, idx:end);

[~, idx] = sort(leftPart(2, :));
leftPart = leftPart(:, idx);

[~, idx] = sort(rightPart(2, :));
rightPart = rightPart(:, idx);

leftBound = nan(2, 999);
rightBound = nan(2, 999);

AxRange = linspace(-3, 3, 100);
for i = 1:100-1
    % find the points within the interval
    startIdx = find(leftPart(2, :) > AxRange(i), 1, 'first');
    endIdx   = find(leftPart(2, :) < AxRange(i + 1), 1, 'last');
    filteredLeftRange = leftPart(:, startIdx:endIdx);

    startIdx = find(rightPart(2, :) > AxRange(i), 1, 'first');
    endIdx   = find(rightPart(2, :) < AxRange(i + 1), 1, 'last');
    filteredRightRange = rightPart(:, startIdx:endIdx);

    % find the edge point
    [~, idx] = min(filteredLeftRange(1, :));
    if ~isempty(idx)
        leftBound(:, i) = filteredLeftRange(:, idx);
    end

    [~, idx] = max(filteredRightRange(1, :));
    if ~isempty(idx)
        rightBound(:, i) = filteredRightRange(:, idx);
    end
end

leftBound = leftBound(:, ~isnan(leftBound(1, :)));
rightBound = rightBound(:, ~isnan(rightBound(1, :)));

leftBound = unique(leftBound', 'rows')';
rightBound = unique(rightBound', 'rows')';

scatter(leftBound(1, :), leftBound(2, :))
scatter(rightBound(1, :), rightBound(2, :))

figure;
hold on;

accelBound = [leftBound rightBound];
plot(accelBound(1, :), accelBound(2, :))
figure
hold on
grid on

for i = 1:length(dSteer)

    steer = plot3(saveMzBody(i,:),saveCAyVel(i,:),saveCAxVel(i,:), "Color", "blue", 'LineStyle','--');
    % if mod(i,7) == 0
    %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
    %     % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
    % end

end

for i = 1:length(SA_CG)
    slip = plot3(saveMzBody(:,i),saveCAyVel(:,i),saveCAxVel(:,i), "Color", "red", 'LineStyle','--');
    % if mod(i,7) == 0
    %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
    %     % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
    % end
end

zlim([-2, 2])

view(3)
ylabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
xlabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
zlabel("Normalized Longitudinal Accleration $(C_{Ax})$",'Interpreter','latex')
