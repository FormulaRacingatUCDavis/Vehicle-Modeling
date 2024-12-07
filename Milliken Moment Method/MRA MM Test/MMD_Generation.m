clc ; clear;%close all;
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
% 1  2      Postive Y coord
% 3  4
% Backwards
%
% RIGHT HAND TURN RESULTS IN POSTIVE AY(yes you heard that right)


g = 9.81;                   % Grav constant [m/s^2]
m = 280;                    % Total Mass [kg]
PFront = 51.1 /100;           % Percent Mass Front [0-1]
WB = 1.595;                   % Wheelbase [m]
TWf = 1.193;                  % Trackwidth [m]
TWr = 1.193;
toe_f = -0.5 * (pi/180);     % Toe Angles [radians] (positive is inwards)
toe_r = 0 * (pi/180);
hCG = 0.25346;                  % CG height [m]


% TESTING PARAMETER CHANGES
WB = WB;
TWf = TWf;
TWr = TWr;
PFront = PFront;
hCG = hCG;

% Tire Model Parameters
Idx = 1;                    % Moment of Inertia in x for wheel
TirePressure = 70;          % kPa
TireInclination = 1;        % deg 
TireSR = 0;                 % -
Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
load('Hoosier_R25B_16x75-10x7.mat');

%%% SELECT RANGES FOR BODY SLIP AND STEERING ANGLES
SA_CG = deg2rad(linspace(-12,12,31))';
dSteer = deg2rad(linspace(-20,20,31))';


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


%% FREE ROLLING MMD SECTION
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


%% TESTING LEVEL SURFACES
V = 20; % Velocity [m/s]


%%% SELECT RANGES FOR BODY SLIP AND STEERING ANGLES
SA_CG = deg2rad(linspace(-12,12,31))';
dSteer = deg2rad(linspace(-20,20,31))';

dSteer_W1 = toe_f + dSteer;
dSteer_W2 = -toe_f + dSteer;
dSteer_W3 = toe_r + dSteer.*0;
dSteer_W4 = -toe_r + dSteer.*0;
dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];

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
saveTM_Fy = zeros(4,length(SA_CG));

% Yaw Rate (Omega) iteration relaxation parameter for convergence:
% Parameter is between [0-1], is dependent upon how fast the simulation is
% ran, speeds below tangent speed pr<0.5, speeds above tangent speeds are
% more stable and requires pr>0.6
pr = 0.0; 


for i = 1:length(dSteer)

    for j = 1:length(SA_CG)

        AxGuess = 0;
        AyGuess = 0.1;
        res = 1;
        tol = 1e-3;

        itAyBody = [];
        itAyBody(1,1) = AyGuess;
        itAxBody = [];
        itAxBody(1,1) = AxGuess;
        itCAyBody = [];
        itCAyBody(1,1) = AxGuess/g;
        itCAxBody = [];
        itCAxBody(1,1) = AxGuess/g;
        itFz = [];
        itFyTire = [];

        %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
        itOmega = [];
        itOmega(1,1) = 0;

        c = 1;

        while abs(res) > tol  

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
            R =  abs(VxCurr.^2 / AyVelCurr);
            Omega = itOmega(c);

            for p = 1:4

                %%%%% WHEEL SLIP ANGLE EQUATIONS
                % 
                % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                % SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
                % % 
                if c == 1
                    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 2
                    SA_Wheel(p,1) = atan2( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) , (R.*cos(SA_CG(j)) - coord_AllW(2,p)) )...
                                    - dSteer_AllW(i,p);

                else
                    % %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    % %%% Equation 1
                    % SA_Wheel(p,1) = atan2( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
                    %                     , (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
                     %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
                    %%% Equation 1
                    SA_Wheel(p,1) = atan2( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
                                        , (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);

                end


               %%%%% WHEEL SPEED EQUATIONS

               % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
               % V_Wheel(p,1) = 0;

               %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
               V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );

            end

            dFzf_dAx = (hCG .* m)./(2.* WB);
            dFzf_dAy = 1*(hCG .* m .* PFront)/TWf;
            dFzr_dAy = 1*(hCG .* m .* (1-PFront))/TWr;

            Fz_Wf = (m.*g.* PFront)/2;
            Fz_Wr = (m.*g.* (1-PFront))/2;

            Fz(1,1) = Fz_Wf + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
            Fz(2,1) = Fz_Wf + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
            Fz(3,1) = Fz_Wr - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
            Fz(4,1) = Fz_Wr - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;

            for p = 1:4
                [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire, rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_Wheel(p), Idx, Model);

                %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
                TM_Fx(p) = TM_Fx(p) .* 0; 

                % Calspan TTC Data usual correction factor - 0.7
                TM_Fx(p) = TM_Fx(p) .* 0.7;
                % Tire Model outputs in opposite Y coordinates
                TM_Fy(p) = (1).* TM_Fy(p) .* 0.7;

                FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                FyTire(p,1) = 1*TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));

                % Made it a matrix sum so its not ugly :)
                MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                   -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
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

            % Using dimensionless G's to iteration (apparently improves
            % stability)?
            itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
            itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
            res = itCAyBody(c+1) - itCAyBody(c);
            resAx = itCAxBody(c+1) - itCAxBody(c);
            % Tests the c 
            plot(1:c+1, itAyBody);

            c = c + 1;

            if c > 1000
                iterationCtrl = itAyBody( (c-10) : (c-1) );
                itAyBody(c) = min(iterationCtrl);
                break
            end

        end % while loop end
        % if j == 13
        %     return
        % end
        %saveAyBody(i,j) = (itAyBody(end)+itAyBody(end-1))/2;
        saveTM_Fy(:,j) = TM_Fy;
        saveSA_Wheel(:,j) = SA_Wheel;
        saveFz(:,j) = Fz;
        saveAyBody(i,j) = itAyBody(end);
        saveAxBody(i,j) = itAxBody(end);
        saveMzBody(i,j) = MzBody/(m.*g.*WB);
        saveItFz(:,j) = Fz;
        saveItAyBody{j,1} = itAyBody;

        disp("Steering Angle [" + i + "] Slip Angle [" + j + "] finished, iterations: " +  c)

    end % SA_CG End

end % dSteer End


%% PLOTTING
close all;

saveAxVel = zeros(size(saveAyBody));
saveAyVel = zeros(size(saveAyBody));
saveCAyVel = zeros(size(saveAyBody));
saveCAxVel = zeros(size(saveAyBody));

% Body to velocity coordinate transformation
for i = 1:length(SA_CG)
    saveAxVel(:,i) = saveAxBody(:,i) .* cos(SA_CG) + saveAyBody(:,i) .* sin(SA_CG);
    saveAyVel(:,i) = saveAyBody(:,i) .* cos(SA_CG) - saveAxBody(:,i) .* sin(SA_CG);
    saveCAxVel(:,i) = saveAxVel(:,i)/g;
    saveCAyVel(:,i) = saveAyVel(:,i)/g;
end

figure
hold on
grid on
for i = 1:length(dSteer)
    steer = plot(saveCAyVel(i,:),saveMzBody(i,:), "Color", "blue", 'LineStyle','--');
    if mod(i,7) == 0
        labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
        % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
    end

end

for i = 1:length(SA_CG)
    slip = plot(saveCAyVel(:,i),saveMzBody(:,i), "Color", "red");
    if mod(i,7) == 0
        labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
        % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
    end
end

xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
ylabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
xlim([-2,2])
ylim([-1,1])

if PFront > 0.5
    resp = 'Understeer' ;
elseif PFront < 0.5
    resp = 'Oversteer';
else
    resp = 'Neutral Steer';
end

% 
% title({ ...
%        ['SA: [' num2str(min(rad2deg(SA_CG))) ',' num2str(max(rad2deg(SA_CG))) '] & Steering: [' num2str(min(rad2deg(dSteer))) ',' num2str(max(rad2deg(dSteer))) ']'], ...
%        ['Expected Response: '  resp] ...
%        ['Radius: ' num2str(R) ' m'] ...
%        ['Velocity: ' num2str(V) ' m/s']...
%       });

% 
% title({ ...
%         ['Free Rolling MMD - Infinite Radius']
%        ['SA: [' num2str(min(rad2deg(SA_CG))) ',' num2str(max(rad2deg(SA_CG))) '] & Steering: [' num2str(min(rad2deg(dSteer))) ',' num2str(max(rad2deg(dSteer))) ']'], ...
%        ['Expected Response: '  resp] ...
%        ['Radius: ' num2str(R) ' m'] ...
%        ['Velocity: ' num2str(V) ' m/s']...
%       });
title({['Free Rolling MMD: Constant Velocity'] ...
       ['Velocity = ', num2str(V),' m/s']},'Interpreter','latex')


%% FINDING HIGHEST STEADY STATE AY

zeroMz_CAy = zeros(1,length(SA_CG));

% Find Change Pts of Mz from pos to negative for each SA_CG
for j = 1:length(SA_CG)
    for i = 1:length(dSteer)
        if i == length(SA_CG)
            indexSS = 0;
            break
        end
        diff = saveMzBody(i,j) - saveMzBody(i+1,j);
        if abs(diff) > abs(saveMzBody(i,j))
            indexSS = i;
            break
        end
    end
    % Interpolates for slip angle
    % indexes(j) = indexSS;
    if indexSS == 0
        zeroMz_CAy(j) = 0;
    else
    slope = (saveMzBody(indexSS+1,j) - saveMzBody(indexSS,j)) / (saveCAyVel(indexSS+1,j) - saveCAyVel(indexSS,j));
    b = saveMzBody(indexSS,j) - slope*saveCAyVel(indexSS,j);
    zeroMz_CAy(j) = -b/slope;    
    end
end

[maxThroSA,indMaxThroSA] = max(zeroMz_CAy);

zeroMz_CAy = zeros(1,length(SA_CG));
for i = 1:length(dSteer)
    for j = 1:length(SA_CG)
        if j == length(SA_CG)
            indexSS = 0;
            break
        end
        diff = saveMzBody(i,j) - saveMzBody(i,j+1);
        if abs(diff) > abs(saveMzBody(i,j))
            indexSS = j;
            break
        end
    end
    % Interpolates for slip angle
    % indexes(j) = indexSS;
    if indexSS == 0
        zeroMz_CAy(i) = 0;
    else
    slope = (saveMzBody(i,indexSS+1) - saveMzBody(i,indexSS)) / (saveCAyVel(i,indexSS+1) - saveCAyVel(i,indexSS));
    b = saveMzBody(i,indexSS) - slope*saveCAyVel(i,indexSS);
    zeroMz_CAy(i) = -b/slope;    
    end
end
[maxThroSteer, indMaxThroSteer] = max(zeroMz_CAy);

CAymax = max(maxThroSteer, maxThroSA);
if CAymax == maxThroSteer
    indMax = indMaxThroSteer;
else
    indMax = indMaxThroSA;
end




AyMaxSS = plot(CAymax, 0, "Marker", ".", "MarkerSize", 20, "Color","g");
label = sprintf('C_{Ay_0} = %.4g\nSlip Angle = %.3g°', CAymax, SA_CG(indMaxThroSA) * 180/pi); %,'Interpreter','latex');
% text(CAymax + .2, 0, {'','','','','C_{Ay_0} =', 'SA_CG =', num2str(CAymax), num2str(SA_CG(indMax))}, "FontSize",7 );
text(CAymax - .025, -0.15, label, "FontSize", 8, 'FontWeight','bold');

legend([steer, slip, AyMaxSS],{"Constant Steer", "Constant Slip", "$C_{Ay_{Max SS}}$"}, "Location","northeast",'Interpreter','latex')

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
ylabel("Normalized Longitudinaal Accleration $(C_{Ax})$",'Interpreter','latex')



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

view(3)
ylabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
xlabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
zlabel("Normalized Longitudinaal Accleration $(C_{Ax})$",'Interpreter','latex')



figure
hold on
grid on

[ ~ , dSteerMesh] = meshgrid(ones(size(dSteer)), dSteer);
[SAMesh, ~] = meshgrid( SA_CG ,ones(size(SA_CG)));
[ ~ , indexMesh] = meshgrid(dSteer', 1:length(dSteer));


for i = 1:length(dSteer)
    steer = plot3(dSteerMesh(:,i),SAMesh(:,i),saveAxBody(:,i), "Color", "blue", 'LineStyle','--');
    
end
surf(dSteerMesh, SAMesh, zeros(size(SAMesh)))
surf(dSteerMesh, SAMesh, saveAxBody)
view(3)
xlabel("dsteer angle rads")
ylabel("SAMesh")
zlabel("AX")
