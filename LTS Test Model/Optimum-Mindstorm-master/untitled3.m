% Jonathan Vogel
% Clemson Formula SAE
% Tradespace Analysis Project
% 2019 Michigan Dynamic Event Lap Sim

clear
close all
clc
format long g

% The purpose of this code is to evaluate the points-scoring capacity of a
% virtual vehicle around the 2019 FSAE Michigan Dynamic Event Tracks

% NOTES:
% 10/4/2024: 
%     Looked at the cornering package, but due to non-convergence, changed all
%     rscale to 1 because it was making the force Fy too small for all diff_AY
%     convergence, but that was for the first diffAY iterations, moving onto
%     Mz iterations, it now can get past first variable generation for first Mz 
%     generation, but gets stuck on converging at line 469 for the first diffAY
%     under first Mz generation


%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax
%% Section 1: Input Tire Model
% this section is required, everything should be pre-loaded so no need to
% touch any of this, unless you want to change the tire being evaluated.
% The only things you might want to change are the scaling factors at the
% bottom of the section
disp('2019 Michigan Endurance Points Analysis')
disp('Loading Tire Model')

% First we load in the lateral tire force model, which is a Pacejka model
% created by derek:
global FZ0 LFZO LCX LMUX LEX LKX  LHX LVX LCY LMUY LEY LKY LHY LVY ...
       LGAY Ltr LRES LGAZ LXAL LYKA LVYKA LS LSGKP  LSGAL LGYR KY
load('A1654run21_MF52_Fy_GV12.mat')
% then load in coefficients for Magic Formula 5.2 Tire Model:
load('A1654run21_MF52_Fy_12.mat')

% Next you load in the longitudinal tire model, which for now is just a
% CSAPS spline fit to the TTC data
% find your pathname and filename for the tire you want to load in
filename = 'Hoosier_R25B_18.0x7.5-10_FX_12psi.mat';
load(filename)
tire_radius = 9.05/12; %ft
tyreRadius = tire_radius/3.28; % converts to meters

load('Hoosier_R25B_16x75-10x7.mat');
Pressure    = 70;
Inclination = -1;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

% finally, we have some scaling factors for longitudinal (x) and lateral
% (y) friction. You can use these to tune the lap sim to correlate better 
% to logged data 
sf_x = 0.75;
sf_y = 0.70;   
%% Section 2: Input Powertrain Model
% change whatever you want here, this is the 2018 powertrain package iirc
% just keep your units consistent please
disp('Loading Engine Model')
engineSpeed = [1000:500:5000]; % RPM
% torque should be in N-m:
%engineTq = [115 120 125 127 127 127 125 120 115];
engineTq = [230 230 230 230 230 225 220 215 210];
primaryReduction = 1;
gear = [1]; % transmission gear ratios
finalDrive = 2.91; % large sprocket/small sprocket
shiftpoint = 14000; % optimal shiftpoint for most gears [RPM]
drivetrainLosses = .93; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 90; % differential locking torque (0 =  open, 1 = locked)

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction;
VMAX = floor(3.28*shiftpoint/(gearTot/tyreRadius*60/(2*pi)));
T_lock = T_lock/100;
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive shiftpoint drivetrainLosses};
%% Section 3: Vehicle Architecture
disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 51.5; % Front lateral load transfer distribution (%)
W = SI2FREEDOM(275, "kg"); % vehicle + driver weight (lbs)
WDF = 51; % front weight distribution (%)
cg = 10/12; % center of gravity height (ft)
l = SI2FREEDOM(1.595, "m"); % wheelbase (ft)
twf = SI2FREEDOM(1.193, "m"); % front track width (ft)
twr = twf; % rear track width (ft)

% some intermediary calcs you don't have to touch
LLTD = LLTD/100;
WDF = WDF/100;
m = W/32.2; % mass (lbm)
WF = W*WDF; % front weight
WR = W*(1-WDF); % rear weight
a = l*(1-WDF); % front axle to cg
b = l*WDF; % rear axle to cg
tw = twf;
%% Section 4: Input Suspension Kinematics
disp('Loading Suspension Kinematics')
% this section is actually optional. So if you set everything to zero, you
% can essentially leave this portion out of the analysis. Useful if you are
% only trying to explore some higher level relationships

% Pitch and roll gradients define how much the car's gonna move around
rg_f = 0; % front roll gradient (deg/g)
rg_r = 0; % rear roll gradient (deg/g)
pg = 0; % pitch gradient (deg/g)
WRF = 180; % front and rear ride rates (lbs/in)
WRR = 180; 

% then you can select your camber alignment
IA_staticf = -1; % front static camber angle (deg)
IA_staticr = -1; % rear static camber angle (deg)
IA_compensationf = 0; % front camber compensation (%)
IA_compensationr = 0; % rear camber compensation (%)

% lastly you can select your kingpin axis parameters
casterf = 0; % front caster angle (deg)
KPIf = 0; % front kingpin inclination angle (deg)
casterr = 0;
KPIr = 0;

% intermediary calcs, plz ignore
IA_staticf = deg2rad(IA_staticf); % front static camber angle (deg)
IA_staticr = deg2rad(IA_staticr); % rear static camber angle (deg)
IA_compensationf = IA_compensationf/100; % front camber compensation (%)
IA_compensationr = IA_compensationr/100; % rear camber compensation (%)
casterf = deg2rad(casterf);
KPIf = deg2rad(KPIf);
casterr = deg2rad(casterr);
KPIr = deg2rad(KPIr);
IA_roll_inducedf = asin(2/twf/12);
IA_roll_inducedr = asin(2/twr/12);
IA_gainf = IA_roll_inducedf*IA_compensationf;
IA_gainr = IA_roll_inducedr*IA_compensationr;
%% Section 5: Input Aero Parameters
disp('Loading Aero Model')
Cl = 3.98; %279/418
Cd = 1.374; % .0245
CoP = 50; % front downforce distribution (%)

%%% ADDED MORE ACCURATE AERO DRAG FORCE COEFF
rho = SI2FREEDOM(1.165, "rho"); % kg/m^3
crossA = SI2FREEDOM(0.92, "m^2"); % m^2

% Intermediary Calculations
CoP = CoP/100;

%% Testing

disp('Generating g-g-V Diagram')

deltar = 0;
deltaf = 0;
velocity = 15:5:115; % range of velocities at which sim will evaluate (ft/s)
radii = [15:10:155]; % range of turn radii at which sim will evaluate (ft)

% First we will evaluate our Acceleration Capacity
g = 1; % g is a gear indicator, and it will start at 1
spcount = 1; % spcount is keeping track of how many gearshifts there are
% shift_points tracks the actual shift point velocities
shift_points(1) = 0; 
tol = 1e-5;
disp('     Acceleration Envelope')
for  i = 1:1:length(velocity) % for each velocity
    Ax(1) = 0;
    Ax2(1) = 0;
    j = 1;
    err = 1;
    while tol < abs(err) && j < 80
        gp = g; % Current gear = current gear (wow!)
        V = velocity(i); % find velocity
        DF = (1/2)*rho*crossA*Cl*V^2; % calculate downforce (lbs)
        % calculate f/r suspension drop from downforce (in)
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR;
        % from rh drop, find camber gain (deg)
        IA_0f = IA_staticf - dxf*IA_gainf;
        IA_0r = IA_staticr - dxr*IA_gainr;
        % find load on each tire (lbs)
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        % now we actually sweep through with acceleration
        WS = W/2; % weight of one half-car
        pitch = -Ax(j)*pg*pi/180; % pitch angle (rad)
        % recalculate wheel loads due to load transfer (lbs)
        wf = wf-Ax(j)*cg*WS/l; 
        wr = wr+Ax(j)*cg*WS/l;
        % recalculate camber angles due to pitch
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
        % select a range of slip ratios (sl) [-]
        sl = [0:.01:0.2];
        % evaluate the tractive force capacity from each tire for the range of
        % slip ratios
        for k = 1:length(sl)  
            fxf(k) = fnval([sl(k);-wf;rad2deg(-IA_f)],full_send_x)*sf_x;
            fxr(k) = fnval([sl(k);-wr;rad2deg(-IA_r)],full_send_x)*sf_x;
        end
        for k = 1:length(sl)
            [outFxf(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k),wf*4.44822, Pressure, Inclination, V*0.3048, Idx, Model);
            [outFxr(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k),wf*4.44822, Pressure, Inclination, V*0.3048, Idx, Model);
        end
        outFxf = outFxf .* 0.224809 .* sf_x; %Convert fron Newtons to pound force and multiply by scaling
        outFxr = outFxr .* 0.224809 .* sf_x;

        % find max force capacity from each tire:
        fxf(find(abs(fxf) > 1000)) = [];
        fxr(find(abs(fxr) > 1000)) = [];
        FXF = max(fxf);
        FXR = max(fxr);
        FXF2 = max(outFxf);
        FXR2 = max(outFxr);
        % Calculate total tire tractive force (lbs)
        DragF = (1/2)*rho*crossA*Cd*V^2; %Calculate drag force (lbf)
        FX = abs(2*FXR) - DragF;
        FX2 = abs(2*FXR2) - DragF;
        % calculate total lateral acceleration capacity (g's)
        Ax(j + 1) = FX/W;
        Ax2(j + 1) = FX2/W;
        err = Ax(j+1) - Ax(j);
        j = j + 1;
    end
    AxSave(i) = Ax(j);
    Ax2Save(i) = Ax2(j);

    %%% 2ND WHILE LOOP
    j = 1;
    err = 1;
    Ax3(1) = 0;
    while tol < abs(err)
        gp = g; % Current gear = current gear (wow!)
        V = velocity(i); % find velocity
        DF = (1/2)*rho*crossA*Cl*V^2; % calculate downforce (lbs)
        % calculate f/r suspension drop from downforce (in)
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR;
        % from rh drop, find camber gain (deg)
        IA_0f = IA_staticf - dxf*IA_gainf;
        IA_0r = IA_staticr - dxr*IA_gainr;
        % find load on each tire (lbs)
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        % now we actually sweep through with acceleration
        WS = W/2; % weight of one half-car
        pitch = -Ax3(j)*pg*pi/180; % pitch angle (rad)
        % recalculate wheel loads due to load transfer (lbs)
        wf = wf-Ax3(j)*cg*WS/l; 
        wr = wr+Ax3(j)*cg*WS/l;
        % recalculate camber angles due to pitch
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
        % select a range of slip ratios (sl) [-]
        sl = [0:.01:0.2];
        % evaluate the tractive force capacity from each tire for the range of
        % slip ratio
        for k = 1:length(sl)
            [outFxf(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k),wf*4.44822, Pressure, Inclination, V*0.3048, Idx, Model);
            [outFxr(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k),wf*4.44822, Pressure, Inclination, V*0.3048, Idx, Model);
        end
        outFxf = outFxf .* 0.224809 .* sf_x; %Convert fron Newtons to pound force and multiply by scaling
        outFxr = outFxr .* 0.224809 .* sf_x;

        % find max force capacity from each tire:
        FXF3 = max(outFxf);
        FXR3 = max(outFxr);
        % Calculate total tire tractive force (lbs)
        DragF = (1/2)*rho*crossA*Cd*V^2;
        FX3 = abs(2*FXR3) - DragF;
        % calculate total lateral acceleration capacity (g's)
        Ax3(j+1) = FX3/W;
        err = Ax3(j+1) - Ax3(j);
        j = j + 1
    end
    Ax3Save(i) = Ax3(j);
    outPTLS = PowertrainlapsimV222(V/3.281); % Outputs in newtons for some reason
    FxOut = outPTLS(1);
    FxOut = FxOut * 0.224809 - DragF; %Convert back to pound force
    AxPower(i) = FxOut/W;
end
%A_Xr(A_Xr < 0) = 0;

disp("acceleration package done")
%%
% from these results, you can create the first part of the GGV diagram
% input for the lap sim codes:
% accel is the maximum acceleration capacity as a function of velocity
% (power limited) and grip is the same but (tire limited)
accel = csaps(velocity,AxPower);
grip = csaps(velocity,AxSave);
grip2 = csaps(velocity, Ax2Save);
grip3 = csaps(velocity, Ax3Save);
grip4 = csaps(velocity, min(Ax3Save,AxPower)-0.01);
drag = csaps(velocity, ((1/2).*rho.*crossA.*Cd.*velocity.^2)/W);


%% Testing 2

velocity = 15:5:115; % range of velocities at which sim will evaluate (ft/s)


%% Graphing
close all
fnplt(accel); hold on;
%fnplt(grip); fnplt(grip2); 
fnplt(grip3); 
fnplt(grip4, ".");
fnplt(drag); 
legend(["Power Limited(Max Torque)", "Grip Limited(SRmax = 0.2)", "Final Tractive Force","Drag"],"location", "best");
title("Driveline Model Spline Fits")
xlabel("Velocity (ft/s)")
ylabel("X-acceleration (g)")

%% 
%Changing variables so Cornering Package can Run
grip = grip3;

%% Cornering Package

% Next we explore the cornering envelope. First we define AYP, which is the
% starting guess for lateral acceleration capacity at a given speed
AYP = 1;
disp('     Cornering Envelope')
% for cornering performance, it makes more sense to evaluate a set of
% cornering radii, instead of speeds
for turn = 1:1:length(radii)
    % first define your vehicle characteristics:
        a = l*(1-WDF);
        b = l*WDF;
        R = radii(turn);
        % update speed and downforce
        V = sqrt(R*32.2*AYP);
        DF = (1/2)*rho*crossA*Cl*V^2; 
        % from downforce, update suspension travel (in):
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR; 
        % from suspension heave, update static camber (rad):
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        % update load on each axle (lbs)
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        % guess ackermann steer angle as starting steer angle
        delta = l/R;
        ddelta = delta*.01;
        % assume vehicle sideslip starts at 0 (rad)
        beta = deg2rad(0);
        A_y = V^2/R;
        % calculate lateral load transfer (lbs)
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        % split f/r using LLTD
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        % calculate f/r roll (rad)
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        % update individual wheel loads 
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        % update individual wheel camber (from roll, then from steer
        % effects)
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir;
        % calculate yaw rate
        r = A_y/V;
        % from yaw, sideslip and steer you can get slip angles
        a_f = beta+a*r/V-delta;
        a_r = beta-b*r/V;
        % with slip angles, load and camber, calculate lateral force at
        % the front
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
        % before you calculate the rears, you ned to see what the diff is
        % doing
        % calculate the drag from aero and the front tires
        F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
        % calculate the grip penalty assuming the rears must overcome that
        % drag
        rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
        % now calculate rear tire forces, with said penalty
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        % sum of forces and moments
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2; % incl the differential contribution     
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        % calculate resultant lateral acceleration
        AY = F_y/(W/32.2);
        % compare to the initial guess
        diff_AY = A_y-AY;
        % vary the sideslip angle (B) until the initial guess and resultant
        % AY match up

        % err = 1
        % tol = tol;
        % A_y(1) = 0;
        % beta(1) = 0;
        % while tol < abs(err)
        % 
        % 
        % end




        while diff_AY < 0
            beta = beta + .0025;
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_f = beta+a*r/V-delta;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
            F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
            rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;       
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY; 
        end
        itCount = 0;
        diffDiffAy = -1;
        while diff_AY > 0 %&& diffDiffAy < 0
            itCount = itCount + 1;
            beta = beta - .0025;
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_f = beta+a*r/V-delta;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
            F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
            rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;          
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diffDiffAy = (A_y-AY) - diff_AY
            diff_AY = A_y-AY;
        end

        % at that point, check the yaw moment. Re-run the above loop^ but
        % this time, steer angle is being varied until moment comes out to
        % zero-ish:
        disp("first it done")
        while M_z < 0 
            delta = delta+ddelta;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_f = beta+a*r/V-delta;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
            F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
            rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2;      
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            %%
            itCount = 1;
            while diff_AY > 0  %&& diffDiffAy < 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diffDiffAy = (A_y-AY) - diff_AY
                diff_AY = A_y-AY; 
                
                itCountSave(itCount) = diff_AY;
                itCount = itCount + 1
            end
        end
        while M_z > 0 
            delta = delta-ddelta;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_f = beta+a*r/V-delta;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
            F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
            rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2; 
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
        end
        disp("second it done")
        % then re run all of THAT, slowly increasing your AY guess 
        % until the front tire maxes out aka slip
        % angle of 12
        while a_f > deg2rad(-12)
            AYP = AYP+.005;
            a = l*(1-WDF);
            b = l*WDF;
            R = radii(turn);
            V = sqrt(R*32.2*AYP);
            DF = Cl*V^2; 
            dxf = DF*CoP/2/WRF; 
            dxr = DF*(1-CoP)/2/WRR; 
            IA_0f = IA_staticf - dxf*IA_gainf; 
            IA_0r = IA_staticr - dxr*IA_gainr; 
            wf = (WF+DF*CoP)/2;
            wr = (WR+DF*(1-CoP))/2;
            delta = l/R;
            ddelta = delta*.005;
            beta = deg2rad(0);
            A_y = V^2/R;
            WT = A_y*cg*W/mean([twf twr])/32.2/12;
            WTF = WT*LLTD;
            WTR = WT*(1-LLTD);
            phif = A_y*rg_f*pi/180/32.2;
            phir = A_y*rg_r*pi/180/32.2;
            wfin = wf-WTF;
            wfout = wf+WTF;
            wrin = wr-WTR;
            wrout = wr+WTR;
            IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
            IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
            IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
            IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
            r = A_y/V;
            a_f = beta+a*r/V-delta;
            a_r = beta-b*r/V;
            F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
            F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
            F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
            rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
            F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
            F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
            F_y = F_fin+F_fout+F_rin+F_rout;
            M_z_diff = F_x*T_lock*twr/2; 
            M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
            AY = F_y/(W/32.2);
            diff_AY = A_y-AY;
            while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
            %rad2deg([a_f a_r])
            while M_z > 0 
                delta = delta-ddelta;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
            end
            while M_z < 0 
                delta = delta+ddelta;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    %disp(rad2deg(a_f))
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                    if a_f < deg2rad(-12)
                        diff_AY = -1;
                    end
                end
                if a_f < deg2rad(-12)
                    M_z = 1;
                end
            end
        end
        disp("third it done")
        % once you've exceeded the capability of the fronts, take one small
        % step back and that is your max lateral acceleration capacity
        AYP = AYP-.005;
        a = l*(1-WDF);
        b = l*WDF;
        R = radii(turn);
        V = sqrt(R*32.2*AYP);
        DF = Cl*V^2; 
        dxf = DF*CoP/2/WRF; 
        dxr = DF*(1-CoP)/2/WRR; 
        IA_0f = IA_staticf - dxf*IA_gainf; 
        IA_0r = IA_staticr - dxr*IA_gainr; 
        wf = (WF+DF*CoP)/2;
        wr = (WR+DF*(1-CoP))/2;
        delta = l/R;
        ddelta = delta*.01;
        beta = deg2rad(0);
        A_y = V^2/R;
        WT = A_y*cg*W/mean([twf twr])/32.2/12;
        WTF = WT*LLTD;
        WTR = WT*(1-LLTD);
        phif = A_y*rg_f*pi/180/32.2;
        phir = A_y*rg_r*pi/180/32.2;
        wfin = wf-WTF;
        wfout = wf+WTF;
        wrin = wr-WTR;
        wrout = wr+WTR;
        IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
        IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
        IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
        IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
        r = A_y/V;
        a_f = beta+a*r/V-delta;
        a_r = beta-b*r/V;
        F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
        F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
        F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
        rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
        F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
        F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
        F_y = F_fin+F_fout+F_rin+F_rout;
        M_z_diff = F_x*T_lock*twr/2; 
        M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
        AY = F_y/(W/32.2);
        diff_AY = A_y-AY;
        while diff_AY < 0
                beta = beta + .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
        while diff_AY > 0
                beta = beta - .0025;
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY; 
            end
        %rad2deg([a_f a_r])
        while M_z < 0 
                delta = delta+ddelta;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2;     
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                    if a_f < deg2rad(-12)
                        diff_AY = -1;
                    end
                end
            end
        while M_z > 0 
                delta = delta-ddelta;
                beta = deg2rad(0);
                A_y = V^2/R;
                WT = A_y*cg*W/mean([twf twr])/32.2/12;
                WTF = WT*LLTD;
                WTR = WT*(1-LLTD);
                phif = A_y*rg_f*pi/180/32.2;
                phir = A_y*rg_r*pi/180/32.2;
                wfin = wf-WTF;
                wfout = wf+WTF;
                wrin = wr-WTR;
                wrout = wr+WTR;
                IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                r = A_y/V;
                a_f = beta+a*r/V-delta;
                a_r = beta-b*r/V;        
                F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                F_y = F_fin+F_fout+F_rin+F_rout;
                M_z_diff = F_x*T_lock*twr/2; 
                M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                AY = F_y/(W/32.2);
                diff_AY = A_y-AY;
                while diff_AY < 0
                    beta = beta + .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
                while diff_AY > 0
                    beta = beta - .0025;
                    A_y = V^2/R;
                    WT = A_y*cg*W/mean([twf twr])/32.2/12;
                    WTF = WT*LLTD;
                    WTR = WT*(1-LLTD);
                    phif = A_y*rg_f*pi/180/32.2;
                    phir = A_y*rg_r*pi/180/32.2;
                    wfin = wf-WTF;
                    wfout = wf+WTF;
                    wrin = wr-WTR;
                    wrout = wr+WTR;
                    IA_f_in = -twf*sin(phif)*12/2*IA_gainf - IA_0f - KPIf*(1-cos(delta)) - casterf*sin(delta) +phif;
                    IA_f_out = -twf*sin(phif)*12/2*IA_gainf + IA_0f + KPIf*(1-cos(delta)) - casterf*sin(delta) + phif;
                    IA_r_in = -twr*sin(phir)*12/2*IA_gainr - IA_0r - KPIr*(1-cos(deltar)) - casterf*sin(deltar) +phir;
                    IA_r_out = -twr*sin(phir)*12/2*IA_gainr + IA_0r + KPIr*(1-cos(deltar)) - casterf*sin(deltar) + phir; 
                    r = A_y/V;
                    a_f = beta+a*r/V-delta;
                    a_r = beta-b*r/V;
                    F_fin = -MF52_Fy_fcn(A,[-rad2deg(a_f) wfin -rad2deg(IA_f_in)])*sf_y*cos(delta);
                    F_fout = MF52_Fy_fcn(A,[rad2deg(a_f) wfout -rad2deg(IA_f_out)])*sf_y*cos(delta);
                    F_x = Cd*V^2 + (F_fin+F_fout)*sin(delta)/cos(delta); 
                    rscale = 1; %1-(F_x/W/fnval(grip,V))^2;
                    F_rin = -MF52_Fy_fcn(A,[-rad2deg(a_r) wrin -rad2deg(IA_r_in)])*sf_y*rscale;
                    F_rout = MF52_Fy_fcn(A,[rad2deg(a_r) wrout -rad2deg(IA_r_out)])*sf_y*rscale;
                    F_y = F_fin+F_fout+F_rin+F_rout;
                    M_z_diff = F_x*T_lock*twr/2; 
                    M_z = (F_fin+F_fout)*a-(F_rin+F_rout)*b-M_z_diff;
                    AY = F_y/(W/32.2);
                    diff_AY = A_y-AY; 
                end
        end
        B = rad2deg(beta);
        af = rad2deg(a_f);
        ar = rad2deg(a_r);
        steer = rad2deg(delta);
        UG = rad2deg(delta-l/R)*32.2/AY;
        Ugradient(turn) = UG;
        %F_lat = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*cos(delta);
        %F_drag = fnval([rad2deg(a_f);-wf;0],full_send_y)*.45*sin(delta);
        skid = 2*pi*R/V;
        steering(turn) = steer;
        speed(turn) = V;
        lateralg(turn) = AY/32.2;
        radii(turn)
        %toc
end
% Lateral Acceleration


%% Functions
function out = SI2FREEDOM(num, input)
    if input == "kg"
        out = num * 2.20462; %Output from kg to lbm
    elseif input == "m"
        out = num * 3.28084; %Output from m to ft
    elseif input == "rho"
        out = num / (14.5939*35.3147); %Output from kg/m^3 to slug/ft^3
    elseif input == "m^2"
        out = num * 3.28084^2; %Output from m^2 to ft^2
    elseif input == "N"
        out = num * 0.224809; %Output from newtons to pound Force
    end

end