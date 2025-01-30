clc; clear; close all

%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
global r_max accel grip deccel lateral cornering gear shift_points...
    top_speed r_min path_boundaries tire_radius shift_time...
    powertrainpackage track_width path_boundaries_ax

%% Section 1: Tire Model

% %%% Our Tire Model
% load('Hoosier_43075_16x75-10_R20-8.mat');
load('Hoosier_R25B_16x75-10x7.mat');
Pressure    = 70;
Inclination = -1;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

%%% Vogel Tire Model
%load('Hoosier_R25B_18.0x7.5-10_FX_12psi.mat');

tire_radius = 8/12; % tire radius in feet
tyreRadius = 8/12/3.28; % tire radius in meters

%%% Scaling Factors for Fx and Fy Tires
sf_x = 0.75;
sf_y = 0.70;   

%% Section 2: Powertrain 

% % change whatever you want here, this is the 2018 powertrain package iirc
% % just keep your units consistent please

% disp('Loading Engine Model')
% engineSpeed = [6200:100:14100]; % RPM
% % torque should be in N-m:
% engineTq = [41.57 42.98 44.43 45.65 46.44 47.09 47.52 48.58 49.57 50.41 51.43 51.48 51 49.311 48.94 48.66 49.62 49.60 47.89 47.91 48.09 48.57 49.07 49.31 49.58 49.56 49.84 50.10 50.00 50.00 50.75 51.25 52.01 52.44 52.59 52.73 53.34 53.72 52.11 52.25 51.66 50.5 50.34 50.50 50.50 50.55 50.63 50.17 50.80 49.73 49.35 49.11 48.65 48.28 48.28 47.99 47.68 47.43 47.07 46.67 45.49 45.37 44.67 43.8 43.0 42.3 42.00 41.96 41.70 40.43 39.83 38.60 38.46 37.56 36.34 35.35 33.75 33.54 32.63 31.63];
% primaryReduction = 76/36;
% gear = [33/12, 32/16, 30/18, 26/18, 30/23, 29/24]; % transmission gear ratios
% finalDrive = 40/12; % large sprocket/small sprocket


disp('Loading Engine Model')
engineSpeed = [1000:500:5000]; % RPM
% torque should be in N-m:
%engineTq = [115 120 125 127 127 127 125 120 115];
engineTq = [230 230 230 230 230 225 220 215 210];
primaryReduction = 1;
gear = [1]; % transmission gear ratios
finalDrive = 2.91; % large sprocket/small sprocket


shiftpoint = 14000; % optimal shiftpoint for most gears [RPM]
drivetrainLosses = .85; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 90; % differential locking torque (0 =  open, 1 = locked)

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction;
VMAX = floor(shiftpoint/(gearTot/tyreRadius*60/(2*pi))); %Ft/s
T_lock = T_lock/100;
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive shiftpoint drivetrainLosses};

%% Section 3: Vehicle Parameters
% 
% disp('Loading Vehicle Characteristics')
% % These are the basic vehicle architecture primary inputs:
% LLTD = 51.5/100; % Front lateral load transfer distribution (-)
% W = 660 * 4.44822; % vehicle + driver weight (N)
% WDF = 50/100; % front weight distribution (-)
% cg = 13.2/12/3.28; % center of gravity height (m)
% l = 60.5/12/3.28; % wheelbase (m)
% twf = 46/12/3.28; % front track width (m)
% twr = 44/12/3.28; % rear track width (m)
% 
% m = W / 9.81; % Total Mass (kg)
% WF = W * WDF; % front weight
% WR = W * (1 - WDF); % rear weight
% a = l*(1-WDF); % front axle to cg
% b = l*WDF; % rear axle to cg
% tw = twf; % Track Width


disp('Loading Vehicle Characteristics')
% These are the basic vehicle architecture primary inputs:
LLTD = 51.5; % Front lateral load transfer distribution (%)
W = 275 * 9.81; % vehicle + driver weight (N)
WDF = 51; % front weight distribution (%)
cg = 10/12/3.28; % center of gravity height (m)
l = 1.595; % wheelbase (m)
twf = 1.193; % front track width (m)
twr = twf; % rear track width (m)

% some intermediary calcs you don't have to touch
LLTD = LLTD/100;
WDF = WDF/100;
m = W/9.81; % mass (lbm)
WF = W*WDF; % front weight
WR = W*(1-WDF); % rear weight
a = l*(1-WDF); % front axle to cg
b = l*WDF; % rear axle to cg
tw = twf;

%% Section 4: Input Suspension Kinematics (Ignore For Now)
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
IA_staticf = 0; % front static camber angle (deg)
IA_staticr = 0; % rear static camber angle (deg)
IA_compensationf = 10; % front camber compensation (%)
IA_compensationr = 20; % rear camber compensation (%)

% lastly you can select your kingpin axis parameters
casterf = 0; % front caster angle (deg)
KPIf = 0; % front kingpin inclination angle (deg)
casterr = 4.1568;
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

% 
% Cl = .0418; %279/418
% Cd = .0184; % .0245
% CoP = 48/100; % front downforce distribution (-)

Cl = 3.98; 
Cd = 1.374; 
CoP = 50/100; % front downforce distribution (%)
rho = 1.165; % kg/m^3
crossA = 0.92; % m^2

%% Section 6: Generate GGV Diagram
% this is where the m e a t of the lap sim takes place. The GGV diagram is
% built by finding a maximum cornering, braking, and acceleration capacity
% for any given speed

disp('Generating g-g-V Diagram')

deltar = 0;
deltaf = 0;
velocity = [4:4:36]; % range of velocities at which sim will evaluate (m/s)
radii = [5:5:50]; % range of turn radii at which sim will evaluate (m)
tol = 1e-6;


disp('     Acceleration Envelope')
for i = 1:length(velocity)
    j = 1;
    err = 1;
    Ax(1) = 0;
    while tol < abs(err)
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
        % slip ratio
        for k = 1:length(sl)
            [outFxf(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , wf, Pressure, Inclination, V, Idx, Model);
            [outFxr(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , wr, Pressure, Inclination, V, Idx, Model);
        end
        outFxf = outFxf  .* sf_x; %multiply by scaling
        outFxr = outFxr  .* sf_x;

        % find max force capacity from each tire:
        FXF = max(outFxf);
        FXR = max(outFxr);
        % Calculate total tire tractive force (lbs)
        DragF = (1/2)*rho*crossA*Cd*V^2;
        FX = abs(2*FXR) - DragF;
        % calculate total lateral acceleration capacity (g's)
        Ax(j+1) = FX/W;
        err = Ax(j+1) - Ax(j);
        j = j + 1;
    end
    AxTire(i) = Ax(j); % Saving Iterated Ax from Tire forces

    %%% Finding Engine Power limitted Ax
    gearTot = gear(1) * finalDrive * primaryReduction;
    rpm = V * gearTot / tyreRadius * 60 / (2*pi);

    k = 2;
    while rpm > engineSpeed(k)
        k = k + 1;
    end

    torque_engine = engineTq(k-1) + (engineTq(k)-engineTq(k-1)) / (engineSpeed(k)-engineSpeed(k-1)) * (rpm-engineSpeed(k-1));
    torque_total = torque_engine * gearTot * drivetrainLosses;   % torque output at the wheels [N-m]
    Fx = torque_total/tyreRadius; % Engine torque/radius = Force
    FxOut = Fx - DragF; %Convert back to pound force
    AxPower(i) = FxOut/W; % In Gs

end

%%% Spline Fits for Tire Limited Ax and Engine Limited Ax
accel = csaps(velocity, AxPower);
grip = csaps(velocity, AxTire);
finalAx = csaps(velocity, min(AxTire,AxPower)-0.01);
drag = csaps(velocity, ((1/2).*rho.*crossA.*Cd.*velocity.^2)/W);


%% Graphing
close all

fnplt(accel); hold on;
fnplt(grip); 
fnplt(finalAx); 
fnplt(drag)
legend(["Power Limited(Max Torque)", "Grip Limited(SRmax = 0.2)","Final Tractive Force","Drag"],"location", "best");
title("Driveline Model Spline Fits")
xlabel("Velocity (ft/s)")
ylabel("X-acceleration (g)")






