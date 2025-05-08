clc; clear; close all

%% Section 0: Name all symbolic variables
% Don't touch this. This is just naming a bunch of variables and making
% them global so that all the other functions can access them
% global r_max accel grip deccel lateral cornering gear shift_points...
%     top_speed r_min path_boundaries tire_radius shift_time...
%     powertrainpackage track_width path_boundaries_ax

%% Section 1: Tire Model

% %%% Our Tire Model
load('Hoosier_43075_16x75-10_R20-8.mat');
% load('Hoosier_R25B_16x75-10x7.mat');
Pressure    = 70;
Inclination = -1;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

%%% Vogel Tire Model
%load('Hoosier_R25B_18.0x7.5-10_FX_12psi.mat');

tire_radius = 8/12; % tire radius in feet
tyreRadius = 8/12/3.28; % tire radius in meters

%%% Scaling Factors for Fx and Fy Tires
sf_x = .6;
sf_y = .47;   

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
engineSpeed = [0:500:5000]; % RPM
% torque should be in N-m:
%engineTq = [115 120 125 127 127 127 125 120 115];
engineTq = [230 230 230 230 230 230 230 225 220 215 210];
primaryReduction = 1;
gear = [1]; % transmission gear ratios
finalDrive = 2.91; % large sprocket/small sprocket

drivetrainLosses = .85; % percent of torque that makes it to the rear wheels 
shift_time = .25; % seconds
T_lock = 90; % differential locking torque (0 =  open, 1 = locked)

% Intermediary Calcs/Save your results into the workspace
gearTot = gear(end)*finalDrive*primaryReduction;
VMAX = engineSpeed(end)/gearTot * (2 * pi * tyreRadius)  / 60; %Ft/s
powertrainpackage = {engineSpeed engineTq primaryReduction gear finalDrive drivetrainLosses};

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
weight = 275 * 9.81; % vehicle + driver weight (N)
WDF = 51; % front weight distribution (%)
cg = 10/12/3.28; % center of gravity height (m)
l = 1.595; % wheelbase (m)
twf = 1.193; % front track width (m)
twr = twf; % rear track width (m)

% some intermediary calcs you don't have to touch
LLTD = LLTD/100;
WDF = WDF/100;
mass = weight/9.81; % mass (lbm)
WeightFront = weight*WDF; % front weight
WeightRear = weight*(1-WDF); % rear weight
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

Cl = 2; 
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

AxTireAccel = zeros(length(velocity), 1);
AxPower = zeros(length(velocity), 1);
AxTireBrake = zeros(length(velocity), 1);


disp('     Acceleration Envelope')
for i = 1:length(velocity)
    j = 1;
    err = 1;
    Ax(1) = 0;
    while tol < abs(err)
        V = velocity(i); % find velocity
        FzAero = (1/2)*rho*crossA*Cl*V^2; % calculate downforce (N)
        % calculate f/r suspension drop from downforce (in)
        dxf = FzAero*CoP/2/WRF; 
        dxr = FzAero*(1-CoP)/2/WRR;
        % from rh drop, find camber gain (deg)
        IA_0f = IA_staticf - dxf*IA_gainf;
        IA_0r = IA_staticr - dxr*IA_gainr;
        % find load on each tire (lbs)
        FzFrontStatic = (WeightFront+FzAero*CoP)/2;
        FzRearStatic = (WeightRear+FzAero*(1-CoP))/2;
        % now we actually sweep through with acceleration
        WS = weight/2; % weight of one half-car
        pitch = -Ax(j)*pg*pi/180; % pitch angle (rad)
        % recalculate wheel loads due to load transfer (lbs)
        FzFront = FzFrontStatic-Ax(j)*cg*WS/l; 
        FzRear = FzRearStatic+Ax(j)*cg*WS/l;
        % recalculate camber angles due to pitch
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
        % select a range of slip ratios (sl) [-]
        sl = [0:.01:0.2];
        % evaluate the tractive force capacity from each tire for the range of
        % slip ratio
        for k = 1:length(sl)
            [FxTireFront(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , FzFront, Pressure, Inclination, V, Idx, Model);
            [FxTireRear(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , FzRear, Pressure, Inclination, V, Idx, Model);
        end
        FxTireFront = FxTireFront  .* sf_x; %multiply by scaling
        FxTireRear = FxTireRear  .* sf_x;

        % find max force capacity from each tire:
        FXF = max(FxTireFront);
        FXR = max(FxTireRear);
        % Calculate total tire tractive force (lbs)
        FxDrag = (1/2)*rho*crossA*Cd*V^2;
        FX = abs(2*FXR) - FxDrag;
        % calculate total lateral acceleration capacity (g's)
        Ax(j+1) = FX/weight;
        err = Ax(j+1) - Ax(j);
        j = j + 1;
    end
    AxTireAccel(i) = Ax(j); % Saving Iterated Ax from Tire forces

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
    FxOut = Fx - FxDrag; %Convert back to pound force
    AxPower(i) = FxOut/weight; % In Gs

end

disp('     Braking Envelope')
for i = 1:length(velocity)
    j = 1;
    err = 1;
    Ax(1) = 0;
    while tol < abs(err)
        V = velocity(i); % find velocity
        FzAero = (1/2)*rho*crossA*Cl*V^2; % calculate downforce (N)
        % calculate f/r suspension drop from downforce (in)
        dxf = FzAero*CoP/2/WRF; 
        dxr = FzAero*(1-CoP)/2/WRR;
        % from rh drop, find camber gain (deg)
        IA_0f = IA_staticf - dxf*IA_gainf;
        IA_0r = IA_staticr - dxr*IA_gainr;
        % find load on each tire (lbs)
        FzFrontStatic = (WeightFront+FzAero*CoP)/2;
        FzRearStatic = (WeightRear+FzAero*(1-CoP))/2;
        % now we actually sweep through with acceleration
        WS = weight/2; % weight of one half-car
        pitch = -Ax(j)*pg*pi/180; % pitch angle (rad)
        % recalculate wheel loads due to load transfer (lbs)
        FzFront = FzFrontStatic-Ax(j)*cg*WS/l; 
        FzRear = FzRearStatic+Ax(j)*cg*WS/l;
        % recalculate camber angles due to pitch
        IA_f = -l*12*sin(pitch)/2*IA_gainf + IA_0f;
        IA_r = l*12*sin(pitch)/2*IA_gainr + IA_0r;
        % select a range of slip ratios (sl) [-]
        sl = [0:.01:0.3];
        % evaluate the tractive force capacity from each tire for the range of
        % slip ratio
        for k = 1:length(sl)
            [FxTireFront(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , FzFront, Pressure, Inclination, V, Idx, Model);
            [FxTireRear(k),~ ,~, ~, ~] = ContactPatchLoads(Tire, 0, sl(k) , FzRear, Pressure, Inclination, V, Idx, Model);
        end
        FxTireFront = FxTireFront  .* sf_x; %multiply by scaling
        FxTireRear = FxTireRear  .* sf_x;

        % find max force capacity from each tire:
        FXF = max(FxTireFront);
        FXR = max(FxTireRear);
        % Calculate total tire tractive force (lbs)
        FxDrag = (1/2)*rho*crossA*Cd*V^2;
        FX = abs(4*FXR) - FxDrag;
        % calculate total lateral acceleration capacity (g's)
        Ax(j+1) = FX/weight;
        err = Ax(j+1) - Ax(j);
        j = j + 1;
    end
    AxTireBrake(i) = Ax(j); % Saving Iterated Ax from Tire forces
end

%%% Spline Fits for Tire Limited Ax and Engine Limited Ax
spline_power = csaps(velocity, AxPower);
spline_gripAccel = csaps(velocity, AxTireAccel);
spline_finalAx = csaps(velocity, min(AxTireAccel,AxPower)-0.01);
spline_gripBrake = csaps(velocity, AxTireBrake);
spline_drag = csaps(velocity, ((1/2).*rho.*crossA.*Cd.*velocity.^2)/weight);


%% Graphing
close all

fnplt(spline_power); hold on;
fnplt(spline_gripAccel); 
fnplt(spline_finalAx); 
fnplt(spline_drag)
fnplt(spline_gripBrake)
legend(["Power Limited(Max Torque)", ...
        "Grip Limited(SRmax = 0.3)",...
        "Final Tractive Force", ...
        "Drag", ...
        "Brake Grip(Srmax = 0.3)"], ...
        "location", "best");
title("Driveline Model Spline Fits")
xlabel("Velocity (m/s)")
ylabel("X-acceleration (g)")
grid on


%% Apex based simulation

trackfile = 'OpenTRACK Tracks/OpenTRACK_Donington Park_Closed_Forward.mat' ;
tr = load(trackfile);
Vmax = zeros(length(tr.r),1);
Vsave = zeros(length(tr.r),1);
tps = zeros(length(tr.r),1);
bps = zeros(length(tr.r), 1);
AyMax =  zeros(length(tr.r), 1);
iterations = zeros(length(tr.r), 1);
DataNeed = zeros(length(tr.r), 1);
close all;

for i = 1:length(tr.r)
    % Initializing Constants
    g = 9.81;
    r = tr.r(i); % CURVATURE NOT RADIUS
    %r = 1/20;
    VmaxEngine = engineSpeed(end) * (2*pi*tyreRadius) / 60 / gearTot;
    i
    
    if r == 0
        Vmax(i) = VmaxEngine;
        tpsCurr = 1;
        bpsCurr = 0;
    else
        VGuess = 1; % Initial Guess velocity (m/s)
        tol = 1e-6;
        err = tol * 2;
        iteration = 1;
        VIteration = [];

        % Finding a initial guess point
        while tol < err
            VIter = VGuess;
            FzAero = (1/2)*rho*crossA*Cl*VIter^2; % calculate downforce (N) 
            FxDrag = (1/2)*rho*crossA*Cd*VIter^2;

            FzFront = (WeightFront+FzAero*CoP)/2;
            FzRear = (WeightRear+FzAero*(1-CoP))/2;
            SlipAngle = linspace(0,20,31)';
            SlipRatio = 0;

            FzVector = [FzFront ; FzFront; FzRear; FzRear]';
            [~ , FyTire, ~, ~ ,~] = ContactPatchLoads...
                (Tire, SlipAngle, 0 , FzVector, Pressure, Inclination, VIter, Idx, Model);
            FyTireTot = sum(max(abs(FyTire), [], 2)); 
            FyTireTot = FyTireTot * sign(r);
            VIter = sqrt(FyTireTot/( mass * r));
            err = abs(VIter - VGuess);

            % if VIter > VmaxEngine
            %     VGuess = VIter - VGuess * 1;
            % else
            %     VGuess = VIter - VGuess * 0.01;
            % end

            VGuess = VIter;

            VIteration(iteration) = VGuess;
            plot(1:iteration, VIteration);
            iteration = iteration + 1;

        end

        V = min(VIter, VmaxEngine);
        Vsave(i) = V;
        
        % V = 0;
        adjust_Velocity = true;
        iteration = 1;
        VIteration = [];

        while adjust_Velocity
            FzAero = (1/2)*rho*crossA*Cl*V^2; % calculate downforce (N) 
            FxDrag = (1/2)*rho*crossA*Cd*V^2;
            
            FzFront = (WeightFront+FzAero*CoP)/2;
            FzRear = (WeightRear+FzAero*(1-CoP))/2;

            SlipAngle = linspace(0,20,31)';
            SlipRatio = 0;
            
            FzVector = [FzFront ; FzFront; FzRear; FzRear]';
            [~ , FyTire, ~, ~ ,~] = ContactPatchLoads...
                (Tire, SlipAngle, SlipRatio , FzVector, Pressure, Inclination, 0, Idx, Model);

            % FyTire outputs 31x4 matrix, max( , ,2) takes max of columns
            FyTireTot = sum(max(abs(FyTire), [], 2));
            FyTireTot = FyTireTot * sign(r);

            AyTireMax = FyTireTot/mass;
            AyNeed = V^2 * r;
            AxDrag = FxDrag / mass ;
            
            if AxDrag >= 0
                SlipAngle = 0;
                SlipRatio = linspace(0,0.3,31)';
            
                FzVector = [FzFront ; FzFront; FzRear; FzRear]';
                [FxTire , ~, ~, ~ ,~] = ContactPatchLoads...
                    (Tire, SlipAngle, SlipRatio , FzVector, Pressure, Inclination, 0, Idx, Model);
                FxTireAccelTot = sum(max(abs(FxTire(:,3:4)), [], 2));

                AxTireMax = FxTireAccelTot/ mass;
                RPM = V * gearTot / (2*pi*tyreRadius) * 60;
                RPM = min(RPM, max(engineSpeed)); 
                AxPowerMax = interp1(engineSpeed, engineTq, RPM, "linear") / tyreRadius;
                
                Ay = AyTireMax * sqrt(1 - (AxDrag/AxTireMax)^2);
                AxAccel = AxTireMax * sqrt(1 - (AyNeed/AyTireMax)^2);

                scale = min(AxDrag, AxAccel)/ AxPowerMax;
                tpsCurr = max(min(1, scale), 0);
                bpsCurr = 0;

                if AyNeed/AyTireMax >= 1 && AxDrag/AxTireMax >= 1
                    disp("Problem");
                elseif scale >= 1
                    disp("Problem");
                end
            % else
            %     disp("Yurrr")
            %     SlipAngle = 0;
            %     SlipRatio = linspace(0,0.3,31)';
            % 
            %     FzVector = [FzFront ; FzFront; FzRear; FzRear]';
            %     [FxTire , ~, ~, ~ ,~] = ContactPatchLoads...
            %         (Tire, SlipAngle, SlipRatio , FzVector, Pressure, Inclination, 0, Idx, Model);
            %     FxTireDeccelTot = sum(max(abs(FxTire), [], 2));
            % 
            %     AxTireMax = FxTireDeccelTot/ mass;
            % 
            %     Ay = AyTireMax * sqrt(1 - (AxDrag/AxTireMax)^2);
            %     AxDeccel = AxTireMax * sqrt(1 - (AyNeed/AyTireMax)^2);
            % 
            %     bpsCurr = min(AxDrag, AxDeccel);
            %     tpsCurr = 0;
            % 
            %     if AyNeed/AyTireMax > 1
            %         disp("Problem");
            %     end
            end
            
            err = sqrt(Ay/r) - V;
            if abs(Ay/AyNeed) <= 1 || err > tol
                V = sqrt(Ay/r) ;
            else
                adjust_Velocity = false;
            end
            
            VIteration(iteration) = V;
            plot(1:iteration, VIteration);
            iteration = iteration + 1;
        end
        
        tps(i) = tpsCurr;
        bps(i) = bpsCurr;
        Vmax(i) = V;
        AyMax(i) = Ay;
        iterations(i,1) = iteration;
        DataNeed(i,1) = AxTireMax;

    end

end

close all;
plot( Vmax);
title("Maximum Radius and Velocity")
xlabel("Radius")
ylabel("Maximum Velocity")
grid on

figure
plot(DataNeed);
ylabel("Value");
xlabel("Index");



