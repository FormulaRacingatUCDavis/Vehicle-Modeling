clc; clear; close all
load('Hoosier_43075_16x75-10_R20-8.mat');
Tire = Tire;
fun = @(V) func1(V, Tire);
J0 = 0;
mat_A = [];
val_b = [];
nonlMat_A = [];
nonlVal_B = [];
lowerBound = [];
upperBound = [];

[x, funcVal] = fmincon(fun, J0, mat_A, val_b, nonlMat_A, nonlVal_B, lowerBound,...
    upperBound);

x

funcVal


function outVal = func1(V, Tire)
    
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

    Cl = 2; 
    Cd = 1.374; 
    CoP = 50/100; % front downforce distribution (%)
    rho = 1.165; % kg/m^3
    crossA = 0.92; % m^2

    r = 0.1; % 10 meter turn radius (this is curvature 1/r)

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

    AyTireMax = FyTireTot/mass * sign(r);
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
    outVal = V;
end