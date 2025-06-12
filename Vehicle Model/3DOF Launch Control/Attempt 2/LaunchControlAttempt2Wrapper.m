clc; clear; close all
% Load tire parameter structure from .mat file
load('Hoosier_R25B_16x75-10x7.mat');        % e.g., 'TireData.mat' contains a struct named Tire
% load('constants.mat')
%% Short cuts
RPM_to_radS = 2*pi/60;

%% Drive Train Parameters (DTParameters)
DTParameters.torquePeak = 220;        %[Nm]   peak torque
DTParameters.powerPeak = 124e3;       %[W]    peak power
DTParameters.w_base = DTParameters.powerPeak/DTParameters.torquePeak; % [rad/s] rpm
DTParameters.w_max = 6500*2*pi/60;    % [rad/s]
%RPM
DTParameters.torque_vs_RPM_table = [220, 220, 170, 0];    %[Nm] - From data table
DTParameters.torque_vs_RPM_breakpoints = [0, 5500, 6048, 6500];  %[RPM]
%RPS
DTParameters.torque_vs_radS_table = DTParameters.torque_vs_RPM_table*RPM_to_radS;
DTParameters.torque_vs_radS_breakpoints = DTParameters.torque_vs_RPM_breakpoints*RPM_to_radS;  
DTParameters.finalDrive = 2.91;       %[:]     final-drive reduction
DTParameters.driveTrainEfficiency = 0.95; %[:] drivetrain efficiency

%% Vehicle Parameters (VParameters)
VParameters.perc_front = 0.543;       % Front weight distribution [%]
VParameters.wheelBase = 1.582;        % Wheelbase [m]
VParameters.lf = VParameters.wheelBase * (1 - VParameters.perc_front); % [m]
VParameters.lr = VParameters.wheelBase * VParameters.perc_front;     % [m]
VParameters.twF = 1.189;              % Front track width [m]
VParameters.twR = 1.189;              % Rear track width [m]
VParameters.hCG = 0.314;              % CG height [m]

% Mass properties
VParameters.m.vehicle = 203;          % Vehicle mass without driver [kg]
VParameters.m.driver = 68;            % Driver mass [kg]
VParameters.m.total = VParameters.m.vehicle + VParameters.m.driver; % [kg]

% Alignment
VParameters.toeF = -0.5 * (pi/180);   % Static toe front [rad] (-0.5°)
VParameters.toeR = 0 * (pi/180);      % Static toe rear [rad] (0°)
VParameters.camberF = -1.3 * (pi/180); % Static camber front [rad] (-1.3°)
VParameters.camberR = -1.0 * (pi/180); % Static camber rear [rad] (-1.0°)

% Roll centers
VParameters.hRCF = 0.070866;          % Front roll center height [m]
VParameters.hRCR = 0.090932;          % Rear roll center height [m]

%% Steering Parameters
SteerParameters.acker.front = 0.63;    % Static Ackermann front [%]
SteerParameters.acker.rear = 0;        % Rear Ackermann [%]
SteerParameters.ratio = 5;             % Steering ratio (:1)
SteerParameters.cFactor = 0.08788;     % C-factor [m] (87.88 mm)
SteerParameters.armLength = 0.07009;   % Steer arm length [m] (70.09 mm)
SteerParameters.caster = 2.5 * (pi/180); % Caster angle [rad]
SteerParameters.trail = 0.01081;       % Trail [m]
SteerParameters.scrubRadius = 0.000889; % Scrub radius [m]
SteerParameters.kingpinInclination = 11.89 * (pi/180); % Kingpin inclination [rad]
SteerParameters.kingpinOffset = 0.0221; % Kingpin offset [m]

%% Tire Parameters
% Tire specs

TireParameters.Pacejka = Tire.Pacejka;

TireParameters.tire.make = 'Hoosier R20'; % Tire model
TireParameters.tire.OD = 0.4064;       % Tire outer diameter [m] (16 in)
TireParameters.tire.width = 0.1905;    % Tire width [m] (7.5 in)
TireParameters.tire.offset = -0.010;   % Tire design offset [m] (-10 mm)

% Wheel specs
TireParameters.wheel.make = 'OZ Racing 10" Cast Mag';
TireParameters.wheel.diameter = 0.254; % Bead-seat diameter [m] (10 in)
TireParameters.wheel.width = 0.1778;   % Wheel width [m] (7 in)
TireParameters.wheel.offset = 0.022;   % Wheel offset [m] (22 mm)
TireParameters.wheel.circumfrence = pi*TireParameters.wheel.diameter;

%% Suspension Parameters (SuspParameters)

% Hub dimensions
SuspParameters.hub.centerLock.diam = 0.254; % Hub diameter [m]
SuspParameters.hub.centerLock.width = 0.190; % Hub width [m]

% Suspension travel
SuspParameters.travel.jounce.front = 0.027; % Front jounce travel [m]
SuspParameters.travel.rebound.front = 0.027; % Front rebound travel [m]
SuspParameters.travel.jounce.rear = 0.027;  % Rear jounce travel [m]
SuspParameters.travel.rebound.rear = 0.027; % Rear rebound travel [m]

% Rates and frequencies
SuspParameters.wheelRate_lin.front = 42100; % Front wheel rate [N/m]
SuspParameters.wheelRate_lin.rear = 46100;  % Rear wheel rate [N/m]
SuspParameters.rollRate.front = 740;        % Front roll rate [N*m/deg]
SuspParameters.rollRate.rear = 801;         % Rear roll rate [N*m/deg]
SuspParameters.natFreq.front = 2.33;        % Front natural freq [Hz]
SuspParameters.natFreq.rear = 2.42;         % Rear natural freq [Hz]

% Damping
SuspParameters.damping.jounce.front = 0.65; % Front jounce damping [%]
SuspParameters.damping.jounce.vel.front = 0.06; % Front jounce velocity [m/s]
SuspParameters.damping.rebound.front = 0.80; % Front rebound damping [%]
SuspParameters.damping.rebound.vel.front = 0.04; % Front rebound velocity [m/s]
SuspParameters.damping.jounce.rear = 0.65;  % Rear jounce damping [%]
SuspParameters.damping.jounce.vel.rear = 0.06; % Rear jounce velocity [m/s]
SuspParameters.damping.rebound.rear = 0.80; % Rear rebound damping [%]
SuspParameters.damping.rebound.vel.rear = 0.04; % Rear rebound velocity [m/s]

% Motion ratios
SuspParameters.motionRatio.front = 1.125;   % Front motion ratio (:1)
SuspParameters.motionRatio.rear = 1.125;    % Rear motion ratio (:1)
SuspParameters.motionType.front = 'Progressive'; % Progressive spring curve
SuspParameters.motionType.rear = 'Progressive';  % Progressive spring curve

% Camber rates
SuspParameters.camberRate.ride.front = 60;  % Ride camber change [deg/m]
SuspParameters.camberRate.ride.rear = 58;   % Ride camber change [deg/m]
SuspParameters.camberRate.roll.front = -0.439; % Roll camber change [deg/deg]
SuspParameters.camberRate.roll.rear = -0.386; % Roll camber change [deg/deg]

% Anti-dive/squat
SuspParameters.antiDive = 30;              % Anti-dive [%]
SuspParameters.antiSquat = 0;              % Anti-squat [%]

% Roll centers at 1g
SuspParameters.hRC_1g.front.height = 0.072; % Front RC @1g [m]
SuspParameters.hRC_1g.front.lateral = 0.020; % Front RC lat @1g [m]
SuspParameters.hRC_1g.rear.height = 0.092;  % Rear RC @1g [m]
SuspParameters.hRC_1g.rear.lateral = 0.021; % Rear RC lat @1g [m]

%% Aerodynamic Parameters (AeroParameters)
AeroParameters.rho = 1.162;           % Air density [kg/m^3]
AeroParameters.A_ref = 0.9237;        % Reference frontal area [m^2]
AeroParameters.Cd = 1.468;            % Drag coefficient [-]
AeroParameters.downforce_80kph = 718.64; % Downforce at 80 kph [N]
AeroParameters.distribution.front = 0.1441; % Front downforce distribution [-]
AeroParameters.distribution.rear = 1 - AeroParameters.distribution.front; % Rear

% Derived metrics
V_calc = 80 / 3.6;                    % 80 kph in m/s
AeroParameters.Cl_downforce = AeroParameters.downforce_80kph / ...
    (0.5 * AeroParameters.rho * V_calc^2 * AeroParameters.A_ref); % Lift coeff
AeroParameters.Drag_80kph = 0.5 * AeroParameters.rho * V_calc^2 * ...
    AeroParameters.A_ref * AeroParameters.Cd; % Drag force at 80 kph [N]


%% Sim
% sim("launchControl.slx")

%% TESTING STUFF
addpath( genpath( fileparts( which( 'regressSRFunction.m' ) ) ) );
Fz = 600;
motorForce = 0;
maxSR = 0.3;
load('Hoosier_R25B_16x75-10x7.mat');  

TireParameters.Pacejka = Tire.Pacejka;

regressSRFunction(Fz, motorForce, maxSR, TireParameters)