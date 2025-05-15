%% Tuning
%PID Loop
PID.Kp = m.total/5;
PID.Ki = m.total/25;
PID.Kd = m.total/10;

%% Short cuts
RPM_to_radS = 2*pi/60;

%% motor specs (EMRAX 228HV LC) 
T_peak       = 220;        %[Nm]   peak torque
P_peak       = 124e3;      %[W]    peak power
w_base = P_peak/T_peak;    % [rad/s] rpm
w_max = 6500*2*pi/60;      % [rad/s]
%RPM
torque_vs_RPM_table         = [220, 220, 170, 0];    %[Nm] - From data table, May change to match graph
torque_vs_RPM_breakpoints   = [0, 5500, 6048, 6500];  %[RPM]
%RPS
torque_vs_radS_table         = torque_vs_RPM_table*RPM_to_radS;
torque_vs_radS_breakpoints   = torque_vs_RPM_breakpoints*RPM_to_radS;  



%%  drivetrain & vehicle 
i_g          = 2.91;       %[:]     final-drive reduction
drive_train_eff        = 0.95;       %[:]     drivetrain efficiency (≈chain drive)
R_w          = 0.1778;     %[m]     wheel radius Asuming 7 in wheels
n_g          = 0.95;        % Drive train eff / chain loss

s_slip =  0.1;              % TUNE THE SLIP 
t_cntrl = 0.05;             % TIME STEP %% UPDATE WITH time stiep of system

%% Geometry and mass distribution
perc_front              = 0.543;              % Front weight distribution [%] (with driver)
W                       = 1.582;              % Wheelbase [m]
lf                      = W * (1 - perc_front); % Distance from front axle to CG [m]
lr                      = W * perc_front;     % Distance from rear axle to CG [m]
twidth.front            = 1.189;              % Front track width [m]
twidth.rear             = 1.189;              % Rear track width [m]

%% Static alignment settings
% Toe: negative is toe-out, positive is toe-in
dtoe.front              = -0.5 * (pi/180);    % Static toe front [rad] (-0.5°)
dtoe.rear               =  0   * (pi/180);    % Static toe rear [rad] (0°)

% Camber: negative is camber-out at top
staticCamber.front      = -1.3 * (pi/180);     % Static camber front [rad] (-1.3°)
staticCamber.rear       = -1.0 * (pi/180);     % Static camber rear [rad] (-1.0°)

%% Steering geometry
steer.acker.front           = 0.63;          % Static Ackermann front [%]
steer.acker.rear            = 0;             % Rear Ackermann [%]
steer.ratio                 = 5;             % Steering ratio (:1)
steer.cFactor               = 0.08788;       % C-factor [m] (87.88 mm)
steer.armLength             = 0.07009;       % Steer arm length [m] (70.09 mm)
steer.caster                = 2.5 * (pi/180);% Caster angle [rad]
steer.trail                 = 0.01081;       % Trail [m]
steer.scrubRadius           = 0.000889;      % Scrub radius [m]
steer.kingpinInclination    = 11.89 * (pi/180); % Kingpin inclination [rad]
steer.kingpinOffset         = 0.0221;        % Kingpin offset [m]

%% Mass properties
m.vehicle           = 203;                % Vehicle mass without driver [kg]
m.driver            = 68;                 % Driver mass [kg]
m.total             = m.vehicle + m.driver; % Total mass [kg]

%% Center of Gravity
hCG                 = 0.314;              % CG height [m]

%% Roll center heights (static)
hRC.front           = 0.070866;           % Front roll center height [m]
hRC.rear            = 0.090932;           % Rear roll center height [m]

%% Suspension parameters
% Tire specs
tire.make           = 'Hoosier R20';      % Tire model
% Outer diameter and width
tire.OD             = 0.4064;             % Tire outer diameter [m] (16 in)
tire.width          = 0.1905;             % Tire width [m] (7.5 in)
tire.offset         = -0.010;             % Tire design offset [m] (-10 mm)
                
% Wheel specs
wheel.make          = 'OZ Racing 10" Cast Mag';
wheel.diameter      = 0.254;              % Bead-seat diameter [m] (10 in)
wheel.width         = 0.1778;             % Wheel width [m] (7 in)
wheel.offset        = 0.022;              % Wheel offset [m] (22 mm)
wheel.circumfrence  = pi*wheel.diameter;

Pressure    = 70;
Inclination = 1;
Velocity    = 10;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
% Center-lock hub dimensions
hub.centerLock.diam     = 0.254;              % Hub center-lock diameter [m] (254 mm)
hub.centerLock.width    = 0.190;              % Hub center-lock width [m] (190 mm)

% Suspension travel
susp.travel.jounce.front    = 0.027;         % Front jounce travel [m]
susp.travel.rebound.front   = 0.027;         % Front rebound travel [m]
susp.travel.jounce.rear     = 0.027;         % Rear jounce travel [m]
susp.travel.rebound.rear    = 0.027;         % Rear rebound travel [m]

% Linear wheel rate (chassis to wheel center)
susp.wheelRate_lin.front    = 42100;         % Front wheel rate [N/m]
susp.wheelRate_lin.rear     = 46100;         % Rear wheel rate [N/m]

% Roll rate (chassis to wheel center)
susp.rollRate.front         = 740;           % Front roll rate [N*m/deg]
susp.rollRate.rear          = 801;           % Rear roll rate [N*m/deg]

% Sprung mass natural frequencies
susp.natFreq.front  = 2.33;                 % Front natural freq [Hz]
susp.natFreq.rear   = 2.42;                 % Rear natural freq [Hz]

% Damping (percent critical at specified velocity)
susp.damping.jounce.front       = 0.65;       % Front jounce damping [%]
susp.damping.jounce.vel.front   = 0.06;       % Front jounce test velocity [m/s]
susp.damping.rebound.front      = 0.80;       % Front rebound damping [%]
susp.damping.rebound.vel.front  =0.04;       % Front rebound velocity [m/s]
susp.damping.jounce.rear        = 0.65;       % Rear jounce damping [%]
susp.damping.jounce.vel.rear    = 0.06;       % Rear jounce velocity [m/s]
susp.damping.rebound.rear       = 0.80;       % Rear rebound damping [%]
susp.damping.rebound.vel.rear   = 0.04;       % Rear rebound velocity [m/s]

% Motion ratio and type
susp.motionRatio.front  = 1.125;            % Front motion ratio (:1)
susp.motionRatio.rear   = 1.125;            % Rear motion ratio (:1)
susp.motionType.front   = 'Progressive';    % Progressive spring curve
susp.motionType.rear    = 'Progressive';    % Progressive spring curve  

% Camber change rates
susp.camberRate.ride.front  = 60;           % Ride camber change [deg/m]
susp.camberRate.ride.rear   = 58;           % Ride camber change [deg/m]
susp.camberRate.roll.front  = -0.439;       % Roll camber change [deg/deg]
susp.camberRate.roll.rear   = -0.386;       % Roll camber change [deg/deg]

% Anti-dive / Anti-squat
susp.antiDive   = 30;                        % Anti-dive [%]
susp.antiSquat  = 0;                         % Anti-squat [%]

% Roll center at 1g lateral acceleration
hRC_1g.front.height     = 0.072;              % Front RC @1g [m]
hRC_1g.front.lateral    = 0.020;              % Front RC lat @1g [m]
hRC_1g.rear.height      = 0.092;              % Rear RC @1g [m]
hRC_1g.rear.lateral     = 0.021;              % Rear RC lat @1g [m]

%% Aerodynamic parameters
% Air properties and reference area (from spec sheet)
rho             = 1.162;              % Air density [kg/m^3]
A_ref           = 0.9237;             % Reference frontal area [m^2]

% Aerodynamic coefficients
Cd                  = 1.468;              % Drag coefficient [-]
downforce_80kph     = 718.64;             % Downforce at 80 kph [N]
% Distribution of downforce between front and rear
aero.distribution.front     = 0.1441;           % Front downforce distribution [-]
aero.distribution.rear      = 1 - aero.distribution.front; % Rear distribution [-]

% Derived aerodynamic metrics
V_calc               = 80 / 3.6;           % 80 kph in m/s
Cl_downforce         = downforce_80kph / (0.5 * rho * V_calc^2 * A_ref); % Lift coeff (~negative)
Drag_80kph           = 0.5 * rho * V_calc^2 * A_ref * Cd; % Drag force at 80 kph [N]

%% Gravitational constant
g                     = 9.81;               % Gravitational acceleration [m/s^2]
