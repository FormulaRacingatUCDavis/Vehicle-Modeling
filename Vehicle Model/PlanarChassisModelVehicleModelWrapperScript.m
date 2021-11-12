 clc; clear; close all;  

%% Vehicle Parameters

%%%Controls Brake Model
Parameter.Mass.m  = 270;          % Mass [kg]
Parameter.Mass.h  = 206.18./1000; % C.G. Height [mm -> m]
Parameter.Mass.pf = .48;           % Percent Front Weight Distribution [ ]

Parameter.Susp.L = 1.525; % Wheelbase [m]

Parameter.Wheel.J = 0.148; % Wheel Spin Inertia [kg-m^2]

Parameter.Brake.Db = [0.59; 0.8125] .* 0.0254; % Cylinder Bore Diameter [in -> m]
Parameter.Brake.mu = 0.55;                   % Pad Friction [ ]
Parameter.Brake.Ap = [2.9; 1.45] * 0.0254^2;        % Brake Pad Area [in^2 -> m^2]
Parameter.Brake.Rr = 3.3 * 0.0254;           % Rotor Radius [in -> m] 

Parameter.Pedal.eta = 5.7 ; % Pedal Ratio [ ]
Parameter.Pedal.pbb = .5;  % Balance Bar Setting [ ]

%load('Hoosier_R25B_16x75-10x7.mat'); 
Parameter.Pacejka = Tire.Pacejka;
Tire.Pacejka.L.mu.x = 2/3;
Parameter.Pacejka.L.mu.x = 2/3;

%%% Kinematics Evaluation
Parameter.Wheelbase =;
Parameter.%w =;
Parameter.MotionRatio =;
Parameter. =;
Parameter.SprungMass =;
Parameter.RollCenterHeight =;
Parameter.Trackwidth =;
Parameter. =;
Parameter. =;
Parameter. =;
Parameter. =;
Parameter. =;
Parameter. =;


Parameter.SuspensionSpringStiff = 80000;
Parameter.TireStiff             = 87.45;

Parameter.SprungMass   = 2500;
Parameter.UnsprungMass = 320;

Parameter.DampingCoeff = 350;

%% Run Model / Designat Model Input 

Out = sim('PlanarChassisVehicleModel.slx');