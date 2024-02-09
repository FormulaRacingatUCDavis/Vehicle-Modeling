clc; clear; close all;  

%% Vehicle Parameters 
%%% Chassis Parameters
    Wheelbase  = 1.575; 
    TrackWidth = 1.22*ones(1,2); 
    Mass       = 275;   
    YawInertia = 130; 
    PercentFront = 0.5;
    CoG        = [(0.47-0.5)*Wheelbase, 0, 0.25];
    AMz = 0;
    AFy = 0;
%%% Steering Kinematics Parameters
% Parameter.Steer.MaxWheel  =  130  ; % Max Steering Wheel Angle     [deg]
% Parameter.Steer.MaxTire   =   28  ; % Max Tire Steer Angle         [deg]
% Parameter.Steer.Quadratic =    1.4; % Quadratic Steer Nonlinearity [ ] 
% Parameter.Steer.Cubic     = -  1.2; % Cubic Steer Nonlinearity     [ ] 
ToeFront = -0.0 ; % [Front] Static Toe            [deg]
ToeRear  = 0.0 ; % [Rear] Static Toe              [deg]

CoP  = [(0.4-0.5)*Wheelbase, 0, 0];
PerLLT = 0.6;

AckermanFront = 0;
AckermanRear = 0;
TirePos = [0.84 0.66 0; 0.84 -0.66 0; -0.67 0.66 0; -0.67 -0.66 0;];
SpinRate = 100 .* ones(4,1);
EffRadius = 0.19 .* ones(4,1);
%%%Aero Parameters
AirDensity = 1.225;
RefArea = 1.1;
DragCoeff = 0.6;
LiftCoeff = -1.9;
%%Tire Parameters
Pressure    = 80;
Inclination = 1;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

load('Hoosier_R25B_16x75-10x7.mat');
% Tire.Source(4).FileName = 'B1965run4';
% Tire.Source(5).FileName = 'B1965run5';
% Tire.Source(6).FileName = 'B1965run6';
% Tire.Radius.Effective = [];
% Tire.Radius.Loaded = [];
%% Run Model / Designat Model Input 

Out = sim('SimpleVehicleModelAttempt2.slx');
