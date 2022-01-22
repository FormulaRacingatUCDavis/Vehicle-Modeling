clc; clear; close all;  

%% Vehicle Parameters 
%%% Chassis Parameters
    Parameter.Wheelbase  = 1.575; 
    Parameter.TrackWidth = 1.22*ones(1,2); 
    Parameter.Mass       = 275;   
    Parameter.YawInertia = 130; 
    Parameter.CoG        = [(0.47-0.5)*Parameter.Wheelbase, 0, 0.25];
    Parameter.AMz = 0;
    Parameter.AFy = 0;
%%% Steering Kinematics Parameters
% Parameter.Steer.MaxWheel  =  130  ; % Max Steering Wheel Angle     [deg]
% Parameter.Steer.MaxTire   =   28  ; % Max Tire Steer Angle         [deg]
% Parameter.Steer.Quadratic =    1.4; % Quadratic Steer Nonlinearity [ ] 
% Parameter.Steer.Cubic     = -  1.2; % Cubic Steer Nonlinearity     [ ] 
ToeFront    = -0.0 ; % [Front] Static Toe            [deg]
ToeRear    = 0.0 ; % [Rear] Static Toe            [deg]

 Parameter.CoP  = [(0.4-0.5)*Parameter.Wheelbase, 0, 0];
 Parameter.PerLLT = 0.6;

AckermanFront = 0.2;
AckermanRear = 0.1;
Parameter.TirePos = [0.84 0.66 0; 0.84 -0.66 0; -0.67 0.66 0; -0.67 -0.66 0;];
Parameter.SpinRate = 100 .* ones(4,1);
Parameter.EffRadius = 0.19 .* ones(4,1);
%%%Aero Parameters
   Parameter.AirDensity = 1.225;
   Parameter.RefArea = 1.1;
   Parameter.DragCoeff = 0.6;
   Parameter.LiftCoeff = -1.9;
%%Tire Parameters
Parameter.Pressure    = 70;
Parameter.Inclination = 1;
Parameter.Idx         = 1;
Parameter.Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );

load('Tire.mat');
Tire.Source(4).FileName = 'B1965run4';
Tire.Source(5).FileName = 'B1965run5';
Tire.Source(6).FileName = 'B1965run6';
Tire.Radius.Effective = [];
Tire.Radius.Loaded = [];
%% Run Model / Designat Model Input 

Out = sim('SimpleVehicleModel.slx');