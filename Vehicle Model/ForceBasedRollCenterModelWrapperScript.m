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
ToeFront = -0.0 ; % [Front] Static Toe            [deg]
ToeRear  = 0.0 ; % [Rear] Static Toe              [deg]

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

%FIXME
TFx = zeros(1,4);
    TFy = [250, 250, 0, 0];
    TMz = zeros(1,4);
    
    AFx = -100;
    AFy = 0;
    AMz = 0;
    AFz = 0;
    AMy = 0;
    
    Wheelbase  = 1.575; 
    TrackWidth = 1.22;               %*ones(1,2); (FIXME)did a quick fix for array sizes 
    TrackWidthTest = 1.22*ones(1,2);
    Steer      = [18, 15, 1, -1];
    
    Mass         = 275;   
    YawInertia   = 130; 
    CoG          = [(0.47-0.5)*Wheelbase, 0, 0.25];
    PercentFront = 0.5;
    Sprung_COG_Height = 0;
    
    LongVel = 15;
    LatVel  = 0;
    YawVel  = 0.5;

    Front_View_Suspension_Arm = 0;   %calculation
    Side_View_Suspension_Arm = 0;
    Front_View_Jacking_Angle = 0;    
    Side_View_Jacking_Angle = 0;    
    
    Front_Roll_Stiffness = 0;
    Rear_Roll_Stiffness = 0;
    Roll_Dampening_Coeff_Front = 0;
    Roll_Dampening_Coeff_Rear = 0;
    Roll_Speed = 0;
    Roll = 0;

    Pitch_Stiffness = 0;
    Pitch_Dampening = 0;
    Pitch = 0;
    Pitch_Velocity = 0;

    Ride_Stiffness = 0;
    Ride = 0;
    Ride_Dampening = 0;
    Ride_Velocity = 0;

    Iyy = 0;
    Ixx = 0;

    z = 0;

%% Run Model / Designat Model Input 

Out = sim('ForceBasedRollCenterVehicleModel.slx');