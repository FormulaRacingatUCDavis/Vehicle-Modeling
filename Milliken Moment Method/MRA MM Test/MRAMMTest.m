clc; clear; close all;

%% MRAMMTest - MRA Moment Method Development Script
% This scripts runs a simplistic planar, rigid roll distribution model to
% estimate yaw behavior. The model will be used to experiment generation
% techniques to generate Milliken Moment Diagrams.
% 
% Inputs:
% 
% Outputs:
%
% Notes:
% Do I solve for turning radius?
% 
% Graphing
% Why do I have two lateral accelerations
%
% Author(s):
% Tristan Pham       (atlpham@ucdavis.edu)
% 
% Last Updated: 20-Jan-2022

%% Solver Initialization
Solver.i = 0;
Solver.d = 0;
Solver.q = 0;

%%Plotting initialize
lat_Acc_Array = [];
yaw_Acc_Array = [];

while Solver.i < 24
DelSW = 0 + Solver.d;                     %[0:5:130, 0:-5:-130]; % Steering Wheel Angle (degree)
    %while solver.q < 10
%% Vehicle Parameters

%%% Aero
Parameter.g = 9.8;                            % Acceleration due to gravity (m/s^2)
Parameter.air_density = 1.225;                % Air Density 
Parameter.aero_Distribution_Front = 0;        % Front Aero Distribution
Parameter.RefArea = 1.1;
Parameter.DownForceCoeff = 1.5;

%%% Tire
load('Hoosier_LCO_16x75-10x7.mat');           % Loading Tire Data (Tire Repo);
Parameter.rearsteer = 1/6;                    % Rear Steer coefficient (may not apply to us)
Parameter.turning_radius = 10;                % CHECK Turning Radius (m)
Parameter.SlipRatio = 0.01;                   % Slip Ratio (degree)
Parameter.Pressure = 80;                      % Tire Pressure (KPa)
Parameter.Inclination = 0;                    % Tire Inclination (degrees)
Parameter.Idx = 1;
Fidelity = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
%Velocity = tire_vel(1:4)

%%% Mass
Parameter.Mass = 200 + 68;                    % Mass of the car + Driver Mass
Parameter.Pf = 0.50;                          % Percent Front Weight Distribution
Parameter.I_CoG = Parameter.Mass .* 0.7^2;    % Moment of Inertia

%%% Suspension
Parameter.ackerman_Front = 0.0;                % Front Ackerman (degree)
Parameter.ackerman_Rear = 0.0;                 % Rear Ackerman (degree)
Parameter.toe_Front = 0.0;                     % Front Toe (degree)
Parameter.toe_Rear = 0.0;                      % Rear Toe (degree)
Parameter.unsprung_mass = 77;                  % Unsprung Mass (kg)
Parameter.unsprung_CG_Height = 0.220;          % Unsprung CoG Height (m)
Parameter.suspended_mass = 191;                % Suspended Mass (kg)
Parameter.height_CG_Suspended = 0.230;         % Suspendd CoG Height (m) 
Parameter.roll_Center_Height = 0.1;            % Roll Center Height (m) 
Parameter.roll_SDF = 0.60;                     % Percent Front Roll Stiffness Distribution Front (%)
Parameter.tw = 1.220;                          % Track Width
Parameter.L = 1.525;                           % Wheel Base (m)

%%% Wheel Base Distances
Parameter.a = Parameter.L .* (1-Parameter.Pf); % Front Distance From CoG
Parameter.b = Parameter.L .* Parameter.Pf;     % Rear Distance From CoG

%% Variable Initialization
alpha = zeros(1,4);                            % Slip Angle (degree)
delta = zeros(1,4);                            % Steer Angle (degree)
tire_vel = zeros(1:4);                         % Tire Velocities (m/s)
Fy = zeros(1:4);                               % Tire Velocities (m/s)
Fz = zeros(1,4);                               % Normal Loads (N)

%% Iterated Variables
beta = -10 + Solver.q;                         % CHECK Body Slip Angle (degree)
long_Vel = 10;                                 % Longitudinal Velocity (m/s)

%% Steering Calculation (Currently having 0 steer for rear tires)
delta(1) = -Parameter.toe_Front + (DelSW + Parameter.ackerman_Front .* DelSW.^2);
delta(2) = Parameter.toe_Front + (DelSW - Parameter.ackerman_Front .* DelSW.^2);
delta(3) = 0; %-Parameter.toe_Rear - Parameter.rearsteer .* (DelSW - Parameter.ackerman_Rear .* DelSW.^2);
delta(4) = 0; %Parameter.toe_Front - Parameter.rearsteer .* (DelSW + Parameter.ackerman_Rear .* DelSW.^2);

%% Contact Patch Points
T_1 = [Parameter.a; Parameter.tw/2; 0];  % Tire location in reference if body coordinate system
T_2 = [Parameter.a; -Parameter.tw/2; 0];
T_3 = [Parameter.b; Parameter.tw/2; 0];
T_4 = [Parameter.b; -Parameter.tw/2; 0];

%% Acceleration Calculation
lat_Vel = tanh(beta).*long_Vel;         % Double Check
velocity_CoG = [long_Vel; lat_Vel; 0];
yaw_rate = sqrt(long_Vel.^2 + lat_Vel.^2) ./ Parameter.turning_radius;
omega = [0;0;yaw_rate];
latAcc = (long_Vel.^2 + lat_Vel.^2)./Parameter.turning_radius;   %DOUBLE CHECK

%% Unsprung Weight Transfer
deltaFz_LFU = ( Parameter.unsprung_mass .* latAcc .*...
    Parameter.unsprung_CG_Height) ./ Parameter.tw; 
% Lateral Front Unsprung Weight Transfer

deltaFz_LRU = ( Parameter.unsprung_mass .* latAcc .*...
    Parameter.unsprung_CG_Height) ./ Parameter.tw; 
% Lateral Rear Unsprung Weight Transfer

%% Suspended Weight Transfer Geometric
deltaFz_Lateral_Front_Suspended_Geometric = (Parameter.suspended_mass .*...
    (1-Parameter.Pf) .* latAcc .* Parameter.roll_Center_Height) ./ Parameter.tw;
% Lateral Front Geometric Weight Transfer (Suspended)

deltaFz_Lateral_Rear_Suspended_Geometric = (Parameter.suspended_mass .*...
    (Parameter.Pf) .* latAcc .* Parameter.roll_Center_Height) ./ Parameter.tw;
% Lateral Rear Geometric Weight Transfer (Suspended)

%% Suspended Weight Transfer Elastic
deltaFz_Lateral_Front_Suspended_Elastic = ((Parameter.suspended_mass .* latAcc ...
    .* (Parameter.height_CG_Suspended - Parameter.roll_Center_Height)) ...
    ./ Parameter.tw ) .* Parameter.roll_SDF;
% Lateral Front Elastic Weight Transfer (Suspended)

deltaFz_Lateral_Rear_Suspended_Elastic = ((Parameter.suspended_mass .* latAcc ...
    .* (Parameter.height_CG_Suspended - Parameter.roll_Center_Height)) ...
    ./ Parameter.tw ) .* (1-Parameter.roll_SDF);
% Lateral Rear Elastic Transfer (Suspended)

%% Aero
F_aero = 0.5.^3 .* Parameter.air_density .*Parameter.RefArea .* long_Vel .^2 ...
.* Parameter.DownForceCoeff;

%% Normal Force Calculation
deltaFz_Lateral_Front = deltaFz_LFU + ...
    deltaFz_Lateral_Front_Suspended_Geometric + ...
    deltaFz_Lateral_Front_Suspended_Elastic-F_aero;
deltaFz_Lateral_Rear = deltaFz_LRU + ...
    deltaFz_Lateral_Rear_Suspended_Geometric + ...
    deltaFz_Lateral_Rear_Suspended_Elastic-F_aero;

%% Normal Force Calculation (Positive steering) (VALUES TOO SMALL)
Fz(1) = 0.5 .* Parameter.Mass .* Parameter.g .* Parameter.Pf - deltaFz_Lateral_Front; + 0.5 .* F_aero .* Parameter.aero_Distribution_Front;
Fz(2) = 0.5 .* Parameter.Mass .* Parameter.g .* Parameter.Pf - deltaFz_Lateral_Front; + 0.5 .* F_aero .* Parameter.aero_Distribution_Front;
Fz(3) = 0.5 .* Parameter.Mass .* Parameter.g .* (1-Parameter.Pf) - deltaFz_Lateral_Front; + 0.5 .* F_aero .* (1-Parameter.aero_Distribution_Front);
Fz(4) = 0.5 .* Parameter.Mass .* Parameter.g .* (1-Parameter.Pf) - deltaFz_Lateral_Front; + 0.5 .* F_aero .* (1-Parameter.aero_Distribution_Front);

%% Slip angle Calulations
tire_vel_1 = velocity_CoG + cross(omega, T_1);
tire_vel_2 = velocity_CoG + cross(omega, T_2);
tire_vel_3 = velocity_CoG + cross(omega, T_3);
tire_vel_4 = velocity_CoG + cross(omega, T_4);

alpha(1) = atan2d( (lat_Vel + Parameter.a.*yaw_rate) ./ ( long_Vel - ... 
    ( (Parameter.tw/2).*yaw_rate ) ), long_Vel) - delta(1);

alpha(2) = atan2d( (lat_Vel + Parameter.a.*yaw_rate) ./ ( long_Vel + ...
    ( (Parameter.tw/2).*yaw_rate ) ), long_Vel) - delta(2);

alpha(3) = atan2d( (lat_Vel - Parameter.b.*yaw_rate)./ (long_Vel - ...
    ( (Parameter.tw/2).*yaw_rate) ), long_Vel) - delta(3);                                      

alpha(4) = - atan2d( (lat_Vel - Parameter.b.*yaw_rate) ./ (long_Vel + ...
    ( (Parameter.tw/2).*yaw_rate) ) , long_Vel) - delta(4);

%% Tire Model
[~, Fy(1), ~, ~, ~] = ContactPatchLoads( Tire, alpha(1), Parameter.SlipRatio, ...
    Fz(1), Parameter.Pressure, Parameter.Inclination, tire_vel_1, ...
    Parameter.Idx, Fidelity );
[~, Fy(2), ~, ~, ~] = ContactPatchLoads( Tire, alpha(2), Parameter.SlipRatio, ...
    Fz(2), Parameter.Pressure, Parameter.Inclination, tire_vel_2, ...
    Parameter.Idx, Fidelity );
[~, Fy(3), ~, ~, ~] = ContactPatchLoads( Tire, alpha(3), Parameter.SlipRatio, ...
    Fz(3), Parameter.Pressure, Parameter.Inclination, tire_vel_3, ...
    Parameter.Idx, Fidelity );
[~, Fy(4), ~, ~, ~] = ContactPatchLoads( Tire, alpha(4), Parameter.SlipRatio, ...
    Fz(4), Parameter.Pressure, Parameter.Inclination, tire_vel_4, ...
    Parameter.Idx, Fidelity );

%% Force Calculation
F_1 = [-Fy(1).*sin(delta(1));Fy(1).*cos(delta(1));Fz(1)];
F_2 = [-Fy(2).*sin(delta(2));Fy(2).*cos(delta(2));Fz(2)];
F_3 = [-Fy(3).*sin(delta(3));Fy(3).*cos(delta(3));Fz(3)];
F_4 = [-Fy(4).*sin(delta(4));Fy(4).*cos(delta(4));Fz(4)];

%% Lateral Acc and Yaw Acc  DOUBLE CHECK
lat_Acc = (Fy(1) + Fy(2) + Fy(3) + Fy(4))./Parameter.Mass

yaw_Acc = (cross(T_1,F_1) + cross(T_2,F_2) + cross(T_3,F_3)...
    + cross(T_4,F_4))./ Parameter.I_CoG;  %Shouldn't be a column vector?
yaw_Acc = yaw_Acc(3)

Solver.i = Solver.i + 1;
Solver.q = Solver.q + 1;
    %end
Solver.d = Solver.d + 5;
lat_Acc_Array(end+1) = lat_Acc;
yaw_Acc_Array(end+1) = yaw_Acc;
end

plot(lat_Acc_Array,yaw_Acc_Array)

%         figure( 'Name', 'Beta Isolines')
%         plot3( , ,  , 'r' ); hold on;
%         plot3( , ,  ,  'b' );
%         xlabel( 'deltaSW [deg]' ); ylabel( 'ay [g]' ); zlabel( 'psiDDot [rad/s^2]' );
%         
%         figure( 'Name', 'Beta Isolines')
