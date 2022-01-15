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
% Author(s):
% Tristan Pham       (atlpham@ucdavis.edu)
% 
% Last Updated: 4-Jan-2022


%% Vehicle Parameters
g = 9.8;                            % Acceleration due to gravity (m/s^2)
air_density = 1.225;
DelSW = 20;  %[0:5:130, 0:-5:-130]; % Steering Wheel Angle (degree)

tw = 1.220;                         % Track Width
long_Vel = 15;                      % Longitudinal Velocity

aero_Distribution_Front = 0;        % Front Aero Distribution
Wheelbase = 1.525;                  % Wheel Base (m)
alpha = zeros(1,4);                 % Slip Angle (degree)
delta = zeros(1,4);                 % Steer Angle (degree)
rearsteer = 1/6;
yaw_rate = 0;
turing_Radius = 0;                  % Turning Radius (m)

%%% Slip angle Calulations
velocity_CoG = [long_Vel; lat_Vel; 0];
omega = [0;0;yaw_rate];
lat_Vel = tan2d(beta).*long_Vel;
yaw_rate = sqrt(long_Vel.^2 + lat_Vel.^2) ./ turning_Radius;

tire_vel(1) = velocity_CoG + cross(omega, r(1));
tire_vel(2) = velocity_CoG + cross(omega, r(2));
tire_vel(3) = velocity_CoG + cross(omega, r(3));
tire_vel(4) = velocity_CoG + cross(omega, r(4));

alpha(1) = atan2d( (lat_Vel + a.*yaw_rate) ./ ( long_Vel - ... 
    ( (tw/2).*psi_dot ) ), long_Vel) - delta(1);

alpha(2) = atan2d( (lat_Vel + a.*yaw_rate) ./ ( long_Vel + ...
    ( (tw/2).*psi_dot ) ), long_Vel) - delta(2);

alpha(3) = atan2d( (lat_Vel - b.*yaw_rate)./ (long_Vel - ...
    ( (tw/2).*psi_dot) ), long_Vel) - delta(3);                                      

alpha(4) = - atan2d( (y_dot - b.*yaw_rate) ./ (long_Vel + ...
    ( (tw/2).*yaw_rate) ) , long_Vel) - delta(4);

%%% Tire Model
Tire = load('Hoosier_LCO_16x75-10x7.mat'); % Loading Tire Data (Tire Repo)

[~, Fy(1), ~, ~, ~] = ContactPatchLoads( Tire, alpha(1), SlipRatio, ...
    NormalLoad, Pressure, Inclination, Velocity, ...
    Idx, Model );
[~, Fy(2), ~, ~, ~] = ContactPatchLoads( Tire, alpha(2), SlipRatio, ...
    NormalLoad, Pressure, Inclination, Velocity, ...
    Idx, Model );
[~, Fy(3), ~, ~, ~] = ContactPatchLoads( Tire, alpha(3), SlipRatio, ...
    NormalLoad, Pressure, Inclination, Velocity, ...
    Idx, Model );
[~, Fy(4), ~, ~, ~] = ContactPatchLoads( Tire, alpha(4), SlipRatio, ...
    NormalLoad, Pressure, Inclination, Velocity, ...
    Idx, Model );

%%% Mass
Parameter.Mass = 200 + 68;                % Mass of the car + Driver Mass
Parameter.Pf = 0.50;                      % Percent Front Weight Distribution
I_CoG = Parameter.Mass .* 0.7^2;

%%% Suspension
ackerman_Front = 0.0;                     % Front Ackerman (degree)
ackerman_Rear = 0.0;                      % Rear Ackerman (degree)
toe_Front = 0.0;                          % Front Toe (degree)
toe_Rear = 0.0;                           % Rear Toe (degree)
unsprung_mass = 77;                       % Unsprung Mass (kg)
unsprung_CG_Height = 0.220;               % Unsprung CoG Height (m)
suspended_mass = 77;                      % Suspended Mass (kg)
height_CG_Suspended = 0.230;              % Suspendd CoG Height (m) 
roll_Center_Height = 0.1;                 % Roll Center Height (m) 
roll_Stiffness_Distribution_Front = 0.60; % Percent Front Roll Stiffness (%)


%%%% Wheel Base Distances
a = Wheelbase .* (1-Parameter.Pf);        % Front Distance From CoG
b = Wheelbase .* Parameter.Pf;            % Rear Distance From CoG

%%%% Steering Calculation
delta(1) = -toe_Front + (DelSW + ackerman_Front .* DelSW.^2);
delta(2) = toe_Front + (DelSW - ackerman_Front .* DelSW.^2);
delta(3) = -toe_Rear - rearsteer .* (DelSW - ackerman_Rear .* DelSW.^2);
delta(4) = toe_Front - rearsteer .* (DelSW + ackerman_Rear .* DelSW.^2);

%%%% Contact Patch Points
r{1}= [a;tw/2;0];
r{2}= [a;-tw/2;0];
r{3}= [b;tw/2;0];
r{4}= [b;-tw/2;0];

%%%% Force Calculation
F = zeros(1,4);
Fz = zeros(1,4);
F(1) = [-Fy(1).*sin(delta(1));Fy(1).*cos(delta(1));Fz(1)];
F(2) = [-Fy(2).*sin(delta(2));Fy(2).*cos(delta(2));Fz(2)];
F(3) = [-Fy(3).*sin(delta(3));Fy(3).*cos(delta(3));Fz(3)];
F(4) = [-Fy(4).*sin(delta(4));Fy(4).*cos(delta(4));Fz(4)];

%%%% Acceleration Calculation
latAcc = (longVel.^2 + latVel.^2)./R_CG;

%%%% Normal Force Calculation
deltaFz_Lateral_Front = deltaFz_Lateral_Front_Unsprung + ...
    deltaFz_Lateral_Front_Suspended_Geometric + ...
    deltaFz_Lateral_Front_Suspended_Elastic;
deltaFz_Lateral_Rear = deltaFz_Lateral_Rear_Unsprung + ...
    deltaFz_Lateral_Rear_Suspended_Geometric + ...
    deltaFz_Lateral_Rear_Suspended_Elastic;
%F_aero = 0.5 .* air_density .* cross_Sectional_Area .* longVel .^2 .* Downforce_Coef;

%%%%% Unsprung Weight Transfer
deltaFz_Lateral_Front_Unsrung = (2 .* unsprung_mass .* latAcc .*...
    unsprung_CG_Height) ./ tw;
deltaFz_Lateral_Rear_Unsrung = (2 .* unsprung_mass .* latAcc .*...
    unsprung_CG_Height) ./ tw;

%%%%% Suspended Weight Transfer Geometric (double Check)
deltaFz_Lateral_Front_Suspended_Geometric = (suspended_mass .*...
    (1-Parameter.Pf) .* latAcc .* roll_Center_Height_Front)./tw;
deltaFz_Lateral_Rear_Suspended_Geometric = (suspended_mass .*...
    (Parameter.Pf) .* latAcc .* roll_Center_Height_Front)./tw;

%%%%% Suspended Weight Transfer Elastic (double Check)
deltaFz_Lateral_Front_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* roll_Stiffness_Distribution_Front;
deltaFz_Lateral_Front_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* (1-roll_Stiffness_Distribution_Front);

%%% Normal Force Calculation (Positive steering)
Fz(1) = 0.5 .* Parameter.Mass .* g .* Parameter.Pf - deltaFz_Lateral_Front; %+ 0.5 .* F_aero .* aero_Distribution_Front;
Fz(2) = 0.5 .* Parameter.Mass .* g .* Parameter.Pf - deltaFz_Lateral_Front; %+ 0.5 .* F_aero .* aero_Distribution_Front;
Fz(3) = 0.5 .* Parameter.Mass .* g .* (1-Parameter.Pf) - deltaFz_Lateral_Front; %+ 0.5 .* F_aero .* (1-aero_Distribution_Front);
Fz(4) = 0.5 .* Parameter.Mass .* g .* (1-Parameter.Pf) - deltaFz_Lateral_Front; %+ 0.5 .* F_aero .* (1-aero_Distribution_Front);



%%% Lateral Acc and Yaw Acc
lat_Acc = (F_y(1) + F_y(2) + F_y(3) + F_y(4))./Parameter.Mass

yaw_Acc = (cross(r{1},Fz(1)) + cross(r{2},Fz(2)) + cross(r{3},Fz(3))...
    + cross(r{4},Fz(4)))./ I_CoG

%% Solver
