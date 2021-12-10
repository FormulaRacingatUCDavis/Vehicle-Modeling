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
% Last Updated: 9-Dec-2021


%% Vehicle Parameters
%%% Mass


%%% Suspension


%%% Tire Model
load('Hoosier_LCO_16x75-10x7.mat');

% Pacejka Scaling Factors


%%% Controls


%%% Calculated Parameters

%%%% Wheel Base Distances
a = m * g * pf;
b = m * (1-pf);

%%%% Steering Calculation
delta(1) = -toe_Front + (delta + ackerman_Front .* delta.^2);
delta(2) = toe_Front + (delta - ackerman_Front .* delta.^2);
delta(3) = -toe_Rear - rearsteer .* (delta - ackerman_Rear .* delta.^2);
delta(4) = toe_Front - (delta + ackerman_Rear .* delta.^2);

%%%% Contact Patch Points
r(1)= [a;tw/2;0];
r(2)= [a;-tw/2;0];
r(3)= [b;tw/2;0];
r(4)= [b;-tw/2;0];

%%%% Force Calculation
F(1:4) = [-Fy(i).*sin(delta(i));Fy(i).*cos(delta(i));Fz(i)];

%%%% Acceleration Calculation
latAcc = (longVel.^2 + latVel.^2)./R_CG;

%%%% Normal Force Calculation
deltaFz_Lateral_Front = deltaFz_Lateral_Front_Unsprung + ...
    deltaFz_Lateral_Front_Suspended_Geometric + ...
    deltaFz_Lateral_Front_Suspended_Elastic;
deltaFz_Lateral_Front = deltaFz_Lateral_Rear_Unsprung + ...
    deltaFz_Lateral_Rear_Suspended_Geometric + ...
    deltaFz_Lateral_Rear_Suspended_Elastic;
F_aero = 0.5 .* air_density .* cross_Sectional_Area .* longVel .^2 .* Downforce_Coef;

%%%%% Unsprung Weight Transfer
deltaFz_Lateral_Front_Unsrung = (2 .* unsprung_mass .* latAcc .* unsprung_CG_Height) ./ tw;
deltaFz_Lateral_Rear_Unsrung = (2 .* unsprung_mass .* latAcc .* unsprung_CG_Height) ./ tw;

%%%%% Suspended Weight Transfer Geometric (double Check)
deltaFz_Lateral_Front_Suspended_Geometric = (suspended_mass .* (1-pf) .* latAcc .* roll_Center_Height_Front)./tw;
deltaFz_Lateral_Rear_Suspended_Geometric = (suspended_mass .* (pf) .* latAcc .* roll_Center_Height_Front)./tw;

%%%%% Suspended Weight Transfer Elastic (double Check)
deltaFz_Lateral_Front_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* roll_Stiffness_Distribution_Front;
deltaFz_Lateral_Front_Suspended_Elastic = ((suspended_mass .* latAcc .* (height_CG_Suspended)...
    - roll_Center_Height) ./ tw ) .* (1-roll_Stiffness_Distribution_Front);

%%% Normal Force Calculation
Fz(1) = 0.5 .* m .* g .* percentWeight - deltaFz_Lateral_Front + 0.5 .* F_aero .* aero_Distribution_Front;
Fz(2) = ;
Fz(3) = ;
Fz(4) = ;

%%% Slip angle Calulations
veocity_CoG = [long_Vel; lat_Vel; 0];
omega = [0;0;yaw_rate];
lat_Vel = tan2d(beta).*long_Vel;
yaw_rate = sqrt(long_Vel.^2 + lat_Vel.^2) ./ turning_Radius;
tire_vel(i) = velocity_CoG + cross(omega, r(i);
alpha(i) = arctan2d(v_y(i)./v_x(i)) - delta(i); % How Do I Get v_y and v_x

%%% Lateral Acc and Yaw Acc
lat_Acc = (F_y(1) + F_y(2) + F_y(3) + F_y(4))./m; 
yaw_Acc = (cross(r(i),Fz(i)))./ I_CoG;

%% Initializing Vehicle States


%% Solver
