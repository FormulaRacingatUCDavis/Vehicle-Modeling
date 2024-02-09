clearvars; clc
close all
clearvars; clc
close all

%% Vehicle Parameters

%global m_total weight_front weight_rear m g r_gy j track weight_left weight_right a b k_spring r_damper k_tire_l k_tire_r x F h mt

%Dynamic
u = 80; %km/hr, longitudinal speed

%Sprung mass
m_total = 237.6; %kg, total mass of vehicle
Izz = 89.99 ; %kg-m^2, Yaw inertia respect to Z-axis
weight_front = 0.5; % percent, weight distribution at front
weight_rear = 1 - weight_front; % percent, weight distribution at rear
mf = m_total*weight_front; %kg, vehicle mass at front
mr = m_total*weight_rear; %kg, vehicle mass at rear
g = 9.81; %m/s^2, acceleration due to gravity
r_gy = 0.795; %m, radius of gyration
jf = mf*r_gy^2; %kg-m^2, rotational inertia respect to X-axis at front
jr = mr*r_gy^2; %kg-m^2, rotational inertia respect to X-axis at rear

%Wheelbase information
wheelbase = 1.575; %m, wheelbase of the car
aa = weight_front*wheelbase; %m, distance between center of mass and front end
bb = weight_rear*wheelbase; %m, distance between center of mass and front end

%Track information
track = 1.248; %m, track of the car
weight_right = 0.5; %percent, weight distribution at right
weight_left = 1 - weight_right; % percent, weight distribution at right
a = weight_left*track; %kg, distance between center of mass and right end of track
b = weight_right*track; %kg, distance between center of mass and left end of track

%Spring and damper
k_springf = 16640; %N/m, spring constant at front
r_damperf = 1416.7; %N.s/m damping constant at front
k_springr = 16640; %N/m, spring constant at rear
r_damperr = 1416.7; %N.s/m damping constant at rear

%Tire
mt = 27.4/2; %kg, mass of tire
k_tire_l = 128571.4; %N/m, spring constant of left tire
k_tire_r = 128571.4; %N/m, spring constant of right tire

% Lateral acceleration input
%x = 1.5; %g, lateral acceleration 
%F = m*x*g; %N, Lateral force act on center of mass of carh 
hf = 0.20729; %m, height of certer of mass at front
hr = 0.20729; %m, height of certer of mass ar rear

% Angular velocity of steering wheel
steer_angle = 35.122; %deg, streer angle
steer_time = 0.15; %sec, sample time


%% Initial Conditions

initial_3 = [0 0 0 0 (mr*g*weight_left/k_springr) (mr*g*weight_right/k_springr) ((mr*weight_left+mt)*g/k_tire_l) ((mr*weight_right+mt)*g/k_tire_r) 0];
%% Time Control Parameters

sim('Weight_transfer_sim')


%% Plots

figure('Name','Roll_angle vs. Time','NumberTitle','off','Color','white')
plot(roll_angle); grid on;
title('Time vs. Roll Angle')
ylabel('Roll Angle (rad)')
xlabel('Time (s)')

figure('Name','Force on Left Tire vs. Force on Right Tire','NumberTitle','off','Color','white')
plot(force_on_lefttire,'b')
hold on; grid on;
plot(force_on_righttire,'k')
hold on;
plot(input_force,'r');
title('Force on Left Tire vs. Force on Right Tire')
legend('Force on Left Tire','Force on Right Tire','Force input')
ylabel('Force (N)')
xlabel('Time (s)')
