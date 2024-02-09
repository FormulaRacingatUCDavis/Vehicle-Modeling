clearvars; clc
close all

%% Vehicle Parameters

global m_total weight_front weight_rear m g r_gy j track weight_left weight_right a b k_spring r_damper k_tire_l k_tire_r x F h mt

%Sprung mass
m_total = 237.6; %kg, total mass of vehicle
weight_front = 0.5; % percent, weight distribution at front
weight_rear = 1 - weight_front; % percent, weight distribution at rear
m = m_total*weight_front; %kg, vehicle mass at front
g = 9.81; %m/s^2, acceleration due to gravity
r_gy = 0.795; %m, radius of gyration
j = m*r_gy^2; %kg-m^2, rotational inertia

%Track information
track = 1.248; %m, track of the car
weight_right = 0.5; % percent, weight distribution at right
weight_left = 1 - weight_right; % percent, weight distribution at right
a = weight_left*track; %kg, distance between center of mass and right end of track
b = weight_right*track; %kg, distance between center of mass and left end of track

%Spring and damper
k_spring = 16640; %N/m, spring constant
r_damper = 1416.7; %N.s/m damping constant

%Tire
mt = 27.4; %kg, mass of tire
k_tire_l = 128571.4; %N/m, spring constant of left tire
k_tire_r = 128571.4; %N/m, spring constant of right tire

% Lateral acceleration input
x = 1.5; %g, lateral acceleration 
F = m*x*g; %N, Lateral force act on center of mass of carh 
h = 0.20729; %m, height of certer of mass

%% Initial Conditions

initial = [0 0 0 0 (m*g*weight_left/k_spring) (m*g*weight_right/k_spring) ((m*weight_left+mt)*g/k_tire_l) ((m*weight_right+mt)*g/k_tire_r) 0];

%% Time Control Parameters

%sim('Weight_transfer_sim')

%% Trandistional
tspan=linspace(0,20,100);
[t,s]=ode45(@weight_tr_fcn,tspan,initial);

Pj=s(:,1);
Pv=s(:,2); 
Ptl=s(:,3); 
ptr=s(:,4); 
qsl=s(:,5); 
qsr=s(:,6); 
qslt=s(:,7); 
qsrt=s(:,8); 
roll_angle=s(:,9);

ext=zeros(length(t),3);
ds=zeros(length(t),9);
for i=1:length(t)
 [ds(i,:),ext(i,:)] = weight_tr_fcn(t(i),s(i,:));

end

force_input=ext(:,1);
force_on_lefttire= ext(:,2); 
force_on_righttire= ext(:,3); 


%% Plots

figure('Name','Roll_angle vs. Time','NumberTitle','off','Color','white')
plot(t,roll_angle*180/pi);grid on
title('Time vs. Roll Angle')
ylabel('Roll Angle (rad)')
xlabel('Time (s)')

figure('Name','Force on Left Tire vs. Force on Right Tire','NumberTitle','off','Color','white')
plot(t,force_on_lefttire,'b',t,force_on_righttire,'k',t,force_input,'r','LineWidth',3);grid on
title('Force on Left Tire vs. Force on Right Tire')
legend('Force on Left Tire','Force on Right Tire','Force input')
ylabel('Force (N)')
xlabel('Time (s)')
