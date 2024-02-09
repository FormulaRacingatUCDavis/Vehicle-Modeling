clearvars; clc
close all

%% Load files

load Traction_Control_parameter
load CarSim_FE4.mat
load Vehicle_information_test_0118_2017.mat

inertiaDriveShaft = 12;
ratioDiff = 2;
radiusWheel = 16*2.54/100;
massVehicle = 1500;
inertiaLateral = 25*(radiusWheel)^2;
beta = 15;
engineRPM0 = 800;
tireStiffness = 200000;
tireDamping = 1000;

%% Youla Controler Prameter

wn = 1;
zeta = 0.707;
A = 1.5;
tau_tire = 0.1;
tau_c = 0.03;
kc = 1;
Slip_r = 0.2; %Slip target

%% Controller Switch 
FB_S = 1; % On=1 , Off = 0

%% Simulation 

sim('Traction_Control_Testing_3_1_2017')
real_time_slip_control = real_time_slip;
vehicle_speed_control = vehicle_speed;

FB_S = 0;

sim('Traction_Control_Testing_3_1_2017')
real_time_slip_nocontrol = real_time_slip;
vehicle_speed_nocontrol = vehicle_speed;
%% Display
figure('units','normalized','outerposition',[0 0 1 1])
plot(throttle_input,'black');
hold on
plot(real_time_slip_control,'b');
hold on
plot(real_time_slip_nocontrol,'r');
hold on
y=real_time_slip.Time*0+0.2;
plot(real_time_slip.Time,y,'black--');
title('Slip vs. time')
xlabel('Time [s]')
ylabel('Slip')
legend('throttle input','Slip (w/ controller)','Slip (w/o controller)')

figure('units','normalized','outerposition',[0 0 1 1])
plot(throttle_input*110,'black');
hold on
plot(vehicle_speed_control,'b');
hold on
plot(vehicle_speed_nocontrol,'r');
title('Speed vs. time')
xlabel('Time [s]')
ylabel('Speed (km/hr')
legend('throttle input','Speed (w/ controller)','Speed (w/o controller)')