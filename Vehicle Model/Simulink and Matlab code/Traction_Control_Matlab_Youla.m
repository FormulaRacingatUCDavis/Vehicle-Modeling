clearvars; clc
close all

%% Load files

load Traction_Control_parameter


%% Youla Controller Parameter

wn = 70;
zeta = 1;
A = 70;
tau_tire = 0.1; % Time delay from motor to tire
tau_c = 0.03;
kc = 1;

%% Controller Switch 
FB_S = 1; % On=1 , Off = 0

%% Simulation (Youla)

sim('Traction_Control_main_code_Youla3')
real_time_slip_Youla_2nd = real_time_slip;

sim('Traction_Control_main_code_Youla2')
real_time_slip_Youla = real_time_slip;

sim('Traction_Control_main_code')
real_time_slip_PID = real_time_slip;

%% Display
figure('units','normalized','outerposition',[0 0 1 1])
plot(throttle_input,'black');
hold on
plot(real_time_slip_Youla,'r');
hold on
plot(real_time_slip_PID,'b');
hold on
plot(real_time_slip_Youla_2nd,'r--');
hold on
y=real_time_slip.Time*0+0.2;
plot(real_time_slip.Time,y,'black--');
title('Slip vs. time')
xlabel('Time [s]')
ylabel('Slip')
legend('throttle input','Slip (Youla)','Slip (PID)','Slip (Youla 2nd)')