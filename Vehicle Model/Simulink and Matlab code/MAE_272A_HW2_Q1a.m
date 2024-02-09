clearvars; clc
close all

%% Parameters

tau = 2; %sec
tau_0 = 10; %sec
A = 10; %ft^2
t = (0:0.1:20);

%% Metric of state 

a = [(-2/tau - 1/tau_0) 1/tau 1/tau; 1/tau -2/tau 1/tau; 1/tau 1/tau -2/tau];
b = [1/A 0; 0 1/A; 0 0];
b1 =[1/A ; 0 ; 0 ];
b2 =[0; 1/A; 0];
c = [0 1 0];
d = [0];

%% System 1, step inputs u1 and u2 both active
%system_Q1 = ss(a,b,c,d);
%step(system_Q1, t);

%% System 2, only step input u1 active 
system_Q2 = ss(a,b1,c,d);


%% System 3, only step input u2 active 
system_Q3 = ss(a,b2,c,d);

%% Plot

step(system_Q2, t);
hold on;
title('Step Response to Hydralic System');
xlabel('Time [s]');
ylabel(' Y [Height]');
step(system_Q3, t);
legend('U1(t)', 'U2(t)')
