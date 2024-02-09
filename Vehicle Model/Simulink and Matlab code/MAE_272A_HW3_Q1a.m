%non-minimum phase example with actuator saturation criteria
clearvars; clc
close all
load Traction_Control_parameter
load CarSim_FE4.mat
load Vehicle_information_test_0118_2017.mat


s=tf('s');

wn = 70;
zeta = 1;
A = 140;
tau_tire = 0.1; % Time delay from motor to tire
tau_c = 0.03;
kc = 1;
k = (wn^2)/A;

%Gd = -0.01/(s*s*(s+10));
Gp1 = 1/(tau_tire*s+1);
Gp2 =  alpha*r/(T*TR)/(M/R*s+1);
Y = k*(tau_tire*s+1)*(s+A)/(s*s+2*zeta*wn*s+wn*wn);
T1 = minreal(Y*Gp1);
S1 = 1 - T1;
S1 = minreal(S1);
Gc = Y/S1;
T2 =  minreal(T1*Gp2);
S2 = minreal(S1*Gp2);

figure;
bode(T1,S1);
hold on
bode(Y);
hold off

figure
bode(T2,S2);