%Final Q1
clearvars; clc
close all

s=tf('s');

%tau in Gp
tau1 = 0;
tau2 = 0.1;

%tau in Y ans T
tau3 = 0.036; %tau1 in Y
tau4 = 0.1; %tau2 in Y
k = 1.2;

%Y
Y = (k*(s-1)*(0.05*s+1))/((tau3*s+1)*(tau4*s+1)); 

%Gp
Gp1 = (exp(-tau1*s))/(s-1); %tau = o
Gp2 = (exp(-tau2*s))/(s-1); %tau = 1
Gp3 = -(0.05*s-1)/((s-1)*(0.05*s+1)); % 1st order estimation

%T
T1 = minreal(Gp1*Y)
T2 = minreal(Gp2*Y)
T3 = minreal(Gp3*Y)

%S
S1 = minreal(1 - T1)
S2 = minreal(1 - T2)
S3 = minreal(1 - T3)

%Gc
Gc1 = zpk(minreal(Y/S1))
Gc2 = zpk(minreal(Y/S2))
Gc3 = zpk(minreal(Y/S3))

%L
L1 = Gc1*Gp1
L2 = Gc2*Gp2
L3 = Gc3*Gp3