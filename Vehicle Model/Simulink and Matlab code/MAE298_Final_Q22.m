clearvars; 
clear all
close all
clc
s = tf('s');

%tau in Gp
tau1 = 0;
tau2 = 0.1;

%Gp
Gp1 = (exp(-tau1*s))/(s-1); %tau = o
Gp2 = (exp(-tau2*s))/(s-1); %tau = 1
Gp = -(0.05*s-1)/((s-1)*(0.05*s+1)); % 1st order estimation

%w 
W1= (s+20)/(8*(s+2));% just select and tune this filter

W2= 0.01; % Assume 2.
    
W3= 37.5*(s+20)/(s+900); % leave this blank.

P = augtf(Gp,W1,W2,W3);

[K,CL,GAM] = hinfsyn(P);

L = minreal(Gp*K);
S = inv(1+L);
T = 1-S;

K=minreal(K);
Gc=minreal(tf(K));
Y=minreal(Gc/(1+Gc*Gp));

bode(T)
hold on
bode(S)
bode(1/W1)
bode(1/W3)
hold off
legend('T','S','invW1','invW3')