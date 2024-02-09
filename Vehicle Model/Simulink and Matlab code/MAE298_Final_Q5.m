clear all
close all
clc
s = tf('s');

%Gp
Gp = (s+1)/(s*s*s)

wn = 10.95;
zeta = 0.707;

C=2;
D=3;
E=4;
F=5;
K=C*D*E*F/(wn*wn);

%Y
Y = zpk(minreal(K*s*s*s*(s^2+2*zeta*wn*s+wn^2)/((s+C)*(s+D)*(s+E)*(s+F)*(s+1))))

%T
T = zpk(minreal(Y*Gp))

%S
S = zpk(minreal(1 - T))

bode(Y)