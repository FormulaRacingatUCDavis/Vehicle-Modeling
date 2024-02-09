clear all
close all
clc
s = tf('s');
Gp=400/(s^2+2*s+400);
W1=350*((s/350)+1)/(3.5*(1*s+1));% just select and tune this filter

W2=[]; % leave this blank.
    
W3=(s/350+1); % leave this blank.

P = augtf(Gp,W1,W2,W3);

[K,CL,GAM] = hinfsyn(P);

L = minreal(Gp*K);
S = inv(1+L);
T = 1-S;

bode(T)
hold on
bode(S)
bode(1/W1)
bode(1/W3)
hold off
legend('T','S','invW1','invW3')