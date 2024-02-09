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
W1= (s+8)/(3.2*(s+2));% just select and tune this filter

W2= 0.01; % Assume 2.
    
W3= (s+20)/(0.0267*(s+900)); % leave this blank.

P = augtf(Gp,W1,W2,W3);

[K,CL,GAM] = hinfsyn(P);

K = minreal(tf(K))
L = minreal(Gp*K);
S = inv(1+L);
T = minreal(1-S);

K=minreal(K);
Gc=minreal(tf(K));
Y=minreal(Gc/(1+Gc*Gp));


%tau in Y ans T
tau3 = 0.036; %tau1 in Y
tau4 = 0.1; %tau2 in Y
k1 = 1.2;

%Y
Y1 = (k1*(s-1)*(0.05*s+1))/((tau3*s+1)*(tau4*s+1)); 

%Gp
Gp1 = (exp(-tau1*s))/(s-1); %tau = o

Gp3 = -(0.05*s-1)/((s-1)*(0.05*s+1)); % 1st order estimation

%T
T1 = minreal(Gp1*Y1)

T3 = minreal(Gp3*Y1)

%S
S1 = minreal(1 - T1)

S3 = minreal(1 - T3)

%Gc
Gc1 = zpk(minreal(Y1/S1))

Gc3 = zpk(minreal(Y1/S3))


bode(T)
hold on
bode(S)
hold on
bode(1/W1)
hold on
bode(1/W3)
hold off
legend('T','S','invW1','invW3')