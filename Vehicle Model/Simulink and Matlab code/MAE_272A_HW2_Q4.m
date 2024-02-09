clearvars; clc
close all

%% Metric of state 

A = [-2 -1 0; 1 0 0; 0 1 0];
Bu = [1;0;4];
Bd = [0;0;2];
C = [0 0 1];
D = [0];

%% State Space to Transfer Function

[num_u,den_u] = ss2tf(A, Bu, C, D);
[num_d,den_d] = ss2tf(A, Bd, C, D);
TF_u = tf(num_u,den_u);
TF_d = tf(num_d,den_d);
t = (0:0.1:20);
[zu,pu,ku]= ss2zp(A, Bu, C, D);
[zd,pd,kd]= ss2zp(A, Bd, C, D);

%% Display

x1 = ['The transfer function of Y/u is '];
x2 = ['The transfer function of Y/d is '];

disp(x1);
TF_u
disp('where');
Zero = zu 
Pole = pu
Gain = ku

disp(x2);
TF_d
disp('where');
Zero = zd 
Pole = pd
Gain = kd

