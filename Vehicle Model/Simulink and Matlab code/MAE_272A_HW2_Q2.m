clearvars; clc
close all

%% parameters
n = 6; %dimension
Wn = 5/6; %rad/s
sample = 50;

%% Butter filter
[b,a] = butter(n,Wn);

%% Transfer function to State- Space form
[A,B,C,D] = tf2ss(b,a);
A
B
C
D
sys1 = ss(A,B,C,D);
t = (0:0.1:20);

%% Impaulse response
%impz(b,a,sample)
impulse(sys1,t)
