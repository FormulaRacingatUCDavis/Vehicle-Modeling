clearvars; clc
close all

syms tau
%tau = 1;
A = [0 -1; 4 -4]
B = [0 1; 1 4] 
C = [1 0; 0 1]
D = 0
[X]=int(expm(A*(1-tau))*B*[1;1],0,1);
X1 = X(1)
X2 = X(2)