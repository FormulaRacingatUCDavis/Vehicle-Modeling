%% EME185 - Bicycle Demo
% Here is a really interesting comment.  Yes.
clearvars; clc
close all
%% Parameters
m = 1600; %kg, vehicle total mass
L = 2.4; %m, wheelbase
abyb = 0.6; % Ratio of a to b
b = L/(1 + abyb); a = (L - b);
Jy = 0.4*m*a*b; %kg-m-m, vehicle centroidal mass moment of inertia
u0 = 65*8.94/20; %m/s, forward speed
Cf = 5e4; %N/rad, front axle stiffness
Ku = 1; % >1 = understeering, <1 oversteering
Cr = Ku*abyb*Cf; %N/rad, rear axle stiffness
strMag = 0.95; %deg, tire-steered angle step input
tauS = 1/2/pi/2; % time constant for steering LPF
%% Initial conditions
inits = zeros(5,1);
%% Simulation Control Parameters
% another freakin comment
t_final = 6; %s, ...
maxstep = 0.01; %...
sim('bikecar')
%% Post/plots
v = x(:,1); wy = x(:,2);
%% Some Plots
% Set some defaults
set(0,'DefaultLineLineWidth',2.0)
set(0,'DefaultAxesFontSize',12)
% INERTIAL DISPLACEMENT
xP = downsample(x,30);
figure('Name','inertial','NumberTitle','off','Color','white')
    quiver(xP(:,4),xP(:,5),cos(xP(:,3)),sin(xP(:,3)),0.25), axis equal
    title('Top-down view','FontSize',14)
    ylabel('displacement (m)','FontSize',14)
    xlabel('displacement (m)','FontSize',14)
% STEERED ANGLE
figure('Name','delta','NumberTitle','off','Color','white')
    plot(t,delta*180/pi,'b')
    title('tire-steered angle')
    ylabel('angle (deg)')
    xlabel('time (s)')
% YAW
figure('Name','yawrate','NumberTitle','off','Color','white')
    plot(t,wy,'k')
    title('yaw rate')
    ylabel('angular velocity (rad/s)')
    xlabel('time (s)')
% Fy
figure('Name','Fys','NumberTitle','off','Color','white')
    plot(t,Cf*int(:,1),t,Cr*int(:,2)); grid on
    title('lateral tire force')
    ylabel('force (rad/s)')
    legend('F','R')
    xlabel('time (s)')
% Lateral acceleration
figure('Name','Lateracc','NumberTitle','off','Color','white')
    plot(t,int(:,3)); grid on
    title('lateral acceleration')
    ylabel('force (G)')
    xlabel('time (s)')
% CHASSIS SIDESLIP
figure('Name','sideslip','NumberTitle','off','Color','white')
    plot(t,atan2(v,u0)*180/pi); grid on
    title('chassis sideslip')
    ylabel('angle (deg)')
    xlabel('time (s)')