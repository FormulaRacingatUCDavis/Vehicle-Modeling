clearvars; clc
close all

%% Vehicle Parameters

m = 1800; %kg, vehicle mass
g = 9.81; %m/s^2, acceleration due to gravity
mr = 1.25; %kg, rack mass
Jy = 4250; %kg-m^2, vehicle yaw moment of inertia
Jwd = 0.45; %kg-m^2, tire diametric moment of inertia
Jm = 3.5e-4; %kg-m^2, steering motor moment of inertia
a = 1.15; %m, CG to front axle 
b = 1.65; %m, rear axle to CG
l = 0.135; %m, effective steering arm
Ca = 1*0.025; %m, caster trail
rp = 0.01; %m, pinion radius
gr = 1/20; % motor gearing ratio
T = 0.0533; %N-m/rad, motor constant
bL = 60; %N-s/m, torsional viscous damping, lower column
kL = 1400; %N/m, torsional stiffness, lower column
kU = 140; %N/m, torsional stiffness, upper column (TAS)
Nf = b*m*g/(a + b); %N, front axle normal force
Nr = a*m*g/(a + b); %N, rear axle normal force
Cy = 2*20000; %N/rad, linear tire stiffness (both tires)
bm = 0.005;


%% Initial Conditions

initial = [0 0 0 0 0 0 0 0 0 0];

%% Time Control Parameters

sim('eme185')

%% Movie
X = s(:,9);
Y = s(:,10);
psi = s(:,8);
delta = s(:,7);
% X = int64(X);
% Y = int64(Y);
% 
% 
% close all
% v = VideoWriter('movie002.mp4');
% open(v);
% X_min = min(X); X_max = max(X);
% Y_min = min(Y); Y_max = max(Y);
% [X_car, Y_car] = draw_car(X,Y,psi,delta);
% figure('units','normalized','outerposition',[0 0 1 1])
% axis equal
% axis([X_min-10 X_max+10 Y_min-10 Y_max+10])
% set(gca,'Color',[0.15 0.12 0.12])
% for i = linspace(1,length(t),101);
%     plot(X(1:i),Y(1:i),'k--');
%     hold on
%     plot_car(X_car,Y_car,i);
%     hold off
%     set(gca,'Color',[0.7 0.7 0.7])
%     axis equal
%     axis([X_min-10 X_max+10 Y_min-10 Y_max+10])
%     frame = getframe;
%     writeVideo(v,frame);
%     pause(0.001)
% end
% close(v);

%% Plots

figure('units','normalized','outerposition',[0 0 1 1])
subplot(4,1,1)
plot(Wsw*180/pi)
axis([0 10 -10 210])
title('Angular Velocity of Steering Wheel')
xlabel('Time [s]')
ylabel('\omega_{sw} [deg/s]')

subplot(4,1,2)
plot(t,s(:,6)/Jm)
title('Current in Steering Motor')
xlabel('Time [s]')
ylabel('i_C [A]')

subplot(4,1,3)
plot(t,s(:,5)*180/pi)
title('Angular Displacement of Upper Column')
xlabel('Time [s]')
ylabel('\theta_L [deg]')

subplot(4,1,4)
plot(t,s(:,4)*180/pi)
title('Angular Displacement of Lower Column')
xlabel('Time [s]')
ylabel('\theta_L [deg]')

figure('units','normalized','outerposition',[0 0 1 1])
plot(Fyf,'b');
hold on
plot(Fyr,'r');
title('Lateral Forces on Tires')
xlabel('Time [s]')
ylabel('Force [N]')
legend('Front Wheel','Rear Wheel')

figure('units','normalized','outerposition',[0 0 1 1])
plot(alat/g);
hold on
title('Lateral Acceleration')
xlabel('Time [s]')
ylabel('Acceleration [g]')
t_max = t((alat==max(alat)));

figure('units','normalized','outerposition',[0 0 1 1])
plot(alphaf*180/pi,'b');
hold on
plot(alphar*180/pi,'r');
title('Slip Angles of Tires')
xlabel('Time [s]')
ylabel('Slip Angle [deg]')
legend('Front Wheel','Rear Wheel')

figure('units','normalized','outerposition',[0 0 1 1])
plot(t,s(:,5)*kU,'b')
hold on
plot(t,s(:,4)*kL + bL*(-l/rp*(s(:,3)/Jwd - s(:,2)/Jy)+ gr*s(:,6)/Jm),'r')
i = ic.Data;
Pm_dot = dPm.Data;
taoFG = (T*i - bm*s(:,6)/Jm - Pm_dot)*gr;
plot(t,taoFG)
plot(t,s(:,6)/Jm*gr*bm,'g')
title('Torques from Lower Column, Upper Column, and EPS unit')
xlabel('Time [s]')
ylabel('Torque [N*m]')

figure('units','normalized','outerposition',[0 0 1 1])
plot(X,Y,'k--')
hold on
u0 = 100*1000/3600;
quiver(X(1:500:end),Y(1:500:end),u0*cos(psi(1:500:end)),u0*sin(psi(1:500:end)))

figure('units','normalized','outerposition',[0 0 1 1])
ratio = Fyf/Fyr;
plot(t,ratio)

% Torque
% figure(1)
% subplot(4,2,1)
% plot(Wsw)
% title('\omega_{sw}')
% subplot(4,2,2)
% plot(t,s(:,7)*180/pi)
% title('\delta')
% subplot(4,2,3)
% plot(t,s(:,2))
% title('p_y')
% subplot(4,2,4)
% plot(t,s(:,6))
% title('p_m')
% subplot(4,2,5)
% plot(t,s(:,1))
% title('p_v')
% subplot(4,2,6)
% plot(t,s(:,5)*180/pi)
% title('q_U')
% subplot(4,2,7)
% plot(Fv)
% title('F_v')
% subplot(4,2,8)
% plot(t,s(:,4))
% title('q_L')
% figure(2)
% plot(Fyf)
% hold on
% plot(Fyr,'r')
% title('front and rear forces')
% figure(3)
% plot(s(:,9),s(:,10))
% axis equal
% figure(4)
% plot(alat/9.81)
% title('Alateral/g')
% figure(5)
% plot(alphaf*pi/180)
% hold on
% plot(alphar*pi/180,'r')
% title('front and rear slip angle')
% figure(6)
% plot(ic)
% title('Current')

    
 

