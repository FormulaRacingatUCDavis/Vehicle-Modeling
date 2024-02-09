%% Animation for bicycle car
% Date: 02/28/2017
% Author: Ehsan Arasteh
% Hyundai Center of Excellence in Vehicle Dynamic Systems & Control
% Detail:
% This code gets the trajectory of a vehicle and the tire forces and plots
% the animation of the vehicle trajectory with tire trails

%%
clear;clc;close all;

master %Runs the bicycle model

% Front tire force
F_f = Cf*int(:,1);
% Rear tire force
% F_r = Cr*int(:,2);

%Normalized tire forces (for the sake of visualization)
F_f_normalized = (F_f-min(F_f))/(max(F_f)-min(F_f));
% F_r_normalized = (F_r-min(F_r))/(max(F_r)-min(F_r));

% Body corner coordinates
body = [-1, 1, 1, -1;-1, -1, 1, 1];
% Length and Width of the car
L = 2.5;
W = 1;
% Stretching the car
body(1,:) = body(1,:)*L;
body(2,:) = body(2,:)*W;

% Tire corners
tire = [-1, 1, 1, -1;-1, -1, 1, 1];
% Stretching the tire
tire(1,:) = 0.5*tire(1,:);
tire(2,:) = 0.25*tire(2,:);

% New figure
figure(2);
hold on
% Filling in the colors
h1_body = fill(body(1,:),body(2,:),'r'); % Body
h1_tire_rr = fill(tire(1,:),tire(2,:),'b'); % Tire Rear Right
h1_tire_lr = fill(tire(1,:),tire(2,:),'b'); % Tire Rear Left
h1_tire_lf = fill(tire(1,:),tire(2,:),'b'); % Tire Front Left
h1_tire_rf = fill(tire(1,:),tire(2,:),'b'); % Tire Front Right

X = x(:,4); Y = x(:,5); Psi = x(:,3);

axis equal % Equal distance axis
axis([-5 max(X)+10 -5 max(Y)+10]) % bounding X and Y axis

%Defining the animated lines for each tire
rr_curve = animatedline;
lr_curve = animatedline;
lf_curve = animatedline;
rf_curve = animatedline;

N = numel(X); % size of X matrix will be used for the number of loops

%%Saving into .avi stuff
ax = gca;
ax.NextPlot = 'replaceChildren';
F(N) = struct('cdata',[],'colormap',[]);
v = VideoWriter('cars2.avi');
open(v);

for i = 1:N
    %%%BODY
    %Rotation
    XY_rot = [cos(Psi(i)),-sin(Psi(i));sin(Psi(i)),cos(Psi(i))]*[body(1,:);body(2,:)];  %Rotation Matrix * [X;Y]
    %Translation
    X_body = XY_rot(1,:) + X(i);
    Y_body = XY_rot(2,:) + Y(i);
    %setting the object to the new position
    set(h1_body,'XData',X_body,'YData',Y_body);
    
    %%%TIRE: Rear Right
    XY_rot = [cos(Psi(i)),-sin(Psi(i));sin(Psi(i)),cos(Psi(i))]*[tire(1,:);tire(2,:)];  %Rotation Matrix * [X;Y]
    %Translation
    X_tire_rr = XY_rot(1,:) + X_body(1);
    Y_tire_rr = XY_rot(2,:) + Y_body(1);
    set(h1_tire_rr,'XData',X_tire_rr,'YData',Y_tire_rr);
    % Adding the new point to the tire trail
    addpoints(rr_curve,X_body(1),Y_body(1));
    
    %%%TIRE: Rear Left
    XY_rot = [cos(Psi(i)),-sin(Psi(i));sin(Psi(i)),cos(Psi(i))]*[tire(1,:);tire(2,:)];  %Rotation Matrix * [X;Y]
    %Translation
    X_tire_lr = XY_rot(1,:) + X_body(4);
    Y_tire_lr = XY_rot(2,:) + Y_body(4);
    set(h1_tire_lr,'XData',X_tire_lr,'YData',Y_tire_lr);
    addpoints(lr_curve,X_body(4),Y_body(4));
    
    %%%TIRE: Front Left
    XY_rot = [cos(Psi(i)+delta(i)),-sin(Psi(i)+delta(i));sin(Psi(i)+delta(i)),cos(Psi(i)+delta(i))]*[tire(1,:);tire(2,:)];  %Rotation Matrix * [X;Y]
    %Translation
    X_tire_lf = XY_rot(1,:) + X_body(3);
    Y_tire_lf = XY_rot(2,:) + Y_body(3);
    set(h1_tire_lf,'XData',X_tire_lf,'YData',Y_tire_lf);
    % Setting the color based on the normalized force
    set(h1_tire_lf,'FaceColor',[0 0 1-F_f_normalized(i)]);
    addpoints(lf_curve,X_body(3),Y_body(3));
    
    %%%TIRE: Front Right
    XY_rot = [cos(Psi(i)+delta(i)),-sin(Psi(i)+delta(i));sin(Psi(i)+delta(i)),cos(Psi(i)+delta(i))]*[tire(1,:);tire(2,:)];  %Rotation Matrix * [X;Y]
    %Translation
    X_tire_rf = XY_rot(1,:) + X_body(2);
    Y_tire_rf = XY_rot(2,:) + Y_body(2);
    set(h1_tire_rf,'XData',X_tire_rf,'YData',Y_tire_rf);
    set(h1_tire_rf,'FaceColor',[0 0 1-F_f_normalized(i)]);
    addpoints(rf_curve,X_body(2),Y_body(2));
    % Position of X_CG and Y_CG
    X_CG = (X_body(1)+X_body(2))/2;
    Y_CG = (Y_body(1)+Y_body(2))/2;
    % For Car view uncomment this
%     axis([X_CG-10 X_CG+10 Y_CG-5 Y_CG+5])
    
    hold on;
    pause(0.05)
    F(i) = getframe();
    % Movie stuff
    writeVideo(v,F(i));
end

% Movie stuff
close(v);

% show for multiple times
movie(F,3)
