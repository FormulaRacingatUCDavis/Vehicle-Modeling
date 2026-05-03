clear; clc; close all;

% load data
% Data = readtable("FE13_BlueMax_Jonah_Warmup.csv");
% Data = Data(9e4:end, :);
Data = readtable("FE13_BlueMax_Jian_Warmup.csv");

[~, idx] = unique(Data.time);
Data = Data(idx, :);
%% Match the axsis
time = datetime(Data.time);
time_s = seconds(time - time(1));

% with switch on the top (Jonah)
% Ax = Data.AccX_g_';
% Ay = Data.AccY_g_';
% Az = Data.AccZ_g_';
% Acc = [Az; Ay; Ax];
% 
% function [Ax, Ay, Az] = transform_data(Acc, eul)
%     R = eul2rotm(eul);
%     Acc_transformed = R * Acc;
% 
%     Az = Acc_transformed(1, :)';
%     Ay = Acc_transformed(2, :)';
%     Ax = Acc_transformed(3, :)';
% end
% 
% function loss = loss(x, Acc, time_s)
%     [Ax, ~, ~] = transform_data(Acc, [0, pi/2 + x, 0]);
%     Vx = cumtrapz(time_s, Ax, 1);
%     loss = var(Vx);
% end
% 
% f = @(x) loss(x, Acc, time_s);
% delta_angle = fminbnd(f, -pi/4, pi/4);
% 
% [Ax, Ay, Az] = transform_data(Acc, [0, pi/2 + delta_angle, 0]);
% 
% yaw = Data.AngleX___;
% yawRate = Data.AsX___s_;


% with switch on the left (Jian)
Ax = Data.AccY_g_;
Ay = -Data.AccX_g_;
Az = Data.AccZ_g_;
yaw = Data.AngleZ___;
yawRate = Data.AsZ___s_;

Vx = cumtrapz(time_s, Ax);
X = Data.time;% cumtrapz(time_s, Vx);

%% test plot
hold on
plot(Data.time, Ax);
plot(Data.time, Ay);
plot(Data.time, Az);
yline(0)

xlabel("Time")
ylabel("Accel (g)")
legend('AccX', 'AccY', 'AccZ');
title('Raw accel data');

%% Smoothing
windowSize = 50; 
smoothAx = smoothdata(Ax, 'movmean', windowSize);
smoothAy = smoothdata(Ay, 'movmean', windowSize);
smoothAz = smoothdata(Az, 'movmean', windowSize);

figure;
hold on;
plot(X, smoothAx);
plot(X, smoothAy);

% xlabel("Time");
xlabel("X (m)")
ylabel("Smoothed Accel (g)");
legend('Smooth AccX', 'Smooth AccY');
title('Smoothed Accel Data');

%% GG diagram
figure;
scatter(Ay, Ax)

xlabel("Ay(g)")
ylabel("Ax(g)")

%% Smoothed GG diagram

figure;
scatter(smoothAy, smoothAx)

xlabel("Ay(g)")
ylabel("Ax(g)")

%% yaw

figure;
plot(X, yaw)

xlabel("X (m)")
ylabel("yaw (deg)")

%% yaw rate

figure;
plot(X, yawRate)

% xlabel("time")
xlabel("X (m)")
ylabel("yaw rate (deg/s)")

%%

figure;
plot(Data.time, Vx);

%%

save("FE13_BlueMax_Jian")