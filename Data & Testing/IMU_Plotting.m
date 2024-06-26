clc; clear; close all;

data = readmatrix('004_endurance_parsed.csv');

rows = data(:, 1) == 101;

% Extract the corresponding values from column 10
filteredaccelData = data(rows, [2,4,6,10]);
xData = filteredaccelData(:,1);
trueyData = filteredaccelData(:,2);
trueyData = smoothdata(trueyData) ;
zData = filteredaccelData(:,3);
truezData = zData.*sind(53)-xData.*sind(37);
truezData = smoothdata(truezData);
truexData = zData.*cosd(53)+xData.*cosd(37);
truexData = smoothdata(truexData);
TimeData = filteredaccelData(:,4);

velxData = cumtrapz(TimeData,truexData);
velyData = cumtrapz(TimeData,trueyData);
velzData = cumtrapz(TimeData,truezData);

totalVel = sqrt(velxData.^2+velyData.^2);
totalVel = smoothdata(totalVel);
maxVel = max(totalVel);

figure(1)
%Smoothed_Data_Real = smoothdata(Real_Readings,"gaussian");
%Conversion = (Real_Readings-P_1(2))./P_1(1);
%g_force = ((Smoothed_Conversion+120).*32.174)./(Smoothed_Conversion.*32.174);
subplot(2,2,1)
hold on
plot(TimeData,truexData)
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,2)
hold on
plot(TimeData,trueyData)
%plot(Time,Conversion)
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,3)
plot(TimeData,truezData);
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,4)
plot(truexData,trueyData);

rows = data(:, 1) == 100;

% Extract the corresponding values from column 10
filteredaccelData = data(rows, [2,4,6,10]);
ax = filteredaccelData(:,1);
ay = filteredaccelData(:,2);
az = filteredaccelData(:,3);
TimeData_Ang = filteredaccelData(:,4);

% Rotation angle around y-axis in degrees
rotation_angle = 53; % 57 degrees counterclockwise

% Convert rotation angle to radians
rotation_angle_rad = deg2rad(rotation_angle);

% Construct rotation matrix for rotation around y-axis
Ry = [cos(rotation_angle_rad), 0, sin(rotation_angle_rad);
      0, 1, 0;
      -sin(rotation_angle_rad), 0, cos(rotation_angle_rad)];
n = length(ax);
% Transform IMU angular acceleration to system axes
for i = 1:n
    imu_angular_acceleration = [ax(i); ay(i); az(i)];
    system_angular_acceleration(:,i) = Ry * imu_angular_acceleration;
end

figure(2)
%Smoothed_Data_Real = smoothdata(Real_Readings,"gaussian");
%Conversion = (Real_Readings-P_1(2))./P_1(1);
%g_force = ((Smoothed_Conversion+120).*32.174)./(Smoothed_Conversion.*32.174);
subplot(2,2,1)
hold on
plot(TimeData_Ang,system_angular_acceleration(1,:))
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,2)
hold on
plot(TimeData_Ang,system_angular_acceleration(2,:))
%plot(Time,Conversion)
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')
subplot(2,2,3)
plot(TimeData_Ang,system_angular_acceleration(3,:));
title('Time vs Signal')
ylabel('Signal')
xlabel('Time')

figure(3)
subplot(2,2,1)
plot(TimeData,totalVel)
subplot(2,2,2)
plot(TimeData,velxData)
subplot(2,2,3)
plot(TimeData,velyData)