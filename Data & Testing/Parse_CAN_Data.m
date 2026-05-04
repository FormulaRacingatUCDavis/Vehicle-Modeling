clear; clc; close all

function T = process_CAN_data(T)
    FDR = 2.91;   % final drive ratio
    tire_diameter_in = 16;      % [in] tire diameter
    tire_radius_m = (tire_diameter_in * 0.0254) / 2;    % [m]
    T.Wheel_Speed = -T.INV_Motor_Speed ./ FDR;
    T.Wheel_Omega = T.Wheel_Speed * 2*pi/60;              % [rad/s]
    T.Speed_mps   = T.Wheel_Omega * tire_radius_m;      % [m/s]
end

Data = readtable("FE13CAN_20260417_165614-parsed.csv");

Data = Data(~isnan(Data.INV_Motor_Speed), :);

start_time = datetime("20260417_165614", "InputFormat","uuuuMMdd_HHmmss");
Data.time = start_time + seconds(Data.Timestamp_s_);
Data.time.Format = 'yyyy-MM-dd HH:mm:ss.SS';
Data = process_CAN_data(Data);

% smoothed_v = smoothdata(Data.Speed_mps, "movmean", 3);
smoothed_v = Data.Speed_mps;

plot(smoothed_v)

non_zero_speeds = Data.Speed_mps(Data.Speed_mps > 1);
disp("avg speed: " + mean(non_zero_speeds))
disp("max speed: "+ max(smoothed_v))
