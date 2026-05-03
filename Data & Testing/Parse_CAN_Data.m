clear; clc; close all

function T = process_CAN_data(T)
    FDR = 2.91;   % final drive ratio
    tire_diameter_in = 16;      % [in] tire diameter
    tire_radius_m = (tire_diameter_in * 0.0254) / 2;    % [m]
    T.Wheel_Speed = -T.INV_Motor_Speed ./ FDR;
    T.Wheel_Omega = T.Wheel_Speed * 2*pi/60;              % [rad/s]
    T.Speed_mps   = T.Wheel_Omega * tire_radius_m;      % [m/s]
end

Data = readtable("FE13CAN_20260417_164027.csv");

Data = Data(~isnan(Data.INV_Motor_Speed), :);

start_time = datetime("20260417_164027", "InputFormat","uuuuMMdd_HHmmss");
Data.time = start_time + seconds(Data.Timestamp_s_);
Data.time.Format = 'yyyy-MM-dd HH:mm:ss.SS';

plot(Data.time, Data.INV_Motor_Speed)
