clear; clc; close all;

carParams = Cars.FE13();

carParams.tire.CorrectionFactorLat = 0.72;
carParams.tire.CorrectionFactorLong = 0.7;

% no aero
% carParams.Cl = -0.13;
% carParams.Cd = 0.78;
% carParams.CoP = 0.0765;
% carParams.crossA = 0.62;

% drs deactivated
carParams.Cl = 4.04;
carParams.Cd = 1.46;
carParams.crossA = 0.956;
carParams.CoP = 57.5/100;

mmd = MMD.MMD(carParams);
%% 
V = 14;
GGV_data = generate_GGV(mmd, V);

%%
close all

idx = GGV_data(:, 1) >= 0;
ggv_x_g_zero = GGV_data(idx, 1);
ggv_x_b_zero = GGV_data(~idx, 1);
ggv_y_g_zero = GGV_data(idx, 2);
ggv_y_b_zero = GGV_data(~idx, 2);

[ggv_y_g_zero, idx] = sort(ggv_y_g_zero);
ggv_x_g_zero = ggv_x_g_zero(idx);

[ggv_y_b_zero, idx] = sort(ggv_y_b_zero, "descend");
ggv_x_b_zero = ggv_x_b_zero(idx);



xlabel("Ay")
ylabel("AX")
zlabel("V")

xlim([-2, 2])
ylim([-2, 2])

load("FE13_BlueMax_Jonah.mat")
hold on

windowSize = 15;
scatter(smoothdata(Ay, 'movmean', windowSize), smoothdata(Ax, 'movmean', windowSize))
plot([ggv_y_g_zero; ggv_y_b_zero], [ggv_x_g_zero; ggv_x_b_zero])

xlabel("a_y (g)")
ylabel("a_x (g)")

legend(["MMD @ 14m/s", "track day data"])