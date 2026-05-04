clear; clc; close all;

carParams = Cars.FE12();

carParams.tire.CorrectionFactorLat = 0.7;
carParams.tire.CorrectionFactorLong = 0.7;

% no aero
% carParams.Cl = -0.13;
% carParams.Cd = 0.78;
% carParams.CoP = 0.0765;
% carParams.crossA = 0.62;

% drs deactivated
% carParams.Cl = 4.17;
% carParams.Cd = 1.37;
% carParams.crossA = 0.9594448;
% carParams.CoP = 57.5/100;

%HDF MB
carParams.Cl = @(yaw) 3.2868 - 0.0195.* yaw;
carParams.Cd = 1.463;
carParams.crossA = 0.99;
carParams.CoP = 53.36/100;

models.aeroModel = @MMD.models.aeroWithYaw;

mmd = MMD.MMD(carParams, models);

% Example use: generate and plot MMD
grid.SA_CG  = -8:0.5:8;
grid.dSteer = -20:20;
grid.V      = 15;
result1 = mmd.evaluate(grid, "drive", inf);
result2 = mmd.evaluate(grid, "level_surface", 0);
result3 = mmd.evaluate(grid, "brake", -inf);

fig = figure;
MMD.plot.plot3DMMD(result1, fig)
MMD.plot.plot3DMMD(result2, fig)
MMD.plot.plot3DMMD(result3, fig)

% SS = MMD.analysis.getSteadyStateCAy(result);
% MMD.plot.plotMMD(result, SS)
% MMD.plot.plot3DMMD(result)

% Example use: generate GGV
%% 
% V = 14;
% GGV_data = generate_GGV(mmd, V);
% 
% %%
% 
% idx = GGV_data(:, 1) >= 0;
% ggv_x_g_zero = GGV_data(idx, 1);
% ggv_x_b_zero = GGV_data(~idx, 1);
% ggv_y_g_zero = GGV_data(idx, 2);
% ggv_y_b_zero = GGV_data(~idx, 2);
% 
% [ggv_y_g_zero, idx] = sort(ggv_y_g_zero);
% ggv_x_g_zero = ggv_x_g_zero(idx);
% 
% [ggv_y_b_zero, idx] = sort(ggv_y_b_zero, "descend");
% ggv_x_b_zero = ggv_x_b_zero(idx);
% 
% 
% plot([ggv_y_g_zero; ggv_y_b_zero], [ggv_x_g_zero; ggv_x_b_zero])
% 
% 
% xlabel("Ay")
% ylabel("AX")
% zlabel("V")
% 
% xlim([-2, 2])
% ylim([-2, 2])
