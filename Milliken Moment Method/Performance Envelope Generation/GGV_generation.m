clear; clc; close all;

carParams = Cars.FE13();

% % car parameter override
% carParas.m = 114514; % (random number)
% 
% % model override
% models.weightTransferFn = @SampoWeightTransferModel;
% 
% % extra configurations (not quite implemented)
% config.log     = true;
% config.maxIter = 500;

% HDF HB
carParams.m = carParams.m + (42-28)*0.453592;
carParams.Cl = @(yaw) 3.42663 - 0.02214 .* yaw;
% carParams.Cl = 4.200;
carParams.Cd = 1.446;
carParams.crossA = 0.99;
carParams.CoP = 57.93;

models.aeroModel = @MMD.models.aeroWithYaw;

mmd = MMD.MMD(carParams, models);


% % Example use: generate and plot MMD
% grid.SA_CG  = -5:5;
% grid.dSteer = -15:15;
% grid.V      = 30;
% result = mmd.evaluate(grid, "free_rolling");
% MMD.plot.plot3DMMD(result)

% Example use: generate GGV
V = linspace(9, 40, 20);
%% 
GGV_data = generate_GGV(mmd, V);

%%
scatter3(GGV_data(:, 1), GGV_data(:, 2), GGV_data(:, 3))

xlabel("Ax")
ylabel("Ay")
zlabel("V")

xlim([-5, 5])
ylim([-5, 5])

%%
[filename, pathname] = uiputfile('*.mat', 'save data');
if ~(isequal(filename,0) || isequal(pathname,0))
    save(fullfile(pathname, filename), "GGV_data", "carParams");
    disp(['GGV data saved' fullfile(pathname, filename)]);
end
