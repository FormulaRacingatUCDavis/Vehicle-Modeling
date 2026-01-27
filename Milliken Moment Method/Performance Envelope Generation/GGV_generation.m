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

carParams.m = carParams.m + (42-28)*0.453592;
%HDF HB
% carParams.Cl = @(yaw) 3.42663 - 0.02214 .* yaw;
% carParams.Cd = 1.446;
% carParams.crossA = 0.99;
% carParams.CoP = ?/100;

%HDF MB
% carParams.Cl = @(yaw) 3.2868 - 0.0195.* yaw;
% carParams.Cd = 1.463;
% carParams.crossA = 0.99;
% carParams.CoP = ?/100;

%HDF LB
% carParams.Cl = @(yaw) 3.1363 -0.0251 .* yaw;
% carParams.Cd = 1.428;
% carParams.crossA = 0.99;
% carParams.CoP = ?/100;

%LDF HB
% carParams.Cl = @(yaw) 3.4095 -0.0425 .* yaw;
% carParams.Cd = 1.31;
% carParams.crossA = ?;
% carParams.CoP = 63.79/100;

%LDF MB
% carParams.Cl = @(yaw) 3.0931 -0.0316 .* yaw;
% carParams.Cd = 1.27;
% carParams.crossA = ?;
% carParams.CoP = 54.31/100;

%LDF LB
% carParams.Cl = @(yaw) 2.9822 -0.0405 .* yaw;
% carParams.Cd = 1.22;
% carParams.crossA = ?;
% carParams.CoP = 45.46/100;

models.aeroModel = @MMD.models.aeroWithYaw;

mmd = MMD.MMD(carParams, models);


% % Example use: generate and plot MMD
% grid.SA_CG  = -5:5;
% grid.dSteer = -15:15;
% grid.V      = 30;
% result = mmd.evaluate(grid, "free_rolling");
% MMD.plot.plot3DMMD(result)

% Example use: generate GGV
V = linspace(10, 40, 20);
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
