clear; clc; close all;

carParams = Cars.FE12();

% % car parameter override
% carParas.m = 114514; % (random number)
% 
% % model override
% models.weightTransferFn = @SampoWeightTransferModel;
% 
% % extra configurations (not quite implemented)
% config.log     = true;
% config.maxIter = 500;

carParams.m = 280;
carParams.Cl = -0.13;
carParams.Cd = 0.78;
carParams.crossA = 0.62;
carParams.CoP = 7.65/100;
carParams.tire.CorrectionFactor = 0.7;

mmd = MMD.MMD(carParams);

% % Example use: generate and plot MMD
% grid.SA_CG  = -5:5;
% grid.dSteer = -15:15;
% grid.V      = 30;
% result = mmd.evaluate(grid, "free_rolling");
% MMD.plot.plot3DMMD(result)

% Example use: generate GGV
V = linspace(7, 50, 20);
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
