clear; clc; close all

% load car
carParams = Cars.FE13();

% no aero
% carParams.Cl = -0.13;
% carParams.Cd = 0.78;
% carParams.CoP = 0.0765;
% carParams.crossA = 0.62;

% HDF MB
% carParams.Cl = @(yaw) 3.42663 - 0.02214 .* yaw;
% carParams.Cd = 1.446;
% carParams.crossA = 0.99;
% carParams.CoP = 57.94/100;

% Run data with different velocity
V = linspace(15, 30, 10);
models = struct();
% models.aeroModel = @MMD.models.aeroWithYaw;
mmd = MMD.MMD(carParams, models);
Ay = zeros(size(V));
dSteer = zeros(size(V));

grid.SA_CG = linspace(-8, 2, 10);
grid.dSteer = linspace(-20, 20, 20);

for i = 1:length(V)
    grid.V = V(i);
    result = mmd.evaluate(grid, "level_surface", 0);
    steadystate = MMD.analysis.getSteadyStateCAy(result, 0);
    Ay(i) = steadystate.CAy;
    dSteer(i) = steadystate.dSteer;
end

%%

% [Ay, idx] = sort(Ay);
% dSteer = dSteer(idx);

plot(Ay, dSteer)
xlabel("Ay (g)")
ylabel("Steering angle (deg)")