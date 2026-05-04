clear; clc; close all

% load car
FE13 = Cars.FE13();

% no aero
carParams.Cl = -0.13;
carParams.Cd = 0.78;
carParams.CoP = 0.0765;
carParams.crossA = 0.62;

% Run data with different velocity
V = linspace(10, 30, 20);
mmd = MMD.MMD(FE13);
Ay = zeros(size(V));
dSteer = zeros(size(V));

grid.SA_CG = linspace(-5, 1, 6);
grid.dSteer = linspace(-20, 20, 30);

for i = 1:length(V)
    grid.V = V(i);
    result = mmd.evaluate(grid, "level_surface", 0);
    steadystate = MMD.analysis.getSteadyStateCAy(result, 0);
    Ay(i) = steadystate.CAy;
    dSteer(i) = steadystate.dSteer;
end

%%

[Ay, idx] = sort(Ay);
dSteer = dSteer(idx);

plot(Ay, dSteer)
xlabel("Ay (g)")
ylabel("Steering angle (deg)")