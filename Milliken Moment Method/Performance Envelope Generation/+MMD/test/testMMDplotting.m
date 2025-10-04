clear;

grid.SA_CG = -5:1;
grid.dSteer = -15:15;
grid.V = 30;

result = MMD.core.solve(grid, Cars.FE12(), "brake", -inf);



%%
close all;
maxFy = MMD.analysis.getSteadyStateCAy(result, 0);

MMD.plot.plotMMD(result, maxFy)
% fig = MMD.plot.plot3DMMD(result)
% MMD.plot.plot3DMMD(result2, fig)
