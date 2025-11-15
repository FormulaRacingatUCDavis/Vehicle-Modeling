clear;

grid.SA_CG = linspace(-8, 1, 10);
grid.dSteer = linspace(-20, 20, 20);
grid.V = 30;

load("+Cars/fe13SampoParams");
FE13 = Cars.FE13(fe13params);
FE13.m_s = FE13.m_s + (FE13.m - fe13params.m);

result = MMD.core.solve(grid, FE13, "brake", -inf);

%%
close all;
maxFy = MMD.analysis.getSteadyStateCAy(result, -1.8);

MMD.plot.plotMMD(result, maxFy)
MMD.plot.plot3DMMD(result)
