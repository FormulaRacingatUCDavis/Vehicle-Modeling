clear; close all;

models = struct();
models.weightTransferFn = @MMD.models.SampoWeightTransfer;

load("+Cars/fe13SampoParams");
FE13 = Cars.FE13(fe13params);
FE13.m_s = FE13.m_s + (FE13.m - fe13params.m);
% FE13.kF = 99999;

mmd = MMD.MMD(FE13, models);

grid.SA_CG = -8:8;
grid.dSteer = -20:20;
grid.V = 30;

result = mmd.evaluate(grid, "free_rolling");

MMD.plot.plotMMD(result);

[Stability, Control] = MMD.analysis.getStabilityNControl(result)

Stability = Stability
Control   = Control
