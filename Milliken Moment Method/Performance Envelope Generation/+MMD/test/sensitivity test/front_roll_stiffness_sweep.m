clear; clc; close all;

fe13params = load("+Cars/fe13SampoParams").fe13params;
FE13 = Cars.FE13(fe13params);

ax = linspace(-1, 1, 10);
carParamSweep = repmat(FE13, size(ax));

% for i = 1:length(ax)
%     carParamSweep(i).kR = ax(i);
% end

%%

models.weightTransferFn = @MMD.models.SampoWeightTransfer;
grid.SA_CG = -12:0.05:1;
grid.dSteer = -20:20;
grid.V = 30;

stability = zeros(size(ax));
control = zeros(size(ax));
maxAy = zeros(size(ax));
SA = zeros(size(ax));
dS = zeros(size(ax));

for i = 1:length(ax)
    carParams = carParamSweep(i);
    mmd = MMD.MMD(carParams, models);
    result = mmd.evaluate(grid, "level_surface", ax(i));
    [stability(i), control(i)] = MMD.analysis.getStabilityNControl(result);
    MaxAyResult = MMD.analysis.getSteadyStateCAy(result);
    maxAy(i) = MaxAyResult.CAy;
    SA(i) = MaxAyResult.SA;
    dS(i) = MaxAyResult.dSteer;
    % MMD.plot.plotMMD(result)
end

%%

tiledlayout(2, 2)

nexttile
plot(ax, stability)
title("stability vs Ax")
xlabel("Ax (g)")
ylabel("stability (Nm/deg)")

nexttile
plot(ax, control)
title("control vs Ax")
xlabel("Ax (g)")
ylabel("control (Nm/deg)")

nexttile
plot(ax, SA)
title("SA under max SS conering vs Ax")
xlabel("Ax (g)")
ylabel("SA (deg)")

nexttile
plot(ax, dS)
title("steering under max SS conering vs Ax")
xlabel("Ax (g)")
ylabel("steer angle (deg)")

figure;
plot(ax, maxAy)
title("Ay vs Ax")
xlabel("Ax (g)")
ylabel("Ay (g)")