clear; clc; close all;

fe13params = load("+Cars/fe13SampoParams").fe13params;
FE13 = Cars.FE13(fe13params);

kR = linspace(200, 2000, 50);
carParamSweep = repmat(FE13, size(kR));

for i = 1:length(kR)
    carParamSweep(i).kR = kR(i);
end

V = linspace(7, 40, 2);
%%

models.weightTransferFn = @MMD.models.SampoWeightTransfer;
grid.SA_CG = -12:0.05:1;
grid.dSteer = -20:20;
grid.V = 30;

stability = zeros(size(kR));
control = zeros(size(kR));
maxAy = zeros(size(kR));
SA = zeros(size(kR));
dS = zeros(size(kR));

for i = 1:length(kR)
    carParams = carParamSweep(i);
    mmd = MMD.MMD(carParams, models);
    result = mmd.evaluate(grid, "free_rolling");
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
plot(kR, stability)
title("stability vs rear roll stiffness")
xlabel("rear roll stiffness (Nm/deg)")
ylabel("stability (Nm/deg)")

nexttile
plot(kR, control)
title("control vs rear roll stiffness")
xlabel("rear roll stiffness (Nm/deg)")
ylabel("control (Nm/deg)")

nexttile
plot(kR, SA)
title("SA under max SS conering vs rear roll stiffness")
xlabel("rear roll stiffness (Nm/deg)")
ylabel("SA (deg)")

nexttile
plot(kR, dS)
title("steering under max SS conering vs rear roll stiffness")
xlabel("rear roll stiffness (Nm/deg)")
ylabel("steer angle (deg)")