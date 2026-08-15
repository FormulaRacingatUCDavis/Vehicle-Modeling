clear; close all; clc;
%%

% fe13params = load("+Cars/fe13params").fe13params;
% carParams = Cars.FE13(fe13params);

carParams = Cars.FE13();

carParams.tire.CorrectionFactorLat = 0.7;
carParams.tire.CorrectionFactorLong = 0.7;

% Spec B MB
carParams.Cl = 2.95; % @(yaw) 3.2868 - 0.0195.* yaw;
carParams.Cd = 1.27;
carParams.crossA = 0.913;
carParams.CoP = 54/100;

% models.weightTransferFn = @MMD.models.SampoWeightTransfer;

mmd = MMD.MMD(carParams);

n = 9;

all_hCG = [carParams.hCG, linspace(0.2, 0.5, n)];
[all_hCG, idx] = sort(all_hCG);
org_hcg_idx = idx(1);

all_Cl  = [carParams.Cl, linspace(3, 5, n)];
[all_Cl, idx] = sort(all_Cl);
org_cl_idx = idx(1);

n = n+1;
[all_hCG, idx] = sort(all_hCG);
all_SSAy = zeros(n);
all_stability = zeros(n);
all_control = zeros(n);

MMDgrid.SA_CG  = -8:0.5:8;
MMDgrid.dSteer = -20:20;
MMDgrid.V      = 20;
%%

tiledlayout(n, n)

for i=1:n
    for j=1:n
        fig = nexttile;
    
        carParams.hCG = all_hCG(i);
        carParams.Cl = all_Cl(j);
        mmd = MMD.MMD(carParams);
    
        result = mmd.evaluate(MMDgrid, "free_rolling");
        SSAy = MMD.analysis.getSteadyStateCAy(result);
        
        MMD.plot.plotMMD(result, SSAy, fig)
        [stability, control] = MMD.analysis.getStabilityNControl(result);
        
        stability = stability  / (carParams.WB * carParams.m * 9.81);
        control = control  / (carParams.WB * carParams.m * 9.81);
    
        all_SSAy(i, j) = SSAy.CAy;
        all_stability(i, j) = stability;
        all_control(i, j) = control;
    end
end

%% Baseline values
base_hCG       = all_hCG(org_hcg_idx);
base_Cl        = all_Cl(org_cl_idx);
base_Ay        = all_SSAy(org_hcg_idx, org_cl_idx);
base_stability = all_stability(org_hcg_idx, org_cl_idx);
base_control   = all_control(org_hcg_idx, org_cl_idx);

[all_Cl, all_hCG] = meshgrid(all_Cl, all_hCG);

%% Percentage changes
hCG_change = 100 .* (all_hCG - base_hCG) ./ base_hCG;

Ay_change = ...
    100 .* (all_SSAy - base_Ay) ./ base_Ay;

stability_change = ...
    100 .* (all_stability - base_stability) ./ base_stability;

control_change = ...
    100 .* (all_control - base_control) ./ base_control;

Cl_change = ...
    100 .* (all_Cl - base_Cl) ./ base_Cl;


%% Ay: original value
figure;
surf(all_Cl, all_hCG, all_SSAy)
xlabel("CL")
ylabel("CG Height (m)");
zlabel("Ay (g)");
title("Ay vs CG Height" + " @ " + MMDgrid.V + "m/s");

grid on;
hold on;
box on;



%% Ay: percentage change
figure;
surf(Cl_change, hCG_change, Ay_change);
xlabel("CL Change (%)")
ylabel("CG Height Change (%)");
zlabel("Ay Change (%)");
title("Ay Change vs CG Height Change" + " @ " + MMDgrid.V + "m/s");

grid on;
box on;


%% Stability: original value vs original CG height
figure;
surf(all_Cl, all_hCG, all_stability);

xlabel("CL")
ylabel("CG Height (m)");
zlabel("Stability");
title("Stability vs CG Height" + " @ " + MMDgrid.V + "m/s");

grid on;
box on;


%% Stability: percentage change vs CG height percentage change
figure;
surf(Cl_change, hCG_change, stability_change);

xlabel("CL Change (%)")
ylabel("CG Height Change (%)");
zlabel("Stability Change (%)");
title("Stability Change vs CG Height Change" + " @ " + MMDgrid.V + "m/s");

grid on;
box on;


%% Control: original value vs original CG height
figure;
surf(all_Cl, all_hCG, all_control);

xlabel("CL")
ylabel("CG Height (m)");
zlabel("Control");
title("Control vs CG Height" + " @ " + MMDgrid.V + "m/s");

grid on;
box on;


%% Control: percentage change vs CG height percentage change
figure;
surf(Cl_change, hCG_change, control_change);

xlabel("CL Change (%)")
ylabel("CG Height Change (%)");
zlabel("Control Change (%)");
title("Control Change vs CG Height Change" + " @ " + MMDgrid.V + "m/s");

grid on;
box on;