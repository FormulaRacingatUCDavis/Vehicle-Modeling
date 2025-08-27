% clear;

load("OpenTRACK_Paul Ricard_Closed_Forward.mat")
load("GGV_Data.mat")

%%
% v_samples = linspace(5, 70, 20);
% Fy_samples = zeros(size(v_samples));
% parfor i  = 1:length(v_samples)
%     disp("calculating v = " + v_samples(i));
%     Fy_samples(i) = calcFy(i, carParams);
%     disp("Fy(" + v_samples(i) + ") = " + Fy_samples(i));
% end
mask = dataPoints(3, :) == 0 & dataPoints(2, :) > 0;

v_samples = dataPoints(1, mask);
Fy_samples = dataPoints(2, mask) .* 9.81 .* carParams.m;

%%



pp_Fy = pchip(v_samples, Fy_samples);

max_speed_trace = zeros(size(r));
parfor i = 1:length(r)
    max_speed_trace(i) = interp_GV(carParams, r(i), pp_Fy);
end

% max_speed_trace = smoothdata(max_speed_trace, 'movmean', 50);
[v_apex,apex] = findpeaks(-max_speed_trace,...
                          'MinPeakProminence', 3);
v_apex = -v_apex;

figure;
hold on;
grid on;

plot(max_speed_trace)
% x = 1:length(idx);
scatter(apex, v_apex)


%%

function v = interp_GV(carParams, r, pp_Fy)
     % speed solution
     if false%abs(r) < 1e-6 % At strait
         v = inf;
     else % Cornering
         r = abs(r);
         f = @(v) (v < 5)*1e6 + ppval(pp_Fy, v) - carParams.m*r*v.^2;
         disp("r=" + r)
         opts = optimset('Display', 'iter', 'FunValCheck', 'on');
         v = fzero(f, 50);
     end
end

function Fy = calcFy(v, carParams)
    rangeSA = [-5,5];
    rangeSteer = [-15,15];
    [SSAy, ~, ~] = MMD_Generation(carParams, rangeSA, rangeSteer, v, false, 0);
    Fy = SSAy.CAy * carParams.m * 9.81;
end