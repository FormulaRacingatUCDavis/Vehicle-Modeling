clc; clear; close all
format short g
%% Evaluation

vParam.m = 280;                    % Total Mass [kg]
vParam.PFront = 51.1 /100;           % Percent Mass Front [0-1]
vParam.WB = 1.595;                   % Wheelbase [m]
vParam.TWf = 1.193;                  % Trackwidth [m]
vParam.TWr = 1.193;
vParam.toe_f = 0.5 ;     % Toe Angles [degrees] (positive is inwards)
vParam.toe_r = 0 ;
vParam.hCG = 0.25346;                  % CG height [m]

% Tire Model Parameters
tParam.Idx = 1;                    % Moment of Inertia in x for wheel
tParam.TirePress = 70;          % kPa
tParam.TireIncl = 1;        % deg 
tParam.TireSR = 0;                 % -
tParam.Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
load('Hoosier_43075_16x75-10_R20-8.mat');
tParam.Tire = Tire;

R = [5:5:120]';
SA_CG = linspace(-12,12,31);
dSteer = linspace(-30,30,31);
debug = 0;
saveCAY = zeros(length(R),1);
saveSA = zeros(length(R),1);
matCAY = zeros(length(dSteer), length(SA_CG), length(R));
matMZ = zeros(length(dSteer), length(SA_CG), length(R));
for i = 1:length(R)
    disp("For Radius: " + R(i))
    [saveCAY(i), saveSA(i),matCAY(:,:,i), matMZ(:,:,i)] = mmd_funct(vParam, tParam, SA_CG, dSteer, R(i), debug);
end
%%
saveAY = saveCAY .* 9.81;

%% Plotting
figure
hold on
grid()
plot(R.*3.28084, 3.28084.*sqrt(saveAY .* R .* sin(saveSA)))
title("Turning Radius and Vy")
ylabel("Vx (ft/s)")
xlabel("Radius (ft)")
hold off

figure
hold on
grid()
plot(R.*3.28084, saveAY./9.81)
title("Turning Radius and C_{ay}")
ylabel("C_{ay}")
xlabel("Radius (ft)")
hold off


save('MMD_DATA(R-5-5-120).mat', 'R', 'saveAY', 'saveSA')

