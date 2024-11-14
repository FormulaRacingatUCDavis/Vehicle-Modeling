vParam.m = 250;                    % Total Mass [kg]
vParam.PFront = 50 /100;           % Percent Mass Front [0-1]
vParam.WB = 1.5;                   % Wheelbase [m]
vParam.TWf = 1.2;                  % Trackwidth [m]
vParam.TWr = 1.2;
vParam.toe_f = 0.5 ;     % Toe Angles [radians] (positive is inwards)
vParam.toe_r = 0 ;
vParam.hCG = 0.2;                  % CG height [m]

% Tire Model Parameters
tParam.Idx = 1;                    % Moment of Inertia in x for wheel
tParam.TirePress = 70;          % kPa
tParam.TireIncl = 0;        % deg 
tParam.TireSR = 0;                 % -
tParam.Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
load('Hoosier_R25B_16x75-10x7.mat');
tParam.Tire = Tire;

R = linspace(10,100,200)';
SA_CG = linspace(-12,12,31);
dSteer = linspace(-30,30,31);
debug = 0;
saveCAY = zeros(length(R),1);
saveSA = zeros(length(R),1);
for i = 1:length(R)
    disp("For Radius: " + R(i))
    [saveCAY(i), saveSA(i)] = mmd_funct(vParam, tParam, SA_CG, dSteer, R(i), debug);
end

figure
hold on
grid()
plot(R, sqrt(saveCAY .* R .* sin(saveSA)))
title("Turning Radius and Vy")
ylabel("Vy (m/s)")
xlabel("Radius (m)")
hold off

figure
hold on
grid()
plot(R, saveCAY)
title("Turning Radius and C_{ay}")
ylabel("C_{ay}")
xlabel("Radius (m)")
hold off

save('MMD_DATA.mat', 'R', 'saveCAY', 'saveSA')

