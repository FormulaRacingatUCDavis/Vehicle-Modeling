clear;

% FE12 constants
% Chasis/suspension constants
carParams.m = 270;                        % Total Mass [kg]
carParams.PFront = 53.4/100;              % Percent Mass Front [0-1]
carParams.WB = 1.582;                     % Wheelbase [m]
carParams.TWf = 1.240;                    % Trackwidth [m]
carParams.TWr = 1.240;
carParams.toe_f = -0.5 * (pi/180);        % Toe Angles [radians] (positive is inwards)
carParams.toe_r = 0.5 * (pi/180);
carParams.hCG = 0.314;                    % CG height [m]
carParams.TireInclinationFront = -1.3; % deg 
carParams.TireInclinationRear = -1;    % deg   

% Aero constants
carParams.Cl = 3.215;
carParams.Cd = 1.468; 
carParams.CoP = 45/100;                   % front downforce distribution (%)
carParams.rho = 1.165;                    % kg/m^3
carParams.crossA = 0.9237;                % m^2

% braking system
% THIS IS MADE UP, NEED TO GET THE ACTUAL VALUE
carParams.B_FBB = 55/45;                    % Front brake bias

% Tire
tire = load('Hoosier_R20_16(18)x75(60)-10x8(7).mat');
tire.Idx = 1;                     % Moment of Inertia in x for wheel
tire.TirePressure = 70;           % kPa
tire.Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
carParams.tire = tire;

velocity = linspace(20, 55, 8);

dataPoints = [];

rangeSA = [-5,5];
rangeSteer = [-30,30];

stepSize = 0.1;

%%
parfor i = 1:length(velocity)
    targetCAx = 0;
    [SSAy, ~, ~] = MMD_Generation(carParams, rangeSA, rangeSteer, velocity(i), false, targetCAx)
    while SSAy.CAy ~= 0
        disp("targetCAx: " + targetCAx + " velocity: " + velocity(i))
        point = [velocity(i); SSAy.CAy; targetCAx];
        dataPoints = [dataPoints point];
        [SSAy, ~, ~] = MMD_Generation(carParams, rangeSA, rangeSteer, velocity(i), false, targetCAx)
        targetCAx = targetCAx + stepSize;
    end

    targetCAx = -stepSize;
    [SSAy, ~, ~] = MMD_Generation(carParams, rangeSA, rangeSteer, velocity(i), false, targetCAx)
    while SSAy.CAy ~= 0
        disp("targetCAx: " + targetCAx + " velocity: " + velocity(i))
        point = [velocity(i); SSAy.CAy; targetCAx];
        dataPoints = [dataPoints point];
        [SSAy, ~, ~] = MMD_Generation(carParams, rangeSA, rangeSteer, velocity(i), false, targetCAx)
        targetCAx = targetCAx - stepSize;
    end
end

%% Mirror data to the left side

mask = dataPoints(1, :) == 10;
dataPoints = [dataPoints dataPoints .* [1; -1; 1]];
base = dataPoints(:, mask);
for i = 1:length(base)
    dataPoints = [dataPoints, [ones(1, 10) .* base(1, i); linspace(-base(2, i), base(2, i), 10); ones(1, 10) .* base(3, i)]];
end

%%

x = dataPoints(3, :);
y = dataPoints(2, :);
z = dataPoints(1, :);

xrange = linspace(min(x), max(x), 100);
yrange = linspace(min(y), max(y), 100);
[xq, yq] = meshgrid(xrange, yrange);

F = scatteredInterpolant(x', y', z', 'natural');
zq = F(xq, yq);

figure;

hold on
surf(xq, yq, zq)
% scatter3(x, y, z)
view(3)
xlim([-4, 4])
ylim([-4, 4])
zlim([10, 60])

xlabel("Normalized Longitudinal Accelaration(g)")
ylabel("Normalized Lateral Accelaration(g)")
zlabel("Velocity(m/s)")

%%

mask = dataPoints(3, :) < 0;

x = dataPoints(1, mask);
y = dataPoints(2, mask);
z = dataPoints(3, mask);

xrange = linspace(10, 60 , 100);
yrange = linspace(min(y), max(y), 100);
[xq, yq] = meshgrid(xrange, yrange);

F = scatteredInterpolant(x', y', z', "linear", "nearest");
zq = F(xq, yq);

figure;

hold on
surf(xq, yq, zq)
scatter3(x, y, z)

xlabel("Velocity(m/s)")
ylabel("Normalized Lateral Accelaration(g)")
zlabel("Normalized Longitudinal Accelaration(g)")