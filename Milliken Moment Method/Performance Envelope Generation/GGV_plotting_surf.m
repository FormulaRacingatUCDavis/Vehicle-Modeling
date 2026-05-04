clear; clc; close all;

% This shi is purely AI generated
% but it works well

%% Load .mat file
[file, path] = uigetfile('*.mat', 'Select GGV .mat file');
S = load(fullfile(path, file));

names = fieldnames(S);
P = [];

for i = 1:numel(names)
    X = S.(names{i});
    if isnumeric(X) && ismatrix(X) && size(X,2) >= 3
        P = X(:,1:3);
        fprintf("Using variable: %s\n", names{i});
        break;
    end
end

if isempty(P)
    error('No numeric matrix with at least 3 columns was found.');
end

Ax = P(:,1);
Ay = P(:,2);
V  = P(:,3);

valid = isfinite(Ax) & isfinite(Ay) & isfinite(V);
Ax = Ax(valid);
Ay = Ay(valid);
V  = V(valid);

%% Settings
nVBins = 35;        % number of velocity slices
nTheta = 181;      % angular resolution around each Ax-Ay slice
shrink2D = 0.85;   % 1 = convex boundary, smaller = tighter boundary
minPts = 8;

%% Bin points by velocity
vEdges = linspace(min(V), max(V), nVBins + 1);
vCenters = 0.5 * (vEdges(1:end-1) + vEdges(2:end));
vBin = discretize(V, vEdges);

thetaGrid = linspace(-pi, pi, nTheta).';

AxSurf = nan(nTheta, nVBins);
AySurf = nan(nTheta, nVBins);
VSurf  = nan(nTheta, nVBins);

%% Build one 2D boundary ring per velocity slice
for i = 1:nVBins
    idx = vBin == i;

    if sum(idx) < minPts
        continue;
    end

    x = Ax(idx);
    y = Ay(idx);

    % 2D boundary of this velocity slice
    try
        k = boundary(x, y, shrink2D);
    catch
        k = convhull(x, y);
    end

    xb = x(k);
    yb = y(k);

    % Remove duplicated closing point if present
    if hypot(xb(end) - xb(1), yb(end) - yb(1)) < 1e-10
        xb(end) = [];
        yb(end) = [];
    end

    % Center of this slice
    cx = mean(xb);
    cy = mean(yb);

    % Parameterize boundary by angle
    theta = atan2(yb - cy, xb - cx);

    [theta, order] = sort(theta);
    xb = xb(order);
    yb = yb(order);

    % Remove duplicate angle values
    [theta, ia] = unique(theta, 'stable');
    xb = xb(ia);
    yb = yb(ia);

    if numel(theta) < minPts
        continue;
    end

    % Periodic extension for interpolation across -pi/pi
    thetaExt = [theta - 2*pi; theta; theta + 2*pi];
    xbExt = [xb; xb; xb];
    ybExt = [yb; yb; yb];

    AxSurf(:,i) = interp1(thetaExt, xbExt, thetaGrid, 'linear');
    AySurf(:,i) = interp1(thetaExt, ybExt, thetaGrid, 'linear');
    VSurf(:,i)  = vCenters(i);
end

%% Remove empty velocity slices
keep = all(isfinite(AxSurf), 1);
AxSurf = AxSurf(:, keep);
AySurf = AySurf(:, keep);
VSurf  = VSurf(:, keep);

%% Close the surface around theta direction
AxSurf = [AxSurf; AxSurf(1,:)];
AySurf = [AySurf; AySurf(1,:)];
VSurf  = [VSurf;  VSurf(1,:)];

%% Plot hollow GGV envelope
figure;

surf(AxSurf, AySurf, VSurf, VSurf, ...
    'EdgeColor', 'none', ...
    'FaceColor', 'interp', ...
    'FaceAlpha', 0.95);

xlabel('A_x (g)');
ylabel('A_y (g)');
zlabel('V (m/s)');

title('GGV graph, FE13 Spec B Medium Balance');

grid on;
axis tight;
view(45, 25);

colormap turbo;
c = colorbar;
c.Label.String = 'Velocity';

camlight headlight;
lighting gouraud;
material dull;