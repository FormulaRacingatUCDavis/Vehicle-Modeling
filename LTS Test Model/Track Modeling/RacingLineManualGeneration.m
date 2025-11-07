clc; clear; close all

%% Initialization
f1 = figure;
j = 1;
imageFile = 'Screenshot_20250909-185228_7.png';
image = imread(imageFile);
imshow(imageFile);
hold on;

%% Active Plotting

rawX = zeros(10000,1);
rawY = zeros(10000,1);
scaleX = zeros(2,1);
scaleY = zeros(2,1);

%%% Calculates scaling needed (add points at the scale points)
while true
    title("Select Scaling Points (Limit: 2)", FontSize = 15)
    try 
        if j == 3
           disp("------------------------------------------------------")
           scaleDist = input("Distance between marked points (unit) = ");
           disp("------------------------------------------------------")
           break
        end
        % ginput(number) throws error when 'ENTER' is pressed
        [scaleX(j), scaleY(j)] = ginput(1);
        plot(scaleX(j), scaleY(j),'ro','MarkerFaceColor','r','MarkerSize',3)
        
        if j == 2
            plot(scaleX, scaleY,'LineWidth',2,'color','red');
        end

        j=j+1;

    catch % Executes the break if error happens in try()
        break;
    end
    
    % In case something other than enter is pressed
    if f1.CurrentCharacter > 0
        break;
    end
end
totPixel = abs(diff(scaleX));
scalePixel = scaleDist/totPixel;

if isempty(scaleDist) == 1
    error("Scaling process error occured, retry scaling");
else
    disp("-----------------------------------------")
    disp("Scaling Factor = " + scalePixel + " unit/pixel")
    disp("-----------------------------------------")
end

%%% Plotting Racing Line
close all;
f1 = figure;
imshow(imageFile);
hold on;
j = 1;

while true
    title("Select Racing Path Points (Limit = 10000) ENTER when done", FontSize = 15)
    try 
        % ginput(number) throws error when 'ENTER' is pressed
        [rawX(j), rawY(j)] = ginput(1);
        plot(rawX(j),rawY(j),'ro','MarkerFaceColor','r','MarkerSize',3)

        if j > 1
            plot(rawX(1:j),rawY(1:j),'LineWidth',2,'color','red');
        end

        j=j+1;

    catch
        disp("Plotting ended, total number of points = " + nnz(rawX));
        
        break;
    end

    % In case something other than enter is pressed
    if f1.CurrentCharacter > 0
        break;
    end

end
rawX(j) = rawX(1);
rawY(j) = rawY(1);

%% Plotting

numPlotted = nnz(rawX);
scaledX = rawX(1:numPlotted) .* scalePixel;
scaledY = rawY(1:numPlotted) .* scalePixel;
[pixelLimY, pixelLimX, ~] = size(image);
distLimX = pixelLimX .* scalePixel;
distLimY = pixelLimY .* scalePixel;

totDist = sum(sqrt(diff(scaledX).^2 + diff(scaledY).^2));

disp("------------------------------------------------------")
disp("Total Distance Tracked = " + totDist + " unit distance")
disp("------------------------------------------------------")

figure;
hold on;
grid on;
plot(scaledX, scaledY, 'ro-', LineWidth = 2);
title("Plotted Path (Not Processed)")
xlabel("X Coordinate (distance unit)")
ylabel("Y Coordinate (distance unit)")
xlim([0, distLimX])
ylim([0, distLimY])


%% Scaling and Interpolation
% Flipping BlueMax Around bc its backwards
scaledY = -scaledY;
scaledY = scaledY + max(abs(scaledY));

scaledCoords = [scaledX, scaledY];
numPlottedMat = 1:numPlotted;
extraPoints = linspace(1, length(scaledX), length(scaledCoords)*10)';

ppFunc = pchip(numPlottedMat, scaledCoords');
extraCoords = ppval(ppFunc, extraPoints);
extraCoords = extraCoords';

figure
hold on
grid on
plot(extraCoords(:,1), extraCoords(:,2), 'o-')
plot(scaledX, scaledY, LineWidth = 2)
title("Track Plotting Comparison")
legend(["Extended", "Original"])
xlabel("X Coordinate (distance unit)")
ylabel("Y Coordinate (distance unit)")
axis equal

%% Finding Curvature and Vectors

[lengthMat, radiusMat, vectorMat] = curvature(extraCoords);

figure;
grid on
hold on
axis equal
plot(extraCoords(:,1), extraCoords(:,2), '.');
quiver(extraCoords(:,1), extraCoords(:,2),vectorMat(:,1),vectorMat(:,2));

TrackInfo.coords = extraCoords;
TrackInfo.r = radiusMat;
TrackInfo.x = lengthMat;
TrackInfo.dx = diff(lengthMat);
TrackInfo.n = length(radiusMat);

%% Saving Track Information
SaveTrack = questdlg( 'Save Track?', '', 'Yes', 'No', 'No' );

if strcmpi( SaveTrack, 'Yes' )
    Directory.Tool = fileparts( matlab.desktop.editor.getActiveFilename );
    Directory.Name = [Directory.Tool(1:max(strfind( Directory.Tool,'\' ))),...
        '\Track Modeling\Tracks'];
    addpath( genpath( Directory.Tool      ) );
    addpath( genpath( Directory.Name      ) );

    trackName = inputdlg( {'Enter Track File Name'} );
    if isempty(trackName)
        error( 'Track file naming cancelled, track is not saved' )
    else
        save([Directory.Name , '\', char(trackName), '.mat'], 'TrackInfo')
    end
    
    disp([char(trackName), '.mat saved'])
end


%% Functions

%%% Curvature and Circumcenter from MATLAB exchange
%%% Link: https://www.mathworks.com/matlabcentral/fileexchange/69452-curvature-of-a-1d-curve-in-a-2d-or-3d-space

function [L,R,k] = curvature(X)
% Radius of curvature and curvature vector for 2D or 3D curve
%  [L,R,k] = curvature(X)
%   X:   2 or 3 column array of x, y (and possibly z) coordiates
%   L:   Cumulative arc length
%   R:   Radius of curvature
%   k:   Curvature vector
% The scalar curvature value is 1./R
% Version 2.6: Calculates end point values for closed curve
  N = size(X,1);
  dims = size(X,2);
  if dims == 2
    X = [X,zeros(N,1)];  % Use 3D expressions for 2D as well
  end
  L = zeros(N,1);
  R = NaN(N,1);
  k = NaN(N,3);
  for i = 2:N-1
    [R(i),~,k(i,:)] = circumcenter(X(i,:)',X(i-1,:)',X(i+1,:)');
    L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  end
  if norm(X(1,:)-X(end,:)) < 1e-10 % Closed curve. 
    [R(1),~,k(1,:)] = circumcenter(X(end-1,:)',X(1,:)',X(2,:)');
    R(end) = R(1);
    k(end,:) = k(1,:);
    L(end) = L(end-1) + norm(X(end,:)-X(end-1,:));
  end
  i = N;
  L(i) = L(i-1)+norm(X(i,:)-X(i-1,:));
  if dims == 2
    k = k(:,1:2);
  end
end

function [R,M,k] = circumcenter(A,B,C)
% Center and radius of the circumscribed circle for the triangle ABC
%  A,B,C  3D coordinate vectors for the triangle corners
%  R      Radius
%  M      3D coordinate vector for the center
%  k      Vector of length 1/R in the direction from A towards M
%         (Curvature vector)
  D = cross(B-A,C-A);
  b = norm(A-C);
  c = norm(A-B);
  if nargout == 1
    a = norm(B-C);     % slightly faster if only R is required
    R = a*b*c/2/norm(D);
    if norm(D) == 0
      R = Inf;
    end
    return
  end
  E = cross(D,B-A);
  F = cross(D,C-A); 
  G = (b^2*E-c^2*F)/norm(D)^2/2;
  M = A + G;
  R = norm(G);  % Radius of curvature
  if R == 0
    k = G;
  elseif norm(D) == 0
    R = Inf;
    k = D;
  else
    k = G'/R^2;   % Curvature vector
  end
end
