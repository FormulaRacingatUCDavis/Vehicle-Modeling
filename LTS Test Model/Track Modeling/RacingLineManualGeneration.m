clc; clear; close all

%% Initialization
f1 = figure;
j = 1;
imageFile = 'Screenshot 2024-11-04 174632.jpg';
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
plot(scaledX, -scaledY, 'ro-', LineWidth = 2);
title("Plotted Path (Not Processed)")
xlabel("X Coordinate (distance unit)")
ylabel("Y Coordinate (distance unit)")
xlim([0, distLimX])
ylim([-distLimY, 0])

%% TESTING

lineThingy = spline(scaledX, scaledY);
lineThingy2 = pchip(scaledX, scaledY);
figure
hold on
plot(scaledX, scaledY)
fnplt(lineThingy2)
% fnplt(lineThingy)

