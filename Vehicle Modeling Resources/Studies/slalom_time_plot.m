clear; close all; clc

carParams = Cars.FE13();
track_width = linspace(1.2, 2, 10);
time = zeros(1, 10);
d = 9.144; % 30' spacing
n = 10;


for i=1:10
    carParams.Twf = track_width(i);
    carParams.Twr = track_width(i);

    time(i) = slalom_time(carParams, d, n);
    disp("============ finished =============")
end

%%

plot(track_width, time)