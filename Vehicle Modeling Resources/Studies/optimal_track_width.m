clear; close all; clc

% slaloms in the modeled track
% 30' spacing * 7
% 35' spacing * 2
% 35'--> 33' (34'?) spacing * 3

d = [30 35 34] .* 0.3048;
n = [7 2 3];

samples = 10;

carParams = Cars.FE13();
track_width = linspace(1, 2, samples);
slalom_total_time = zeros(1, samples);
ss_total_time = zeros(1, samples);

for i=1:samples
    carParams.Twf = track_width(i);
    carParams.Twr = track_width(i);

    % slalom time
    for j=1:length(n)
        slalom_total_time(i) = slalom_total_time(i) + slalom_time(carParams, d(j), n(j));
    end

    % SS corner time
    for j=1:length(dx)
        ss_total_time(i) = ss_total_time(i) + ss_corner_time(carParams, r(j), dx(j));
    end
end

%%

figure;
hold on
grid on
plot(track_width, slalom_total_time)
plot(track_width, ss_total_time)
plot(track_width, ss_total_time + slalom_total_time)

xlabel("track width (m)")
ylabel("time (s)")

legend(["slalom time", "steady-state corner time", "total time"])
