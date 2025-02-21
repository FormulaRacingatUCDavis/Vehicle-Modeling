% Load stuff
track_data = load('maxar_coords.mat');
percentages_data = load('maxar_racing_line_opt.mat');

% Extract info
data_matrix = track_data.data;
percentages = percentages_data.x;

% Make it a column vector
percentages = percentages(:);

% Extract the columns for Outside Track and Inside Track
outside_x = data_matrix(:, 2);
outside_y = data_matrix(:, 3);
inside_x = data_matrix(:, 4);
inside_y = data_matrix(:, 5);

% % debugging stuff, checks sizes
% disp("Size of outside_x: "), disp(size(outside_x));
% disp("Size of inside_x: "), disp(size(inside_x));
% disp("Size of percentages: "), disp(size(percentages));
% 
% % Check if percentages are within expected range
% disp("Min percentage: "), disp(min(percentages));
% disp("Max percentage: "), disp(max(percentages));

% calculations for optimized line
optimized_x = inside_x + percentages .* (outside_x - inside_x);
optimized_y = inside_y + percentages .* (outside_y - inside_y);

% smoothing optimized line
t_original = 1:length(optimized_x);
t_smooth = linspace(1, length(optimized_x), 50000); % More points for smoothness
smooth_x = spline(t_original, optimized_x, t_smooth);
smooth_y = spline(t_original, optimized_y, t_smooth);

% Plot everything
figure;
plot(outside_x, outside_y, 'b-', 'LineWidth', 2);
hold on;
plot(inside_x, inside_y, 'r-', 'LineWidth', 2);
plot(smooth_x, smooth_y, 'g-', 'LineWidth', 2);

% Labels and legend
title('💀');
legend({'Outside Track', 'Inside Track', '"Optimized" Racing Line'}, 'Location', 'Best');
hold off;
