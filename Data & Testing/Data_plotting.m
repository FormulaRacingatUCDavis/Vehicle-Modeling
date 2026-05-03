clear; clc; close all;

jian_data = load("FE13_BlueMax_Jian.mat");
jonah_data = load("FE13_BlueMax_Jonah.mat");

figure
hold on
scatter(jonah_data.smoothAy, jonah_data.smoothAx)
scatter(jian_data.smoothAy, jian_data.smoothAx)
legend('Jonah', 'Jian');
xlabel('Ax');
ylabel('Ay');
title('Scatter Plot of Ax vs Ay');