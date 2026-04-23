clear; clc; close all;

jian_data = load("FE13_BlueMax_Jian.mat");
jonah_data = load("FE13_BlueMax_Jonah.mat");

figure
hold on
scatter(jonah_data.Ay, jonah_data.Ax)
scatter(jian_data.Ay, jian_data.Ax)
legend('Jonah', 'Jian');
xlabel('Ax');
ylabel('Ay');
title('Scatter Plot of Ax vs Ay');