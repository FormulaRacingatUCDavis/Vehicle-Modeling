clear; clc; close all;

mmd = MMD.MMD(Cars.FE12());

GGV_data = generate_GGV(mmd, 10:5:45);

GGV_data = [GGV_data; (GGV_data' .* [1; -1; 1])'];

%%
scatter3(GGV_data(:, 1), GGV_data(:, 2), GGV_data(:, 3))

xlabel("Ax")
ylabel("Ay")
zlabel("V")

xlim([-5, 5])
ylim([-5, 5])