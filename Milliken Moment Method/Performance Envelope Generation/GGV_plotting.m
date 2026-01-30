clear;clc;close all;

Files = ["FE13_HDF_HB" "FE13_HDF_MB" "FE13_HDF_LB" "FE13_LDF_HB" "FE13_LDF_MB" "FE13_LDF_LB"];

for(i = 1:length(Files))
    GGV_data_points{i} = load(Files(i)).GGV_data;
end

tiledlayout(2, 3)
for(i = 1:length(Files))
    nexttile
    GGV_data = GGV_data_points{i};

    scatter3(GGV_data(:, 1), GGV_data(:, 2), GGV_data(:, 3))
    
    xlabel("Ax")
    ylabel("Ay")
    zlabel("V")

    title(Files(i))

    view(2)
    
    xlim([-3, 3])
    ylim([-3, 3])
end

figure;
hold on
for(i = 1:length(Files))
    GGV_data = GGV_data_points{i};
    scatter3(GGV_data(:, 1), GGV_data(:, 2), GGV_data(:, 3))
end

xlabel("Ax")
ylabel("Ay")
zlabel("V")

xlim([-3, 3])
ylim([-3, 3])

legend(Files)
view(3)