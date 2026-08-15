
load('try1deleted1000.mat'); % Loads radii
carParams = Cars.FE13();

mmd = MMD.MMD(carParams); 

num_points = length(radiusMat);
Cay = zeros(1, num_points);
v_max = zeros(1, num_points); % at each dx


for i = 1:num_points
    current_R = radiusMat(i);
    
    result = mmd.evaluate(); %which input variables don't give errors??

%get Cay
    Cay(i) = result.Cay;
    v_max(i) = sqrt(Cay(i) * current_R);

    fprintf(v_max)

    %time for each segment
    dt(i) = dx / v_max(i);
end

fprintf('Point %d Max Speed: %.2f m/s\n', i, v_max(i));

%Sum time 
total_time = sum(dt);

