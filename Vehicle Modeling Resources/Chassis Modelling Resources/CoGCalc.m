clear; clc; close all;

% initial measurements
F_0 = [
    90.6	81.5;
    65.1	74.2;
    ];

F_front_0 = sum(F_0(1, :));
F_rear_0 = sum(F_0(2, :));
W_0 = sum(F_0(:));

disp("Front %: " + 100 * F_front_0/W_0)

% front raised
F_p = [
    90.75	79.3;
    72.5	70.3;
    ];

F_front_p = sum(F_p(1, :));
F_rear_p = sum(F_p(2, :));
W_p = sum(F_p(:));

disp("Initial total weight: " + W_0 + "(kg)")
disp("Raised total weight: " + W_p + "(kg)")
disp("Differnce: " + (W_p - W_0) + "(kg)")

%
L = 1.572; % wheelbase (m)
h = 0.265; % height raised
r = 0.2032; % effective radius (8 inch)


theta = asin(h/L);
hCG = r + L * (F_front_0-F_front_p)/(W_0*tan(theta));
disp("CoG Height: " + hCG  + "(m)")