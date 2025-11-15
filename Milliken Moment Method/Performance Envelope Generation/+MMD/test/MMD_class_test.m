clear;clc;close all;

mmd = MMD.MMD(Cars.FE13());

grid.SA_CG = 0:5;
grid.dSteer = 0:30;
grid.V = 30;

result = mmd.evaluate(grid, "free_rolling")

grid.V = 35

result = mmd.evaluate(grid, "free_rolling", [], result)
