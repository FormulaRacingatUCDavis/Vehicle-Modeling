clear;

fe12_mmd = mmd.MMD(Cars.FE12());

grid.SA_CG = -5:5;
grid.dSteer = -30:30;
grid.V = 30;

result = fe12_mmd.evaluate(grid, "free_rolling")

grid.V = 35

result = fe12_mmd.evaluate(grid, "free_rolling", [], result)