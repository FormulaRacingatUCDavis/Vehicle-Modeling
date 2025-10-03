function absoluteCAy = getAbsoluteCAy(result,grid)
    SA_CG = grid.SA_CG;
    dSteer = grid.dSteer;
    [SA_CG, dSteer] = meshgrid(SA_CG, dSteer);
    dataPoints = [result.CAyVel(1:end);
                  SA_CG(1:end);
                  dSteer(1:end);
                  result.MzBody(1:end)];

    [absoluteCAy.CAy, idx] = max(dataPoints(1, :));
    absoluteCAy.SA = dataPoints(2, idx);
    absoluteCAy.dSteer = dataPoints(3, idx);
    absoluteCAy.Mz = dataPoints(4, idx);

end