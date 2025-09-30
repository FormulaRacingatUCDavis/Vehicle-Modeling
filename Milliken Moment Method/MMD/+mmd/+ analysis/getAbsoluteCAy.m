function absoluteCAy = getAbsoluteCAy(result.rawData, result.SA_CG, result.dSteer)

    [SA_CG, dSteer] = meshgrid(SA_CG, dSteer);
    dataPoints = [rawData.saveCAyVel(1:end);
                  SA_CG(1:end);
                  dSteer(1:end);
                  rawData.saveMzBody(1:end)];

    [absoluteCAy.CAy, idx] = max(dataPoints(1, :));
    absoluteCAy.SA = dataPoints(2, idx);
    absoluteCAy.dSteer = dataPoints(3, idx);
    absoluteCAy.Mz = dataPoints(4, idx);

end