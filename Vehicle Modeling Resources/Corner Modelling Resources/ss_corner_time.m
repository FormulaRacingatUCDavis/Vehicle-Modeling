function dt = ss_corner_time(carParams, r, dx)
    % Initialization
    mmd = MMD.MMD(carParams);
    v = 20;
    v_last = 20;
    e = inf;

    % Find v_max
    while e > 0.1
        ay = calcAy(v, mmd);
        v = sqrt(ay * r);

        e = abs(v - v_last);
        v_last = v;
    end

    % time for each segment
    dt = dx / v;
end

function ay = calcAy(v, mmd)
    grid.V = v;
    grid.dSteer = -20:20;
    grid.SA = -5:5;
    
    results = mmd.evaluate(grid, "free_rolling");
    SS = MMD.analysis.getSteadyStateCAy(results);

    ay = SS.CAy;
end