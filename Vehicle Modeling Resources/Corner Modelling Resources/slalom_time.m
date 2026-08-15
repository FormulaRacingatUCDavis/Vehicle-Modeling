function time = slalom_time(carParams, d, n)
    % d: cone distance, not period
    % n: number of intervals
    g = 9.81;
    amplitude = max(carParams.Twf, carParams.Twr) / 2;
    grid.SA_CG = linspace(-5, 1, 6);
    grid.dSteer = linspace(-20, 20, 30);

    v = 15;
    tol = 0.01;
    max_iter = 1000;
    iter_count = 0;
    e = inf;

    while e > tol & iter_count < max_iter
        grid.V = v;
        mmd = MMD.MMD(carParams);
        results = mmd.evaluate(grid, "free_rolling");
        SS = MMD.analysis.getSteadyStateCAy(results);
        Ay = SS.CAy;
        v_p = (d / pi) * sqrt(g * Ay / amplitude);

        e = abs(v - v_p);
        v = v_p;
        iter_count = iter_count + 1;
    end

    time = n * pi * sqrt(amplitude / (g * Ay));

end
