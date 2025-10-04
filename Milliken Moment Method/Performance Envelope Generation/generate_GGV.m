function GGV_data = generate_GGV(mmd, V, SA_CG, dSteer, stepSize)
    arguments
        mmd
        V 
        SA_CG = -5:1;
        dSteer = -15:15;
        stepSize = 0.05;
    end

    grid.SA_CG  = SA_CG;
    grid.dSteer = dSteer;
    GGV_data = [];

    driveData = [];
    brakeData = [];

    c = 1;
    % Velocity sweep
    for v = V
        disp(c + "/" + length(V) + ", " + 100*c/length(V) + "%")
        c = c + 1;

        grid.V = v;
        
        driveData = mmd.evaluate(grid, "drive", inf, driveData);
        brakeData = mmd.evaluate(grid, "brake", -inf, brakeData);

        % accel sweep
        targetCAxAccel = max(driveData.CAxVel(:));
        GGV_data = [GGV_data; [targetCAxAccel, 0, v]];
        while true
            targetCAxAccel = targetCAxAccel - stepSize;
            SSAy = MMD.analysis.getSteadyStateCAy(driveData, targetCAxAccel);

            if SSAy.CAy == 0
                break;
            end

            GGV_data = [GGV_data; [targetCAxAccel SSAy.CAy v]];
        end

        % deccel sweep
        targetCAxDecel = min(brakeData.CAxVel(:));
        GGV_data = [GGV_data; [targetCAxDecel, 0, v]];
        while true
            targetCAxDecel = targetCAxDecel + stepSize;
            SSAy = MMD.analysis.getSteadyStateCAy(brakeData, targetCAxDecel);

            if SSAy.CAy == 0
                break;
            end

            GGV_data = [GGV_data; [targetCAxDecel SSAy.CAy v]];
        end
    
        % fill in the blanks
        level_surf_data = [];
        for targetCAx = targetCAxDecel:0.1:targetCAxAccel
            level_surf_data = mmd.evaluate(grid, "level_surface", targetCAx, level_surf_data);

            SSAy = MMD.analysis.getSteadyStateCAy(level_surf_data, targetCAx);
            GGV_data = [GGV_data; [targetCAx SSAy.CAy v]];
        end
    end
end
