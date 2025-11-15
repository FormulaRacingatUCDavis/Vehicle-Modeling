function [stabilityVal, controlVal] = getStabilityNControl(result, sa_cg, dsteer)
    arguments
        result 
        sa_cg = 0
        dsteer = 0
    end
    MzBody = result.CMzBody;
    SA_CG = result.grid.SA_CG;
    dSteer = result.grid.dSteer;
    m = result.carParams.m;
    WB = result.carParams.WB;
    g = 9.81;

    %%% Finds control derivative (Mz/steer slope on the SA line)
    if sum(ismember(SA_CG, 0)) > 0
        deriv1 = gradient(MzBody(:, SA_CG == 0)) ./ rad2deg(gradient(dSteer));
        if sum(ismember(dSteer, 0)) > 0
            controlVal = deriv1((find(dSteer == 0))).* (m.*g.*WB);
        else
            disp('Choose an odd number of array values for dSteer doofus');
        end
    else
        disp('Choose an odd number of array values for SA_CG doofus');
    end
    
    %%% Finds stability derivative (Mz/SA slope on the steer line)
    if sum(ismember(dSteer, 0)) > 0
        deriv2 = gradient(MzBody(dSteer == 0,:))' ./ rad2deg(gradient(SA_CG));
        if sum(ismember(SA_CG, 0)) > 0
            %%% This value is negative because the values for steering starts
            %%% on the right side of the map going to the left side
            %%% ------ Maybe something wrong with the coordinates again?
            %%% No it should be negative because a moment in the other direction
            %%% will be generated with a slip angle
            stabilityVal = -deriv2((find(SA_CG == 0))) .* (m.*g.*WB);
        else
            disp('Choose an odd number of array values for SA_CG doofus');
        end
    else
        disp('Choose an odd number of array values for dSteer doofus');
    end
end

