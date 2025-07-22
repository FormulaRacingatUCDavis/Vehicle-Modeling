function SR = calcSR(FxTarget, Tire, SlipAngle, NormalLoad, Pressure, Inclination, Velocity, Idx, Model)
    %%% Slip-Load Plots 
    SlipRatio = linspace(-1, 1, 100)';

    [Fx, ~, ~, ~, ~] = ContactPatchLoads( Tire, ...
        SlipAngle, SlipRatio, ...
        NormalLoad, Pressure, Inclination, Velocity, ...
        Idx, Model );
    
    [~, max_idx] = max(Fx);
    [~, min_idx] = min(Fx);
    newSR = SlipRatio(min_idx:max_idx);
    newFx = Fx(min_idx:max_idx);
    
    yuh = csapi(newFx, newSR);
    
    SR = fnval(yuh, FxTarget);
end

