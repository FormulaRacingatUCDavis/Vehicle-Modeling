function SR = calcSR(FxTarget, Tire, SlipAngle, NormalLoad, Pressure, Inclination, Velocity, Idx, Model)
    %%% Slip-Load Plots 
    SlipRatio = linspace(-1, 1,100)';

    [Fx, ~, ~, ~, ~] = ContactPatchLoads( Tire, ...
        SlipAngle, SlipRatio, ...
        NormalLoad, Pressure, Inclination, Velocity, ...
        Idx, Model );
    
    [~, ind] = max(Fx);
    newSR = SlipRatio(1:ind);
    newFx = Fx(1:ind);
    
    yuh = csapi(newFx, newSR);
    
    SR = fnval(yuh, FxTarget);
end

