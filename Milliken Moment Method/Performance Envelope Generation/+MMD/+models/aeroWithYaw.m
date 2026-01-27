function [FaeroFront, FaeroRear, FaeroDrag] = aeroWithYaw(carParams, state)
    V = state.V;
    yaw = state.yaw;
    
    Cl = carParams.Cl(yaw);
    Cd = carParams.Cd; 
    CoP = carParams.CoP;            % front downforce distribution (%)
    rho = carParams.rho;            % kg/m^3
    crossA = carParams.crossA;

    FzAero = (1/2)*rho*crossA*Cl*V^2;
    FaeroFront = FzAero .* CoP;
    FaeroRear = FzAero .* (1-CoP);
    FaeroDrag = (1/2)*rho*crossA*Cd*V^2;
end