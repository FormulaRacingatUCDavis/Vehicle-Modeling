function regressedSR = regressSRFunction(Fz, motorForce, maxSR, TireParameters)
    % if nargin == 0
    %     addpath( genpath( fileparts( which( 'regressSRFunction.m' ) ) ) );
    %     Fz = 700;
    %     motorForce = 1500;
    %     maxSR = 0.3;
    %     load('Hoosier_R25B_16x75-10x7.mat');  
    % 
    %     TireParameters.Pacejka = Tire.Pacejka;
    % 
    %     regressSRFunction(Fz, motorForce, maxSr, TireParameters)
    % end



    SlipRatio = linspace(0, 1,100)';
    
    Fx = LongTM(Fz, SlipRatio, TireParameters);
    
    [maxFx, maxFxInd] = max(Fx);
    newSR = SlipRatio(1:maxFxInd);
    newFx = Fx(1:maxFxInd);
    
    regressFunc = csapi(newFx, newSR);
    
    if motorForce > maxFx
        regressedSR = ppval(regressFunc, maxFx);
    else
        regressedSR = ppval(regressFunc, motorForce);
    end

    if regressedSR > maxSR
        regressedSR = maxSR;
    end
end

function Fx = LongTM(Fz, Kappa, TireParameters)
    
    Pi = 70;
    Inc = 0;
    
    dFz = (Fz - TireParameters.Pacejka.Fzo) ./ TireParameters.Pacejka.Fzo;
    dPi = (Pi - TireParameters.Pacejka.Pio) ./ TireParameters.Pacejka.Pio;
    
    Cx = TireParameters.Pacejka.p.C.x(1) .* TireParameters.Pacejka.L.C.x;
    
    Dx = (TireParameters.Pacejka.p.D.x(1) + TireParameters.Pacejka.p.D.x(2).*dFz) .* ...
         (1 + TireParameters.Pacejka.p.P.x(3).*dPi + TireParameters.Pacejka.p.P.x(4).*dPi.^2) .* ...
         (1 - TireParameters.Pacejka.p.D.x(3).*Inc.^2).*Fz .* TireParameters.Pacejka.L.mu.x;
    
    Ex = ( TireParameters.Pacejka.p.E.x(1) + TireParameters.Pacejka.p.E.x(2).*dFz ...
        + TireParameters.Pacejka.p.E.x(3).*dFz.^2 ) .* ( 1 - TireParameters.Pacejka.p.E.x(4).*sign(Kappa) ) .* ...
        TireParameters.Pacejka.L.E.x;
    
    Kxk = Fz.*(TireParameters.Pacejka.p.K.x(1) + TireParameters.Pacejka.p.K.x(2).*dFz ) .* ...
        exp( TireParameters.Pacejka.p.K.x(3) .* dFz ) .* ...
        (1 + TireParameters.Pacejka.p.P.x(1).*dPi + TireParameters.Pacejka.p.P.x(2).*dPi.^2) .* ...
        TireParameters.Pacejka.L.K.x.k;
    
    Bx = Kxk ./ ( Cx.*Dx );
    
    Vx = Fz.*(TireParameters.Pacejka.p.V.x(1) + TireParameters.Pacejka.p.V.x(2).*dFz ) .* ...
        TireParameters.Pacejka.L.V.x;
    
    Hx = (TireParameters.Pacejka.p.H.x(1) + TireParameters.Pacejka.p.H.x(2).*dFz) .* ...
        TireParameters.Pacejka.L.H.x;
    
    Fx = Dx .* sin( Cx .* atan( (1-Ex) .* Bx.*(Kappa + Hx) + ...
        Ex.*atan( Bx.*(Kappa + Hx) ) ) ) + Vx;

end