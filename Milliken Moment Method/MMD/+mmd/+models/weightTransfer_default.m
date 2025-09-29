function [outputArg1,outputArg2] = weightTransfer_default(inputArg1,inputArg2)
    dFzf_dAx = (hCG .* m)./(2.* WB);
    dFzf_dAy = (hCG .* m .* PFront)/TWf;
    dFzr_dAy = (hCG .* m .* (1-PFront))/TWr;

    FzAero = (1/2)*rho*crossA*Cl*V^2;
    Fz_WeightFront = (m.*g.* PFront + FzAero .* CoP)/2;
    Fz_WeightRear = (m.*g.* (1-PFront) + FzAero .* (1 - CoP))/2;

    Fz(1,1) = Fz_WeightFront + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
    Fz(2,1) = Fz_WeightFront + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
    Fz(3,1) = Fz_WeightRear - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
    Fz(4,1) = Fz_WeightRear - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
end

