function [latWT_Front, latWT_Rear, longWT] = weightTransfer_default(carParams, Ax, Ay)
    m = carParams.m;
    hCG = carParams.hCG;
    PFront = carParams.PFront;
    WB = carParams.WB;
    TWf = carParams.TWf;
    TWr = carParams.TWr;

    dFzf_dAx = (hCG .* m)./(2.* WB);
    dFzf_dAy = (hCG .* m .* PFront)/TWf;
    dFzr_dAy = (hCG .* m .* (1-PFront))/TWr;

    latWT_Front = dFzf_dAy .* Ay;
    latWT_Rear  = dFzr_dAy .* Ay;
    longWT      = dFzf_dAx .* Ax;
end

