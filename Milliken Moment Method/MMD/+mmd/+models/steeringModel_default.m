function dSteer_Allw = steeringModel_default(carParams, dSteer)
    dSteer_W1 = carParams.toe_f + dSteer;
    dSteer_W2 = -carParams.toe_f + dSteer;
    dSteer_W3 = carParams.toe_r + dSteer.*0;
    dSteer_W4 = -carParams.toe_r + dSteer.*0;
    dSteer_Allw = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];
end