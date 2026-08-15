function coord_AllW = calcWheelCoords(carParams)
    lf = carParams.WB * (1-carParams.PFront);
    lr = carParams.WB * (carParams.PFront);
    coord_W1 = [lf ; carParams.TWf/2];
    coord_W2 = [lf ; -carParams.TWf/2];
    coord_W3 = [-lr ; carParams.TWr/2];
    coord_W4 = [-lr ; -carParams.TWr/2];
    coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];
end