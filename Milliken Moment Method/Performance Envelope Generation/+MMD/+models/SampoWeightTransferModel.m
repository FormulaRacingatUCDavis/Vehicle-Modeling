% Function to find weight transfer for the front and rear of a car using
% the flexible vehicle model. This function takes an input of a structure
% of car parameters, along with the lateral acceleration (Ay)
function [deltaFzFront, deltaFzRear] = SampoWeightTransfer(carParams, Ay)
    kF = carParams.kF;                                                     % For the sake of simplicity, members of carParams structure are assigned
    kR = carParams.kR;                                                     % to shorter local variables within this function
    kC = carParams.kC;
    m_uF = carParams.m_uF;
    m_uR = carParams.m_uR;
    TWf = carParams.TWf;
    TWr = carParams.TWr;
    h_uF = carParams.h_uF;
    h_uR = carParams.h_uR;
    zF = carParams.zF;
    zR = carParams.zR;
    d_sF = carParams.h_sF - zF;
    d_sR = carParams.h_sR - zR;
    m_sF = carParams.m_s*carParams.b_s/carParams.WB;
    m_sR = carParams.m_s*carParams.a_s/carParams.WB;
    deltaFzFront = ((kF*d_sF*m_sF)/(kF+(kR*kC/(kR+kC)))+((kF*kC/(kF+kC))*d_sR*m_sR)/(kF*kC/(kF+kC)+kR)+zF*m_sF+h_uF*m_uF)*Ay/TWf;
    deltaFzRear = ((kR*kC/(kR+kC)*d_sF*m_sF)/(kF+kR*kC/(kR+kC))+(kR*d_sR*m_sR)/(kF*kC/(kF+kC)+kR)+zR*m_sR+h_uR*m_uR)*Ay/TWr;
end
