function [max_Ax, SA, dsteer] = vehicle_model_comb_test(carParams, V, targetAy, driveCondition, initCon)
    function y = func(x)
        SA_CG = x(1);
        dSteer = x(2);
        targetCAx = x(3);
        [Ax, Ay, Mz] = single_MMD_Point(carParams, V, SA_CG, dSteer, targetCAx, driveCondition);
        y = double(abs(targetAy - Ay) + abs(Mz) + -driveCondition*Ax + (~driveCondition)*Ax);
    end

    options = optimoptions('fmincon', ...
        'Display','off');

    [x, ~] = fmincon(@func, initCon, [], [], [], [], [-5, -15, -10], [5, 15, 10], [], options);

    SA = x(1);
    dsteer = x(2);
    
    max_Ax = x(3) * 9.81;
end

function [Ax, Ay, Mz] = single_MMD_Point(carParams, V, SA_CG, dSteer, targetCAx, driveCondition)
    % constants
    g = 9.81;
    pr = 0;

    % unpack car params
    m = carParams.m;                % Total Mass [kg]
    PFront = carParams.PFront;      % Percent Mass Front [0-1]
    WB = carParams.WB;              % Wheelbase [m]
    TWf = carParams.TWf;            % Trackwidth [m]
    TWr = carParams.TWr;
    hCG = carParams.hCG;            % CG height [m]
    toe_f = carParams.toe_f;
    toe_r = carParams.toe_r;
    TireInclinationFront = carParams.TireInclinationFront;
    TireInclinationRear = carParams.TireInclinationRear;
    
    Cl = carParams.Cl;
    Cd = carParams.Cd; 
    CoP = carParams.CoP;            % front downforce distribution (%)
    rho = carParams.rho;            % kg/m^3
    crossA = carParams.crossA;      % m^2
    
    B_FBB = carParams.B_FBB;        % Front brake bias

    Idx = carParams.tire.Idx;                     % Moment of Inertia in x for wheel
    TirePressure = carParams.tire.TirePressure;           % kPa
    Model = carParams.tire.Model;
    Tire = carParams.tire.Tire;

    SA_CG = deg2rad(SA_CG);
    dSteer = deg2rad(dSteer);

    dSteer_W1 = toe_f + dSteer;
    dSteer_W2 = -toe_f + dSteer;
    dSteer_W3 = toe_r + dSteer.*0;
    dSteer_W4 = -toe_r + dSteer.*0;
    dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];

    lf = WB * (1-PFront);
    lr = WB * (PFront);
    coord_W1 = [lf ; TWf/2];
    coord_W2 = [lf ; -TWf/2];
    coord_W3 = [-lr ; TWr/2];
    coord_W4 = [-lr ; -TWr/2];
    coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];

    AxGuess = 0;
    AyGuess = 0;
    res = 1;
    resCAx = 1;
    tol = 1e-3;

    itAyBody = [];
    itAyBody(1,1) = AyGuess;
    itAxBody = [];
    itAxBody(1,1) = AxGuess;
    itCAyBody = [];
    itCAyBody(1,1) = AyGuess/g;
    itCAxBody = [];
    itCAxBody(1,1) = AxGuess/g;
    tireSR = [0, 0, 0, 0];

    %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
    itOmega = [];
    itOmega(1,1) = 0;

    c = 1;

    while abs(res) > tol || abs(resCAx) > tol || c < 3
    
        if c > 1
        % level surface stuff
            Ferror = m * g * (targetCAx - itAxVel/g);
    
            % could set a tolerance for Ferror here
            if abs(Ferror) > 0
    
                % determine drive/brake condition
                TireFxTarget = zeros(4, 1);
                if driveCondition
                    % drive
                    TireFxTarget([3, 4]) = (sum(TM_Fx([3, 4])) + Ferror) / 2;
                else
                    % brake
                    TireFxTarget([1, 2]) = B_FBB * (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                    TireFxTarget([3, 4]) = (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                end

                if min(Fz) > 0
                    tireSR = calcSR(TireFxTarget, driveCondition, B_FBB, Tire, rad2deg(SA_Wheel), Fz , TirePressure ,...
                            TireInclination, V_Wheel, Idx, Model);
                end
            end
        end
    
        AyCurr = itAyBody(c);
        AxCurr = itAxBody(c);
    
        AyVelCurr = itAyBody(c) .* cos(SA_CG) - itAxBody(c) .* sin(SA_CG);
    
        VyCurr = V .* sin(SA_CG);
        VxCurr = V .* cos(SA_CG);
    
        %%%%% VEHICLE BODY YAW RATE
    
        %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
        %%% Equation 2
        if c == 1
            % Nothing since relaxation parameter requires last index
        else
            itOmega(c) = AyVelCurr/VxCurr * (1-pr) + itOmega(c-1)*pr;
        end
        % R =  VxCurr.^2 / AyVelCurr; this variable is not being
        % used for some reason
        Omega = itOmega(c);
    
        for p = 1:4
            %%%%% WHEEL SLIP ANGLE EQUATIONS
            %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius) -
            %%% Equation 1
            SA_Wheel(p,1) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
                                , (VxCurr - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(p);
    
    
           %%%%% WHEEL SPEED EQUATIONS
           %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
           V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
    
        end
    
    
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
    
        for p = 1:4
            %%% Still need to figure out tire inclination
            %%% stuff--------- camber changes based on wheel SA, body
            %%% SA, and which wheel it is
            if p == 1 
                TireInclination = -sign(SA_Wheel(p)) .* TireInclinationFront;
            elseif p == 2
                TireInclination =  -sign(SA_Wheel(p)).* -TireInclinationFront;
            elseif p == 3
                TireInclination = -sign(SA_Wheel(p)) .* TireInclinationRear;
            else
                TireInclination =  -sign(SA_Wheel(p)) .* -TireInclinationRear;
            end
    
    
            [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire,...
                rad2deg(SA_Wheel(p)), tireSR(p), Fz(p) , TirePressure ,...
                TireInclination, V_Wheel(p), Idx, Model);
    
            % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
            % TM_Fx(p) = TM_Fx(p) .* 0; 
    
            % Calspan TTC Data usual correction factor - 0.7
            TM_Fx(p) = TM_Fx(p) .* 0.7;
            % Tire Model outputs in opposite Y coordinates
            TM_Fy(p) = (1).* TM_Fy(p) .* 0.53;
    
            FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(p)) - TM_Fy(p) .* sin(dSteer_AllW(p));
            FyTire(p,1) = TM_Fx(p) .* sin(dSteer_AllW(p)) + TM_Fy(p) .* cos(dSteer_AllW(p));
    
            % Made it a matrix sum so its not ugly :)
            MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(p)) ;
                               -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(p)) ; 
                                coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(p)) ;
                                coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(p))  ] ); 
        end
    
        FxDrag = (1/2)*rho*crossA*Cd*V^2;
    
        FxBody = sum(FxTire) - FxDrag;
        FyBody = sum(FyTire);
        MzBody = sum(MzTire);
    
        itAyBody(c+1,1) = FyBody/m;
        itAxBody(c+1,1) = FxBody/m;
        itAxVel = itAxBody(c+1,1).* cos(SA_CG) + itAyBody(c+1,1) .* sin(SA_CG);
    
        % Using dimensionless G's to iteration (apparently improves
        % stability)?
        itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
        itCAxBody(c+1,1) = itAxBody(c+1,1)/g;
        res = itCAyBody(c+1) - itCAyBody(c);
        resCAx = itCAxBody(c+1) - itCAxBody(c);
    
        % %%% CAN COMMENT IN AND WILL DISPLAY THE ITERATIONS THAT IT IS
        % %%% STUCK ON BUT MASSIVELY IMPACTS PERFORMANCE - ONLY USE WHEN
        % %%% THE SIMULATION IS SEEMINGLY STUCK
        %
        % % Tests the c 
        % plot(1:c+1, itCAxBody);
    
        c = c + 1;
        
        % If the iterations do not converge, take an average of last
        % 100 values and set that
        if c > 1000
            iterationCtrl = itAyBody( (c-100) : (c-1) );
            itAyBody(c) = min(iterationCtrl);
            itCAxBody(c) = mean(itCAxBody(end-100:end));
            % disp("over iter limit")
            break
        end
    
    end % while loop end
    
   Ax = itAxBody(end) .* cos(SA_CG) + itAyBody(end) .* sin(SA_CG);
   Ay = itAyBody(end) .* cos(SA_CG) - itAxBody(end) .* sin(SA_CG);
   Mz = MzBody/(m.*g.*WB);
    
end