function [CAxBody, CAyBody, CAxVel, CAyVel, MzBody, Omega] = iterateOneCell(carParams, dSteer, SA_CG, V, driveCondition, targetCAx, models, AxBodyinit, AyBodyinit, config)
    % unpack car params
    m = carParams.m;                % Total Mass [kg]
    PFront = carParams.PFront;      % Percent Mass Front [0-1]
    CamberFront = carParams.CamberFront;
    CamberRear = carParams.CamberRear;
    WB = carParams.WB;
    
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

    coord_AllW = carParams.coord_AllW;

    % variable initializations
    itAyBody = [];
    itAyBody(1,1) = AyBodyinit;
    itAxBody = [];
    itAxBody(1,1) = AxBodyinit;
    itOmega = [];
    itOmega(1,1) = 0;

    SA_Wheel = zeros(4, 1);
    V_Wheel  = zeros(4, 1);
    TM_Fx    = zeros(4, 1);
    TM_Fy    = zeros(4, 1);
    FxTire   = zeros(4, 1);
    FyTire   = zeros(4, 1);
    MzTire   = zeros(4, 1);
    tireSR   = zeros(4, 1);

    g = 9.81;
    c = 1;

    resCAy = 1;
    resCAx = 1;

    pr = config.pr;

    dSteer_AllW = models.steeringModel(carParams, dSteer);

    % Iterate
    while abs(resCAy) > config.tol || abs(resCAx) > config.tol || c < 3
    
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

        % level surface
        if ~isempty(targetCAx) && c > 1
            Ferror = m * g * (targetCAx - itAxVel/g);

            TireFxTarget = zeros(4, 1);
            if driveCondition
                % drive
                targetFx = (sum(TM_Fx([3, 4])) + Ferror);
                motorLimFx = models.motorLimitFn(carParams, V);
                if (targetFx > motorLimFx)
                    targetFx = motorLimFx;
                end
                TireFxTarget([3, 4]) = targetFx / 2;
            else
                % brake
                TireFxTarget([1, 2]) = B_FBB * (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                TireFxTarget([3, 4]) = (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
            end
            
            tireSR = calcSR(carParams, TireFxTarget, driveCondition, rad2deg(SA_Wheel), Fz ,...
                    TireInclination, V_Wheel);
        end

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


        [latWT_Front, latWT_Rear, longWT] = models.weightTransferFn(carParams, AxCurr, AyCurr);

        FzAero = (1/2)*rho*crossA*Cl*V^2;
        Fz_WeightFront = (m.*g.* PFront + FzAero .* CoP)/2;
        Fz_WeightRear = (m.*g.* (1-PFront) + FzAero .* (1 - CoP))/2;

        Fz(1,1) = Fz_WeightFront + longWT + latWT_Front;
        Fz(2,1) = Fz_WeightFront + longWT - latWT_Front;
        Fz(3,1) = Fz_WeightRear  - longWT + latWT_Rear;
        Fz(4,1) = Fz_WeightRear  - longWT - latWT_Rear;

        for p = 1:4
            %%% Still need to figure out tire inclination
            %%% stuff--------- camber changes based on wheel SA, body
            %%% SA, and which wheel it is
            if p == 1 
                TireInclination = -sign(SA_Wheel(p)) .* CamberFront;
            elseif p == 2
                TireInclination =  -sign(SA_Wheel(p)).* -CamberFront;
            elseif p == 3
                TireInclination = -sign(SA_Wheel(p)) .* CamberRear;
            else
                TireInclination =  -sign(SA_Wheel(p)) .* -CamberRear;
            end


            [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire,...
                rad2deg(SA_Wheel(p)), tireSR(p), Fz(p) , TirePressure ,...
                TireInclination, V_Wheel(p), Idx, Model);

            % %%% METHOD 1: Free Rolling MMD Assumption (Inf Radius)
            % TM_Fx(p) = TM_Fx(p) .* 0; 

            % Calspan TTC Data usual correction factor - 0.7
            TM_Fx(p) = TM_Fx(p) .* carParams.tire.CorrectionFactor ;
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
        resCAy = (itAyBody(c+1) - itAyBody(c))/g;
        resCAx = (itAxBody(c+1) - itAxBody(c))/g;

        % %%% CAN COMMENT IN AND WILL DISPLAY THE ITERATIONS THAT IT IS
        % %%% STUCK ON BUT MASSIVELY IMPACTS PERFORMANCE - ONLY USE WHEN
        % %%% THE SIMULATION IS SEEMINGLY STUCK
        %
        % % Tests the c 
        % plot(1:c+1, itCAxBody);

        c = c + 1;
        
        % If the iterations do not converge, take an average of last
        % 100 values and set that
        if c > config.maxIter
            iterationCtrl = itAyBody( (c-100) : (c-1) );
            itAyBody(c) = min(iterationCtrl);
            itAxBody(c) = mean(itAxBody(end-100:end));
            % disp("over iter limit")
            break
        end
    end % while loop end
    
    CAxBody = itAxBody(end) / g;
    CAyBody = itAyBody(end) / g;
    MzBody = MzBody/(m.*g.*WB);
    CAxVel = CAxBody * cos(SA_CG) + CAyBody .* sin(SA_CG);
    CAyVel = CAyBody.* cos(SA_CG) - CAxBody .* sin(SA_CG);
    Omega  = itOmega(end);
end
