function [steadyStateCAy, absoluteCAy, rawData] = MMD_Generation(carParams, rangeSA, rangeSteer, velocity, plots, targetCAx)

    % RIGHT HAND SIDE IS POSITIVE Y - RCVD Standard (but lots of industry
    % doesnt like to use this, but for the sake of the paper, we will use this,
    % maybe change it later once I have this all figured out.
    %
    % Tires go 
    %
    % Forwards
    % 2  1      Postive Y coord
    % 4  3
    % Backwards
    %
    % RIGHT HAND TURN RESULTS IN POSTIVE AY(yes you heard that right)

    %%% SA & dSteer range correction
    if mod(sum(abs(rangeSA)), 2) == 0
        numSA = sum(abs(rangeSA)) + 1;
    else
        numSA = sum(abs(rangeSA));
    end
    
    if mod(sum(abs(rangeSteer)), 2) == 0
        numST = sum(abs(rangeSteer)) + 1;
    else
        numST = sum(abs(rangeSteer));
    end
    

    %%% Variable initializations
    SA_CG = deg2rad(linspace(rangeSA(1), rangeSA(2),numSA))';
    dSteer = deg2rad(linspace(rangeSteer(1), rangeSteer(2),numST))';

    dSteer_W1 = carParams.toe_f + dSteer;
    dSteer_W2 = -carParams.toe_f + dSteer;
    dSteer_W3 = carParams.toe_r + dSteer.*0;
    dSteer_W4 = -carParams.toe_r + dSteer.*0;
    dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];
    
    lf = carParams.WB * (1-carParams.PFront);
    lr = carParams.WB * (carParams.PFront);
    coord_W1 = [lf ; carParams.TWf/2];
    coord_W2 = [lf ; -carParams.TWf/2];
    coord_W3 = [-lr ; carParams.TWr/2];
    coord_W4 = [-lr ; -carParams.TWr/2];
    coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];

    rawData.saveAyBody = zeros(length(dSteer), length(SA_CG));
    rawData.saveAxBody = zeros(length(dSteer), length(SA_CG));
    rawData.saveMzBody = zeros(length(dSteer), length(SA_CG));
    
    rawData.saveAxVel = zeros(size(rawData.saveAyBody));
    rawData.saveAyVel = zeros(size(rawData.saveAyBody));
    rawData.saveCAyVel = zeros(size(rawData.saveAyBody));
    rawData.saveCAxVel = zeros(size(rawData.saveAyBody));

    %%% Run the iteration to generate MMD
    rawData = MMD_iteration(carParams, velocity, dSteer, SA_CG, dSteer_AllW, coord_AllW, rawData);

    if exist('targetCAx','var')
        % if targetCAx is present, perform another iteration to converge to
        % targetCAx
        driveCondition = (ones(size(rawData.saveCAxVel)).* targetCAx - rawData.saveCAxVel) > 0;
        rawData = MMD_iteration(carParams, velocity, dSteer, SA_CG, dSteer_AllW, coord_AllW, rawData, driveCondition, targetCAx);
    end

    %%% Find steadyStateCAy/AbsoluteCAy
    if exist('targetCAx','var')
        steadyStateCAy = getSteadyStateCAy(rawData, SA_CG, dSteer, targetCAx);
    else
        steadyStateCAy = getSteadyStateCAy(rawData, SA_CG, dSteer);
    end
    absoluteCAy = getAbsoluteCAy(rawData, SA_CG, dSteer);

    %%% Make the plots
    if ~exist('plots','var')
        plots = false;
    end
    if plots
        plotMMD(rawData, SA_CG, dSteer, velocity, steadyStateCAy)
        testPlots(rawData, SA_CG, dSteer)
    end
    
end

function rawData = MMD_iteration(carParams, ...
                                 V, dSteer, SA_CG, ...
                                 dSteer_AllW, ...
                                 coord_AllW, ...
                                 rawData, ...
                                 driveCondition, ...
                                 targetCAx)

    % param validation
    if nargin == 8
        error("If setting a target CAx, both targetCAx and driveCondition need to be set")
    end

    %%% constants
    g = 9.81;

    % Yaw Rate (Omega) iteration relaxation parameter for convergence:
    % Parameter is between [0-1], is dependent upon how fast the simulation is
    % ran, speeds below tangent speed pr<0.5, speeds above tangent speeds are
    % more stable and requires pr>0.6
    pr = 0.000; 

    % unpack car params
    m = carParams.m;                % Total Mass [kg]
    PFront = carParams.PFront;      % Percent Mass Front [0-1]
    WB = carParams.WB;              % Wheelbase [m]
    TWf = carParams.TWf;            % Trackwidth [m]
    TWr = carParams.TWr;
    hCG = carParams.hCG;            % CG height [m]
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

    % initalize mats
    SA_Wheel = zeros(4,1);
    V_Wheel = zeros(4,1);
    TM_Fx = zeros(4,1);
    TM_Fy = zeros(4,1);
    FxTire = zeros(4,1);
    FyTire = zeros(4,1);
    TireFxTarget = zeros(4, 1);
    MzTire = zeros(4,1);

    % the iteration
    for i = 1:length(dSteer)
    
        sumc = 1;
    
        for j = 1:length(SA_CG)
            tireSR = zeros(4, 1);
    
            res = 1;
            resCAx = 1;
            tol = 1e-3;
    
            itAyBody = [];
            itAyBody(1,1) = rawData.saveAyBody(i, j);
            itAxBody = [];
            itAxBody(1,1) = rawData.saveAxBody(i, j);
            itCAyBody = [];
            itCAyBody(1,1) = rawData.saveAyBody(i, j)/g;
            itCAxBody = [];
            itCAxBody(1,1) = rawData.saveAxBody(i, j)/g;
               
            %%% METHOD 2: Free Rolling MMD Assumption (Finite Radius)
            itOmega = [];
            itOmega(1,1) = 0;
    
            c = 1;
    
            while abs(res) > tol || abs(resCAx) > tol || c < 3
    
                if nargin == 9 && c > 1
                % level surface stuff
                    Ferror = m * g * (targetCAx - itAxVel/g);
    
                    % could set a tolerance for Ferror here
                    if abs(Ferror) > 0
    
                        % determine drive/brake condition
                        if driveCondition(i, j)
                            % drive
                            TireFxTarget = zeros(4, 1);
                            TireFxTarget([3, 4]) = (sum(TM_Fx([3, 4])) + Ferror) / 2;
                        else
                            % brake
                            TireFxTarget([1, 2]) = B_FBB * (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                            TireFxTarget([3, 4]) = (sum(TM_Fx) + Ferror) / (2 * (1 + B_FBB));
                        end
                        
                        tireSR = calcSR(TireFxTarget, driveCondition(i, j), B_FBB, Tire, rad2deg(SA_Wheel), Fz , TirePressure ,...
                                TireInclination, V_Wheel, Idx, Model);
                    end
                end
    
                AyCurr = itAyBody(c);
                AxCurr = itAxBody(c);
    
                AyVelCurr = itAyBody(c) .* cos(SA_CG(j)) - itAxBody(c) .* sin(SA_CG(j));
    
                VyCurr = V .* sin(SA_CG(j));
                VxCurr = V .* cos(SA_CG(j));
    
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
                                        , (VxCurr - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
    
    
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
    
                    FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                    FyTire(p,1) = TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
    
                    % Made it a matrix sum so its not ugly :)
                    MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                       -coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
                                        coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
                                        coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
                end
    
                FxDrag = (1/2)*rho*crossA*Cd*V^2;
    
                FxBody = sum(FxTire) - FxDrag;
                FyBody = sum(FyTire);
                MzBody = sum(MzTire);
    
                itAyBody(c+1,1) = FyBody/m;
                itAxBody(c+1,1) = FxBody/m;
                itAxVel = itAxBody(c+1,1).* cos(SA_CG(j)) + itAyBody(c+1,1) .* sin(SA_CG(j));
    
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
                    itCAxBody(c) = mean(itCAxBody(end-100:end));
                    % disp("over iter limit")
                    break
                end
    
            end % while loop end
        
            sumc = sumc + c;
            % for p = 1:4
            %     saveSA_WheelTerm(p,j) = atan2( (VyCurr + Omega .* coord_AllW(1,p)) ...
            %                                 , (VxCurr - Omega .* coord_AllW(2,p)) );
            % end
    
            % rawData.saveOmega(i,j) = Omega;
            % rawData.saveTM_Fy(:,j,i) = TM_Fy;
            % rawData.saveSA_Wheel(:,j,i) = SA_Wheel;
            % rawData.saveFzWheel(:,j,i) = Fz;
    
            rawData.saveAyBody(i,j) = itAyBody(end);
            rawData.saveAxBody(i,j) = itAxBody(end);
            rawData.saveCAxBody(i,j) = itCAxBody(end);
            rawData.saveMzBody(i,j) = MzBody/(m.*g.*WB);
    
            rawData.saveAxVel(i,j) = rawData.saveAxBody(i,j).* cos(SA_CG(j)) + rawData.saveAyBody(i,j) .* sin(SA_CG(j));
            rawData.saveAyVel(i,j) = rawData.saveAyBody(i,j).* cos(SA_CG(j)) - rawData.saveAxBody(i,j) .* sin(SA_CG(j));
            rawData.saveCAxVel(i,j) = rawData.saveAxVel(i,j)/g;
            rawData.saveCAyVel(i,j) = rawData.saveAyVel(i,j)/g;
    
    
            % saveItFz(:,j) = Fz;
            % saveItAyBody{j,1} = itAyBody;
        end % SA_CG End
        
        % disp("Steering Angle [" + i + "] finished, Avg Iterations: " +  sumc/length(SA_CG))
    end % dSteer End
end

function steadyStateCAy = getSteadyStateCAy(rawData, SA_CG, dSteer, targetCAx)
    zeroMz_CAy_SA = zeros(length(SA_CG),1);
    zeroMz_CAy_ST = zeros(length(dSteer),1);
    zeroMz_SteerDeg = zeros(length(SA_CG),1);
    zeroMz_SADeg = zeros(length(dSteer),1);

    saveMzBody = rawData.saveMzBody;
    saveCAyVel = rawData.saveCAyVel;
    saveCAxVel = rawData.saveCAxVel;
    
    % Go Through the Mz Data through SlipAngle(columns) to check the SA lines
    % ---- CAUTIONS: This only finds the first point along the line that
    % crosses the zero boundary
    % Also interpolates for the other angle values not used (SA or Steer)
    for j = 1:length(SA_CG)
        checkMat = saveMzBody(2:end,j).*saveMzBody(1:end-1,j);
        if exist('targetCAx','var')
            avgAx = (saveCAxVel(2:end, j) + saveCAxVel(1:end-1, j)) ./ 2;
            indexSS = abs(avgAx - targetCAx) < 5e-2 & checkMat < 0;
        else
            indexSS = checkMat < 0;
        end
        indexSS = find(indexSS, 1, 'first');
        if isempty(indexSS)
    
            % The below creates the minimum value of when the line doesnt cross
            % the 0 value, but need to return 0 in case the minimum value is larger
            % than the actual legit maximum SS Ay
            %
            % [minMz,indexMin] = min(abs(saveMzBody(:,j)));
            % zeroMz_CAy_SA(j) = saveCAyVel(indexMin,j);
            % zeroMz_Steer(j) = dSteer(indexMin);
    
            zeroMz_CAy_SA(j) = 0;
            zeroMz_SteerDeg(j) = 0;
        else
            zeroMz_CAy_SA(j) = interp1( saveMzBody(indexSS:indexSS+1, j),...
                                        saveCAyVel(indexSS:indexSS+1, j), 0.0, 'linear');
            zeroMz_SteerDeg(j) = rad2deg(interp1( saveMzBody(indexSS:indexSS+1, j),...
                                       dSteer(indexSS:indexSS+1), 0.0, 'linear'));
        end
    end
    
    % Go Through the Mz Data through Steer Angle(rows) to check the steer lines
    % ---- CAUTIONS: This only finds the first point along the line that
    % crosses the zero boundary
    for i = 1:length(dSteer)
        checkMat = saveMzBody(i, 2:end).*saveMzBody(i, 1:end-1);
        if exist('targetCAx','var')
            avgAx = (saveCAxVel(i, 2:end) + saveCAxVel(i, 1:end-1)) ./ 2;
            indexSS = abs(avgAx - targetCAx) < 5e-2 & checkMat < 0;
        else
            indexSS = checkMat < 0;
        end
        indexSS = find(indexSS, 1, 'first');
        if isempty(indexSS)
            zeroMz_CAy_ST(i) = 0;
            zeroMz_SADeg(i) = 0;
        else
            zeroMz_CAy_ST(i) = interp1( saveMzBody(i, indexSS:indexSS+1),...
                                        saveCAyVel(i, indexSS:indexSS+1), 0.0, 'linear');
            zeroMz_SADeg(i) = rad2deg(interp1( saveMzBody(i, indexSS:indexSS+1),...
                                       SA_CG(indexSS:indexSS+1), 0.0, 'linear'));
        end
    end
    
    % Finds the maximum CAy value in both methods and compares and assigns the
    % label values for SA and Steer
    [maxCAy_SA, maxCAy_SA_ind] = max(zeroMz_CAy_SA);
    [maxCAy_ST, maxCAy_ST_ind] = max(zeroMz_CAy_ST);
    
    steadyStateCAy.CAy = max(maxCAy_SA, maxCAy_ST);
    
    if max(maxCAy_SA, maxCAy_ST) == maxCAy_SA
        steadyStateCAy.SA = rad2deg(SA_CG(maxCAy_SA_ind));
        steadyStateCAy.dSteer = zeroMz_SteerDeg(maxCAy_SA_ind);
    else
        steadyStateCAy.SA = zeroMz_SADeg(maxCAy_ST_ind);
        steadyStateCAy.dSteer = rad2deg(dSteer(maxCAy_ST_ind));
    end

    steadyStateCAy.Mz = 0;
end

function absoluteCAy = getAbsoluteCAy(rawData, SA_CG, dSteer)

    [SA_CG, dSteer] = meshgrid(SA_CG, dSteer);
    dataPoints = [rawData.saveCAyVel(1:end);
                  SA_CG(1:end);
                  dSteer(1:end);
                  rawData.saveMzBody(1:end)];

    [absoluteCAy.CAy, idx] = max(dataPoints(1, :));
    absoluteCAy.SA = dataPoints(2, idx);
    absoluteCAy.dSteer = dataPoints(3, idx);
    absoluteCAy.Mz = dataPoints(4, idx);

end

function plotMMD(rawData, SA_CG, dSteer, V, SSFy)
    figure
    hold on
    grid on
    for i = 1:length(dSteer)
        steer = plot(rawData.saveCAyVel(i,:),rawData.saveMzBody(i,:), "Color", "blue", 'LineStyle','--');
        if dSteer(i) == 0
            plot_zeroSteer = plot(rawData.saveCAyVel(i,:),rawData.saveMzBody(i,:), "Color", "blue", 'LineStyle','-');
        end
    end
    
    for i = 1:length(SA_CG)
        slip = plot(rawData.saveCAyVel(:,i),rawData.saveMzBody(:,i), "Color", "red", 'LineStyle', '--');
        if SA_CG(i) == 0
            plot_zeroSA = plot(rawData.saveCAyVel(:,i),rawData.saveMzBody(:,i), "Color", "red", "LineStyle", '-');
        end
    end
    
    xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
    ylabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
    xlim([-2,2])
    ylim([-1,1])
    
    title({['Free Rolling MMD: Constant Velocity'] ...
           ['Velocity = ', num2str(V),' m/s'] ...
           ['Minimum Radius = ' num2str(V.^2./(SSFy.CAy.*9.81)) ' m']...
          },'Interpreter','latex')
    
    
    %%% Maximum Ay Plotting Stuff
    
    AyMaxSS = plot(SSFy.CAy, 0, "Marker", ".", "MarkerSize", 20, "Color","g");
    label = sprintf(['C0   = %.4g\n',...
                     'SA      = %.3g°\n',...
                     'Steer  = %.3g°'],...
                    SSFy.CAy, SSFy.SA, SSFy.dSteer);
    
    text(SSFy.CAy - 0.07, 0.20, label, "FontSize", 8, 'FontWeight','bold', 'Interpreter', 'latex');
    
    legend([steer, slip, AyMaxSS], ...
            {"Constant Steer", ...
             "Constant Slip", ...
             "$C_{Ay_{Max SS}}$"},...
             "Location","northeast",'Interpreter','latex')
end

function testPlots(rawData, SA_CG, dSteer)
    saveCAyVel = rawData.saveCAyVel;
    saveCAxVel = rawData.saveCAxVel;
    saveMzBody = rawData.saveMzBody;

    figure
    hold on
    grid on
    for i = 1:length(dSteer)
        steer = plot(saveCAyVel(i,:),saveCAxVel(i,:), "Color", "blue", 'LineStyle','--');
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
        %     % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
        % end
    
    end
    
    for i = 1:length(SA_CG)
        slip = plot(saveCAyVel(:,i),saveCAxVel(:,i), "Color", "red");
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
        %     % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
        % end
    end
    
    xlabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
    ylabel("Normalized Longitudinal Accleration $(C_{Ax})$",'Interpreter','latex')
    
    % Get sort the coordinate pairs
    points = [saveCAyVel(1:end); saveCAxVel(1:end)];
    [~, idx] = sort(points(1, :));
    points = points(:, idx);
    
    % scatter(points(1, :), points(2, :))
    
    % split into left and right parts
    [~, idx] = min(abs(points(1, :)));
    leftPart = points(:, 1:idx);
    rightPart = points(:, idx:end);
    
    [~, idx] = sort(leftPart(2, :));
    leftPart = leftPart(:, idx);
    
    [~, idx] = sort(rightPart(2, :));
    rightPart = rightPart(:, idx);
    
    leftBound = nan(2, 999);
    rightBound = nan(2, 999);
    
    AxRange = linspace(-3, 3, 100);
    for i = 1:100-1
        % find the points within the interval
        startIdx = find(leftPart(2, :) > AxRange(i), 1, 'first');
        endIdx   = find(leftPart(2, :) < AxRange(i + 1), 1, 'last');
        filteredLeftRange = leftPart(:, startIdx:endIdx);
    
        startIdx = find(rightPart(2, :) > AxRange(i), 1, 'first');
        endIdx   = find(rightPart(2, :) < AxRange(i + 1), 1, 'last');
        filteredRightRange = rightPart(:, startIdx:endIdx);
    
        % find the edge point
        [~, idx] = min(filteredLeftRange(1, :));
        if ~isempty(idx)
            leftBound(:, i) = filteredLeftRange(:, idx);
        end
    
        [~, idx] = max(filteredRightRange(1, :));
        if ~isempty(idx)
            rightBound(:, i) = filteredRightRange(:, idx);
        end
    end
    
    leftBound = leftBound(:, ~isnan(leftBound(1, :)));
    rightBound = rightBound(:, ~isnan(rightBound(1, :)));
    
    leftBound = unique(leftBound', 'rows')';
    rightBound = unique(rightBound', 'rows')';
    
    scatter(leftBound(1, :), leftBound(2, :))
    scatter(rightBound(1, :), rightBound(2, :))
    
    figure;
    hold on;

    [~, idx] = sort(leftBound(2, :), "ascend");
    leftBound = leftBound(:, idx);
    
    [~, idx] = sort(rightBound(2, :), "descend");
    rightBound = rightBound(:, idx);
    
    accelBound = [leftBound rightBound];
    plot(accelBound(1, :), accelBound(2, :))
    figure
    hold on
    grid on
    
    for i = 1:length(dSteer)
    
        steer = plot3(saveMzBody(i,:),saveCAyVel(i,:),saveCAxVel(i,:), "Color", "blue", 'LineStyle','--');
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg steer'];  % Dynamic label for clarity
        %     % text(saveCAyVel(i,end), saveMzBody(i,end), labelText)
        % end
    
    end
    
    for i = 1:length(SA_CG)
        slip = plot3(saveMzBody(:,i),saveCAyVel(:,i),saveCAxVel(:,i), "Color", "red", 'LineStyle','--');
        % if mod(i,7) == 0
        %     labelText = ['\leftarrow ',num2str(rad2deg(dSteer(i))), ' deg SA'];  % Dynamic label for clarity
        %     % text(saveCAyVel(1,i), saveMzBody(1,i), labelText)
        % end
    end
    
    zlim([-2, 2])
    
    view(3)
    ylabel("Normalized Lateral Acceleration $(C_{Ay})$",'Interpreter','latex')
    xlabel("Normalized Yaw Moment $(C_{Mz})$",'Interpreter','latex')
    zlabel("Normalized Longitudinal Accleration $(C_{Ax})$",'Interpreter','latex')

end