function [cay_max] = mmd_funct(vParam, tParam, SA_CG, dSteer, R, debug)
%MMD_FUNCT Summary of this function goes here
%   Function form of the MMD
%   Need to load Tire Data & Tire Modeling into Workspace

%   Input Variables:
%   vParam: Structure with fields:
%       m:      Total Mass              (m)
%       PFront: Percent mass in front   (0-1)
%       WB:     Wheelbase               (kg)
%       TWf:    Trackwidth front        (m)
%       TWr:    Trackwidth rear         (m)
%       toe_f:  Toe angle front         (deg) (Positive inwards)
%       toe_r:  Toe angle rear          (deg)
%       hCG:    CG height               (m) 

%   tParam: Structure with fields
%       Idx:        Moment of inertia in x for wheel    (m^4)
%       TirePress:  Tire pressure                       (kPa)
%       TireIncl:   Tire inclination                    (deg)
%       TireSR:     -                                   (-)        

%   SA_CG:  Row array of body slip angles               (deg)
%   dSteer: Row array of steering angles                (deg)
%   NOTE:   SA_CG & dSteer need to be same dimensions

%   R: Turning radius   (m)

%   debug: boolean for debugging
%       True will plot MMD and print convergence messages
%       Only useful if you break after running mmd_funct

%   Output Variables:
%   cay_max:    The maximum cay from the mmd    (-)

%   RIGHT HAND SIDE IS POSITIVE Y - RCVD Standard   
    
    % Constants
    g = 9.81;                           % Grav constant [m/s^2]
    m = vParam.m;                       % Total Mass [kg]
    PFront = vParam.PFront;             % Percent Mass Front [0-1]
    WB = vParam.WB;                     % Wheelbase [m]
    TWf = vParam.TWf;                   % Trackwidth [m]
    TWr = vParam.TWr;
    toe_f = vParam.toe_f * (pi/180);    % Toe Angles [radians] (positive is inwards)
    toe_r = vParam.toe_r * (pi/180);
    hCG = vParam.hCG;                   % CG height [m]
    
    % Tire Model Parameters
    Idx = tParam.Idx;                   % Moment of Inertia in x for wheel
    TirePressure = tParam.TirePress;    % kPa
    TireInclination = tParam.TireIncl;  % deg 
    TireSR = tParam.TireSR;             % -
    Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
    load('Hoosier_R25B_16x75-10x7.mat');
    
    % SELECT RANGES FOR BODY SLIP AND STEERING ANGLES
    SA_CG = deg2rad(SA_CG)';
    dSteer = deg2rad(dSteer)';
    
    dSteer_W1 = toe_f + dSteer;
    dSteer_W2 = -toe_f + dSteer;
    dSteer_W3 = toe_r + dSteer.*0;
    dSteer_W4 = -toe_r + dSteer.*0;
    dSteer_AllW = [dSteer_W1, dSteer_W2, dSteer_W3, dSteer_W4];
    
    lf = WB * (1-PFront);
    lr = WB * (PFront);
    coord_W1 = [lf ; -TWf/2];
    coord_W2 = [lf ; TWf/2];
    coord_W3 = [-lr ; -TWr/2];
    coord_W4 = [-lr ; TWr/2];
    coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];
    
    % FREE ROLLING MMD SECTION
    
    % Mat Initialization
    SA_Wheel = zeros(4,1);
    V_Wheel = zeros(4,1);
    TM_Fx = zeros(4,1);
    TM_Fy = zeros(4,1);
    FxTire = zeros(4,1);
    FyTire = zeros(4,1);
    MzTire = zeros(4,1);
    saveAyBody = zeros(length(dSteer), length(SA_CG));
    saveAxBody = zeros(length(dSteer), length(SA_CG));
    saveMzBody = zeros(length(dSteer), length(SA_CG));
    saveItFz = [];
    saveTM_Fy = zeros(4,length(SA_CG));
    
    
    for i = 1:length(dSteer)
    
        for j = 1:length(SA_CG)
    
            AxGuess = 0;
            AyGuess = 0;
            res = 1;
            tol = 1e-3;
    
            itAyBody = [];
            itAyBody(1,1) = AyGuess;
            itAxBody = [];
            itAxBody(1,1) = AxGuess;
            itCAyBody = [];
            itCAyBody(1,1) = AyGuess/g;
            itFz = [];
            itFyTire = [];
    
            c = 1;
        
            while abs(res) > tol && c < 1000
                if c == 1
                    AyCurr = itAyBody(c);
                else
                    AyCurr = itAyBody(c);
                end
    
                AxCurr = itAxBody(c);
                V = sqrt(abs(AyCurr).* R); 
                Omega = V/R;
                
                % Free Rolling MMD Assumption
                Omega = 0;
    
                for p = 1:4
                    
                    % SA_Wheel(p,1) = atan( (V.* sin(SA_CG(j)) + Omega .* coord_AllW(1,p)) ...
                    %                     / (V.* cos(SA_CG(j)) - Omega .* coord_AllW(2,p)) ) - dSteer_AllW(i,p);
                    % SA_Wheel(p,1) = atan( (R.*sin(SA_CG(j)) + coord_AllW(1,p)) / (R.*cos(SA_CG(j)) - coord_AllW(2,p)) )...
                    %                 - dSteer_AllW(i,p);
    
                    % Free Rolling MMD Assumption
                    SA_Wheel(p,1) = SA_CG(j) - dSteer_AllW(i,p);
                
                    V_Wheel(p,1) = V + sqrt(  (-Omega.*coord_AllW(2,p)).^2  +  (Omega.*coord_AllW(1,p)).^2  );
                end
    
                % Free Rolling MMD Assumption
                V_Wheel = V_Wheel .* 0;
                
                dFzf_dAx = (hCG .* m)./(2.* WB);
                dFzf_dAy = 1*(hCG .* m .* PFront)/TWf;
                dFzr_dAy = 1*(hCG .* m .* (1-PFront))/TWr;
                
                Fz_Wf = (m.*g.* PFront)/2;
                Fz_Wr = (m.*g.* (1-PFront))/2;
                    
                Fz(1,1) = Fz_Wf + dFzf_dAx .* AxCurr + dFzf_dAy .* AyCurr;
                Fz(2,1) = Fz_Wf + dFzf_dAx .* AxCurr - dFzf_dAy .* AyCurr;
                Fz(3,1) = Fz_Wr - dFzf_dAx .* AxCurr + dFzr_dAy .* AyCurr;
                Fz(4,1) = Fz_Wr - dFzf_dAx .* AxCurr - dFzr_dAy .* AyCurr;
                
                for p = 1:4
                    [TM_Fx(p,1), TM_Fy(p,1), ~, ~, ~] = ContactPatchLoads(Tire, rad2deg(SA_Wheel(p)), TireSR, Fz(p) , TirePressure , TireInclination, V_Wheel(p), Idx, Model);
                    
                    % Free Rolling MMD Assumption
                    TM_Fx(p) = TM_Fx(p) .* 0; 
                    
                    % Calspan TTC Data usual correction factor - 0.7
                    TM_Fx(p) = TM_Fx(p) .* 0.7;
                    % Tire Model outputs in opposite Y coordinates
                    TM_Fy(p) = (-1).* TM_Fy(p) .* 0.7;
                    
                    FxTire(p,1) = TM_Fx(p) .* cos(dSteer_AllW(i,p)) - TM_Fy(p) .* sin(dSteer_AllW(i,p));
                    FyTire(p,1) = 1*TM_Fx(p) .* sin(dSteer_AllW(i,p)) + TM_Fy(p) .* cos(dSteer_AllW(i,p));
                    
                    % Made it a matrix sum so its not ugly :)
                    MzTire(p,1) = sum( [coord_AllW(1,p) .* TM_Fx(p) .* sin(dSteer_AllW(i,p)) ;
                                      - coord_AllW(2,p) .* TM_Fx(p) .* cos(dSteer_AllW(i,p)) ; 
                                        coord_AllW(2,p) .* TM_Fy(p) .* sin(dSteer_AllW(i,p)) ;
                                        coord_AllW(1,p) .* TM_Fy(p) .* cos(dSteer_AllW(i,p))  ] ); 
                end
                
                FxBody = sum(FxTire);
                FyBody = sum(FyTire);
                MzBody = sum(MzTire);
                
                itFz(:,c) = Fz;
                itFyTire(:,c) =  FyTire;
                itAyBody(c+1,1) = FyBody/m;
                itAxBody(c+1,1) = FxBody/m;
                itCAyBody(c+1,1) = itAyBody(c+1,1)/g;
                res = itCAyBody(c+1) - itCAyBody(c);
        
                % % Tests the weight transfer eqn
                % plot(1:c, itFz);
                % legend(["FzW1" "FzW2" "FzW3" "FzW4"])
                
                % Tests the c 
                plot(1:c+1, itAyBody);
                
                c = c + 1;
        
                if c > 499
                    iterationCtrl = itAyBody( (c-10) : (c-1) );
                    itAyBody(c) = min(iterationCtrl);
                    break
                end
                    
            end 
            saveTM_Fy(:,j) = TM_Fy;
            saveSA_Wheel(:,j) = SA_Wheel;
            saveFz(:,j) = Fz;
            saveAyBody(i,j) = itAyBody(end);
            saveAxBody(i,j) = itAxBody(end);
            saveMzBody(i,j) = MzBody/(m.*g.*WB);
            saveItFz(:,j) = Fz;
            saveItAyBody{j,1} = itAyBody;
            
            if debug
                disp("Steering Angle [" + i + "] Slip Angle [" + j + "] finished, iterations: " +  c)
            end
            
        end % SA_CG End
    
    end % dSteer End
    
    % Processing results
    saveAxVel = zeros(size(saveAyBody));
    saveAyVel = zeros(size(saveAyBody));
    saveCAyVel = zeros(size(saveAyBody));
    
    for i = 1:length(SA_CG)
        saveAxVel(:,i) = saveAxBody(:,i) .* cos(SA_CG) + saveAyBody(:,i) .* sin(SA_CG);
        saveAyVel(:,i) = saveAyBody(:,i) .* cos(SA_CG) - saveAxBody(:,i) .* sin(SA_CG);
        saveCAyVel(:,i) = saveAyVel(:,i)/g;
    end

    % FINDING HIGHEST STEADY STATE AY
    zeroMz_CAy = zeros(1,length(SA_CG));
    
    % Find Change Pts of Mz from pos to negative for each SA_CG
    for j = 1:length(SA_CG)
        for i = 1:length(dSteer)
            diff = saveMzBody(i,j) - saveMzBody(i+1,j);
            if diff > saveMzBody(i,j)
                indexSS = i;
                break
            end
        end
        indexes(j) = indexSS;
        slope = (saveMzBody(indexSS+1,j) - saveMzBody(indexSS,j)) / (saveCAyVel(indexSS+1,j) - saveCAyVel(indexSS,j));
        b = saveMzBody(indexSS,j) - slope*saveCAyVel(indexSS,j);
        zeroMz_CAy(j) = -b/slope;
    end

    % Stores OUTPUT
    cay_max = max(zeroMz_CAy);

    % Plot if debugging
    if debug
        figure
        hold on
        grid on
        for i = 1:length(dSteer)
            steer = plot(saveCAyVel(i,:),saveMzBody(i,:), "Color", "blue", 'LineStyle','--');
        end
        
        for i = 1:length(SA_CG)
            slip = plot(saveCAyVel(:,i),saveMzBody(:,i), "Color", "red");
        end
        
        xlabel("Normalized Lateral Acceleration (C_{Ay})")
        ylabel("Normalized Yaw Moment (C_{Mz})")
        xlim([-2,2])
        ylim([-1,1])
        
        if PFront > 0.5
            resp = 'Understeer' ;
        elseif PFront < 0.5
            resp = 'Oversteer';
        else
            resp = 'Neutral Steer';
        end
        
        title({ ...
               ['SA: [' num2str(min(rad2deg(SA_CG))) ',' num2str(max(rad2deg(SA_CG))) '] & Steering: [' num2str(min(rad2deg(dSteer))) ',' num2str(max(rad2deg(dSteer))) ']'], ...
               ['Expected Response: '  resp] ...
              });
        
        
        AyMaxSS = plot(max(zeroMz_CAy), 0, "Marker", ".", "MarkerSize", 20);
        text(max(zeroMz_CAy), 0, {'','','','','C_{Ay_0} =', num2str(max(zeroMz_CAy))}, "FontSize",7 );
        
        legend([steer, slip,AyMaxSS],{"Constant Steer", "Constant Slip", "C_{Ay_{Max SS}}"}, "Location","northeast")
    end

end

