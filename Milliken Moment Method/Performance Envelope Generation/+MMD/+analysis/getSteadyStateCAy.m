function steadyStateCAy = getSteadyStateCAy(result, targetCAx)
    SA_CG = result.grid.SA_CG;
    dSteer = result.grid.dSteer;
    zeroMz_CAy_SA = zeros(length(SA_CG),1);
    zeroMz_CAy_ST = zeros(length(dSteer),1);
    zeroMz_SteerDeg = zeros(length(SA_CG),1);
    zeroMz_SADeg = zeros(length(dSteer),1);

    MzBody = result.MzBody;
    CAyVel = result.CAyVel;
    CAxVel = result.CAxVel;
    
    % Go Through the Mz Data through SlipAngle(columns) to check the SA lines
    % ---- CAUTIONS: This only finds the first point along the line that
    % crosses the zero boundary
    % Also interpolates for the other angle values not used (SA or Steer)
    for j = 1:length(SA_CG)
        checkMat = MzBody(2:end,j).*MzBody(1:end-1,j);
        if exist('targetCAx','var')
            avgAx = (CAxVel(2:end, j) + CAxVel(1:end-1, j)) ./ 2;
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
            % [minMz,indexMin] = min(abs(MzBody(:,j)));
            % zeroMz_CAy_SA(j) = CAyVel(indexMin,j);
            % zeroMz_Steer(j) = dSteer(indexMin);
    
            zeroMz_CAy_SA(j) = 0;
            zeroMz_SteerDeg(j) = 0;
        else
            zeroMz_CAy_SA(j) = interp1( MzBody(indexSS:indexSS+1, j),...
                                        CAyVel(indexSS:indexSS+1, j), 0.0, 'linear');
            zeroMz_SteerDeg(j) = rad2deg(interp1( MzBody(indexSS:indexSS+1, j),...
                                       dSteer(indexSS:indexSS+1), 0.0, 'linear'));
        end
    end
    
    % Go Through the Mz Data through Steer Angle(rows) to check the steer lines
    % ---- CAUTIONS: This only finds the first point along the line that
    % crosses the zero boundary
    for i = 1:length(dSteer)
        checkMat = MzBody(i, 2:end).*MzBody(i, 1:end-1);
        if exist('targetCAx','var')
            avgAx = (CAxVel(i, 2:end) + CAxVel(i, 1:end-1)) ./ 2;
            indexSS = abs(avgAx - targetCAx) < 5e-2 & checkMat < 0;
        else
            indexSS = checkMat < 0;
        end
        indexSS = find(indexSS, 1, 'first');
        if isempty(indexSS)
            zeroMz_CAy_ST(i) = 0;
            zeroMz_SADeg(i) = 0;
        else
            zeroMz_CAy_ST(i) = interp1( MzBody(i, indexSS:indexSS+1),...
                                        CAyVel(i, indexSS:indexSS+1), 0.0, 'linear');
            zeroMz_SADeg(i) = rad2deg(interp1( MzBody(i, indexSS:indexSS+1),...
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
