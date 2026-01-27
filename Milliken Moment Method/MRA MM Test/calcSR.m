function SR = calcSR(carParams, FxTarget, driveCondtion, SlipAngle, NormalLoad, Inclination, Velocity)
    % The "inverse" tire model for MMD with level surface
    SR = zeros(4, 1);
    tireSRCurve = cell(4, 1);
    minFx = zeros(4, 1);
    maxFx = zeros(4, 1);
    B_FBB = carParams.B_FBB;


    % Get the Fx-slipratio curves
    for p=1:4
        if driveCondtion && any(p==[1,2])
            continue
        end
        [tireSRCurve{p}, minFx(p), maxFx(p)] = getTireSRCurve(carParams, SlipAngle(p), NormalLoad(p), Inclination, Velocity(p));
    end

    % Determine if tires are saturated and reset FxTarget
    if driveCondtion
        % For drive condition, only consider the rear wheels
        tireSaturated(1:2) = false;
        tireSaturated(3:4) = FxTarget(3:4) > maxFx(3:4);

        if any(tireSaturated)
            % Find the most saturated tire and set Reset FxTargets
            [FxLimit, ~] = min(maxFx(tireSaturated));
            FxTarget(3:4) = FxLimit;
        end

    else
        % brake condition
        tireSaturated = FxTarget < minFx;

        if any(tireSaturated)
            % Rescale minFx to find the most saturated tire
            rescaledFxTarget = [minFx(1:2) ./ B_FBB, minFx(3:4)];
            [FxLimit, tireIdx] = max(rescaledFxTarget(tireSaturated));
    
            if tireIdx <= 2
                % It's the front tires
                FxTarget(1:2) = FxLimit * B_FBB;
                FxTarget(3:4) = FxLimit;
            else
                % Rear tires
                FxTarget(1:2) = FxLimit;
                FxTarget(3:4) = FxLimit * B_FBB;
            end
        end
    end

    % Calculate the slip ratios
    for p = 1:4
        if driveCondtion && any(p==[1,2])
            continue
        end

        SR(p) = fnval(tireSRCurve{p}, FxTarget(p));
    end

    % close all;
    % for p=1:4
    %     figure;
    %     hold on;
    %     fnplt(tireSRCurve{p})
    %     scatter(FxTarget(p), SR(p))
    % end
end

function [tireSRCurve, minFx, maxFx] = getTireSRCurve(carParams, SlipAngle, NormalLoad, Inclination, Velocity)
    SlipRatio = linspace(-1, 1, 1000)';

    Tire = carParams.tire.Tire;
    Pressure = carParams.tire.TirePressure;
    Idx = carParams.tire.Idx;
    Model = carParams.tire.Model;
    
    [Fx, ~, ~, ~, ~] = ContactPatchLoads( Tire, ...
        SlipAngle, SlipRatio, ...
        NormalLoad, Pressure, Inclination, Velocity, ...
        Idx, Model );

    Fx = Fx .* carParams.tire.CorrectionFactor; % tire correction factor
    
    [maxFx, max_idx] = max(Fx);
    [minFx, min_idx] = min(Fx);

    if min_idx > max_idx
        temp = max_idx;
        max_idx = min_idx;
        min_idx = temp;
    end

    if max_idx - min_idx <= 2
        newFx = [Fx(1) Fx(1)+1 Fx(1)+2];
        newSR = [-1 0 1];
    else
        newSR = SlipRatio(min_idx:max_idx);
        newFx = Fx(min_idx:max_idx);
    end
    
    tireSRCurve = csapi(newFx, newSR);
end