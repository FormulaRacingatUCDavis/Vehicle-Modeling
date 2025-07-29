function SR = calcSR(FxTarget, driveCondtion, B_FBB, Tire, SlipAngle, NormalLoad, Pressure, Inclination, Velocity, Idx, Model)
    % The "inverse" tire model for MMD with level surface
    SR = zeros(4, 1);
    tireSRCurve = cell(4, 1);
    minFx = zeros(4, 1);
    maxFx = zeros(4, 1);

    % Get the Fx-slipratio curves
    for p=1:4
        if driveCondtion && any(p==[1,2])
            continue
        end
        [tireSRCurve{p}, minFx(p), maxFx(p)] = getTireSRCurve(Tire, SlipAngle(p), NormalLoad(p), Pressure, Inclination, Velocity(p), Idx, Model);
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

function [tireSRCurve, minFx, maxFx] = getTireSRCurve(Tire, SlipAngle, NormalLoad, Pressure, Inclination, Velocity, Idx, Model)
    SlipRatio = linspace(-1, 1, 1000)';
    
    [Fx, ~, ~, ~, ~] = ContactPatchLoads( Tire, ...
        SlipAngle, SlipRatio, ...
        NormalLoad, Pressure, Inclination, Velocity, ...
        Idx, Model );

    Fx = Fx .* 0.7; % tire correction factor
    
    [maxFx, max_idx] = max(Fx);
    [minFx, min_idx] = min(Fx);
    newSR = SlipRatio(min_idx:max_idx);
    newFx = Fx(min_idx:max_idx);
    
    tireSRCurve = csapi(newFx, newSR);
end