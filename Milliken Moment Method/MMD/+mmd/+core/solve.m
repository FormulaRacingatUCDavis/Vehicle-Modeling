function result = solve(grid, carParams, mode, targetCAx, preResult, models, config)
    % Run the iteration to generate the MMD results

    % RIGHT HAND SIDE ISararar POSITIVE Y - RCVD Standard (but lots of industry
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

    arguments
        % grid: 
        grid struct

        % car parameters
        carParams      (1, 1) struct

        % mode: running mode
        mode           (1, 1) string
        targetCAx      (1 ,1) double = []

        % previous result: result from previous run to accel iteration
        preResult             double = []

        % models used in the simulation
        models.moterLimitFn     = @mmd.models.motorLimit_default
        models.weightTransferFn = @mmd.models.weightTransfer_default

        % config: extra configureations
        config.tol     (1,1) double {mustBePositive} = 1e-3
        config.maxIter (1,1) double {mustBeInteger, mustBePositive} = 1000
        config.pr      (1,1) double {mustBeGreaterThanOrEqual(config.pr,0), mustBeLessThanOrEqual(config.pr,1)} = 0.0
        config.log     (1,1) logical = false
    end

    % initialize driving/braking conditions
    if mode == "level_surface"
        initRunResult = sovle(grid, carParams, "free_rolling", preResult, models, config);
        grid.driveCondition = (ones(size(initRunResult.CAxVel)).* targetCAx - initRunResult.CAxVel) > 0;
    elseif mode == "drive"
        grid.driveCondition = ones(length(grid.SA_CG), length(grid.dSteer));
    elseif mode == "brake"
        grid.driveCondition = ones(length(grid.SA_CG), length(grid.dSteer));
    end

    % initialize variables for the parallel for loop
    % this is because it is originally a nested for loop
    [ii, jj] = meshgrid(1:length(grid.SA_CG), 1:length(grid.dSteer));

    parfor p = 1:length(ii)
        i = ii(p);
        j = jj(p);

        cellResult = iterateOneCell(carParams, SA_CG(i), dSteer(j), V, driveBrakeCon)

        
    end



end

function [] = iterateOneCell(carParams, SA_CG, dSteer, V, driveBrakeCon)
end