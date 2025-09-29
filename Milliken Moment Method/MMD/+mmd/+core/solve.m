function result = solve(grid, carParams, mode, targetCAx, prevResult, models, config)
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
        targetCAx      (1 ,1) double = 0

        % previous result: result from previous run to accel iteration
        prevResult            double = []

        % models used in the simulation
        models.moterLimitFn     = @mmd.models.motorLimit_default
        models.weightTransferFn = @mmd.models.weightTransfer_default
        models.steeringModel    = @mmd.models.steeringModel_default

        % config: extra configureations
        config.tol     (1,1) double {mustBePositive} = 1e-3
        config.maxIter (1,1) double {mustBeInteger, mustBePositive} = 1000
        config.pr      (1,1) double {mustBeGreaterThanOrEqual(config.pr,0), mustBeLessThanOrEqual(config.pr,1)} = 0.0
        config.log     (1,1) logical = false
    end

    % initialize driving/braking conditions
    if mode == "level_surface"
        initRunResult = sovle(grid, carParams, "free_rolling", prevResult, models, config);
        grid.driveCondition = (ones(size(initRunResult.CAxVel)).* targetCAx - initRunResult.CAxVel) > 0;
    elseif mode == "drive"
        grid.driveCondition = ones(length(grid.dSteer), length(grid.SA_CG));
    elseif mode == "brake"
        grid.driveCondition = zeros(length(grid.dSteer), length(grid.SA_CG));
    elseif mode == "free_rolling"
        targetCAx = [];
    end

    %% Perform the iteration in parallel

    % initialize mats
    AyBody = zeros(length(grid.dSteer), length(grid.SA_CG));
    AxBody = zeros(length(grid.dSteer), length(grid.SA_CG));

    if ~isempty(prevResult)
        AyBodyinit = prevResult.AyBody;
        AxBodyinit = prevResult.AxBody;
    else
        AyBodyinit = zeros(length(grid.dSteer), length(grid.SA_CG));
        AxBodyinit = zeros(length(grid.dSteer), length(grid.SA_CG));
    end

    % This is for the parallel for loop. It prevents the grid being 
    % copied to all the workers and cost extra memory
    constGrid = parallel.pool.Constant(grid);
    AyBodyinit = parallel.pool.Constant(AyBodyinit);
    AxBodyinit = parallel.pool.Constant(AxBodyinit);
    V = grid.V;

    for i = 1:length(grid.dSteer)

        rowAxBody = zeros(1, length(constGrid.Value.dSteer));
        rowAyBody = zeros(1, length(constGrid.Value.dSteer));
        rowAxVel  = zeros(1, length(constGrid.Value.dSteer));
        rowAyVel  = zeros(1, length(constGrid.Value.dSteer));
        rowMzBody = zeros(1, length(constGrid.Value.dSteer));
        rowOmega  = zeros(1, length(constGrid.Value.dSteer));

        for j = 1:length(constGrid.Value.dSteer)

            cellResult = iterateOneCell(carParams, constGrid.Value.dSteer(i), constGrid.Value.SA_CG(j), V, constGrid.Value.driveCondition(i, j), targetCAx, models, AxBodyinit(i, j), AyBodyinit(i, j));
    
            rowAxBody(j) = cellResult(1);
            rowAyBody(j) = cellResult(2);
            rowAxVel(j)  = cellResult(3);
            rowAyVel(j)  = cellResult(4);
            rowMzBody(j) = cellResult(5);
            rowOmega(j)  = cellResult(6);
        end

        AxBody(i, :) = rowAxBody;
        AyBody(i, :) = rowAyBody;
    end

    result.AxBody = AxBody;
    result.AyBody = AyBody;
end