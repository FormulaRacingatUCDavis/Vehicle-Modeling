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
        targetCAx      (1, 1) double = 0

        % previous result: result from previous run to accel iteration
        prevResult            struct = []

        % models used in the simulation
        models         (1, 1) struct = struct()

        % more configurations
        config         (1, 1) struct = struct()
    end

    if ~isfield(models,'motorLimitFn'),     models.motorLimitFn     = @mmd.models.motorLimit_default; end
    if ~isfield(models,'weightTransferFn'), models.weightTransferFn = @mmd.models.weightTransfer_default; end
    if ~isfield(models,'steeringModel'),    models.steeringModel    = @mmd.models.steeringModel_default; end
    if ~isfield(config,'tol'),     config.tol = 1e-3; end
    if ~isfield(config,'maxIter'), config.maxIter = 1000; end
    if ~isfield(config,'pr'),      config.pr = 0.0; end
    if ~isfield(config,'log'),     config.log = false; end

    % initialize driving/braking conditions
    if mode == "level_surface"
        prevResult = mmd.core.solve(grid, carParams, "free_rolling", 0, prevResult, models, config);
        grid.driveCondition = (ones(size(prevResult.CAxVel)).* targetCAx - prevResult.CAxVel) > 0;
    elseif mode == "drive"
        grid.driveCondition = ones(length(grid.dSteer), length(grid.SA_CG));
    elseif mode == "brake"
        grid.driveCondition = zeros(length(grid.dSteer), length(grid.SA_CG));
    elseif mode == "free_rolling"
        targetCAx = [];
        grid.driveCondition = zeros(length(grid.dSteer), length(grid.SA_CG)); % just a place holder here
    end

    % Wheel coordinates calculation
    carParams.coord_AllW = calcWheelCoords(carParams);

    %% Perform the iteration in parallel

    % initialize mats
    CAyBody = zeros(length(grid.dSteer), length(grid.SA_CG));
    CAxBody = zeros(length(grid.dSteer), length(grid.SA_CG));
    CAxVel  = zeros(length(grid.dSteer), length(grid.SA_CG));
    CAyVel  = zeros(length(grid.dSteer), length(grid.SA_CG));
    MzBody = zeros(length(grid.dSteer), length(grid.SA_CG));
    Omega  = zeros(length(grid.dSteer), length(grid.SA_CG));

    if ~isempty(prevResult)
        AyBodyinit = prevResult.CAyBody .* 9.81;
        AxBodyinit = prevResult.CAxBody .* 9.81;
    else
        AyBodyinit = zeros(length(grid.dSteer), length(grid.SA_CG));
        AxBodyinit = zeros(length(grid.dSteer), length(grid.SA_CG));
    end

    grid.SA_CG = deg2rad(grid.SA_CG);
    grid.dSteer = deg2rad(grid.dSteer);

    % This is for the parallel for loop. It prevents the grid being 
    % copied to all the workers and cost extra memory
    constGrid = parallel.pool.Constant(grid);
    constAyBodyinit = parallel.pool.Constant(AyBodyinit);
    constAxBodyinit = parallel.pool.Constant(AxBodyinit);
    V = grid.V;

    parfor i = 1:length(constGrid.Value.dSteer)

        dSteer = constGrid.Value.dSteer;
        SA_CG = constGrid.Value.SA_CG;


        rowCAxBody = zeros(1, length(SA_CG));
        rowCAyBody = zeros(1, length(SA_CG));
        rowCAxVel  = zeros(1, length(SA_CG));
        rowCAyVel  = zeros(1, length(SA_CG));
        rowMzBody = zeros(1, length(SA_CG));
        rowOmega  = zeros(1, length(SA_CG));

        for j = 1:length(SA_CG)

            [
                rowCAxBody(j),           ...
                rowCAyBody(j),           ...
                rowCAxVel(j),            ...
                rowCAyVel(j),            ...
                rowMzBody(j),           ...
                rowOmega(j)             ...
            ] = mmd.core.iterateOneCell ...
            (                           ...
                carParams,              ...
                dSteer(i),              ...
                SA_CG(j),               ...
                V,                      ...
                constGrid.Value.driveCondition(i, j), ...
                targetCAx,              ...
                models,                 ...
                constAxBodyinit.Value(i, j), ...
                constAyBodyinit.Value(i, j), ...
                config                  ...
            );

        end

        CAxBody(i, :) = rowCAxBody;
        CAyBody(i, :) = rowCAyBody;
        CAxVel(i, :)  = rowCAxVel;
        CAyVel(i, :)  = rowCAyVel;
        MzBody(i, :) = rowMzBody;
        Omega(i, :)  = rowOmega;
    end

    result.CAxBody = CAxBody;
    result.CAyBody = CAyBody;
    result.CAxVel  = CAxVel;
    result.CAyVel  = CAyVel;
    result.MzBody = MzBody;
    result.Omega  = Omega;
    result.grid   = grid;
end

function coord_AllW = calcWheelCoords(carParams)
    lf = carParams.WB * (1-carParams.PFront);
    lr = carParams.WB * (carParams.PFront);
    coord_W1 = [lf ; carParams.TWf/2];
    coord_W2 = [lf ; -carParams.TWf/2];
    coord_W3 = [-lr ; carParams.TWr/2];
    coord_W4 = [-lr ; -carParams.TWr/2];
    coord_AllW = [coord_W1, coord_W2, coord_W3, coord_W4];
end