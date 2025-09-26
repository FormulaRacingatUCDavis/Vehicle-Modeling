classdef MMD
    % Class for Milliken Moment Method
    
    properties (SetAccess = immutable)
        carParams (1,1) struct
        models    (1,1) struct
        config    (1,1) struct
    end
    
    methods
        function obj = MMD(carParams, models, config)
            % Construct a MMD instance

            arguments
                carParams (1, 1) struct

                % models used in the simulation
                models.moterLimitFn     = @mmd.models.motorLimit_default
                models.weightTransferFn = @mmd.models.weightTransfer_default

                % extra config
                config.tol     (1,1) double {mustBePositive} = 1e-3
                config.maxIter (1,1) double {mustBeInteger, mustBePositive} = 1000
                config.pr      (1,1) double {mustBeGreaterThanOrEqual(config.pr,0), mustBeLessThanOrEqual(config.pr,1)} = 0.0
                config.log     (1,1) logical = false
            end
            obj.carParams = carParams;
            obj.models    = models;
            obj.config    = config;
        end
        
        function result = evaluate(obj, grid, mode, targetCAx)
            % Evaluate MMD
        end
    end
end

