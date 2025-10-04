classdef MMD
    % Class for Milliken Moment Method
    
    properties (SetAccess = immutable)
        carParams (1,1) struct
        models    (1,1) struct
        config    (1,1) struct
    end
    
    methods
        function obj = MMD(carParams, models, config)
            arguments
                carParams 
                models = struct()
                config = struct()
            end
            % Construct a MMD instance
            obj.carParams = carParams;
            obj.models    = models;
            obj.config    = config;
        end
        
        function result = evaluate(obj, grid, mode, targetCAx, prevResult)
            arguments
                obj 
                grid 
                mode 
                targetCAx        double = 0
                prevResult       struct = []
            end
            % Evaluate MMD
            result = mmd.core.solve(grid, obj.carParams, mode, targetCAx, prevResult, obj.models, obj.config);
        end
    end
end

