function Fx_max = TireEvalModel(SlipRatio, NormalLoad_Fz, Velocity)
%#codegen

persistent Tire Pressure Inclination Model Idx

if isempty(Tire)
    params = coder.load('TireModelParams.mat');
    Tire        = params.Tire;
    Pressure    = params.Pressure;
    Inclination = params.Inclination;
    Model       = params.Model;
    Idx         = 1; % Assume rear-wheel drive (single axle index)
end

% Since this is launch (straight line), slip angle = 0
SlipAngle = 0;

% Call the ContactPatchLoads function
[Fx, ~, ~, ~, ~] = ContactPatchLoads(Tire, ...
                                     SlipAngle, ...
                                     SlipRatio, ...
                                     NormalLoad_Fz, ...
                                     Pressure, ...
                                     Inclination, ...
                                     Velocity, ...
                                     Idx, ...
                                     Model);

% Output max longitudinal traction force
Fx_max = Fx;

end
