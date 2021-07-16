function [BrakingTorque, LinePressure] = SimplifiedBrakingModel( PedalForce, ...
    PedalRatio, BalanceBar  , BoreDiameter, ...
    PadArea   , PadFriction , RotorRadius )
%% SimplifiedBrakingModel - Braking Pressures and Torques
% Computes brake line pressure and braking torques assuming rigid braking
% components.
% 
% Inputs:
%   PedalForce   - (n,1 numeric) Pedal Force                   {F_p}   [N]
%   PedalRatio   - (n,1 numeric) Pedal Ratio                   {eta_p} [ ]
%   BalanceBar   - (n,1 numeric) Front Balance Bar Ratio       {%_bb}  [ ]
%   BoreDiameter - (n,2 numeric) Master Cylinder Bore Diameter {D_c}   [m]
%   PadArea      - (n,2 numeric) Brake Pad Area                {A_p}   [m^2]
%   PadFriction  - (n,2 numeric) Brake Pad Friction            {mu_p}  [ ]
%   RotorRadius  - (n,2 numeric) Brake Rotor Radius            {r_r}   [m]
%
% Outputs:
%   BrakingTorque - (n,4 numeric) Braking Torque      {tau_b} [N-m]
%   LinePressure  - (n,2 numeric) Brake Line Pressure {P_b}   [Pa]
%
% Notes:
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% 
% Last Updated: 30-May-2021

%% Test Case
if nargin == 0
    PedalForce = 150;
    
    PedalRatio = 4.3;
    BalanceBar = 0.48;
    
    BoreDiameter = [0.59, 0.8125] .* 0.0254;
    
    PadArea = [2.9, 1.45] * 0.0254^2;
    PadFriction = [0.55, 0.55];
    
    RotorRadius = [3.3, 3.3] *0.0254;
    
    [BrakingTorque, LinePressure] = SimplifiedBrakingModel( PedalForce, ...
        PedalRatio, BalanceBar  , BoreDiameter, ...
        PadArea   , PadFriction , RotorRadius ) %#ok<NOPRT>
    
    return
end

%% Computation
CylinderForce = PedalForce .* PedalRatio .* [BalanceBar, 1-BalanceBar];

LinePressure = CylinderForce ./ (pi/4 .* BoreDiameter.^2);

BrakingTorque = LinePressure .* PadArea .* PadFriction .* RotorRadius / 2;
BrakingTorque = [BrakingTorque(:,1).*ones(1,2), BrakingTorque(:,2).*ones(1,2)];

end

