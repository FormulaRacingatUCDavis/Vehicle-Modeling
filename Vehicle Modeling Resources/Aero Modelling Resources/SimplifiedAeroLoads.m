function [Drag, Downforce] = SimplifiedAeroLoads( LongVel, ...
    AirDensity, RefArea, DragCoeff, LiftCoeff )
%% SimplifiedWeightTransfer - Simplified Normal Loads
% Computes tire normal loads using simplified weight transfer and
% aerodynamic loading.
% 
% Inputs:
%   LongVel    - (n,1 numeric) Longitudinal Velocity {dot{x}}  [m/s]
%   AirDensity - (n,1 numeric) Air Density           {rho_air} [kg/m^3]
%   RefArea    - (n,1 numeric) Reference Area        {A}       [m^2]
%   DragCoeff  - (n,1 numeric) Drag Coefficient      {C_D}     [ ]
%   LiftCoeff  - (n,1 numeric) Lift Coefficient      {C_L}     [ ]
%
% Outputs:
%   Drag       - (n,1 numeric) Drag Force            {A_F_x}   [N]
%   Downforce  - (n,1 numeric) Downforce             {A_F_z}   [N]
%
% Notes:
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% 
% Last Updated: 31-May-2021

%% Test Case
if nargin == 0
   LongVel = 15;
   
   AirDensity = 1.225;
   
   RefArea = 1.1;
   
   DragCoeff = 0.6;
   LiftCoeff = -1.9;
   
   [Drag, Downforce] = SimplifiedAeroLoads( LongVel, ...
    AirDensity, RefArea, DragCoeff, LiftCoeff ) %#ok<NOPRT>

    return
end

%% Computation
Drag      = 1/2 .* AirDensity .*  DragCoeff .* RefArea .* LongVel.^2;
Downforce = 1/2 .* AirDensity .* -LiftCoeff .* RefArea .* LongVel.^2;

end

