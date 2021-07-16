function [GlobLongVel, GlobLatVel, Curvature, BodySlipAngle] = ...
    BicycleModelWCS( Yaw, LatAcc, LatVel, LongAcc, LongVel )

%% WCS - World Coordinate System Calculations
% 
% Inputs:
%  Yaw     - (n,1 numeric) Yaw                       {psi} [rad]
%  LatAcc  - (n,1 numeric) Lateral Acceleration      {y_ddot} [m/s^2]
%  LatVel  - (n,1 numeric) Lateral Velocity          {y_dot} [m/s]
%  LongAcc - (n,1 numeric) Longitudinal Acceleration {x_ddot} [m/s^2]
%  LongVel - (n,1 numeric) Longitudinal Velocity     {x_dot} [m/s]
% 
% Outputs:
%  GlobLongVel   - (n,1 numeric) Global Longitudial Velocity {X_dot} [m/s]
%  GlobLatVel    - (n,1 numeric) Golbal Lateral Velocity     {Y_dot} [m/s]
%  Curvature     - (n,1 numeric) Pedal Force                 {R} [m]
%  BodySlipAngle - (n,1 numeric) Body Slip Angle             {beta} [degrees]
%  
% Notes:
% Naming convention for R still not in place
%
% Author(s): 
% Tristan Pham (atlpham@ucdavis.edu) [Sep 2020 - Jun 2021] 

% Last Updated: 27-Mar-2021


%% Test Cases
if nargin == 0
    %%% Test Inputs
    Yaw = ; 
    
    LatAcc = ; 
    LatVel = ; 
    
    LongAcc = ;
    LongVel = ;
    
  
    
    fprintf('Executing World Coordinate System() Test Cases: \n');
    
    
    [GlobLongVel, GlobLatVel, Curvature, BodySlipAngle] = BicycleModelWCS( Yaw, ...
        LatAcc, LatVel, LongAcc, LongVel);
    
 % for i = 1:numel()
      %  fprintf('   Steady State Instance %i: \n', i);
      % fprintf('      tau_i = %5.2f [N-m] \n', InputTorque(i));
   % end
    
    
     %return;   
end
    
%% Computation

% Global Velocity
GlobLongVel = LongVel .* cosd( Yaw ) - LatVel .* sind( Yaw );
GlobLatVel = LongVel .* sind( Yaw ) + LatVel .* cosd( Yaw );  
                 
% Curvature
if abs(LongVel.^2 + LatVel.^2) <= 0.01
    Curvature = 0;
else
    Curvature = ( LatVel.^2 + LatVel.^2 ).^(3/2) ./ ...
        abs( LongAcc.*LatVel - LongVel.*LatAcc );
end

% Body Slip
BodySlipAngle = atan2d( LatVel, LongVel );