function [LongAcc, LatAcc, YawAcc, LongAccTot, LatAccTot] = ...
    FullTrack3DOFAccelerations( TFx, TFy, TMz, AFx, AFy, AMz, ... % Loads
        Wheelbase, TrackWidth, Steer, ...                         % Geometry
        Mass, YawInertia, CoG, ...                                % Inertia
        LongVel, LatVel, YawVel )                                 % Velocities
%% FullTrack3DOFAccelerations - Full Track Planar Accelerations
% Computes planar motion accelerations for a 3DOF full track chassis model
% 
% Inputs:
%   TFx        - (n,4 numeric) Tire Longitudinal Force {T_F_y}    [N]
%   TFy        - (n,4 numeric) Tire Lateral Force      {T_F_y}    [N]
%   TMz        - (n,4 numeric) Tire Aligning Moment    {T_M_z}    [N-m]
%   AFx        - (n,1 numeric) Aerodynamic Drag Force  {A_F_x}    [N]
%   AFy        - (n,1 numeric) Aerodynamic Side Force  {A_F_y}    [N]
%   AMz        - (n,1 numeric) Aerodynamic Yaw Moment  {A_M_z}    [N-m]
%   Wheelbase  - (n,1 numeric) Wheelbase               {L}        [m]
%   TrackWidth - (n,2 numeric) Track Width             {t_w}      [m]
%   Steer      - (n,4 numeric) Tire Steer Angle        {delta}    [deg]
%   Mass       - (n,1 numeric) Total Vehicle Mass      {m}        [kg]
%   YawInertia - (n,1 numeric) Vehicle Yaw Inertia     {I_zz}     [kg-m^2]
%   CoG        - (n,3 numeric) Center of Gravity       {CoG}      [m]
%   LongVel    - (n,1 numeric) Longitudinal Velocity   {dot{x}}   [m/s]
%   LatVel     - (n,1 numeric) Lateral Velocity        {dot{y}}   [m/s]
%   YawVel     - (n,1 numeric) Yaw Velocity            {dot{psi}} [rad/s]
%
% Outputs:
%   LongAcc    - (n,1 numeric) Longitudinal Acceleration       {ddot{x}}  [m/s^2]
%   LatAcc     - (n,1 numeric) Lateral Acceleration            {ddot{y}}  [m/s^2]
%   YawAcc     - (n,1 numeric) Yaw Acceleration                {ddot{psi} [rad/s^2]
%   LongAccTot - (n,1 numeric) Total Longitudinal Acceleration {a_x}      [m/s^2]
%   LatAccTot  - (n,1 numeric) Total Lateral Acceleration      {a_y}      [m/s^2]
%
% Notes:
%
% Author(s): 
% Blake Christierson (bechristierson@ucdavis.edu) [Sep 2018 - Jun 2021] 
% Tristan Pham       (atlpham@ucdavis.edu)        [Oct 2020 - ???     ]
% 
% Last Updated: 30-May-2021

%% Test Case
if nargin == 0
    TFx = zeros(1,4);
    TFy = [250, 250, 0, 0];
    TMz = zeros(1,4);
    
    AFx = -100;
    AFy = 0;
    AMz = 0;
    
    Wheelbase  = 1.575; 
    TrackWidth = 1.22*ones(1,2);
    Steer      = [18, 15, 1, -1];
    
    Mass       = 275;   
    YawInertia = 130; 
    CoG        = [(0.47-0.5)*Wheelbase, 0, 0.25];
    
    LongVel = 15;
    LatVel  = 0;
    YawVel  = 0.5;
    
    [LongAcc, LatAcc, YawAcc, LongAccTot, LatAccTot] = ...
        FullTrack3DOFAccelerations( ...
            TFx, TFy, TMz, AFx, AFy, AMz, ...
            Wheelbase, TrackWidth, Steer, ... 
            Mass, YawInertia, CoG, ... 
            LongVel, LatVel, YawVel ) %#ok<NOPRT>
    
    return;
end

%% Computations
%%% Tire Positions (n,4,3 numeric)
TirePos = cat( 3, Wheelbase/2 + [-1, -1, 1, 1].*CoG(:,1), ...
                  [TrackWidth(:,1).*[1 -1]/2, TrackWidth(:,2).*[1 -1]/2], ...
                  zeros( size(Steer) ) );

%%% Tire Loads (n,4,3 numeric)
TireLoad = cat( 3, TFx, TFy, zeros( size(Steer) ) );

%%% Tire Yaw Moment (n,1 numeric)
TireMoment = cross(TirePos, TireLoad, 3);
TireMoment = sum( TireMoment(:,:,3) ) + sum(TMz,2);

%%% Chassis Accelerations
LongAcc = (sum( TFx.*cosd(Steer) - TFy.*sind(Steer), 2 ) + LatVel.*YawVel  - AFx) ./ Mass;
LatAcc  = (sum( TFy.*cosd(Steer) + TFx.*sind(Steer), 2 ) - LongVel.*YawVel - AFy) ./ Mass;
YawAcc  = (TireMoment - AMz) ./ YawInertia;

LongAccTot = LongAcc - LatVel .*YawVel;
LatAccTot  = LatAcc  + LongVel.*YawVel;
