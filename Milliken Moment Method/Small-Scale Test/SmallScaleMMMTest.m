clc; clear; close all;

%% SmallScaleMMMTest - Milliken Moment Method Development Script
% This scripts runs a simplistic planar, rigid roll distribution model to
% estimate yaw behavior. The model will be used to experiment generation
% techniques to generate Milliken Moment Diagrams.
% 
% Inputs:
% 
% Outputs:
%
% Author(s):
% Tristan Pham       (atlpham@ucdavis.edu       )
% Blake Christierson (bechristierson@ucdavis.edu) 
% 
% Last Updated: 16-Feb-2021

tic

%% Vehicle Parameters
%%% Mass
Parameter.Mass.m   = 200 + 68;                % Vehicle + Driver Mass                    [kg]
Parameter.Mass.hs  =   0.245;                 % Sprung Center of Gravity                 [m]
Parameter.Mass.Pf  =   0.47 ;                 % Percent Front Weight Distribution        [ ]
Parameter.Mass.mu  = [38.5  , 38.5 ];         % [Front, Rear] Unsprung Mass              [kg]
Parameter.Mass.hu  = [ 0.220, 0.220];         % [Front, Rear] Unsprung Center of Gravity [m]
Parameter.Mass.Izz = Parameter.Mass.m .* 0.7; % Yaw Inertia                              [kg-m^2]

%%% Suspension
Parameter.Susp.L      =   1.525;         % Wheelbase                           [m]
Parameter.Susp.tw     = [ 1.220, 1.220]; % [Front, Rear] Track Width           [m]
Parameter.Susp.IC     = [ 0.025, 0.100]; % [Front, Rear] Instant Center Height [m]
Parameter.Susp.Pf     =   0.60 ;         % Percent Front Roll Stiffness        [ ]
Parameter.Susp.Camber = [-1.6  ,-1.1  ]; % [Front, Rear] Static Camber         [deg]
Parameter.Susp.Toe    = [-0.2  , 0.0  ]; % [Front, Rear] Static Toe            [deg]

%%% Tire Model
load('Hoosier_R25B_16x75-10x7.mat'); Parameter.Tire(1) = Tire; % Front Tire [FRUCDTire]
load('Hoosier_R25B_16x75-10x7.mat'); Parameter.Tire(2) = Tire; % Rear Tire  [FRUCDTire]

%%% Controls
Parameter.Steer.MaxWheel  =  130  ; % Max Steering Wheel Angle     [deg]
Parameter.Steer.MaxTire   =   28  ; % Max Tire Steer Angle         [deg]
Parameter.Steer.Quadratic =    1.4; % Quadratic Steer Nonlinearity [ ] 
Parameter.Steer.Cubic     = -  1.2; % Cubic Steer Nonlinearity     [ ] 
Parameter.Steer.Function = SteerFunction( Parameter ); % Steer-Steer Function [deg]

%%% Calculated Parameters
Parameter.Mass.ms = Parameter.Mass.m - sum( Parameter.Mass.mu ); 
Parameter.Mass.a  = Parameter.Susp.L .* (1 - Parameter.Mass.Pf);
Parameter.Mass.b  = Parameter.Susp.L .* Parameter.Mass.Pf;

Parameter.Susp.ds = Parameter.Mass.hs - ...
    interp1( [0 Parameter.Susp.L], Parameter.Susp.IC, Parameter.Mass.a );

clear Tire 

%% Operating Conditions
xDot  =    5: 5:40;       % Longitudinal Speed        [m/s]
xDDot =    0;             % Longitudinal Acceleration [m/s^2]
DelSW =   10:15:130;      % Steering Angle            [deg]
Beta   =  - 3: 8;         % Body Slip Angle           [deg]

[xDot, xDDot, DelSW, Beta] = ndgrid( xDot, xDDot, DelSW, Beta ); 

State.Chassis.Beta   = Beta(:);
State.Chassis.x(:,2) = xDot(:);
State.Chassis.x(:,3) = xDDot(:);
State.Steer.Wheel    = DelSW(:);


clear xDot xDDot DelSW Beta

%% Initializing Vehicle States
State.Chassis.y(:,2) = tand( State.Chassis.Beta ) .* State.Chassis.x(:,2); % Lateral Speed [m/s]
State.Chassis.y(end,3) = 0; % Lateral Acceleration [m/s^2]

State.Chassis.psi = zeros( size( State.Chassis.y ) ); % Yaw States [m, m/s, m/s^2]

State.Tire.delta(:,1) =  Parameter.Steer.Function( State.Steer.Wheel, 1 ); % Front-Left Steer Angle  [deg]
State.Tire.delta(:,2) =  Parameter.Steer.Function( State.Steer.Wheel, 2 ); % Front-Right Steer Angle [deg]
State.Tire.delta(:,3) =  Parameter.Susp.Toe(2); % Rear-Left Steer Angle  [deg]
State.Tire.delta(:,4) = -Parameter.Susp.Toe(2); % Rear-Right Steer Angle [deg]

State.Tire.alpha = zeros( size( State.Tire.delta ) ); % Tire Slip Angle [deg]
State.Tire.kappa = zeros( size( State.Tire.delta ) ); % Tire Slip Ratio [ ]

State.Tire.Fz(:,1:2) = Parameter.Mass.m .* ( 9.81*Parameter.Mass.Pf/2 - ...
    Parameter.Mass.hs .* State.Chassis.x(:,3) ./ Parameter.Susp.L ) .* ones(1,2); % Front Tire Loads [N]
State.Tire.Fz(:,3:4) = Parameter.Mass.m .* ( 9.81*(1-Parameter.Mass.Pf)/2 + ...
    Parameter.Mass.hs .* State.Chassis.x(:,3) ./ Parameter.Susp.L ) .* ones(1,2); % Rear Tire Loads  [N]

State.Tire.Fx = zeros( size( State.Tire.Fz ) ); % Longitudinal Tire Force [N]
State.Tire.Fy = zeros( size( State.Tire.Fz ) ); % Lateral Tire Force      [N]
State.Tire.Mz = zeros( size( State.Tire.Fz ) ); % Aligning Moment         [Nm]

<<<<<<< Updated upstream
%{
Solver.Tolerance = 1E-3;
Solver.Correction = ones( size( State.Chassis.x(:,1) ) );
for i = 1 : numel( State.Chassis.x(:,2) )
=======
Opts = optimoptions( 'fmincon', ...
            'Algorithm', 'sqp', ...
            'MaxFunctionEvaluations', 10000, ...
            'MaxIterations', 10000, ...
            'Display', 'off' );

Solution = ones( [size(State.Chassis.Beta), 3] );

for i = 1 : numel( unique( State.Chassis.xDot ) )
    for j = 1 : numel( unique( State.Chassis.xDDot ) )
        for k = 1 : numel( unique( State.Chassis.Beta ) )
            for l = 1 : numel( unique( State.Steer.Wheel ) ) + 1
                tic
                if State.Chassis.Beta(i,j,k,l) == 0 && State.Steer.Wheel(i,j,k,l) == 0
                    Solution(i,j,k,l,:) = fmincon( ...
                        @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                        [0 0 0], [], [], [], [], [], [], [], Opts );
                elseif State.Steer.Wheel(i,j,k,l) == 0
                    if State.Steer.Wheel(i,j,k,l) >= 0
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            [-1 1 0], [], [], [], [], [], [], [], Opts ); 
                    else
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            [1 -1 0], [], [], [], [], [], [], [], Opts ); 
                    end
                else
                    if State.Steer.Wheel(i,j,k,l) >= 0
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            Solution(i,j,k,l-1,:), [], [], [], [], [], [], [], Opts ); 
                    else
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            Solution(i,j,k,l-1,:), [], [], [], [], [], [], [], Opts ); 
                    end
                end
                
                State = StateFunction( squeeze(Solution(i,j,k,l,:)), Parameter, State, i,j,k,l, 'Eval' );
                
                toc
            end
        end
    end
end
>>>>>>> Stashed changes
    
    while Solver.Correction(i) > Solver.Tolerance
        %%% Slip Estimation
        State.Tire.alpha(i,1) = ???;
        State.Tire.alpha(i,2) = ???;
        State.Tire.alpha(i,3) = ???;
        State.Tire.alpha(i,4) = ???;
        
        %%% Tire Forces
        [State.Tire.Fx(i,1:2), State.Tire.Fy(i,1:2), State.Tire.Mz(i,1:2), ~, ~] = ...
            Parameter.Tire(1).???;
        
        [State.Tire.Fx(i,3:4), State.Tire.Fy(i,3:4), State.Tire.Mz(i,3:4), ~, ~] = ...
            Parameter.Tire(2).???;
        
        %%% Chassis States 
        State.Chassis.psi(i,2) = ???;
        
        State.Chassis.y(i,3) = ???;
        
        State.Chassis.psi(i,3) = ???;
        
        State.Chassis.ax = ???;
    
        State.Chassis.ay = ???;
        
        %%% Weight Transfer

        State.Tire.Fz(:,1) = ???;
        State.Tire.Fz(:,2) = ???;
        State.Tire.Fz(:,3) = ???;
        State.Tire.Fz(:,4) = ???;
        
        %%% Residual / Correction Calculation
        Solver.Correction(i) = ???;
    end
end
%}
    
toc

%% Local Functions
function Function = SteerFunction( Parameter ) 
    Function = @(DelSW, i) Parameter.Susp.Toe(1) .* (-1).^(mod(i,2)+1)        + ...
        Parameter.Steer.MaxTire   .* (DelSW ./ Parameter.Steer.MaxWheel)     + ...
        Parameter.Steer.Quadratic .* (DelSW ./ Parameter.Steer.MaxWheel).^2 .* (-1).^(mod(i,2)+1) + ...
        Parameter.Steer.Cubic     .* (DelSW ./ Parameter.Steer.MaxWheel).^3  ;
end