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
% Last Updated: 11-Nov-2021

tic

%% Vehicle Parameters
%%% Mass
Parameter.Mass.m   = 200 + 68;                  % Vehicle + Driver Mass                    [kg]
Parameter.Mass.hs  =   0.245;                   % Sprung Center of Gravity                 [m]
Parameter.Mass.Pf  =   0.50 ;                   % Percent Front Weight Distribution        [ ]
Parameter.Mass.mu  = [38.5  , 38.5 ];           % [Front, Rear] Unsprung Mass              [kg]
Parameter.Mass.hu  = [ 0.220, 0.220];           % [Front, Rear] Unsprung Center of Gravity [m]
Parameter.Mass.Izz = Parameter.Mass.m .* 0.7^2; % Yaw Inertia                              [kg-m^2]

%%% Suspension
Parameter.Susp.L      =   1.525;         % Wheelbase                           [m]
Parameter.Susp.tw     = [ 1.220, 1.220]; % [Front, Rear] Track Width           [m]
%Parameter.Susp.IC     = [ 0, 0];        % [Front, Rear] Instant Center Height [m]
Parameter.Susp.IC     = [ 0.025, 0.050]; % [Front, Rear] Instant Center Height [m]
Parameter.Susp.Pf     =   0.60 ;         % Percent Front Roll Stiffness        [ ]
Parameter.Susp.Camber = [-1.1  ,-1.1  ]; % [Front, Rear] Static Camber         [deg]
Parameter.Susp.Toe    = [-0.0  , 0.0  ]; % [Front, Rear] Static Toe            [deg]

%%% Tire Model
load('Hoosier_R25B_16x75-10x7.mat'); 
Parameter.Tire(1).Model = Tire; % Front Tire [FRUCDTire]
load('Hoosier_R25B_16x75-10x7.mat'); 
Parameter.Tire(2).Model = Tire; % Rear Tire  [FRUCDTire]

Parameter.Tire(1).Fidelity.Pure     = 'Pacejka';
Parameter.Tire(1).Fidelity.Combined = 'MNC';

Parameter.Tire(1).Pressure = 70; % Front Tire Pressure [kPa]
Parameter.Tire(2).Pressure = 70; % Rear Tire Pressure  [kPa]

% Pacejka Scaling Factors
Parameter.Tire(1).Model.Pacejka.L.mu.x = 2/3;
Parameter.Tire(1).Model.Pacejka.L.mu.y = 2/3;
Parameter.Tire(2).Model.Pacejka.L.mu.x = 2/3;
Parameter.Tire(2).Model.Pacejka.L.mu.y = 2/3;

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

Parameter.Susp.r(:,1) = [ Parameter.Mass.a,  Parameter.Susp.tw(1)/2, 0];
Parameter.Susp.r(:,2) = [ Parameter.Mass.a, -Parameter.Susp.tw(1)/2, 0];
Parameter.Susp.r(:,3) = [-Parameter.Mass.b,  Parameter.Susp.tw(2)/2, 0];
Parameter.Susp.r(:,4) = [-Parameter.Mass.b, -Parameter.Susp.tw(2)/2, 0];

clear Tire 

%% Operating Conditions
xDot  =  15;                   % Longitudinal Speed        [m/s]
xDDot =  0;                    % Longitudinal Acceleration [m/s^2]
Beta  =  [-10:0.5:10];         % Body Slip Angle           [deg]
DelSW =  [0:5:130, 0:-5:-130]; % Steering Angle            [deg]

[xDot, xDDot, Beta, DelSW] = ndgrid( xDot, xDDot, Beta, DelSW ); 

State.Chassis.xDot  = xDot;
State.Chassis.xDDot = xDDot;
State.Chassis.Beta  = Beta;
State.Steer.Wheel   = DelSW;

clear xDot xDDot DelSW Beta

%% Initializing Vehicle States
State.Chassis.yDot  = tand( State.Chassis.Beta ) .* State.Chassis.xDot; % Lateral Speed        [m/s]
State.Chassis.yDDot = zeros( size( State.Chassis.Beta ) );              % Lateral Acceleration [m/s^2]

State.Chassis.psiDot  = zeros( size( State.Chassis.Beta ) ); % Yaw Rate         [m/s]
State.Chassis.psiDDot = zeros( size( State.Chassis.Beta ) ); % Yaw Acceleration [m/s^2]

State.Chassis.ax = zeros( size( State.Chassis.Beta ) ); % Longitudinal Acceleration [m/s^2]
State.Chassis.ay = zeros( size( State.Chassis.Beta ) ); % Lateral Acceleration      [m/s^2]

State.Tire.delta(:,:,:,:,1) =  Parameter.Steer.Function( State.Steer.Wheel, 1 ); % Front-Left Steer Angle  [deg]
State.Tire.delta(:,:,:,:,2) =  Parameter.Steer.Function( State.Steer.Wheel, 2 ); % Front-Right Steer Angle [deg]
State.Tire.delta(:,:,:,:,3) =  Parameter.Susp.Toe(2);                            % Rear-Left Steer Angle  [deg]
State.Tire.delta(:,:,:,:,4) = -Parameter.Susp.Toe(2);                            % Rear-Right Steer Angle [deg]

State.Tire.alpha = zeros( size( State.Tire.delta ) ); % Tire Slip Angle [deg]
State.Tire.kappa = zeros( size( State.Tire.delta ) ); % Tire Slip Ratio [ ]

State.Tire.Fz(:,:,:,:,1:2) = Parameter.Mass.m .* ( 9.81*Parameter.Mass.Pf/2 - ...
    Parameter.Mass.hs .* State.Chassis.xDDot ./ Parameter.Susp.L ) .* ones(1,1,1,1,2); % Front Tire Loads [N]
State.Tire.Fz(:,:,:,:,3:4) = Parameter.Mass.m .* ( 9.81*(1-Parameter.Mass.Pf)/2 + ...
    Parameter.Mass.hs .* State.Chassis.xDDot ./ Parameter.Susp.L ) .* ones(1,1,1,1,2); % Rear Tire Loads  [N]

State.Tire.Fx = zeros( size( State.Tire.Fz ) ); % Longitudinal Tire Force [N]
State.Tire.Fy = zeros( size( State.Tire.Fz ) ); % Lateral Tire Force      [N]
State.Tire.Mz = zeros( size( State.Tire.Fz ) ); % Aligning Moment         [Nm]

%% Solver
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
                    if State.Chassis.Beta(i,j,k,l) >= 0
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            [-1 1 0], [], [], [], [], [], [], [], Opts ); 
                    else
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            [1 -1 0], [], [], [], [], [], [], [], Opts ); 
                    end
                else
%                     if State.Steer.Wheel(i,j,k,l) >= 0
%                         Solution(i,j,k,l,:) = fmincon( ...
%                             @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
%                             Solution(i,j,k,l-1,:), [], [], [], [], [], [], [], Opts ); 
%                     else
                        Solution(i,j,k,l,:) = fmincon( ...
                            @(x) StateFunction( x, Parameter, State, i,j,k,l, 'Opt' ), ...
                            Solution(i,j,k,l-1,:), [], [], [], [], [], [], [], Opts ); 
%                     end
                end
                
                State = StateFunction( squeeze(Solution(i,j,k,l,:)), Parameter, State, i,j,k,l, 'Eval' );
                
                toc
            end
        end
    end
end
    
toc

%% Plotting Function
[~,PlotIdx] = sort( State.Steer.Wheel(1,1,1,:) );
PosIdx = find(State.Steer.Wheel(1,1,1,:) > 0);
NegIdx = find(State.Steer.Wheel(1,1,1,:) < 0);

for i = 1 : numel( unique( State.Chassis.xDot ) )
    for j = 1 : numel( unique( State.Chassis.xDDot ) )
        %figure( 'Name', 'Steer Isolines')
        %plot(  squeeze(State.Chassis.ay(i,j,:,PlotIdx)) ./ 9.81,  squeeze(State.Chassis.psiDDot(i,j,:,PlotIdx)), 'k' ); hold on;
        %plot( -squeeze(State.Chassis.ay(i,j,:,PlotIdx)) ./ 9.81, -squeeze(State.Chassis.psiDDot(i,j,:,PlotIdx)), 'k' );
        
        figure( 'Name', 'Beta Isolines')
        plot3(  squeeze(State.Steer.Wheel(i,j,:,PosIdx))', ...
            squeeze(State.Chassis.ay(i,j,:,PosIdx))' ./ 9.81,  squeeze(State.Chassis.psiDDot(i,j,:,PosIdx))', ...
            'r' ); hold on;
        plot3(  squeeze(State.Steer.Wheel(i,j,:,NegIdx))', ...
            squeeze(State.Chassis.ay(i,j,:,NegIdx))' ./ 9.81,  squeeze(State.Chassis.psiDDot(i,j,:,NegIdx))', ...
            'b' );
        xlabel( 'deltaSW [deg]' ); ylabel( 'ay [g]' ); zlabel( 'psiDDot [rad/s^2]' ); 
        %plot( -squeeze(State.Chassis.ay(i,j,:,PlotIdx))' ./ 9.81, -squeeze(State.Chassis.psiDDot(i,j,:,PlotIdx))', 'k' );
        
        figure( 'Name', 'Beta Diagram')
        plot3(  squeeze(State.Chassis.Beta(i,j,:,PlotIdx)), ...
            squeeze(State.Steer.Wheel(i,j,:,PlotIdx)) ,  squeeze(State.Chassis.psiDDot(i,j,:,PlotIdx)), ...
            'k' ); hold on;
        xlabel( 'beta [deg]' ); ylabel( 'deltaSW [deg]' ); zlabel( 'psiDDot [rad/s^2]' ); 
     
        figure( 'Name', 'Yaw Rate')
        plot3(  squeeze(State.Chassis.Beta(i,j,:,PlotIdx)), ...
            squeeze(State.Steer.Wheel(i,j,:,PlotIdx)) ,  squeeze(State.Chassis.yDDot(i,j,:,PlotIdx)), ...
            'k' ); hold on;
        xlabel( 'beta [deg]' ); ylabel( 'deltaSW [deg]' ); zlabel( 'psiDot [rad/s]' ); 
     
        figure( 'Name', 'Slip Angles' )
        subplot( 2,2,1 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.alpha(i,j,:,PlotIdx,1))' );
        subplot( 2,2,2 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.alpha(i,j,:,PlotIdx,2))' );
        subplot( 2,2,3 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.alpha(i,j,:,PlotIdx,3))' );
        subplot( 2,2,4 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.alpha(i,j,:,PlotIdx,4))' );
        
        figure( 'Name', 'Lateral Forces' )
        subplot( 2,2,1 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fy(i,j,:,PlotIdx,1))' );
        subplot( 2,2,2 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fy(i,j,:,PlotIdx,2))' );
        subplot( 2,2,3 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fy(i,j,:,PlotIdx,3))' );
        subplot( 2,2,4 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fy(i,j,:,PlotIdx,4))' );
        
        figure( 'Name', 'Normal Loads' )
        subplot( 2,2,1 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fz(i,j,:,PlotIdx,1))' );
        subplot( 2,2,2 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fz(i,j,:,PlotIdx,2))' );
        subplot( 2,2,3 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fz(i,j,:,PlotIdx,3))' );
        subplot( 2,2,4 )
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Tire.Fz(i,j,:,PlotIdx,4))' );
        
        figure( 'Name', 'Bounding Plot' )
        subplot(3,1,1)
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Chassis.yDDot(i,j,:,PlotIdx,1))' );
        subplot(3,1,2)
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Chassis.psiDot(i,j,:,PlotIdx,1))' );
        subplot(3,1,3)
        plot3( squeeze(State.Steer.Wheel(i,j,:,PlotIdx))', squeeze(State.Chassis.Beta(i,j,:,PlotIdx))', ...
            squeeze(State.Chassis.psiDDot(i,j,:,PlotIdx,1))' );
        
        figure( 'Name', 'Beta Isolines')
        plot( [0,0], [0,0], 'r' ); hold on;
        plot( [0,0], [0,0], 'b' )
        plot( squeeze(State.Chassis.ay(i,j,:,PosIdx))' ./ 9.81,  squeeze(State.Chassis.psiDDot(i,j,:,PosIdx))', ...
            'r' ); 
        plot( squeeze(State.Chassis.ay(i,j,:,NegIdx))' ./ 9.81,  squeeze(State.Chassis.psiDDot(i,j,:,NegIdx))', ...
            'b' );
      xline(0) 
      yline(0)
        legend( '$\beta > 0, \delta > 0$', '$\beta > 0, \delta < 0$','Interpreter','latex' )
        xlabel( '$a_{y}$ [$g$]','Interpreter','latex' ); ylabel( '$\ddot{\psi}$ [$rad/s^{2}$]','Interpreter','latex' ); 
    end
end

%% Local Functions
function Function = SteerFunction( Parameter ) 
    Function = @(DelSW, i) Parameter.Susp.Toe(1) .* (-1).^(mod(i,2)+1)        + ...
        Parameter.Steer.MaxTire   .* (DelSW ./ Parameter.Steer.MaxWheel)     + ...
        Parameter.Steer.Quadratic .* (DelSW ./ Parameter.Steer.MaxWheel).^2 .* (-1).^(mod(i,2)+1) + ...
        Parameter.Steer.Cubic     .* (DelSW ./ Parameter.Steer.MaxWheel).^3  ;
end

function Out = StateFunction( x, Parameter, State, i,j,k,l, Mode ) 
    %%% Total Acceleration
    State.Chassis.ax(i,j,k,l) = State.Chassis.xDDot(i,j,k,l) - ...
        State.Chassis.yDot(i,j,k,l) .* x(2);

    State.Chassis.ay(i,j,k,l) = x(1) + State.Chassis.xDot(i,j,k,l) .* x(2);
    
    %%% Slip Estimation        
    v_c(:,1) = [State.Chassis.xDot(i,j,k,l), State.Chassis.yDot(i,j,k,l), 0] + ...
        cross( [0, 0, x(2)], Parameter.Susp.r(:,1) );

    v_c(:,2) = [State.Chassis.xDot(i,j,k,l), State.Chassis.yDot(i,j,k,l), 0] + ...
        cross( [0, 0, x(2)], Parameter.Susp.r(:,2) );

    v_c(:,3) = [State.Chassis.xDot(i,j,k,l), State.Chassis.yDot(i,j,k,l), 0] + ...
        cross( [0, 0, x(2)], Parameter.Susp.r(:,3) );

    v_c(:,4) = [State.Chassis.xDot(i,j,k,l), State.Chassis.yDot(i,j,k,l), 0] + ...
        cross( [0, 0, x(2)], Parameter.Susp.r(:,4) );

    State.Tire.alpha(i,j,k,l,1) = atan2d( v_c(2,1), v_c(1,1) ) - State.Tire.delta(i,j,k,l,1);
    State.Tire.alpha(i,j,k,l,2) = atan2d( v_c(2,2), v_c(1,2) ) - State.Tire.delta(i,j,k,l,2);
    State.Tire.alpha(i,j,k,l,3) = atan2d( v_c(2,3), v_c(1,3) ) - State.Tire.delta(i,j,k,l,3);
    State.Tire.alpha(i,j,k,l,4) = atan2d( v_c(2,4), v_c(1,4) ) - State.Tire.delta(i,j,k,l,4);

    %%% Weight Transfer  
    dFz(1) = ( Parameter.Susp.Pf .* Parameter.Susp.ds .* Parameter.Mass.ms + ...
        Parameter.Susp.IC(1) .* Parameter.Mass.Pf .* Parameter.Mass.ms + ...
        Parameter.Mass.hu(1) .* Parameter.Mass.mu(1) ) .* ...
        State.Chassis.ay(i,j,k,l) ./ Parameter.Susp.tw(1);

    dFz(2) = ( (1-Parameter.Susp.Pf) .* Parameter.Susp.ds .* Parameter.Mass.ms + ...
        Parameter.Susp.IC(2) .* (1-Parameter.Mass.Pf) .* Parameter.Mass.ms + ...
        Parameter.Mass.hu(2) .* Parameter.Mass.mu(2) ) .* ...
        State.Chassis.ay(i,j,k,l) ./ Parameter.Susp.tw(2);

    State.Tire.Fz(i,j,k,l,1) = Parameter.Mass.m/2 .* (9.81.*Parameter.Mass.Pf) - dFz(1); % - ...
        % State.Chassis.ax(i,j,k,l) .* Parameter.Mass.hs ./ Parameter.Susp.L );
    State.Tire.Fz(i,j,k,l,2) = Parameter.Mass.m/2 .* (9.81.*Parameter.Mass.Pf) + dFz(1); % - ...
        % State.Chassis.ax(i,j,k,l) .* Parameter.Mass.hs ./ Parameter.Susp.L );
    State.Tire.Fz(i,j,k,l,3) = Parameter.Mass.m/2 .* (9.81.*Parameter.Mass.Pf) - dFz(2); % + ...
        % State.Chassis.ax(i,j,k,l) .* Parameter.Mass.hs ./ Parameter.Susp.L );
    State.Tire.Fz(i,j,k,l,4) = Parameter.Mass.m/2 .* (9.81.*Parameter.Mass.Pf) + dFz(2); % + ...
        % State.Chassis.ax(i,j,k,l) .* Parameter.Mass.hs ./ Parameter.Susp.L );

    if any( State.Tire.Fz(i,j,k,l,:) <= 0 )
       State.Tire.Fz(i,j,k,l, State.Tire.Fz(i,j,k,l,:) <= 0) = 0.1;

       SumFz = sum( State.Tire.Fz(i,j,k,l,:) );

       State.Tire.Fz(i,j,k,l,:) = State.Tire.Fz(i,j,k,l,:) .* Parameter.Mass.m .* 9.81 ./ SumFz;
    end

    %%% Tire Forces   
    [State.Tire.Fx(i,j,k,l,1:2), State.Tire.Fy(i,j,k,l,1:2), State.Tire.Mz(i,j,k,l,1:2), ~, ~] = ...
         ContactPatchLoads( Parameter.Tire(1).Model, ...
        squeeze(State.Tire.alpha(i,j,k,l,1:2)), squeeze(State.Tire.kappa(i,j,k,l,1:2)), ...
        squeeze(State.Tire.Fz(i,j,k,l,1:2)), Parameter.Tire(1).Pressure, Parameter.Susp.Camber(1).*[-1 1]', ...
        0, (1:2)', [Parameter.Tire.Fidelity] );

    [State.Tire.Fx(i,j,k,l,3:4), State.Tire.Fy(i,j,k,l,3:4), State.Tire.Mz(i,j,k,l,3:4), ~, ~] = ...
        ContactPatchLoads( Parameter.Tire(2).Model, ...
        squeeze(State.Tire.alpha(i,j,k,l,3:4)), squeeze(State.Tire.kappa(i,j,k,l,3:4)), ...
        squeeze(State.Tire.Fz(i,j,k,l,3:4)), Parameter.Tire(2).Pressure, Parameter.Susp.Camber(2).*[-1 1]', ...
        0, (3:4)', [Parameter.Tire.Fidelity] );

    % State.Tire.Fx(i,j,k,l,:) = 0;
    
    %%% Chassis States 
    Out(1) = ( sum( State.Tire.Fx(i,j,k,l,:) .* cosd( State.Tire.delta(i,j,k,l,:) ) ) - ...
               sum( State.Tire.Fy(i,j,k,l,:) .* sind( State.Tire.delta(i,j,k,l,:) ) ) ) - ...
             Parameter.Mass.m .* (State.Chassis.xDDot(i,j,k,l) - State.Chassis.yDot(i,j,k,l) .* x(2) );

    Out(2) = ( sum( State.Tire.Fy(i,j,k,l,:) .* cosd( State.Tire.delta(i,j,k,l,:) ) ) + ...
               sum( State.Tire.Fx(i,j,k,l,:) .* sind( State.Tire.delta(i,j,k,l,:) ) ) ) - ...
             Parameter.Mass.m .* (x(1) + State.Chassis.xDot(i,j,k,l) .* x(2) );

    Out(3) = sum( ...
        cross( Parameter.Susp.r(:,1), rotz( State.Tire.delta(i,j,k,l,1) ) * ...
            [State.Tire.Fx(i,j,k,l,1); State.Tire.Fy(i,j,k,l,1); 0] ) + ...
        cross( Parameter.Susp.r(:,2), rotz( State.Tire.delta(i,j,k,l,2) ) * ...
            [State.Tire.Fx(i,j,k,l,2); State.Tire.Fy(i,j,k,l,2); 0] ) + ...
        cross( Parameter.Susp.r(:,3), rotz( State.Tire.delta(i,j,k,l,3) ) * ...
            [State.Tire.Fx(i,j,k,l,3); State.Tire.Fy(i,j,k,l,3); 0] ) + ...
        cross( Parameter.Susp.r(:,4), rotz( State.Tire.delta(i,j,k,l,4) ) * ...
            [State.Tire.Fx(i,j,k,l,4); State.Tire.Fy(i,j,k,l,4); 0] ) ) - ...
        x(3) .* Parameter.Mass.Izz;
    
    Out = sqrt( mean( Out.^2 ) );
    
    %%% Output
    if strcmpi( Mode, 'Eval' )
        State.Chassis.yDDot(i,j,k,l)   = x(1);
        State.Chassis.psiDot(i,j,k,l)  = x(2);
        State.Chassis.psiDDot(i,j,k,l) = x(3);

        Out = State;
    end
end

