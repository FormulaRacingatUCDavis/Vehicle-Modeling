function [YawAcc,LatAccTot,LongAccTot]= BicycleModelAccelerations(LongForce,...
    LatForce,SteeringWheelAngle,Mass,WheelBase,PercentFront)
%% Bicycle Model - Evaluate Accelerations
% 
% Inputs:
%  LongForce          - (n,1 numeric) Longitudinal Force {Fx} [N-m]
%  LatForce           - (n,1 numeric) Lateral Force      {Fy} [N-m]
%  SteeringWheelAngle - (n,1 numeric) SteeringWheelAngle {delta} [degrees]
%  Mass               - (n,1 numeric) Mass               {m} [kg]
%  WheelBase          - (n,1 numeric) Wheel Base         {L} [m]
%  PercentFront       - (n,1 numeric) Percent Front      {pf} [%]
%
% Outputs:
%  YawAcc     - (n,1 numeric) Yaw Acceleration                {psi_ddot} [rad/s^2]
%  LatAccTot  - (n,1 numeric) Total Lateral Acceleration      {a_x} [m/s^2]
%  LongAccTot - (n,1 numeric) Total Longitudinal Acceleration {a_y} [m/s^2]
% Notes: Not sure if test case needs to be changed
% 
%
% Author(s): 
% Tristan Pham (atlpham@ucdavis.edu) [Sep 2020 - Jun 2021] 

% Last Updated: 5/13/2021
%% Test Cases
if nargin == 0
    %%% Test Inputs
    LongitudinalForc = ; 
    
    LateralForce = ; 
    SteeringWheelAngle = ; 
    
    rho_g = ;
    Mass = ;
    
    WheelBase = ;
    PercentFront = ;
    
    fprintf('Executing Bicycle Model Accelerations() Test Cases: \n');
    
    
    [YawAcc,LatAccTot,LongAccTot] = BicycleModelAccelerations(LongForce,...
    LatForce,SteeringWheelAngle,Mass,WheelBase,PercentFront);
    
 % for i = 1:numel()
      %  fprintf('   Steady State Instance %i: \n', i);
      % fprintf('      tau_i = %5.2f [N-m] \n', InputTorque(i));
   % end
    
    
     %return;   
end



%% Computation

% Parameters
a = WheelBase* (1-PercentFront);
b = WheelBase*PercentFront;

YawInertia = Mass*(1.3)^2;

%Accelerations
YawAcc = ( a*( LatForce(1)*cosd(SteeringWheelAngle) + LongForce(1)*sind(SteeringWheelAngle) ) - b*LatForce(2) ) / YawInertia;
LatAccTot = ( LatForce(1)*cosd(SteeringWheelAngle) + LongForce(1)*sind(SteeringWheelAngle) + LatForce(2) ) / Mass;
LongAccTot = ( LongForce(1)*cosd(SteeringWheelAngle) - LatForce(1)*sind(SteeringWheelAngle) + LongForce(2) ) / Mass;

