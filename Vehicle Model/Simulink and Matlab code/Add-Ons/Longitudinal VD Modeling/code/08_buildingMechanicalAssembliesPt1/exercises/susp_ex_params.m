%Copyright 2016 The MathWorks, Inc.
%% Chassis dimensions
chassis_w = 30; %cm
chassis_l = 60; %cm
chassis_h = 40; %cm

%% Suspension arm dimensions
%Arm1 (upper arm)
arm1_chassis_dist = 10; %cm
arm1_l = 40; %cm 
arm1_w = 4; %cm
arm1_r = 0.75; %cm
arm1_extLength = 2; %cm
arm1_numHoles = 2;

%Arm2 (lower arm)
arm2_chassis_dist = -10; %cm
arm2_l = 40; %cm 
arm2_w = 4; %cm
arm2_r = 0.75; %cm
arm2_extLength = 2; %cm
arm2_numHoles = 2;

%% Wheel dimensions
wheel_inner_dia = 18*2.54; %cm, 18"
wheel_outer_dia = 21*2.54; %cm, 21"
wheel_inner_tread = 6*2.54; %cm, 6"
wheel_outer_tread = 7*2.54; %cm, 7"
wheel_rev = [wheel_outer_dia/2 -wheel_outer_tread/2;...
             wheel_outer_dia/2 wheel_outer_tread/2;...
             wheel_inner_dia/2 wheel_outer_tread/2;...
             wheel_inner_dia/2 -wheel_outer_tread/2;];
wheel_bias1 = -15; %cm 
wheel_bias2 = 15; %cm
hub_r = 5; %cm
hub_l = 2; %cm
hubDist = -9; %cm

%% Rim dimensions
rim_inner_dia = 15*2.54; %cm, 15"
rim_outer_dia = 18*2.54; %cm, 18"
rim_inner_tread = 4*2.54; %cm, 4"
rim_outer_tread = 5*2.54; %cm, 5"
rim_inner_hub = 5; %cm
rim_outer_hub = rim_inner_hub + (rim_outer_dia/2 - rim_inner_dia/2); %cm, 5"
rim_rev = [rim_outer_dia/2 -rim_outer_tread/2;...
           rim_outer_dia/2 rim_outer_tread/2;...
           rim_inner_dia/2 rim_outer_tread/2;...
           rim_inner_dia/2 -rim_inner_tread/2;...
           rim_outer_hub -rim_inner_tread/2;...
           rim_outer_hub rim_outer_tread/2;...
           rim_inner_hub rim_outer_tread/2;...
           rim_inner_hub -rim_outer_tread/2;];
       
%% Cylinder-Piston dimensions parameters
Rco = 1.5; %cm
Rci = 0.75; %cm
dr = Rco-Rci; %cm
Lc = 20; %cm 
cyl_xsec = [0 -Lc/2; Rco -Lc/2; 
            Rco Lc/2; Rci Lc/2; 
            Rci -Lc/2+dr; 0 -Lc/2+dr]; %cm
Lp = 10; %cm 
piston_xsec = [0 -Lp/2; Rci -Lp/2; Rci Lp/2; 0 Lp/2]; %cm
Ks = 2000000;%N/m 
Ds = 100000;%N/(m/s)
sa_dist = 10; %cm

%% Initial conditions
eq_pos = 10; %cm Prismatic joint I.C

%% Test bed dimensions
platform_w = 45; %cm
platform_l = 60; %cm
platform_h = 5; %cm

platform_x = 0;
platform_y = chassis_w/2 + arm1_l;
platform_z = -chassis_h/2 - 20;

%% Wheel test bed controller parameters
Kw = -1e6; %proportional gain
Dw = -1e5; %derivative gain

%% Load position input data
load wheels_input