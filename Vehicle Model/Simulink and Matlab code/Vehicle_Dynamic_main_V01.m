clearvars; clc
close all

%% Load files

load CarSim_FE4_2.mat
load Vehicle_information_test_0525_2017.mat
load CarSim0524.mat

%% Vehicle Parameters

%global m_total weight_front weight_rear m g r_gy j track weight_left weight_right a b k_spring r_damper k_tire_l k_tire_r x F h mt

%Dynamic
u = 60; %km/hr, longitudinal speed

%Aerodynamic
Cd = 0.7128; %none, drag coefficient
frontal_area = 0.9576; %m^2, frontal area of the car

%Sprung mass
m_total = 237.6 + 70; %kg, total mass of vehicle + Driver's weight
Izz = 89.99 ; %kg-m^2, Yaw inertia respect to Z-axis
weight_front = 0.5; % percent, weight distribution at front
weight_rear = 1 - weight_front; % percent, weight distribution at rear
mf = m_total*weight_front; %kg, vehicle mass at front
mr = m_total*weight_rear; %kg, vehicle mass at rear
g = 9.81; %m/s^2, acceleration due to gravity
r_gy = 0.795;%0.956; %m, radius of gyration
jf = mf*r_gy^2; %kg-m^2, rotational inertia respect to X-axis at front
jr = mr*r_gy^2; %kg-m^2, rotational inertia respect to X-axis at rear

%Wheelbase information
wheelbase = 1.575; %m, wheelbase of the car
aa = weight_front*wheelbase; %m, distance between center of mass and front end
bb = weight_rear*wheelbase; %m, distance between center of mass and front end

%Track information
track = 1.248; %m, track of the car
weight_right = 0.5; %percent, weight distribution at right
weight_left = 1 - weight_right; % percent, weight distribution at right
a = weight_left*track; %kg, distance between center of mass and right end of track
b = weight_right*track; %kg, distance between center of mass and left end of track

%Spring and damper
k_springf = 16640; %N/m, spring constant at front
r_damperf = 1416.7; %N.s/m damping constant at front
k_springr = 16640; %N/m, spring constant at rear
r_damperr = 1416.7; %N.s/m damping constant at rear

%Tire
mt = 27.4/2; %kg, mass of tire
k_tire_l = 128571.4; %N/m, spring constant of left tire
k_tire_r = 128571.4; %N/m, spring constant of right tire
r_tire = 0.5027/2; %m, radius or tire
jt = mt*r_tire^2; %kg-m^2, rotational inertia of tire

sp_max = 30 ; %rad, maximum restriction of slip angle (usually 0.52)

% Lateral acceleration input
%x = 1.5; %g, lateral acceleration 
%F = m*x*g; %N, Lateral force act on center of mass of carh 
hf = 0.20729; %m, height of certer of mass at front
hr = 0.20729; %m, height of certer of mass ar rear

% Angular velocity of steering wheel
steer_angle = 35.122; %deg, streer angle
steer_time = 0.15; %sec, sample time


%% Initial Conditions

initial_1 = [0 0 0 0];
initial_2 = [0 0 0 0 (mf*g*weight_left/k_springf) (mf*g*weight_right/k_springf) ((mf*weight_left+mt)*g/k_tire_l) ((mf*weight_right+mt)*g/k_tire_r) 0];
initial_3 = [0 0 0 0 (mr*g*weight_left/k_springr) (mr*g*weight_right/k_springr) ((mr*weight_left+mt)*g/k_tire_l) ((mr*weight_right+mt)*g/k_tire_r) 0];

%% Tire Data

Tire_data_1 = [0	10.16645	30.135844	49.972681	69.612462	88.995136	108.063766	126.765417	145.053829	162.887632	180.230352	197.053076	213.331781	229.048232	244.189973	258.748997	272.722636	286.111778	298.921762	311.160594	322.839396	333.970622	344.56984	354.653955	364.239424	373.34582	381.991381	390.195233	397.976949	405.355211	412.349148	418.976995	425.256992	431.206487	436.841937	442.179356	447.234758	452.023267	456.557782	460.852983	464.921325	468.774818	472.425028	475.882629	479.158743	482.262266	485.20254	487.98846	490.628478	493.130157	495.500614	497.746965	499.875438	501.892261	503.803661	505.614532	507.33021	508.95559	510.495564	511.95369	513.334417	514.641749	515.879244	517.05046	518.158067	519.205623	520.196242	521.132147	522.016453	522.850939	523.638719	524.381127	525.081277	525.740058	526.36014	526.942857	527.490433	528.003757	528.48461	528.93477	529.355571	529.747904	530.113548	530.452947	530.767881	531.05924	531.327912	531.574788	531.801203	532.008045	532.19576	532.365237	532.517366	532.652592	532.772249	532.876338	532.965747	533.041366	533.103197	533.152572	533.189047	533.189047;0	44.226471	97.638028	150.780467	203.523012	255.739777	307.310215	358.123122	408.073519	457.067103	505.018025	551.852444	597.504971	641.922227	685.060175	726.885008	767.372262	806.506367	844.279761	880.693335	915.753315	949.473047	981.871658	1012.971388	1042.799372	1071.386302	1098.764652	1124.969116	1150.037059	1174.005403	1196.912847	1218.798534	1239.70072	1259.658548	1278.709385	1296.891929	1314.242211	1330.796262	1346.588777	1361.654009	1376.024428	1389.731618	1402.805827	1415.276411	1427.171396	1438.517026	1449.339101	1459.66253	1469.510444	1478.90464	1487.866913	1496.417727	1504.575317	1512.359258	1519.786006	1526.872465	1533.634204	1540.086792	1546.243573	1552.118782	1557.724874	1563.074303	1568.178636	1573.048992	1577.695158	1582.128254	1586.356732	1590.389933	1594.236309	1597.904311	1601.401946	1604.735442	1607.912806	1610.940709	1613.824935	1616.571711	1619.186819	1621.676043	1624.044276	1626.296854	1628.438227	1630.472843	1632.405595	1634.240041	1635.980184	1637.630029	1639.192689	1640.672612	1642.072022	1643.394922	1644.643538	1645.820982	1646.930368	1647.974365	1648.954753	1649.874644	1650.73582	1651.540948	1652.291807	1652.991067	1653.639618	1653.639618;0	61.944621	133.303411	204.417993	275.169157	345.440357	415.119054	484.096267	552.270577	619.544566	685.827937	751.038843	815.100328	877.94567	939.513927	999.752612	1058.616797	1116.069562	1172.080659	1226.627401	1279.693776	1331.269108	1381.349837	1429.9373	1477.037722	1522.66267	1566.826822	1609.548861	1650.850584	1690.757345	1729.295388	1766.494963	1802.386315	1837.001919	1870.375134	1902.539768	1933.530517	1963.382521	1992.130922	2019.810416	2046.455699	2072.101911	2096.78286	2120.531906	2143.381968	2165.365516	2186.513243	2206.856288	2226.424452	2245.24665	2263.350461	2280.763463	2297.512346	2313.621574	2329.116504	2344.020265	2358.355544	2372.144581	2385.408728	2398.168002	2410.441531	2422.248886	2433.607416	2444.534468	2455.047391	2465.161309	2474.891791	2484.253959	2493.26116	2501.927182	2510.264926	2518.28729	2526.005397	2533.430811	2540.575097	2547.448486	2554.060765	2560.42172	2566.540246	2572.426131	2578.086936	2583.531557	2588.767557	2593.802052	2598.64305	2603.296778	2607.770353	2612.070002	2616.201509	2620.171101	2623.98456	2627.647224	2631.163987	2634.540186	2637.780714	2640.89002	2643.872551	2646.732757	2649.474195	2652.101758	2654.618561	2654.618561];
SL = [0.005 0.015 0.025 0.035 0.045 0.055 0.065 0.075 0.085 0.095 0.105 0.115 0.125 0.135 0.145 0.155 0.165 0.175 0.185 0.195 0.205 0.215 0.25 0.3 0.35 0.4 0.45 1];
Tire_nu = [0.175432451 0.610821376 0.997596235 1.320289407 1.456828812 1.777508204 1.930240974 2.046315274 2.134442831 2.201432689 2.252427804 2.291255514 2.320755254 2.343041513 2.359699259 2.371929007 2.380647527 2.386562981 2.390228069 2.392076798 2.392454057 2.391636796 2.382222904 2.359255659 2.332294025 2.304909574 2.278593357 0.3];



%% Damper Data

damper_input = [-127 -57.15 -50.8 -44.45 -38.1 -31.75 -25.4 -19.05 -12.7 -6.35 0 6.35 12.7 19.05 25.4 31.75 38.1 44.45 50.8 88.9 127];
damper_slope = [4.265905512 7.816622922 8.606692913 9.408548931 10.47769029 11.97448819 13.47125984 15.06787402 17.21338583 11.97448819 10 12.87251969 13.7705748 11.67512336 9.57957874 7.963023622 6.885328084 6.030011249 5.463385827 3.699257593 2.933748031];


%% Time Control Parameters

sim('Vehicle_Dynamic_sim_V01')

%% Understeer Coefficient 
Ca_f = (0.5*Fyf_L.Data)./(0.5.*(alphaf_l_L.Data + alphaf_r_L.Data));
Ca_r = (0.5*Fyr_L.Data)./(0.5.*(alphar_l_L.Data + alphar_r_L.Data));
k_steer = abs(Ca_r).*bb - abs(Ca_f).*aa;

%% Plot

figure('units','normalized','outerposition',[0 0 1 1])
subplot(4,1,1)
plot(Steer_angle_L)
title('Angle of Steering Wheel')
xlabel('Time [s]')
ylabel(' [deg]')

subplot(4,1,2)
plot(Steering_rate_L*pi/180)
title('Angular Velocity of Steering Wheel')
xlabel('Time [s]')
ylabel('[rad/s]')

subplot(4,1,3)
plot(ay_L/9.8)
title('Lateral acceleration(g)')
xlabel('Time [s]')
ylabel(' [m/sec^2]')

subplot(4,1,4)
plot(yaw_rate_linear)
title('Yaw Rate')
xlabel('Time [s]')
ylabel('[rad/s]')

figure('units','normalized','outerposition',[0 0 1 1])
plot(Fyf_L,'b');
hold on
plot(Fyr_L,'r');
title('Lateral Forces on Tires')
xlabel('Time [s]')
ylabel('Force [N]')
legend('Front Wheel','Rear Wheel')

figure('units','normalized','outerposition',[0 0 1 1])
plot(alphaf_l_L*180/pi,'b--');
hold on
plot(alphaf_r_L*180/pi,'b');
hold on
plot(alphar_l_L*180/pi,'r--');
hold on
plot(alphar_r_L*180/pi,'r');
title('Slip Angles of Tires')
xlabel('Time [s]')
ylabel('Slip Angle [deg]')
legend('Front Left Wheel','Front Right Wheel','Rear Left Wheel','Rear Right Wheel')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(Steer_angle_L)
hold on
length_of_time = size(t_L.Data);
k_ref = zeros(length_of_time);
plot(t_L.Data,k_ref,'r');
title('Angle of Steering Wheel')
xlabel('Time [s]')
ylabel(' [deg]')

subplot(3,1,2)
plot(X.Data(:,1), Y.Data(:,1),'k--')
title('Car trace')
xlabel('X-Axis Position [m]')
ylabel('Y-Axis Position [m]')

%hold on
%quiver(X.Data(:,1),Y.Data(:,1),u*cos(theta.Data(:,1)),u*sin(theta.Data(:,1)))

subplot(3,1,3)
plot(t_L.Data,k_steer,'b');
hold on
plot(t_L.Data,k_ref,'r');
title('Understeer Coefficient (K)')
xlabel('Time [s]')
ylabel('K')

%Yaw rate compare
figure('units','normalized','outerposition',[0 0 1 1])
plot(Yaw_CS,'black');
hold on
plot(yaw_rate_linear,'b');
hold on
plot(yaw_rate_simple,'r');
hold on
plot(yaw_rate_desired, 'b--');
title('Yaw Rate Comparison ');
xlabel('Time [s]');
ylabel('Yaw rate [deg/s]');
legend('CarSim','Ping-Linear','Ping-Simple','Ideal');
ylim([-110 110]);

%Lateral acceleration compare
figure('units','normalized','outerposition',[0 0 1 1])
plot(Ay_CS,'black');
hold on
plot(Ay_L,'b');
hold on
plot(Ay_Simple,'r');

title('Lateral Acceleration Comparison ');
xlabel('Time [s]');
ylabel('Lateral Acceleration [G]');
legend('CarSim','Ping-Linear','Ping-Simple');
ylim([-2 2]);


%Normal force compare (left front tire)
figure('units','normalized','outerposition',[0 0 1 1])
plot(normal_force_on_lefttire_front_L,'b');
hold on
plot(normal_force_on_lefttire_front_simple,'g');

title('Normal force on Left Tire');
xlabel('Time [s]');
ylabel('Normal force [N]');
legend('Ping-Linear','Simple-Model');
%ylim([-30 30]);
%ratio = Fyf/Fyr;
%plot(t,ratio)