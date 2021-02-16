%% Yaw acce;eration 
psi_ddot = ( a*( Fy(1)*cosd(delta) + Fx(1)*sind(delta) ) - b*Fy(2) ) / Izz;
%% Yaw Velocity
v_psi = a_x*v_y - ( Fx(1)*cosd(delta) - Fy(1)*sind(delta) + Fx(2) ) / (m*v_y) ;
v_psi = ( Fy(1)*cosd(delta) + Fx(1)*sind(delta) + Fy(2) ) / (m*v_x) - (a_y/v_x);
%% y_ddot
a_y = ( Fy(1)*cosd(delta) + Fx(1)*sind(delta) + Fy(2) ) / m - (v_x * v_psi);
