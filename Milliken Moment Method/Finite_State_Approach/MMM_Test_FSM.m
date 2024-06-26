clc; clear; close all;

%% Moment Method Diagram

C_n = N ./ (m .* g .* (l_f + l_r));

l_f = 1.25;
l_r = 1.35;
I_z = 2500;
k_phi_f = 110000;
m = 1500;
t_wf=1.5;
t_wr = 1.5;
k_phi_r = 70000;
h_cog = 0.5;
SR = 15;

%% Vehicle Model
x_ddot = ((Fx(1)+Fx(2)).*cosd(delta)-(Fy(1)+Fy(2)).*sind(delta)+Fx(3)+...
    Fx(4)) ./m + y_dot.*r_dot;
y_ddot = ((Fy(1)+Fy(2)).*cosd(delta)+(Fx(1)+Fx(2)).*sind(delta)+Fy(3)+...
    Fy(4)) ./m + x_dot.*r_dot;
r_ddot = (((Fy(1)+Fy(2)).*cosd(delta)+(Fx(1)+Fx(2)).*sind(delta)).*l_f -...
    (Fy(3)+Fy(4)).*l_r ) ./ I_z;

%%% Check if its supposed to be atand or atan, and if alpha is radians 
alpha(1) = delta - atand( (r_dot.*l_f)./(x_dot - r_dot.*t_wf./2) +...
    y_dot./x_dot);
alpha(2) = delta - atand( (r_dot.*l_f)./(x_dot + r_dot.*t_wf./2) +...
    y_dot./x_dot);
alpha(3) = - atand( -1.*(r_dot.*l_f)./(x_dot - r_dot.*t_wf./2) +...
    y_dot./x_dot);
alpha(4) = - atand( -1.*(r_dot.*l_f)./(x_dot + r_dot.*t_wf./2) +...
    y_dot./x_dot);

%%% Weight transfer CHECK THESE EQUATIONS
delta_Fz_long = (m.*h_cog)./(2.*W.*B).*x_ddot;
delta_Fz_lat_f =((m.*h_cog)./t_wf) .* (k_phi_f./(k_phi_f+k_phi_r)).*y_ddot;
delta_Fz_lat_r =((m.*h_cog)./t_wr) .* (k_phi_r./(k_phi_f+k_phi_r)).*y_ddot;

Fz(1) = (m.*g.*l_f) ./(2.*W.*B) + delta_Fz_long + delta_Fz_lat_f;
Fz(2) = (m.*g.*l_f) ./(2.*W.*B) + delta_Fz_long - delta_Fz_lat_f;
Fz(3) = (m.*g.*l_r) ./(2.*W.*B) - delta_Fz_long + delta_Fz_lat_r;
Fz(4) = (m.*g.*l_r) ./(2.*W.*B) - delta_Fz_long - delta_Fz_lat_r;

%% Tire Model 

mu_x = [1.2,1.2];
mu_y = [0.935, 0.961];
Cx = [1.69, 1.69];
Cy = [1.19, 1.69];
Bx = [11.7, 11.1];
By = [8.86, 9.3];
Ex = [0.377, 0.362];
Ey = [-1.21, -1.11];
Bx1 = [12.4, 12.4];
Bx2 = [-10.8, -10.8];
By1 = [6.46, 6.46];
By2 = [4.20, 4.20];
Cx_alpha = [1.09, 1.09];
Cy_lambda = [1.08, 1.08];

%Radians? or Degrees
Fx0 = mu_x.*Fz.*sin(Cx.*atan(Bx.*gamma-Ex(Bx.*gamma-atan(...
    Bx.*gamma))));
Fy0 = mu_y.*Fz.*sin(Cy.*atan(By.*alpha-Ey(By.*alpha-atan(...
    By.*alpha))));
Hx_alpha = Bx1.*cos(atan(Bx2.*gamma));
Gx_alpha = cos(Cx_alpha.*atan(Hx_alpha.*alpha));
Hy_lambda = By1.*cos(atan(By2.*alpha));
Gy_lambda = cos(Cy_lambda.*atan(Hy_lambda.*lambda));
Fx=Fx0.*Gx_alpha;
Fy = Fy0.*Gy_lambda;