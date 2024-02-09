function [ds,ext] = weight_tr_fcn(t,s,Fcl,Fcr,Vroad_left,Vroad_right)

global m g j a b k_spring r_damper k_tire_l k_tire_r F h mt 

% state variables
Pj = s(1); %roll momentum of sprung mass
Pv = s(2); %heave momentum of sprung mass
Ptl = s(3); %momentum of tire at left
Ptr = s(4); %momentum of tire at right
qsl = s(5); %displacement of spring at left
qsr = s(6); %displacement of spring at right
qslt = s(7); %displacement of tire at left
qsrt = s(8); %displacement of tire at right
roll_angle = s(9); %Roll angle

%Lateral acceleration input
if t>5 && t<15
    Fs=F;
else
    Fs=0;
end

if qslt < 0
    k_tire_l_s = 0;
else
    k_tire_l_s = k_tire_l;
end

if qsrt < 0
    k_tire_r_s = 0;
else
    k_tire_r_s = k_tire_r;
end

Fcl = 0;
Fcr = 0;
Vroad_left = 0;
Vroad_right = 0;

%State variable
dPj = (-(r_damper/j)*(b^2+a^2))*Pj + ((r_damper/m)*(b-a))*Pv - ((b*r_damper)/mt)*Ptl + ((a*r_damper/mt))*Ptr + b*k_spring*qsl - a*k_spring*qsr + Fs*h*cos(roll_angle) + m*g*sin(roll_angle) + b*Fcl - a*Fcr;
      
dPv = (r_damper/j)*(b-a)*Pj - (2*r_damper/m)*Pv + (r_damper/mt)*Ptl + (r_damper/mt)*Ptr - k_spring*qsl - k_spring*qsr - Fcl - Fcr + m*g;
      
dPtl = (-b*r_damper/j)*Pj + (r_damper/m)*Pv - (r_damper/mt)*Ptl + k_spring*qsl - k_tire_l_s*qslt + Fcl + mt*g;
     
dPtr = (a*r_damper/j)*Pj + (r_damper/m)*Pv - (r_damper/mt)*Ptr + k_spring*qsr - k_tire_r_s*qsrt + Fcr + mt*g;
      
dqsl = (-b/j)*Pj + (1/m)*Pv - (1/m)*Ptl;

dqsr = (a/j)*Pj + (1/m)*Pv - (1/m)*Ptr;

dqslt = (1/mt)*Ptl - Vroad_left;

dqsrt = (1/mt)*Ptr - Vroad_right;

droll_angle = Pj/j;

%Output
force_input = Fs;

force_on_lefttire = k_tire_l_s*qslt;

force_on_righttire = k_tire_r_s*qsrt;

ds = [dPj;dPv;dPtl;dPtr;dqsl;dqsr;dqslt;dqsrt;droll_angle];
ext = [force_input;force_on_lefttire;force_on_righttire];

    