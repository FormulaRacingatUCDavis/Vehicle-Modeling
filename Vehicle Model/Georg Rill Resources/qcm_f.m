function xdot = qcm_f(t,x)
% quarter car model with trailing arm suspension

global g s a b h r0
global mC mK mW ThetaK ThetaW
global Ts0 cs ds cx cz dx
global ustep tstep

% state variables
z = x(1); beta = x(2); phi = x(3);
zd = x(4); betad = x(5); phid = x(6);

% step input to actuator @ t = tstep
if t < tstep, u = 0; else u = ustep; end

% torque in revolute joint
Ts = - ( Ts0 + cs*beta + ds*betad );

% tire deflection (static tire radius)
rS = h + z - b + a*sin(beta) - u ;

% longitudinal tire force (adhesion assumed)
Fx = - cx *( a*(1-cos(beta)) - rS*phi ) ...
    - dx *( a*sin(beta)*betad - rS*phid ) ;

% vertical tire force (contact assumed)
Fz = cz *( r0 - rS );

% mass atrix
Massma=[     mC+mK+mW       (s*mK+a*mW)*cos(beta) 0  ; ...
       (s*mK+a*mW)*cos(beta) ThetaK+s^2*mK+a^2*mW 0  ; ...
                0                     0         ThetaW ];

%vector of generalized forces and torques
qgen=[ Fz-(mC+mK+mW)*g+(s*mK+a*mW)*sin(beta)*betad^2 ; ...
      Ts-(s*mK+a*mW)*cos(beta)*g+a*(Fx*sin(beta)+Fz*cos(beta));...
       -rS*Fx ];

% state derivatives
xdot = [ zd; betad; phid; Massma\qgen ];

end