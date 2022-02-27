qcm_data

% steady state tire forces
Fx = 0; Fz = (mC + mK + mW ) * g ;

% knuckle and trailing arm
mS = mK + mW; sS = ( s*mK + a*mK ) / mS;

% solve non-linear equation by matlab-function fzero
be0=fzero(@(be) -(Ts0+cs*be)-sS*mS*cos(be)*g+a*(Fx*sin(be)+Fz*cos(be)),0);
disp(['be_st=',num2str(be0*180/pi),' [Grad]')