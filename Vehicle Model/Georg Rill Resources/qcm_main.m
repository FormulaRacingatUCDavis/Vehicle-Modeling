% globals
global g s a b h r0
global mC mK mW ThetaK ThetaW
global Ts0 cs ds cx cz dx
global ustep tstep

% vehicle data
qcm_data

% define step input
ustep=0.05;
tstep=0.75;

% initial states
x0 = [ 0; 0; 0; 0; 0; 0];

% time siulation
t0=0; tE=1.5;
[tout,xout] = ode45(@qcm_f,[t0,tE],x0);

% plot results
subplot(3,1,1), plot(tout,xout(:,1)), grid on
              xlabel('t [s]'), ylabel('z [m]')
subplot(3,1,2), plot(tout,xout(:,2)*180/pi), grid on
              xlabel('t [s]'), ylabel('\beta [deg]')
subplot(3,1,3), plot(tout,xout(:,3)*180/pi), grid on
              xlabel('t [s]'), ylabel('\phi [deg]')
