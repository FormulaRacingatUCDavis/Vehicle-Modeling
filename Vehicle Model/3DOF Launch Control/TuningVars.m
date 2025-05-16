%% Tuning
%PID Loop
PID.Kp = m.total/5;
PID.Ki = m.total/25;
PID.Kd = m.total/10;


 %%% Nominal Test Case Conditions
Pressure    = 70;
Inclination = 1;
Velocity    = 10;
Idx         = 1;
Model       = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );