function F_trac = motorLimit_default(carParams, v)

    M_torque = carParams.drivetrain.M_torque;                    % [N·m] max motor torque (torque cap)
    Power = carParams.drivetrain.Power;                          % [kW] motor power (constant-power region)
    KMV = carParams.drivetrain.KMV;                              % [RPM/V] motor speed constant
    Max_V = carParams.drivetrain.Max_V;                          % [V] max DC link voltage
    M_RPM    = KMV * Max_V;                                      % [RPM] motor max speed (no-load)
    tire_diameter_in = carParams.drivetrain.tire_diameter_in;    % [in] tire diameter
    FDR = carParams.drivetrain.FDR;                              % final drive ratio
    eta_drivetrain = carParams.drivetrain.eta_drivetrain;   

    %% Derived geometry
    tire_radius_m = (tire_diameter_in * 0.0254) / 2;    % [m]
    
    %% Torque & power maps vs RPM 
    RPM    = 0:M_RPM;                                        % [RPM], includes 0
    Torque = 9.549297 * (Power*1000) ./ max(RPM, 1);         % [N·m]
    Torque = min(Torque, M_torque);                          % clamp to max torque

    wheel_omega = v / tire_radius_m;                           % [rad/s]
    motor_RPM   = (wheel_omega * 60/(2*pi)) * FDR;             % [RPM]

    if motor_RPM >= M_RPM 
        F_trac = 0; 
        return
    end

    % Available motor torque at this RPM 
    Tm = interp1(RPM, Torque, motor_RPM, 'linear', 'extrap'); 
    if Tm < 0, Tm = 0; end

    % Wheel traction force 
    Twheel = Tm * FDR * eta_drivetrain;                       
    F_trac = Twheel / tire_radius_m;  
end
