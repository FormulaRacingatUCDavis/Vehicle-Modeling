function carParams = FE13(carParams)
    arguments
        carParams = struct()
    end
    
    % FE12 constants
    % Chasis/suspension constants
    carParams.m = 264.3;                        % Total Mass [kg]
    carParams.PFront = 52.6/100;              % Percent Mass Front [0-1]
    carParams.WB = 1.572;                     % Wheelbase [m]
    carParams.TWf = 1.223;                    % Trackwidth [m]
    carParams.TWr = 1.241;
    carParams.toe_f = -0.5 * (pi/180);        % Toe Angles [radians] (positive is inwards)
    carParams.toe_r = 0.0 * (pi/180);
    carParams.hCG = 0.292;                    % CG height [m]
    carParams.CamberFront = -1.248; % deg 
    carParams.CamberRear = -1.58;    % deg   

    % Sampo model
    
    
    % Aero constants
    carParams.Cl = 2.805385008;
    carParams.Cd = 1.185948207; 
    carParams.CoP = 63.789/100;                   % front downforce distribution (%)
    carParams.rho = 1.165;                    % kg/m^3
    carParams.crossA = 0.9138362;                % m^2
    
    % braking system
    carParams.B_FBB = 55/45;                    % Front brake bias
    
    % powertrain
    carParams.drivetrain.M_torque = 220;             % [N·m] max motor torque (torque cap)
    carParams.drivetrain.Power    = 80;              % [kW] motor power (constant-power region)
    carParams.drivetrain.KMV      = 12;              % [RPM/V] motor speed constant
    carParams.drivetrain.Max_V    = 504;             % [V] max DC link voltage
    carParams.drivetrain.tire_diameter_in = 16;      % [in] tire diameter
    carParams.drivetrain.FDR               = 2.91;   % final drive ratio
    carParams.drivetrain.eta_drivetrain    = 0.92;    
    
    % Tire
    tire = load('Hoosier_R20_16(18)x75(60)-10x8(7).mat');
    tire.Idx = 1;                     % Moment of Inertia in x for wheel
    tire.TirePressure = 70;           % kPa
    tire.Model = struct( 'Pure', 'Pacejka', 'Combined', 'MNC' );
    tire.CorrectionFactor = 1;
    carParams.tire = tire;
end