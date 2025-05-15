%% Input Conditions
Input.SlipRatio = 0.1;                 % Example Slip Ratio [unitless]
Input.NormalLoad_Fz = 272*(1-0.543)*9.8;            % Example Normal Load [N]
Input.Velocity = 0;                   % Example Vehicle Longitudinal Velocity [m/s]

%% Load Tire Parameters
load('Hoosier_R25B_16x75-10x7.mat');   % Loads "Tire" struct
Tire.Pacejka.L.mu.x = 2/3;             % Modify friction if needed

%% Define Parameters Struct
Parameter.Tire = Tire;                 % Store tire data
Parameter.Pressure = 70;               % Inflation Pressure [psi]
Parameter.Inclination = 1;             % Inclination Angle [deg]
Parameter.Model = 'Pacejka';           % Model type

%% Run Simulink Model
simOut = sim('tire2model.slx', ...
    'SimulationMode', 'normal', ...
    'SrcWorkspace', 'current');

%% Extract Simulink Output
Fx_max = simOut.logsout.getElement('Fx_max').Values.Data;

%% Display Results
fprintf('Computed Max Longitudinal Force: %.2f N\n', Fx_max);
