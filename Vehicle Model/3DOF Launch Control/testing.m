%% Input Conditions
Input.SlipRatio = 0.1;                 % Example Slip Ratio [unitless]
Input.NormalLoad_Fz = 272*(1-0.543)*9.8;            % Example Normal Load [N]
Input.Velocity = 0;                   % Example Vehicle Longitudinal Velocity [m/s]

%% Load Tire Parameters
load('Hoosier_R25B_16x75-10x7.mat');   % Loads "Tire" struct
Tire.Pacejka.L.mu.x = 2/3;             % Modify friction if needed

%% Define Parameters Struct
Parameter.Tire = Tire;                 % Store tire data
Parameter.Pressure = Pressure;               % Inflation Pressure [psi]
Parameter.Inclination = Inclination;             % Inclination Angle [deg]
Parameter.Model = 'Pacejka';           % Model type


%% Simulik Inputs
SimulinkInput = Simulink.SimulationInput('tire2model');
SimulinkInput = SimulinkInput.setVariable('SlipRatio', Input.SlipRatio);
SimulinkInput = SimulinkInput.setVariable('NormalLoad_Fz', Input.NormalLoad_Fz);
SimulinkInput = SimulinkInput.setVariable('Velocity', Input.Velocity);
SimulinkInput = SimulinkInput.setVariable('Tire', Parameter.Tire);
SimulinkInput = SimulinkInput.setVariable('Pressure', Parameter.Pressure);
SimulinkInput = SimulinkInput.setVariable('Inclination', Parameter.Inclination);
SimulinkInput = SimulinkInput.setVariable('Model', Parameter.Model);

%% Run Simulation
simOut = sim(SimulinkInput);

%% Retrieve output (ensure Fx_max is logged)
Fx_max = simOut.logsout.getElement('Fx_max').Values.Data;

fprintf('Computed Max Longitudinal Force: %.2f N\n', Fx_max);


