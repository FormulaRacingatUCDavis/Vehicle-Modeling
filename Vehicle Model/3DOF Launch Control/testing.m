% Load tire parameter structure from .mat file
load('Hoosier_R25B_16x75-10x7.mat');        % e.g., 'TireData.mat' contains a struct named Tire
load('vehcileConstants2025.m')

ContactPatchLoads( Tire, 0, 0.01, Fz0 , 80, 0, 10, 1, Fidelity ) 

%{
SlipRatio_in = 0.1;   
Fz_in        = m.total * g;  % example normal load [N]
V_in         = 15;    % example velocity [m/s]

% Assign inputs to base workspace 
SlipRatio = SlipRatio_in;
NormalLoad = Fz_in;
Velocity = V_in;

modelName = "tire2model";        
load_system(modelName);            % ensure the model is loaded


simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('Tire', Tire.Value);       % pass the Tire struct value
simIn = simIn.setVariable('SlipRatio', SlipRatio_in);
simIn = simIn.setVariable('NormalLoad', Fz_in);
simIn = simIn.setVariable('Velocity', V_in);
simOut = sim(simIn);

% Retrieve output from simulation
logsout = simOut.logsout;                    % SimulationOutput logs
Fx_signal = logsout.get("Fx_max");           % get the logged signal (assuming signal name 'Fx_max')
Fx_max_time = Fx_signal.Values.Time;         % time points (if any dynamic variation; for static calc this might be just [0])
Fx_max_data = Fx_signal.Values.Data;         % the computed Fx_max value(s)

% Display or use the result
disp("Computed Fx_max = " + Fx_max_data + " N");

}%
