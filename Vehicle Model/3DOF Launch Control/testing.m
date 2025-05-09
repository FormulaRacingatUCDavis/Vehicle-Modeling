function runLaunchControl(SlipAngle, SlipRatio, NormalLoad, Pressure, Inclination, Velocity, Idx, Model)
% runLaunchControl  Wrapper to execute 3DOF Launch Control tests
%   Uses external tire data and ContactPatchLoads from separate repos.

    % Determine this script's directory
    thisDir = fileparts(mfilename('fullpath'));
    % --- Run local constants script ---
    run(fullfile(thisDir, 'vehcileConstants2025.m'));

    % --- Constants and SIM tuning  ---
    sim.Vlong    = 0;    % Initial vehicle velocity [m/s]
    sim.VGoal    = 100;
    Kp           = m.total/5;
    Ki           = m.total/25;
    Kd           = m.total/10;
    FilterCoefN  = 100;
    %% I am not liable for how cooked these enxt few lines are
    SlipAngle    = 0.5;  
    SlipRatio    = 0.1;
    NormalLoad   = 1;
    Pressure     = 70;
    Inclination  = 1;
    Velocity     = 10;
    Idx          = 1;
    Model        = struct('Pure','Pacejka','Combined','MNC');

    % --- External resources ---
    tireMatFile = 'C:\Users\nhola\OneDrive\Documents\GitHub\Tire-Data\Models\Hoosier_R25B_16x75-10x7.mat';
    tireModelingRepo = 'C:\Users\nhola\OneDrive\Documents\GitHub\Tire-Modeling';

    % Load tire data from .mat
    tireData = load(tireMatFile);

    % Add the entire Tire-Modeling repo to path (includes ContactPatchLoads and all subroutines)
    addpath(genpath(tireModelingRepo));

    % Call ContactPatchLoads (now all dependencies are on the path)
    [Fx, Fy, Mz, Mx, My] = ContactPatchLoads(...
        tireData.Tire, SlipAngle, SlipRatio, NormalLoad, ...
        Pressure, Inclination, Velocity, Idx, Model);

    % Display results
    fprintf('Fx: %g, Fy: %g, Mz: %g, Mx: %g, My: %g\n', Fx, Fy, Mz, Mx, My);

    % Clean up path
    rmpath(genpath(tireModelingRepo));
end
