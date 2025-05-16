% Load tire parameter structure from .mat file
load('Hoosier_R25B_16x75-10x7.mat');        % e.g., 'TireData.mat' contains a struct named Tire
load('constants.mat')
Parameter.Pacejka = Tire.Pacejka;

% ContactPatchLoads( Tire, SlipAngle, SlipRatio, FzR , Pressure, Inclination, Velocity, Idx, Fidelity ) 
sim("launchControl.slx")
