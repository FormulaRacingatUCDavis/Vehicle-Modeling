%function [Chassis, Tire, Suspension, Controls, Powertrain, Aero] = ...
%    VehicleBusDefinition( )
%% Chassis Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'Longitudinal';
elems(1).Dimensions = [1,3];

elems(2) = Simulink.BusElement;
elems(2).Name = 'Lateral';
elems(2).Dimensions = [1,3];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Ride';
elems(3).Dimensions = [1,3];

elems(4) = Simulink.BusElement;
elems(4).Name = 'Roll';
elems(4).Dimensions = [2,3];

elems(5) = Simulink.BusElement;
elems(5).Name = 'Pitch';
elems(5).Dimensions = [1,3];

elems(6) = Simulink.BusElement;
elems(6).Name = 'Yaw';
elems(6).Dimensions = [1,3];

elems(7) = Simulink.BusElement;
elems(7).Name = 'Global_Displacement';
elems(7).Dimensions = [2,1];

elems(8) = Simulink.BusElement;
elems(8).Name = 'Curvature';

elems(9) = Simulink.BusElement;
elems(9).Name = 'Body_Slip';

Chassis = Simulink.Bus;
Chassis.Elements = elems;
clear elems

%% Tire Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'Spin_Rate';
elems(1).Dimensions = [4,1];

elems(2) = Simulink.BusElement;
elems(2).Name = 'Slip_Ratio';
elems(2).Dimensions = [4,1];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Slip_Angle';
elems(3).Dimensions = [4,1];

elems(4) = Simulink.BusElement;
elems(4).Name = 'Longitudinal_Force';
elems(4).Dimensions = [4,1];

elems(5) = Simulink.BusElement;
elems(5).Name = 'Lateral_Force';
elems(5).Dimensions = [4,1];

elems(6) = Simulink.BusElement;
elems(6).Name = 'Normal_Load';
elems(6).Dimensions = [4,1];

elems(7) = Simulink.BusElement;
elems(7).Name = 'Overturning_Moment';
elems(7).Dimensions = [4,1];

elems(8) = Simulink.BusElement;
elems(8).Name = 'Rolling_Resistance';
elems(8).Dimensions = [4,1];

elems(9) = Simulink.BusElement;
elems(9).Name = 'Aligning_Moment';
elems(9).Dimensions = [4,1];

elems(10) = Simulink.BusElement;
elems(10).Name = 'Effective_Radius';
elems(10).Dimensions = [4,1];

elems(11) = Simulink.BusElement;
elems(11).Name = 'Loaded_Radius';
elems(11).Dimensions = [4,1];

elems(12) = Simulink.BusElement;
elems(12).Name = 'Pressure';
elems(12).Dimensions = [4,1];

elems(13) = Simulink.BusElement;
elems(13).Name = 'Carcass_Temperature';
elems(13).Dimensions = [4,1];

elems(14) = Simulink.BusElement;
elems(14).Name = 'Tread_Temperature';
elems(14).Dimensions = [4,1];

Tire = Simulink.Bus;
Tire.Elements = elems;
clear elems

%% Suspension Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'Steer';
elems(1).Dimensions = [4,1];

elems(2) = Simulink.BusElement;
elems(2).Name = 'Inclination';
elems(2).Dimensions = [4,1];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Base';
elems(3).Dimensions = [4,1];

elems(4) = Simulink.BusElement;
elems(4).Name = 'Track';
elems(4).Dimensions = [4,1];

elems(5) = Simulink.BusElement;
elems(5).Name = 'Jounce';
elems(5).Dimensions = [4,1];

elems(6) = Simulink.BusElement;
elems(6).Name = 'Shock';
elems(6).Dimensions = [4,1];

elems(7) = Simulink.BusElement;
elems(7).Name = 'ARB';
elems(7).Dimensions = [4,1];

elems(8) = Simulink.BusElement;
elems(8).Name = 'Roll_IC';
elems(8).Dimensions = [4,2];

elems(9) = Simulink.BusElement;
elems(9).Name = 'Pitch_IC';
elems(9).Dimensions = [4,2];

Suspension = Simulink.Bus;
Suspension.Elements = elems;
clear elems

%% Controls Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'Steer_Angle';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Rack_Displacement';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Steer_Effort';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Brake_Pedal_Force';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Brake_Line_Pressure';
elems(5).Dimensions = [2,1];

elems(6) = Simulink.BusElement;
elems(6).Name = 'Brake_Torque';
elems(6).Dimensions = [4,1];

elems(7) = Simulink.BusElement;
elems(7).Name = 'Torque_Request';

Controls = Simulink.Bus;
Controls.Elements = elems;
clear elems

%% Powertrain Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'SOC';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Pack_Voltage';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Pack_Current';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Motor_Torque';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Motor_Speed';

elems(6) = Simulink.BusElement;
elems(6).Name = 'Drive_Torque';
elems(6).Dimensions = [4,1];

elems(7) = Simulink.BusElement;
elems(7).Name = 'Diff_State';

elems(8) = Simulink.BusElement;
elems(8).Name = 'Efficiency';

Powertrain = Simulink.Bus;
Powertrain.Elements = elems;
clear elems

%% Aero Bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'Drag';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Side_Force';

elems(3) = Simulink.BusElement;
elems(3).Name = 'Downforce';

elems(4) = Simulink.BusElement;
elems(4).Name = 'Roll_Moment';

elems(5) = Simulink.BusElement;
elems(5).Name = 'Pitch_Moment';

elems(6) = Simulink.BusElement;
elems(6).Name = 'Yaw_Moment';

Aero = Simulink.Bus;
Aero.Elements = elems;
clear elems