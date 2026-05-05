% Load preprocess data สำหรับ Parameter Estimation
tmp = load('/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Preprocess_Data/chirp05_run1.mat');
t_in   = tmp.t;
Vin_in = tmp.Vin;
% Format สำหรับ From Workspace [time, signal]
Vin_ws = [t_in, Vin_in];
% Stop time ให้ตรงกับ data
set_param('Simulation_OL', 'StopTime', num2str(t_in(end)));


% Output data สำหรับ match
omega_ws = [t_in, tmp.omega_f];

set_param('Simulation_OL', 'Solver', 'ode4');
set_param('Simulation_OL', 'SolverType', 'Fixed-step');
set_param('Simulation_OL', 'FixedStep', '0.001');
set_param('Simulation_OL', 'StopTime', '60');