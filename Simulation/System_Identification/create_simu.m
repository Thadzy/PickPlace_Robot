% =============================================
% create_simu.m
% สร้าง First Principles Motor Model
% สำหรับ Parameter Estimation
% Input: V_in (From Workspace), Output: omega_arm
% =============================================

model_name = 'Params_Estimate';
save_path  = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);

% --- Initial Parameter Values ---
L_m     = 0.00147261;
R_m     = 1.53982909;
N_total = 70;
K_e     = 0.0321;
K_t     = 0.0382;
B       = 0.001;
J       = 0.2546;
eta     = 0.85;

% --- Load Data ---
preprocess_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Preprocess_Data/';
tmp      = load(fullfile(preprocess_path, 'chirp05_run1.mat'));
t_in     = tmp.t;
Vin_in   = tmp.Vin;
omega_ws = tmp.omega_f;
Vin_ws   = [t_in, Vin_in];

% --- Block Positions ---
pos_vin     = [30  193  130 207];
pos_sum1    = [175 185  205 215];
pos_1overLm = [240 188  290 212];
pos_integ_i = [325 188  375 212];
pos_Rm      = [240 243  290 267];
pos_KeN     = [240 298  290 322];
pos_Kt      = [410 188  460 212];
pos_eta     = [485 188  535 212];
pos_N       = [560 188  610 212];
pos_sum2    = [645 185  675 215];
pos_1overJ  = [710 188  760 212];
pos_integ_w = [795 188  845 212];
pos_B       = [710 243  760 267];
pos_out     = [895 193  925 207];

% --- Add Blocks ---
add_block('simulink/Sources/From Workspace', [model_name '/Vin_Source'], ...
    'VariableName', 'Vin_ws', ...
    'SampleTime',   '0.001', ...
    'Position', pos_vin);

add_block('simulink/Math Operations/Sum', [model_name '/Sum1'], ...
    'Inputs', '|+--', 'Position', pos_sum1);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_1overLm'], ...
    'Gain', '1/L_m', 'Position', pos_1overLm);

add_block('simulink/Continuous/Integrator', [model_name '/Integrator_i'], ...
    'Position', pos_integ_i);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Rm'], ...
    'Gain', 'R_m', 'Position', pos_Rm);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_KeN'], ...
    'Gain', 'K_e * N_total', 'Position', pos_KeN);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kt'], ...
    'Gain', 'K_t', 'Position', pos_Kt);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_eta'], ...
    'Gain', 'eta', 'Position', pos_eta);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_N'], ...
    'Gain', 'N_total', 'Position', pos_N);

add_block('simulink/Math Operations/Sum', [model_name '/Sum2'], ...
    'Inputs', '|+-', 'Position', pos_sum2);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_1overJ'], ...
    'Gain', '1/J', 'Position', pos_1overJ);

add_block('simulink/Continuous/Integrator', [model_name '/Integrator_omega'], ...
    'Position', pos_integ_w);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_B'], ...
    'Gain', 'B', 'Position', pos_B);

add_block('simulink/Sinks/Out1', [model_name '/omega_arm'], ...
    'Position', pos_out);

% --- Add Lines ---
add_line(model_name, 'Vin_Source/1',       'Sum1/1');
add_line(model_name, 'Sum1/1',             'Gain_1overLm/1');
add_line(model_name, 'Gain_1overLm/1',     'Integrator_i/1');
add_line(model_name, 'Integrator_i/1',     'Gain_Kt/1');
add_line(model_name, 'Gain_Kt/1',          'Gain_eta/1');
add_line(model_name, 'Gain_eta/1',         'Gain_N/1');
add_line(model_name, 'Gain_N/1',           'Sum2/1');
add_line(model_name, 'Integrator_i/1',     'Gain_Rm/1');
add_line(model_name, 'Gain_Rm/1',          'Sum1/2');
add_line(model_name, 'Sum2/1',             'Gain_1overJ/1');
add_line(model_name, 'Gain_1overJ/1',      'Integrator_omega/1');
add_line(model_name, 'Integrator_omega/1', 'omega_arm/1');
add_line(model_name, 'Integrator_omega/1', 'Gain_B/1');
add_line(model_name, 'Gain_B/1',           'Sum2/2');
add_line(model_name, 'Integrator_omega/1', 'Gain_KeN/1');
add_line(model_name, 'Gain_KeN/1',         'Sum1/3');

% --- Solver Settings ---
set_param(model_name, 'Solver',     'ode4');
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'FixedStep',  '0.001');
set_param(model_name, 'StopTime',   num2str(t_in(end)));

% --- Save ---
save_system(model_name, save_path);
fprintf('Model saved: %s\n', save_path);

% --- Test ---
fprintf('Testing simulation...\n');
tic
out = sim(model_name);
toc

omega_sim    = out.yout{1}.Values.Data;
t_sim        = out.yout{1}.Values.Time;
omega_interp = interp1(t_sim, omega_sim, t_in, 'linear', 'extrap');

ss_res    = sum((omega_ws - omega_interp).^2);
ss_tot    = sum((omega_ws - mean(omega_ws)).^2);
fit_score = (1 - ss_res/ss_tot) * 100;

fprintf('omega_sim max     = %.4f rad/s\n', max(abs(omega_sim)));
fprintf('omega_meas max    = %.4f rad/s\n', max(abs(omega_ws)));
fprintf('Initial Fit Score = %.2f%%\n', fit_score);

figure('Name','Before Optimization','NumberTitle','off');
plot(t_in, omega_ws,     'b',   'LineWidth', 1.2, 'DisplayName', 'Measured');
hold on;
plot(t_in, omega_interp, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Simulated');
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Before Optimization — Initial Parameters');
legend('Location','best'); grid on;

% --- เปิด Parameter Estimator App ---
fprintf('\nOpening Parameter Estimator App...\n');
parameterEstimator(model_name);