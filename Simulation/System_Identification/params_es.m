% =============================================
% Script-based Parameter Estimation
% G6 Circular Pick and Place Robot
% =============================================

% --- เปิด model ก่อน ---
model_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/Simulation_OL.slx';
model_name = 'Simulation_OL';

if ~bdIsLoaded(model_name)
    load_system(model_path);
end

preprocess_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Preprocess_Data/';

% --- Load Chirp 0.5Hz run1 ---
tmp        = load(fullfile(preprocess_path, 'chirp05_run1.mat'));
t_meas     = tmp.t;
omega_meas = tmp.omega_f;
Vin_in     = tmp.Vin;
Vin_ws     = [t_meas, Vin_in];

set_param(model_name, 'StopTime', num2str(t_meas(end)));

% --- ทดสอบ simulation ครั้งเดียวก่อน ---
fprintf('Testing single simulation...\n');
k_e     = 0.0321;
k_t     = 0.0382;
B_arm   = 0.001;
eta     = 0.85;
J_scale = 1.0;

tic
out = sim(model_name);
elapsed = toc;
fprintf('Simulation time: %.2f s\n', elapsed);

t_sim     = out.omega_arm.Time;
omega_sim = out.omega_arm.Data;
fprintf('omega_sim max = %.4f rad/s\n', max(abs(omega_sim)));
fprintf('omega_meas max = %.4f rad/s\n', max(abs(omega_meas)));
fprintf('Samples sim  = %d\n', length(t_sim));
fprintf('Samples meas = %d\n', length(t_meas));

% --- Plot เปรียบเทียบก่อน optimize ---
figure('Name', 'Before Optimization', 'NumberTitle', 'off');
omega_interp = interp1(t_sim, omega_sim, t_meas, 'linear', 'extrap');
plot(t_meas, omega_meas,   'b',  'LineWidth', 1.2, 'DisplayName', 'Measured');
hold on;
plot(t_meas, omega_interp, 'r--','LineWidth', 1.2, 'DisplayName', 'Simulated (initial)');
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Before Optimization — Initial Parameters');
legend('Location','best'); grid on;

ss_res = sum((omega_meas - omega_interp).^2);
ss_tot = sum((omega_meas - mean(omega_meas)).^2);
fit_0  = (1 - ss_res/ss_tot) * 100;
fprintf('Initial Fit Score = %.2f%%\n', fit_0);