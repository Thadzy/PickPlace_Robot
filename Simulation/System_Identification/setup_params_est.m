% =============================================
% setup_param_est.m
% Setup สำหรับ Parameter Estimation
% G6 Circular Pick and Place Robot
% =============================================
% วิธีใช้:
% 1. รัน script นี้
% 2. เปิด Parameter Estimator App ใน Simulink
% 3. กด Update from workspace ใน Edit Experiment
% 4. กด Estimate
% =============================================

clc;

% =============================================
% 1. Motor Parameters (Fix — ไม่ต้อง estimate)
% =============================================
R_m     = 1.45336105;    % [Ohm] วัดจาก Locked Rotor Test 9V
L_m     = 0.00144802;    % [H]   วัดจาก Locked Rotor Test 9V
N_total = 70;            % [-]   Gear ratio รวม (Planetary x Belt)

% =============================================
% 2. Initial Guess (Parameters ที่จะ Estimate)
% =============================================
K_e = 0.043219;   % [V/(rad/s)] Back-EMF constant
K_t = 0.040527;   % [N.m/A]     Torque constant
B   = 0.19835;    % [N.m/(rad/s)] Damping coefficient
J   = 0.45190;    % [kg.m^2]    Inertia (arm only, ยังไม่รวม Gripper+Rod)
eta = 0.84954;    % [-]         Motor efficiency

% =============================================
% 3. เลือก Dataset ที่ต้องการใช้
% =============================================
% เปลี่ยน filename ตรงนี้เพื่อเปลี่ยน dataset
% ตัวเลือกที่มี:
%   chirp05_run1.mat  chirp05_run2.mat  chirp05_run3.mat
%   chirp1_run1.mat   chirp1_run2.mat   chirp1_run3.mat
%   ss_run1.mat       ss_run2.mat       ss_run3.mat

preprocess_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Preprocess_Data';
filename        = 'chirp2_run3.mat';   % <-- เปลี่ยนตรงนี้

% โหลด data
tmp      = load(fullfile(preprocess_path, filename));
t_in     = tmp.t;
Vin_in   = tmp.Vin;
omega_ws = tmp.omega_f;
Vin_ws   = [t_in, Vin_in];

fprintf('Loaded: %s\n', filename);
fprintf('Duration : %.1f s\n', t_in(end));
fprintf('Samples  : %d\n', length(t_in));
fprintf('Vin max  : %.3f V\n', max(abs(Vin_in)));
fprintf('omega max: %.4f rad/s\n', max(abs(omega_ws)));

% =============================================
% 4. เปิด Model และตั้ง Stop Time
% =============================================
model_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';
model_name = 'Params_Estimate';

if ~bdIsLoaded(model_name)
    load_system(model_path);
    fprintf('Model loaded: %s\n', model_name);
else
    fprintf('Model already loaded: %s\n', model_name);
end

set_param(model_name, 'StopTime', num2str(t_in(end)));

% =============================================
% 5. ทดสอบ Simulation ก่อน Estimate
% =============================================
fprintf('\nTesting simulation...\n');
tic
out = sim(model_name);
toc

omega_sim    = out.yout{1}.Values.Data;
t_sim        = out.yout{1}.Values.Time;
omega_interp = interp1(t_sim, omega_sim, t_in, 'linear', 'extrap');

ss_res    = sum((omega_ws - omega_interp).^2);
ss_tot    = sum((omega_ws - mean(omega_ws)).^2);
fit_score = (1 - ss_res/ss_tot) * 100;

fprintf('omega_sim max  = %.4f rad/s\n', max(abs(omega_sim)));
fprintf('omega_meas max = %.4f rad/s\n', max(abs(omega_ws)));
fprintf('Fit Score      = %.2f%%\n', fit_score);

% =============================================
% 6. Plot เปรียบเทียบ Initial vs Measured
% =============================================
figure('Name', sprintf('Initial vs Measured — %s', filename), 'NumberTitle', 'off');
plot(t_in, omega_ws,     'b',   'LineWidth', 1.2, 'DisplayName', 'Measured');
hold on;
plot(t_in, omega_interp, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Simulated (initial)');
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title(sprintf('Initial Parameters — %s', filename));
legend('Location', 'best'); grid on;

% % =============================================
% % 7. Save Result หลัง Estimate เสร็จ
% % =============================================
% % รันฟังก์ชันนี้หลังจาก Estimate ใน App เสร็จแล้ว
% % โดยดูค่าจาก Preview panel แล้วใส่มือ
% 
% fprintf('\n=== หลัง Estimate เสร็จ ใส่ค่าที่ได้ตรงนี้ ===\n');
% % K_e_est = ???;
% % K_t_est = ???;
% % B_est   = ???;
% % J_est   = ???;
% % eta_est = ???;
% 
% % แล้วรัน save_result.m เพื่อบันทึก

% % =============================================
% % 8. คำนวณ J จริง (รวม Gripper + Rod)
% % =============================================
% J_gripper = 0.16854;   % [kg.m^2] จาก Parallel Axis Theorem
% J_rod     = 0.04093;   % [kg.m^2] จาก Parallel Axis Theorem

% fprintf('\n=== J Correction ===\n');
% fprintf('J_est     = %.5f kg.m^2  (arm only)\n', J);
% fprintf('J_gripper = %.5f kg.m^2\n', J_gripper);
% fprintf('J_rod     = %.5f kg.m^2\n', J_rod);
% fprintf('J_real    = %.5f kg.m^2  (full system)\n', J + J_gripper + J_rod);

fprintf('\nSetup complete. เปิด Parameter Estimator App ใน Simulink ได้เลยครับ\n');