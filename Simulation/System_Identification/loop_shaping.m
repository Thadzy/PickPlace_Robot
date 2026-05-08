% =========================================================
% loop_shaping_design.m
% ออกแบบ PID ด้วย Loop Shaping ผ่าน MATLAB pidtune
% =========================================================

clc; clear; close all;

% Plant parameters
R_m     = 1.45336;
L_m     = 0.00144802;
N_total = 70;
K_e     = 0.04165;
K_t     = 0.04065;
B_damp  = 0.19279;
J       = 0.72762;
eta     = 0.83607;

% Velocity plant
K_num = K_t * eta * N_total / (L_m * J);
a1    = R_m/L_m + B_damp/J;
a0    = (R_m*B_damp + K_e*K_t*N_total^2) / (L_m*J);

G_vel = tf(K_num, [1, a1, a0]);
G_pos = tf(K_num, [1, a1, a0, 0]);

% =========================================================
% SECTION 1: Velocity Loop Design
% =========================================================
% Target: crossover = 47 rad/s, PM = 70 deg
opts_vel = pidtuneOptions('CrossoverFrequency', 47, ...
                          'PhaseMargin', 70);
[C_vel, info_vel] = pidtune(G_vel, 'PID', opts_vel);

fprintf('=== Velocity Loop PID (Loop Shaping) ===\n');
fprintf('Kp = %.6f\n', C_vel.Kp);
fprintf('Ki = %.6f\n', C_vel.Ki);
fprintf('Kd = %.6f\n', C_vel.Kd);
fprintf('Achieved PM    = %.2f deg\n', info_vel.PhaseMargin);
fprintf('Achieved wc    = %.2f rad/s\n', info_vel.CrossoverFrequency);
fprintf('Stable         = %d\n', info_vel.Stable);

% =========================================================
% SECTION 2: Position Loop Design
% =========================================================
% สำหรับ position loop ใช้ velocity closed-loop เป็น plant
% approximate ว่า velocity loop fast enough -> use 1/s
G_pos_loop = tf(1, [1, 0]);   % integrator = theta/omega

opts_pos = pidtuneOptions('CrossoverFrequency', 9.41, ...
                          'PhaseMargin', 80);
[C_pos, info_pos] = pidtune(G_pos_loop, 'PID', opts_pos);

fprintf('\n=== Position Loop PID (Loop Shaping) ===\n');
fprintf('Kp = %.6f\n', C_pos.Kp);
fprintf('Ki = %.6f\n', C_pos.Ki);
fprintf('Kd = %.6f\n', C_pos.Kd);
fprintf('Achieved PM    = %.2f deg\n', info_pos.PhaseMargin);
fprintf('Achieved wc    = %.2f rad/s\n', info_pos.CrossoverFrequency);
fprintf('Stable         = %d\n', info_pos.Stable);

% =========================================================
% SECTION 3: Verify Open-loop Bode
% =========================================================
L_vel = C_vel * G_vel;
L_pos = C_pos * G_pos_loop;

figure('Name', 'Open-loop Bode', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1200, 700]);

omega_range = logspace(-1, 4, 5000);

% Velocity open-loop
[mag_v, phase_v] = bode(L_vel, omega_range);
mag_v   = squeeze(mag_v);
phase_v = squeeze(phase_v);
[~, idx_gc_v] = min(abs(mag_v - 1));

subplot(2, 2, 1);
semilogx(omega_range, 20*log10(mag_v), 'b', 'LineWidth', 1.5);
hold on;
yline(0, 'r--', '0 dB crossover');
xline(omega_range(idx_gc_v), 'k--', ...
    sprintf('\\omega_c = %.1f rad/s', omega_range(idx_gc_v)));
ylabel('Magnitude (dB)');
title('Open-loop L_{vel}(s) = C_{vel} \cdot G_{vel}');
grid on;

subplot(2, 2, 3);
semilogx(omega_range, phase_v, 'b', 'LineWidth', 1.5);
hold on;
yline(-180, 'r--', '-180°');
xline(omega_range(idx_gc_v), 'k--');
pm_v = 180 + phase_v(idx_gc_v);
yline(-180 + pm_v, 'g--', sprintf('PM = %.1f°', pm_v));
ylabel('Phase (deg)');
xlabel('\omega (rad/s)');
title('Phase L_{vel}(s)');
grid on;

% Position open-loop
[mag_p, phase_p] = bode(L_pos, omega_range);
mag_p   = squeeze(mag_p);
phase_p = squeeze(phase_p);
[~, idx_gc_p] = min(abs(mag_p - 1));

subplot(2, 2, 2);
semilogx(omega_range, 20*log10(mag_p), 'r', 'LineWidth', 1.5);
hold on;
yline(0, 'b--', '0 dB crossover');
xline(omega_range(idx_gc_p), 'k--', ...
    sprintf('\\omega_c = %.1f rad/s', omega_range(idx_gc_p)));
ylabel('Magnitude (dB)');
title('Open-loop L_{pos}(s) = C_{pos} \cdot (1/s)');
grid on;

subplot(2, 2, 4);
semilogx(omega_range, phase_p, 'r', 'LineWidth', 1.5);
hold on;
yline(-180, 'b--', '-180°');
xline(omega_range(idx_gc_p), 'k--');
pm_p = 180 + phase_p(idx_gc_p);
yline(-180 + pm_p, 'g--', sprintf('PM = %.1f°', pm_p));
ylabel('Phase (deg)');
xlabel('\omega (rad/s)');
title('Phase L_{pos}(s)');
grid on;

sgtitle('Open-loop Bode: Velocity and Position Loops', ...
    'FontSize', 13, 'FontWeight', 'bold');

% =========================================================
% SECTION 4: Step Response ของ closed-loop แต่ละ loop
% =========================================================
figure('Name', 'Closed-loop Step Response', 'NumberTitle', 'off', ...
    'Position', [100, 100, 1000, 500]);

T_vel = feedback(C_vel * G_vel, 1);
T_pos = feedback(C_pos * G_pos_loop, 1);

subplot(1, 2, 1);
step(T_vel);
title('Velocity Loop Step Response');
grid on;

subplot(1, 2, 2);
step(T_pos);
title('Position Loop Step Response');
grid on;

% =========================================================
% SECTION 5: สรุปค่า PID สำหรับใส่ใน simulation
% =========================================================
fprintf('\n=== Final PID Gains for Simulation ===\n');
fprintf('--- Velocity Inner Loop ---\n');
fprintf('Kp_vel = %.4f;\n', C_vel.Kp);
fprintf('Ki_vel = %.4f;\n', C_vel.Ki);
fprintf('Kd_vel = %.4f;\n', C_vel.Kd);
fprintf('\n--- Position Outer Loop ---\n');
fprintf('Kp_pos = %.4f;\n', C_pos.Kp);
fprintf('Ki_pos = %.4f;\n', C_pos.Ki);
fprintf('Kd_pos = %.4f;\n', C_pos.Kd);