% =========================================================
% compute_scurve_v2.m
% S-curve Trajectory — แก้ไข numerical integration
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Parameters
% =========================================================
J_est        = 0.72762;
B_est        = 0.19279;
K_t          = 0.04065;
eta          = 0.83607;
N_total      = 70;
i_max        = 10;
omega_max_hw = 8.116;

% Motion limits
v_max = 0.9 * omega_max_hw;

tau_out_max = K_t * i_max * eta * N_total;
tau_net     = tau_out_max - B_est * omega_max_hw;
a_max       = 0.9 * (tau_net / J_est);

omega_n_vel = 56.6;
j_max       = 0.9 * a_max * omega_n_vel;

fprintf('=== Motion Limits ===\n');
fprintf('v_max = %.4f rad/s\n',   v_max);
fprintf('a_max = %.4f rad/s^2\n', a_max);
fprintf('j_max = %.2f rad/s^3\n', j_max);

% =========================================================
% SECTION 2: Analytical S-curve Timing
% =========================================================
q_total = 2 * pi;

% Phase durations
t_j = a_max / j_max;
t_a = v_max / a_max - t_j;

if t_a < 0
    t_a = 0;
    t_j = sqrt(v_max / j_max);
    v_peak = j_max * t_j^2;
    fprintf('WARNING: No constant accel phase, v_peak=%.4f\n', v_peak);
end

% ระยะทางช่วง accel (phases 1-3) คำนวณ analytical
% Phase 1: jerk up
d1 = 0.5 * j_max * t_j^2 * t_j / 3;   % = j*t^3/6
v1 = 0.5 * j_max * t_j^2;              % velocity หลัง phase 1
a1 = j_max * t_j;                       % accel หลัง phase 1
d1 = j_max * t_j^3 / 6;

% Phase 2: constant accel
d2 = v1 * t_a + 0.5 * a1 * t_a^2;
v2 = v1 + a1 * t_a;

% Phase 3: jerk down
d3 = v2 * t_j + 0.5 * a1 * t_j^2 - j_max * t_j^3 / 6;
% v หลัง phase 3 = v_max

d_accel = d1 + d2 + d3;    % ระยะทาง accel (phases 1-3)
d_decel = d_accel;          % symmetric

% Phase 4: cruise
d_cruise = q_total - d_accel - d_decel;

if d_cruise < 0
    % ไม่มี cruise phase -- ลด v_max
    t_v = 0;
    % แก้: หา v_peak จาก d_accel + d_decel = q_total
    % สมการ: 2*(j*tj^3/6 + v1*ta + 0.5*a*ta^2 + v2*tj + 0.5*a*tj^2 - j*tj^3/6) = q
    % approximate: q = v_max*(ta + tj)  สำหรับ symmetric
    fprintf('WARNING: No cruise phase\n');
    v_max  = sqrt(a_max * q_total / 2 - a_max^2 / (2*j_max));
    t_j    = a_max / j_max;
    t_a    = v_max / a_max - t_j;
    if t_a < 0; t_a = 0; end
    d_cruise = 0;
else
    t_v = d_cruise / v_max;
end

% เวลารวม (7 phases: tj + ta + tj + tv + tj + ta + tj)
t_total = 4*t_j + 2*t_a + t_v;

fprintf('\n=== S-curve Timing (Analytical) ===\n');
fprintf('t_j     = %.5f s\n', t_j);
fprintf('t_a     = %.5f s\n', t_a);
fprintf('t_v     = %.5f s\n', t_v);
fprintf('t_total = %.5f s\n', t_total);
fprintf('d_accel = %.5f rad\n', d_accel);
fprintf('d_cruise= %.5f rad\n', d_cruise);
fprintf('d_total = %.5f rad (target %.5f)\n', ...
    2*d_accel + d_cruise, q_total);

% =========================================================
% SECTION 3: Generate Profile (Analytical ทุก phase)
% =========================================================
dt = 0.0001;   % ลด dt เพื่อความแม่นยำ
t  = 0:dt:t_total + 0.01;
n  = length(t);

jerk_p = zeros(1, n);
accel_p = zeros(1, n);
vel_p   = zeros(1, n);
pos_p   = zeros(1, n);

% Phase boundaries
T = zeros(1, 8);
T(1) = 0;
T(2) = T(1) + t_j;       % end of phase 1
T(3) = T(2) + t_a;       % end of phase 2
T(4) = T(3) + t_j;       % end of phase 3
T(5) = T(4) + t_v;       % end of phase 4 (cruise)
T(6) = T(5) + t_j;       % end of phase 5
T(7) = T(6) + t_a;       % end of phase 6
T(8) = T(7) + t_j;       % end of phase 7

% State ต้นของแต่ละ phase
% [a0, v0, p0] ที่เริ่มต้นของแต่ละ phase
phase_init = zeros(7, 3);   % [accel, vel, pos]
phase_init(1, :) = [0, 0, 0];

for k = 1:n
    tk = t(k);

    % หา phase ปัจจุบัน
    if tk <= T(2)
        ph = 1; j_now = +j_max; t0 = T(1);
        a0 = 0; v0 = 0; p0 = 0;
    elseif tk <= T(3)
        ph = 2; j_now = 0; t0 = T(2);
        a0 = a_max; v0 = v_max/2 - a_max*t_j/2;
        % คำนวณจาก end of phase 1
        a0 = j_max * t_j;
        v0 = 0.5 * j_max * t_j^2;
        p0 = j_max * t_j^3 / 6;
    elseif tk <= T(4)
        ph = 3; j_now = -j_max; t0 = T(3);
        a0 = j_max * t_j;
        v0 = 0.5 * j_max * t_j^2 + j_max * t_j * t_a;
        p0 = d1 + d2;
    elseif tk <= T(5)
        ph = 4; j_now = 0; t0 = T(4);
        a0 = 0; v0 = v_max; p0 = d_accel;
    elseif tk <= T(6)
        ph = 5; j_now = -j_max; t0 = T(5);
        a0 = 0; v0 = v_max; p0 = d_accel + d_cruise;
    elseif tk <= T(7)
        ph = 6; j_now = 0; t0 = T(6);
        a0 = -j_max * t_j;
        v0 = v_max - 0.5 * j_max * t_j^2;
        p0 = d_accel + d_cruise + d1;
    elseif tk <= T(8)
        ph = 7; j_now = +j_max; t0 = T(7);
        a0 = -j_max * t_j;
        v0 = 0.5 * j_max * t_j^2 + j_max * t_j * t_a - j_max*t_j^2;
        % ใช้ exact: v0 ที่ end of phase 6
        v0 = v_max - 0.5*j_max*t_j^2 - j_max*t_j*t_a;
        p0 = d_accel + d_cruise + d1 + d2;
    else
        ph = 8; j_now = 0; t0 = T(8);
        a0 = 0; v0 = 0; p0 = q_total;
    end

    tau = tk - t0;

    jerk_p(k)  = j_now;
    accel_p(k) = a0 + j_now * tau;
    vel_p(k)   = v0 + a0 * tau + 0.5 * j_now * tau^2;
    pos_p(k)   = p0 + v0 * tau + 0.5 * a0 * tau^2 + j_now * tau^3 / 6;
end

% =========================================================
% SECTION 4: Verify
% =========================================================
fprintf('\n=== Verification ===\n');
fprintf('v_max reached  = %.5f rad/s   (target %.5f)\n', ...
    max(vel_p), v_max);
fprintf('a_max reached  = %.5f rad/s^2 (target %.5f)\n', ...
    max(accel_p), a_max);
fprintf('Total distance = %.5f rad     (target %.5f)\n', ...
    pos_p(end), q_total);
fprintf('Error distance = %.6f rad = %.4f deg\n', ...
    abs(pos_p(end) - q_total), abs(pos_p(end) - q_total)*180/pi);

% =========================================================
% SECTION 5: Cycle Time
% =========================================================
t_settle   = 0.5;
t_pick     = 4.0;   % สมมติ -- ปรับตามจริง
n_pieces   = 4;

t_cycle = n_pieces * (t_total + t_settle + t_pick);

fprintf('\n=== Cycle Time ===\n');
fprintf('Move time (360 deg) = %.4f s\n', t_total);
fprintf('Settling time       = %.4f s\n', t_settle);
fprintf('Pick+Place time     = %.4f s (สมมติ)\n', t_pick);
fprintf('Cycle (4 pieces)    = %.4f s (req <= 35 s)\n', t_cycle);
if t_cycle <= 35
    fprintf('==> PASS\n');
else
    fprintf('==> FAIL\n');
end

% =========================================================
% SECTION 6: Plot
% =========================================================
figure('Name', 'S-curve v2', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1000, 750]);

subplot(4,1,1);
plot(t, jerk_p, 'k', 'LineWidth', 1.2);
ylabel('Jerk (rad/s^3)');
title('S-curve Trajectory Profile v2 — 360°');
grid on;

subplot(4,1,2);
plot(t, accel_p, 'b', 'LineWidth', 1.2);
yline(a_max,  'r--', 'a_{max}', 'LineWidth', 1);
yline(-a_max, 'r--', 'LineWidth', 1);
ylabel('Accel (rad/s^2)');
grid on;

subplot(4,1,3);
plot(t, vel_p, 'g', 'LineWidth', 1.2);
yline(v_max, 'r--', 'v_{max}', 'LineWidth', 1);
ylabel('Velocity (rad/s)');
grid on;

subplot(4,1,4);
plot(t, pos_p * 180/pi, 'm', 'LineWidth', 1.2);
yline(360, 'r--', '360°', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Position (deg)');
grid on;