% =========================================================
% Script Name: simulate_cascade_pid_v3.m
% Description: Cascade PID + S-curve + State Space (ZOH discretization)
%              - Current-based Vin clamp (predictive)
%              - Derivative filter on both loops
%              - Anti-windup on both loops
%              - Configured for strict black font rendering and clean legends.
% =========================================================
clc; clear; close all;

% =========================================================
% SECTION 1: Plant Parameters (from System ID - Mean chirp1)
% =========================================================
R_m     = 1.45336;
L_m     = 0.00144802;
N_total = 70;
K_e     = 0.04165;
K_t     = 0.04065;
B_damp  = 0.19279;
J       = 0.72762;
eta     = 0.83607;
i_max   = 10.0;     % A  -- Cytron MD10C current limit

% =========================================================
% SECTION 2: State Space Model (continuous)
% x = [i; omega; theta],  u = Vin,  y = [omega; theta]
% =========================================================
A_c = [-R_m/L_m,            -K_e*N_total/L_m,  0;
        K_t*eta*N_total/J,  -B_damp/J,          0;
        0,                   1,                  0];
B_c = [1/L_m; 0; 0];
C_c = [0, 1, 0;    % output 1: omega
       0, 0, 1];   % output 2: theta
D_c = [0; 0];

fprintf('=== Open-loop Eigenvalues (continuous) ===\n');
ev = eig(A_c);
for ii = 1:3
    fprintf('  p%d = %+.4f %+.4fj\n', ii, real(ev(ii)), imag(ev(ii)));
end

% =========================================================
% SECTION 2b: Discretize plant using ZOH (exact)
% dt must be defined before c2d
% =========================================================
dt = 0.0001;    % 10 kHz -- ต้องเล็กกว่า tau_elec = L_m/R_m = 0.001 s
sys_c = ss(A_c, B_c, C_c, D_c);
sys_d = c2d(sys_c, dt, 'zoh');

Ad    = sys_d.A;   % 3x3 discrete A matrix
Bd    = sys_d.B;   % 3x1 discrete B matrix

fprintf('\n=== Discrete plant (ZOH, dt=%.4f s) ===\n', dt);
fprintf('tau_elec = %.5f s  (dt/tau = %.2f -- stable if < 1)\n', ...
    L_m/R_m, dt/(L_m/R_m));

% =========================================================
% SECTION 3: PID Gains
% =========================================================
% --- Velocity Inner Loop (Loop Shaping: wc=47, PM=79.2 deg) ---
Kp_vel = 20.0364;
Ki_vel = 376.9228;
Kd_vel = 0.0325;
N_vel  = 100;

% --- Position Outer Loop (Loop Shaping: wc=9.41, PM=80 deg) ---
Kp_pos = 9.2670;
Ki_pos = 8;
Kd_pos = 0.08;
N_pos  = 20;

% --- Feedforward (exact inverse model) ---
Kvff = 3.034;       % V per (rad/s)
Kaff = 0.4450;      % V per (rad/s^2)

% --- Saturation limits ---
Vin_max  = 24.0;    % V
vref_max = 7.3044;  % rad/s = v_max จาก S-curve

% --- Anti-windup gain ---
Kaw = 0.5;

% --- Feedback lowpass filter ---
fc_fb = 100;
wc_fb = 2 * pi * fc_fb;

fprintf('\n=== PID Gains ===\n');
fprintf('Velocity: Kp=%.3f  Ki=%.2f  Kd=%.4f  N=%d\n', ...
    Kp_vel, Ki_vel, Kd_vel, N_vel);
fprintf('Position: Kp=%.3f  Ki=%.2f  Kd=%.4f  N=%d\n', ...
    Kp_pos, Ki_pos, Kd_pos, N_pos);

% =========================================================
% SECTION 4: S-curve Trajectory (360 deg)
% =========================================================
v_max   = 7.3044;
a_max   = 27.4912;
j_max   = 1400.40;
q_total = 2 * pi;   % 360 deg in rad

% Timing
t_j = a_max / j_max;
t_a = v_max / a_max - t_j;

% Key velocities and distances per phase (analytical)
v1 = 0.5 * j_max * t_j^2;
a1 = j_max * t_j;
d1 = j_max * t_j^3 / 6;
d2 = v1 * t_a + 0.5 * a1 * t_a^2;
v2 = v1 + a1 * t_a;
d3 = v2 * t_j + 0.5 * a1 * t_j^2 - j_max * t_j^3 / 6;

d_accel  = d1 + d2 + d3;
d_cruise = q_total - 2 * d_accel;
t_v      = d_cruise / v_max;
t_total  = 4 * t_j + 2 * t_a + t_v;

% Phase end-times
T    = zeros(1, 9);
T(2) = t_j;
T(3) = T(2) + t_a;
T(4) = T(3) + t_j;
T(5) = T(4) + t_v;
T(6) = T(5) + t_j;
T(7) = T(6) + t_a;
T(8) = T(7) + t_j;

fprintf('\n=== Trajectory (S-curve 360 deg) ===\n');
fprintf('t_j = %.5f s,  t_a = %.5f s,  t_v = %.5f s\n', t_j, t_a, t_v);
fprintf('t_total = %.5f s\n', t_total);

% =========================================================
% SECTION 5: Simulation Setup
% =========================================================
t_end = t_total + 2.0;
t_sim = 0:dt:t_end;
N_sim = length(t_sim);

% Pre-allocate state and log arrays
x           = zeros(3, N_sim);    % [i; omega; theta]
theta_r_log = zeros(1, N_sim);
vref_log    = zeros(1, N_sim);
acc_r_log   = zeros(1, N_sim);
Vin_log     = zeros(1, N_sim);
omega_f_log = zeros(1, N_sim);
theta_f_log = zeros(1, N_sim);
i_cut_log   = false(1, N_sim);

% Controller states
int_vel      = 0;
int_pos      = 0;
dfilt_vel    = 0;
dfilt_pos    = 0;
err_vel_prev = 0;
err_pos_prev = 0;
aw_vel       = 0;
aw_pos       = 0;

% Filter states
omega_f_prev = 0;
theta_f_prev = 0;

% Tustin alpha for feedback filter
alpha_fb = wc_fb * dt / (1 + wc_fb * dt);

% =========================================================
% SECTION 6: Simulation Loop
% =========================================================
for k = 1:N_sim - 1
    tk = t_sim(k);
    
    % -------------------------------------------------------
    % 6.1 Analytical S-curve reference at time tk
    % -------------------------------------------------------
    if     tk <= T(2)
        j_n=+j_max; a0=0;    v0=0;              p0=0;
        t0=T(1);
    elseif tk <= T(3)
        j_n=0;      a0=+a1;  v0=v1;             p0=d1;
        t0=T(2);
    elseif tk <= T(4)
        j_n=-j_max; a0=+a1;  v0=v2;             p0=d1+d2;
        t0=T(3);
    elseif tk <= T(5)
        j_n=0;      a0=0;    v0=v_max;          p0=d_accel;
        t0=T(4);
    elseif tk <= T(6)
        j_n=-j_max; a0=0;    v0=v_max;          p0=d_accel+d_cruise;
        t0=T(5);
    elseif tk <= T(7)
        j_n=0;      a0=-a1;  v0=v_max-v1;       p0=d_accel+d_cruise+d1;
        t0=T(6);
    elseif tk <= T(8)
        j_n=+j_max; a0=-a1;  v0=v_max-v1-a1*t_a; p0=d_accel+d_cruise+d1+d2;
        t0=T(7);
    else
        j_n=0;      a0=0;    v0=0;              p0=q_total;
        t0=T(8);
    end
    
    tau    = tk - t0;
    traj_p = min(p0 + v0*tau + 0.5*a0*tau^2 + j_n*tau^3/6, q_total);
    traj_v = v0 + a0*tau + 0.5*j_n*tau^2;
    traj_a = a0 + j_n*tau;
    
    theta_r_log(k) = traj_p;
    acc_r_log(k)   = traj_a;
    
    % -------------------------------------------------------
    % 6.2 Feedback lowpass filter (Tustin first-order)
    % -------------------------------------------------------
    omega_f      = alpha_fb * x(2,k) + (1 - alpha_fb) * omega_f_prev;
    theta_f      = alpha_fb * x(3,k) + (1 - alpha_fb) * theta_f_prev;
    
    omega_f_prev = omega_f;
    theta_f_prev = theta_f;
    
    omega_f_log(k) = omega_f;
    theta_f_log(k) = theta_f;
    
    % -------------------------------------------------------
    % 6.3 Position PID (Outer loop) --> v_ref
    % -------------------------------------------------------
    err_pos = traj_p - theta_f;
    
    % Derivative with filter: dfilt(k) = (1-N*dt)*dfilt(k-1) + Kd*N*de
    dfilt_pos    = (1 - N_pos*dt) * dfilt_pos + ...
                   Kd_pos * N_pos * (err_pos - err_pos_prev);
    err_pos_prev = err_pos;
    
    % Integral with anti-windup
    int_pos  = int_pos + err_pos * dt + Kaw * aw_pos * dt;
    
    % PID output + trajectory velocity feedforward
    vref_pid = Kp_pos * err_pos + Ki_pos * int_pos + dfilt_pos;
    vref_cmd = vref_pid + traj_v;
    vref_cmd = max(-vref_max, min(vref_max, vref_cmd));
    
    % Anti-windup signal for position
    aw_pos = max(-vref_max, min(vref_max, vref_pid)) - vref_pid;
    vref_log(k) = vref_cmd;
    
    % -------------------------------------------------------
    % 6.4 Velocity PID (Inner loop) --> V_pid
    % -------------------------------------------------------
    err_vel = vref_cmd - omega_f;
    
    % Derivative with filter
    dfilt_vel    = (1 - N_vel*dt) * dfilt_vel + ...
                   Kd_vel * N_vel * (err_vel - err_vel_prev);
    err_vel_prev = err_vel;
    
    % Integral with anti-windup
    int_vel = int_vel + err_vel * dt + Kaw * aw_vel * dt;
    
    V_pid = Kp_vel * err_vel + Ki_vel * int_vel + dfilt_vel;
    
    % Trajectory feedforward
    Vff = Kvff * vref_cmd + Kaff * traj_a;
    
    % Total Vin before saturation
    Vin_raw = V_pid + Vff;
    
    % Voltage saturation
    Vin = max(-Vin_max, min(Vin_max, Vin_raw));
    
    % Anti-windup signal for velocity
    aw_vel = Vin - Vin_raw;
    
    % -------------------------------------------------------
    % 6.5 Predictive Current Limit
    % -------------------------------------------------------
    i_now     = x(1, k);
    omega_now = x(2, k);
    V_bemf    = K_e * N_total * omega_now;
    
    Vin_max_i = L_m * (i_max  - i_now) / dt + R_m * i_now + V_bemf;
    Vin_min_i = L_m * (-i_max - i_now) / dt + R_m * i_now + V_bemf;
    
    Vin_clamped = max(Vin_min_i, min(Vin_max_i, Vin));
    
    % Hardware voltage limit
    Vin_clamped = max(-Vin_max, min(Vin_max, Vin_clamped));
    
    % Log current clamp status
    i_cut_log(k) = (abs(Vin_clamped) < abs(Vin) - 0.01);
    Vin_log(k) = Vin_clamped;
    
    % -------------------------------------------------------
    % 6.6 Plant Integration (ZOH exact discrete)
    % -------------------------------------------------------
    x(:, k+1) = Ad * x(:, k) + Bd * Vin_clamped;
end

% Fill last reference point
theta_r_log(end) = q_total;

% =========================================================
% SECTION 7: Performance Metrics
% =========================================================
band     = 0.01 * q_total;     % 2% of 360 deg = 7.2 deg
idx_end  = find(t_sim >= T(8), 1);
settled         = false;
t_settle_actual = NaN;
t_settle_start  = NaN;

for k = idx_end:N_sim
    in_band = abs(x(3, k) - q_total) < band;
    if in_band
        if ~settled
            t_settle_start = t_sim(k);
            settled = true;
        end
        if (t_sim(k) - t_settle_start) > 0.05
            t_settle_actual = t_settle_start - T(8);
            break;
        end
    else
        settled = false;
    end
end

theta_max = max(x(3, idx_end:end));
overshoot = max(0, (theta_max - q_total) / q_total * 100);
i_cut_pct = sum(i_cut_log) / N_sim * 100;
max_i     = max(abs(x(1, :)));

fprintf('\n=== Performance Results ===\n');
if isnan(t_settle_actual)
    fprintf('Settling time : DID NOT SETTLE\n');
else
    fprintf('Settling time : %.4f s  (req <= 0.5 s)  %s\n', ...
        t_settle_actual, pass_fail(t_settle_actual <= 0.5));
end
fprintf('Overshoot     : %.4f %%  (req <= 1 %%)   %s\n', ...
    overshoot, pass_fail(overshoot <= 1));
fprintf('Final position: %.5f rad  (target %.5f rad)\n', x(3,end), q_total);
fprintf('Steady error  : %.5f rad = %.4f deg\n', ...
    abs(x(3,end) - q_total), abs(x(3,end) - q_total)*180/pi);
fprintf('Max current   : %.4f A   (limit %.1f A)  %s\n', ...
    max_i, i_max, pass_fail(max_i <= i_max));
fprintf('Current-clamp : %.2f %% of time\n', i_cut_pct);

% =========================================================
% SECTION 8: Plots
% =========================================================
figure('Name', 'Cascade PID v3', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1300, 900], ...
    'Color', 'white');

% Set global default colors to black
set(gcf, 'DefaultTextColor',          'black');
set(gcf, 'DefaultAxesColor',          'white');
set(gcf, 'DefaultAxesXColor',         'black');
set(gcf, 'DefaultAxesYColor',         'black');
set(gcf, 'DefaultAxesGridColor',      [0.85 0.85 0.85]);
set(gcf, 'DefaultLegendTextColor',    'black');

% Padding for text alignment at the right edge
text_x_pos = t_sim(end) * 0.98; 

% --- Position Subplot ---
subplot(3, 2, 1);
plot(t_sim, theta_r_log*180/pi, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
hold on;
plot(t_sim, x(3,:)*180/pi, 'r', 'LineWidth', 1.2, 'DisplayName', 'Actual');

% Black lines automatically format their labels as black
xline(T(8), 'k--', 'Traj end', 'LineWidth', 1, 'HandleVisibility', 'off', 'LabelVerticalAlignment', 'bottom');

% Magenta line: Render line separately, insert black text manually
yline(360, 'm:', 'LineWidth', 1, 'HandleVisibility', 'off');
text(text_x_pos, 360, '360°', 'Color', 'k', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8);

ylabel('\theta (deg)');
title('Position');
legend('Location', 'southeast');
grid on;

% --- Position Error Subplot ---
subplot(3, 2, 2);
pos_err_deg = (theta_r_log - x(3,:)) * 180/pi;
plot(t_sim, pos_err_deg, 'r', 'LineWidth', 1.2);

yline(+0.02*360, 'b--', 'HandleVisibility', 'off');
text(text_x_pos, +0.02*360, '+2% band', 'Color', 'k', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8);

yline(-0.02*360, 'b--', 'HandleVisibility', 'off');
text(text_x_pos, -0.02*360, '-2% band', 'Color', 'k', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 8);

xline(T(8), 'k--', 'HandleVisibility', 'off');

ylabel('Error (deg)');
title('Position Error');
grid on;

% --- Velocity Subplot ---
subplot(3, 2, 3);
plot(t_sim, vref_log, 'b--', 'LineWidth', 1.2, 'DisplayName', 'v_{ref}');
hold on;
plot(t_sim, x(2,:), 'r', 'LineWidth', 1.2, 'DisplayName', '\omega actual');
plot(t_sim, omega_f_log, 'g:', 'LineWidth', 1.0, 'DisplayName', '\omega filtered');

yline(+v_max, 'k--', 'v_{max}', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'right');
yline(-v_max, 'k--', 'HandleVisibility', 'off');

ylabel('\omega (rad/s)');
title('Velocity');
legend('Location', 'best');
grid on;

% --- Velocity Error Subplot ---
subplot(3, 2, 4);
plot(t_sim, vref_log - omega_f_log, 'r', 'LineWidth', 1.2);
ylabel('Error (rad/s)');
title('Velocity Error');
grid on;

% --- Motor Voltage Subplot ---
subplot(3, 2, 5);
plot(t_sim, Vin_log, 'b', 'LineWidth', 1.2);

yline(+Vin_max, 'r--', 'HandleVisibility', 'off');
text(text_x_pos, +Vin_max, '+24V', 'Color', 'k', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8);

yline(-Vin_max, 'r--', 'HandleVisibility', 'off');
text(text_x_pos, -Vin_max, '-24V', 'Color', 'k', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 8);

xlabel('Time (s)');
ylabel('V_{in} (V)');
title('Motor Voltage');
grid on;

% --- Motor Current Subplot ---
subplot(3, 2, 6);
plot(t_sim, x(1,:), 'b', 'LineWidth', 1.2, 'DisplayName', 'Current');
hold on;
area(t_sim, double(i_cut_log) * i_max, ...
    'FaceColor', [1 0.5 0.5], 'FaceAlpha', 0.3, ...
    'EdgeColor', 'none', 'DisplayName', 'Clamp zone');

yline(+i_max, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(text_x_pos, +i_max, '+10A', 'Color', 'k', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8);

yline(-i_max, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
text(text_x_pos, -i_max, '-10A', 'Color', 'k', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'FontSize', 8);

xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('Motor Current  (max=%.2f A, clamp=%.1f%%)', max_i, i_cut_pct));
legend('Location', 'best');
grid on;

% --- Main Figure Title ---
st = sgtitle('Cascade PID v3 + S-curve + ZOH Plant + Current Limit', ...
    'FontSize', 13, 'FontWeight', 'bold');
st.Color = 'black'; 

% --- Final Cleanup Loop ---
ax_all = findall(gcf, 'Type', 'axes');
for ax = ax_all'
    lg = get(ax, 'Legend');
    if ~isempty(lg)
        set(lg, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
    end
    set(get(ax, 'Title'),  'Color', 'black');
    set(get(ax, 'XLabel'), 'Color', 'black');
    set(get(ax, 'YLabel'), 'Color', 'black');
    set(ax, 'XColor', 'black', 'YColor', 'black', 'GridColor', [0.85 0.85 0.85]);
end

txt_objs = findall(gcf, 'Type', 'text');
set(txt_objs, 'Color', 'black');

% =========================================================
% Helper function
% =========================================================

function s = pass_fail(cond)
    if cond
        s = 'PASS';
    else
        s = 'FAIL';
    end
end