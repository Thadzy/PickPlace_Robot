% =========================================================
% Script: simulate_cascade_pid_with_rod.m
% Description: 
%   Simulates a cascade PID controller with S-curve trajectory generation
%   and a Zero-Vibration (ZV) Input Shaper. It includes the continuous and 
%   discrete state-space models of a robotic arm and a tangential rod pendulum.
%   The script evaluates settling time, maximum overshoot, and cycle times.
%
% Author: AI Assistant
% Date: May 9, 2026
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Plant Parameters
% =========================================================
% Motor and mechanical parameters for the arm
R_m     = 1.45336;      % Motor resistance (Ohms)
L_m     = 0.00144802;   % Motor inductance (H)
N_total = 70;           % Gear ratio
K_e     = 0.04165;      % Back-EMF constant (V/(rad/s))
K_t     = 0.04065;      % Torque constant (Nm/A)
B_damp  = 0.19279;      % Damping coefficient (Nm/(rad/s))
J       = 0.72762;      % System inertia (kg.m^2)
eta     = 0.83607;      % Efficiency
i_max   = 10.0;         % Maximum current limit (A)

% =========================================================
% SECTION 2: Rod Parameters (Tangential Pendulum)
% =========================================================
m_rod   = 0.16408;      % Mass of the rod (kg)
L_rod   = 0.10;         % Length of the rod (m)
g       = 9.81;         % Acceleration due to gravity (m/s^2)
r_arm   = 0.5;          % Radius of the arm (m)

% Uniform slender rod with pivot at top end
l_cm    = L_rod / 2;                        % Center of mass distance (m)
I_pivot = (1/3) * m_rod * L_rod^2;          % Moment of inertia at pivot (kg.m^2)

% Natural frequency and damping characteristics
wn_rod   = sqrt(m_rod * g * l_cm / I_pivot); % Natural frequency (rad/s)
zeta_rod = 0.05;                             % Damping ratio (lightly damped)
wd_rod   = wn_rod * sqrt(1 - zeta_rod^2);    % Damped natural frequency (rad/s)

fprintf('=== Rod Pendulum Parameters ===\n');
fprintf('m_rod    = %.5f kg\n',  m_rod);
fprintf('L_rod    = %.4f m\n',   L_rod);
fprintf('l_cm     = %.4f m\n',   l_cm);
fprintf('I_pivot  = %.6f kg.m^2\n', I_pivot);
fprintf('wn_rod   = %.4f rad/s (fn = %.4f Hz)\n', wn_rod, wn_rod/(2*pi));
fprintf('zeta_rod = %.4f\n',     zeta_rod);

% =========================================================
% SECTION 3: State Space Model (Arm only, continuous)
% =========================================================
% State vector: x_arm = [current; angular_velocity; angle]
A_c = [-R_m/L_m,            -K_e*N_total/L_m,  0;
        K_t*eta*N_total/J,  -B_damp/J,          0;
        0,                   1,                  0];
B_c = [1/L_m; 0; 0];
C_c = [0, 1, 0; 0, 0, 1];
D_c = [0; 0];

% Discretize arm plant using Zero-Order Hold (ZOH)
dt     = 0.0001;        % Sample time (s)
sys_c  = ss(A_c, B_c, C_c, D_c);
sys_d  = c2d(sys_c, dt, 'zoh');
Ad     = sys_d.A;
Bd     = sys_d.B;

fprintf('\n=== Open-loop Eigenvalues (continuous) ===\n');
ev = eig(A_c);
for ii = 1:3
    fprintf('  p%d = %+.4f %+.4fj\n', ii, real(ev(ii)), imag(ev(ii)));
end

% =========================================================
% SECTION 4: Rod State Space (continuous)
% =========================================================
% State vector: x_rod = [phi; phi_dot]
% phi = angle of rod from vertical (rad)
% Input = alpha_arm (arm angular acceleration)
A_rod = [0,                    1;
         -wn_rod^2,  -2*zeta_rod*wn_rod];
B_rod = [0;
         -m_rod * l_cm * r_arm / I_pivot];

% Discretize rod model
sys_rod_c = ss(A_rod, B_rod, [1 0; 0 1], [0; 0]);
sys_rod_d = c2d(sys_rod_c, dt, 'zoh');
Ad_rod    = sys_rod_d.A;
Bd_rod    = sys_rod_d.B;

fprintf('\n=== Rod Natural Frequency ===\n');
fprintf('wn_rod = %.4f rad/s  =>  T_rod = %.4f s\n', wn_rod, 2*pi/wn_rod);
fprintf('t_settle_rod (5%%) ~ %.4f s\n', 3 / (zeta_rod * wn_rod));

% =========================================================
% SECTION 5: PID and Controller Parameters
% =========================================================
% Velocity loop gains
Kp_vel = 20.0364;
Ki_vel = 376.9228;
Kd_vel = 0.0325;
N_vel  = 100;

% Position loop gains
Kp_pos = 9.2670;
Ki_pos = 10.5;
Kd_pos = 0.08;
N_pos  = 20;

% Feedforward and Limits
Kvff     = 3.034;
Kaff     = 0.4450;
Vin_max  = 24.0;
vref_max = 7.3044;
Kaw      = 0.5;         % Anti-windup gain

% Feedback filtering
fc_fb = 100;
wc_fb = 2 * pi * fc_fb;

% =========================================================
% SECTION 6: S-curve Trajectory & Simulation Setup
% =========================================================
v_max   = 7.3044;
a_max   = 27.4912;
j_max   = 1400.40;
q_total = 2 * pi;       % 360 degrees in radians

% Calculate time segments for S-curve
t_j = a_max / j_max;
t_a = v_max / a_max - t_j;

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

% Trajectory milestones
T    = zeros(1, 9);
T(2) = t_j;
T(3) = T(2) + t_a;
T(4) = T(3) + t_j;
T(5) = T(4) + t_v;
T(6) = T(5) + t_j;
T(7) = T(6) + t_a;
T(8) = T(7) + t_j;

% --- Define Simulation Timeline Before Trajectory Processing ---
t_end = t_total + 3.0;    % Add buffer time for rod settling
t_sim = 0:dt:t_end;
N_sim = length(t_sim);

% =========================================================
% SECTION 7: ZV Input Shaper Processing
% =========================================================
% Compute impulse amplitudes and timing based on rod natural frequency
omega_d = wn_rod * sqrt(1 - zeta_rod^2);
K_zv    = exp(-zeta_rod * pi / sqrt(1 - zeta_rod^2));
t2_zv   = pi / omega_d;
A1_zv   = 1 / (1 + K_zv);
A2_zv   = K_zv / (1 + K_zv);

fprintf('\n=== ZV Input Shaper ===\n');
fprintf('omega_d = %.4f rad/s\n', omega_d);
fprintf('t2      = %.4f s\n',     t2_zv);
fprintf('A1      = %.4f\n',       A1_zv);
fprintf('A2      = %.4f\n',       A2_zv);
fprintf('A1+A2   = %.4f (should = 1.0)\n', A1_zv + A2_zv);

% Pre-compute raw unshaped trajectory
traj_p_raw = zeros(1, N_sim);
traj_v_raw = zeros(1, N_sim);
traj_a_raw = zeros(1, N_sim);

for k = 1:N_sim
    tk = t_sim(k);
    if     tk <= T(2); j_n=+j_max; a0=0;    v0=0;      p0=0;                        t0=T(1);
    elseif tk <= T(3); j_n=0;      a0=+a1;  v0=v1;     p0=d1;                       t0=T(2);
    elseif tk <= T(4); j_n=-j_max; a0=+a1;  v0=v2;     p0=d1+d2;                    t0=T(3);
    elseif tk <= T(5); j_n=0;      a0=0;    v0=v_max;  p0=d_accel;                  t0=T(4);
    elseif tk <= T(6); j_n=-j_max; a0=0;    v0=v_max;  p0=d_accel+d_cruise;         t0=T(5);
    elseif tk <= T(7); j_n=0;      a0=-a1;  v0=v_max-v1; p0=d_accel+d_cruise+d1;    t0=T(6);
    elseif tk <= T(8); j_n=+j_max; a0=-a1;  v0=v_max-v1-a1*t_a; p0=d_accel+d_cruise+d1+d2; t0=T(7);
    else;              j_n=0;      a0=0;    v0=0;      p0=q_total;                  t0=T(8);
    end
    
    tau = tk - t0;
    traj_p_raw(k) = min(p0 + v0*tau + 0.5*a0*tau^2 + j_n*tau^3/6, q_total);
    traj_v_raw(k) = v0 + a0*tau + 0.5*j_n*tau^2;
    traj_a_raw(k) = a0 + j_n*tau;
end

% Apply ZV shaping
delay_steps = round(t2_zv / dt);
traj_p_shaped = zeros(1, N_sim);
traj_v_shaped = zeros(1, N_sim);
traj_a_shaped = zeros(1, N_sim);

for k = 1:N_sim
    k_delay = k - delay_steps;
    if k_delay > 0
        traj_p_shaped(k) = A1_zv * traj_p_raw(k) + A2_zv * traj_p_raw(k_delay);
        traj_v_shaped(k) = A1_zv * traj_v_raw(k) + A2_zv * traj_v_raw(k_delay);
        traj_a_shaped(k) = A1_zv * traj_a_raw(k) + A2_zv * traj_a_raw(k_delay);
    else
        traj_p_shaped(k) = A1_zv * traj_p_raw(k);
        traj_v_shaped(k) = A1_zv * traj_v_raw(k);
        traj_a_shaped(k) = A1_zv * traj_a_raw(k);
    end
end

% =========================================================
% SECTION 8: Initialization of Logs and Controller States
% =========================================================
x_arm        = zeros(3, N_sim);   
x_rod        = zeros(2, N_sim);   
theta_r_log  = zeros(1, N_sim);
vref_log     = zeros(1, N_sim);
acc_r_log    = zeros(1, N_sim);
Vin_log      = zeros(1, N_sim);
omega_f_log  = zeros(1, N_sim);
phi_log      = zeros(1, N_sim);   
phidot_log   = zeros(1, N_sim);   
alpha_log    = zeros(1, N_sim);   

int_vel      = 0; int_pos      = 0;
dfilt_vel    = 0; dfilt_pos    = 0;
err_vel_prev = 0; err_pos_prev = 0;
aw_vel       = 0; aw_pos       = 0;
omega_f_prev = 0; theta_f_prev = 0;
alpha_prev   = 0;

alpha_fb = wc_fb * dt / (1 + wc_fb * dt);

% =========================================================
% SECTION 9: Main Simulation Loop
% =========================================================
for k = 1:N_sim - 1
    
    % ใช้ shaped trajectory แทน raw S-curve
    traj_p = traj_p_shaped(k);
    traj_v = traj_v_shaped(k);
    traj_a = traj_a_shaped(k);
    
    theta_r_log(k) = traj_p;
    acc_r_log(k)   = traj_a;
    
    % Calculate arm angular acceleration
    alpha_now  = (x_arm(2,k) - alpha_prev) / dt;
    alpha_prev = x_arm(2,k);
    alpha_log(k) = alpha_now;
    
    % Update rod dynamics
    x_rod(:, k+1) = Ad_rod * x_rod(:, k) + Bd_rod * alpha_now;
    phi_log(k)    = x_rod(1, k);
    phidot_log(k) = x_rod(2, k);
    
    % Apply feedback filter
    omega_f      = alpha_fb * x_arm(2,k) + (1-alpha_fb) * omega_f_prev;
    theta_f      = alpha_fb * x_arm(3,k) + (1-alpha_fb) * theta_f_prev;
    omega_f_prev = omega_f;
    theta_f_prev = theta_f;
    omega_f_log(k) = omega_f;
    
    % Position PID
    err_pos      = traj_p - theta_f;
    dfilt_pos    = (1-N_pos*dt)*dfilt_pos + Kd_pos*N_pos*(err_pos-err_pos_prev);
    err_pos_prev = err_pos;
    int_pos      = int_pos + err_pos*dt + Kaw*aw_pos*dt;
    vref_pid     = Kp_pos*err_pos + Ki_pos*int_pos + dfilt_pos;
    vref_cmd     = max(-vref_max, min(vref_max, vref_pid + traj_v));
    aw_pos       = max(-vref_max, min(vref_max, vref_pid)) - vref_pid;
    vref_log(k)  = vref_cmd;
    
    % Velocity PID
    err_vel      = vref_cmd - omega_f;
    dfilt_vel    = (1-N_vel*dt)*dfilt_vel + Kd_vel*N_vel*(err_vel-err_vel_prev);
    err_vel_prev = err_vel;
    int_vel      = int_vel + err_vel*dt + Kaw*aw_vel*dt;
    V_pid        = Kp_vel*err_vel + Ki_vel*int_vel + dfilt_vel;
    Vff          = Kvff*vref_cmd + Kaff*traj_a;
    Vin_raw      = V_pid + Vff;
    Vin          = max(-Vin_max, min(Vin_max, Vin_raw));
    aw_vel       = Vin - Vin_raw;
    
    % Current limit
    i_now     = x_arm(1,k);
    omega_now = x_arm(2,k);
    V_bemf    = K_e * N_total * omega_now;
    Vin_max_i = L_m*(i_max  - i_now)/dt + R_m*i_now + V_bemf;
    Vin_min_i = L_m*(-i_max - i_now)/dt + R_m*i_now + V_bemf;
    Vin_clamped = max(-Vin_max, min(Vin_max, max(Vin_min_i, min(Vin_max_i, Vin))));
    Vin_log(k)  = Vin_clamped;
    
    % Update arm plant
    x_arm(:, k+1) = Ad * x_arm(:, k) + Bd * Vin_clamped;
end

% =========================================================
% SECTION 10: Performance Metrics Calculation
% =========================================================
band     = 0.01 * q_total;
idx_end  = find(t_sim >= T(8), 1);
settled         = false;
t_settle_actual = NaN;
t_settle_start  = NaN;

for k = idx_end:N_sim
    in_band = abs(x_arm(3,k) - q_total) < band;
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

phi_max_deg    = max(abs(phi_log)) * 180/pi;
phi_after      = phi_log(idx_end:end);
t_after        = t_sim(idx_end:end);

rod_settled = false;
t_rod_settle = NaN;
phi_threshold = 0.05 * max(abs(phi_log));

for k = 1:length(phi_after)
    if abs(phi_after(k)) < phi_threshold
        if ~rod_settled
            t_rod_settle = t_after(k) - T(8);
            rod_settled  = true;
        end
    else
        rod_settled = false;
    end
end

theta_max = max(x_arm(3, idx_end:end));
overshoot = max(0, (theta_max - q_total)/q_total*100);
max_i     = max(abs(x_arm(1,:)));

fprintf('\n=== Arm Performance ===\n');
if isnan(t_settle_actual)
    fprintf('Settling time : DID NOT SETTLE\n');
else
    fprintf('Settling time : %.4f s  (req <= 0.5 s)  %s\n', ...
        t_settle_actual, pass_fail(t_settle_actual <= 0.5));
end
fprintf('Overshoot     : %.4f %%  (req <= 1 %%)   %s\n', ...
    overshoot, pass_fail(overshoot <= 1));
fprintf('Max current   : %.4f A  (limit %.1f A)  %s\n', ...
    max_i, i_max, pass_fail(max_i <= i_max));

fprintf('\n=== Rod Performance ===\n');
fprintf('Max rod swing : %.4f deg\n', phi_max_deg);
fprintf('wn_rod        : %.4f rad/s\n', wn_rod);
if isnan(t_rod_settle)
    fprintf('Rod settle    : DID NOT SETTLE within simulation\n');
else
    fprintf('Rod settle (5%%): %.4f s\n', t_rod_settle);
end

% =========================================================
% SECTION 11: Plot Arm Diagnostics
% =========================================================
figure('Name', 'Cascade PID + Rod Dynamics', ...
    'Position', [50 50 1400 900], 'Color', 'white');
set(gcf, 'DefaultTextColor',       'black');
set(gcf, 'DefaultAxesColor',       'white');
set(gcf, 'DefaultAxesXColor',      'black');
set(gcf, 'DefaultAxesYColor',      'black');
set(gcf, 'DefaultAxesGridColor',   [0.85 0.85 0.85]);
set(gcf, 'DefaultLegendTextColor', 'black');

% Arm Position Plot
subplot(3,2,1);
plot(t_sim, theta_r_log*180/pi, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
hold on;
plot(t_sim, x_arm(3,:)*180/pi, 'r',  'LineWidth', 1.2, 'DisplayName', 'Actual');
xline(T(8), 'k--', 'Traj end');
yline(360, 'm:', '360°');
ylabel('\theta (deg)'); title('Arm Position'); grid on;
lg=legend('Location','southeast'); set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);

% Position Error Plot
subplot(3,2,2);
plot(t_sim, (theta_r_log - x_arm(3,:))*180/pi, 'r', 'LineWidth', 1.2);
yline(+0.01*360, 'b--', '+1% band');
yline(-0.01*360, 'b--', '-1% band');
xline(T(8), 'k--');
ylabel('Error (deg)'); title('Position Error'); grid on;

% Arm Velocity Plot
subplot(3,2,3);
plot(t_sim, vref_log,    'b--', 'LineWidth', 1.2, 'DisplayName', 'v_{ref}');
hold on;
plot(t_sim, x_arm(2,:), 'r',   'LineWidth', 1.2, 'DisplayName', '\omega actual');
yline(+v_max, 'k--', 'v_{max}');
ylabel('\omega (rad/s)'); title('Arm Velocity'); grid on;
lg=legend('Location','best'); set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);

% Rod Angle Plot
subplot(3,2,4);
plot(t_sim, phi_log*180/pi, 'b', 'LineWidth', 1.2);
xline(T(8), 'k--', 'Traj end');
yline(0, 'k:');
xlabel('Time (s)'); ylabel('\phi (deg)');
title(sprintf('Rod Swing  (max = %.2f deg)', phi_max_deg));
grid on;

% Motor Voltage Plot
subplot(3,2,5);
plot(t_sim, Vin_log, 'b', 'LineWidth', 1.2);
yline(+Vin_max, 'r--', '+24V');
yline(-Vin_max, 'r--', '-24V');
xlabel('Time (s)'); ylabel('V_{in} (V)'); title('Motor Voltage'); grid on;

% Motor Current Plot
subplot(3,2,6);
plot(t_sim, x_arm(1,:), 'b', 'LineWidth', 1.2);
yline(+i_max, 'r--', '+10A');
yline(-i_max, 'r--', '-10A');
xlabel('Time (s)'); ylabel('Current (A)');
title(sprintf('Motor Current  (max = %.2f A)', max_i));
grid on;

sgtitle('Cascade PID + Rod Pendulum Dynamics', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', 'black');

% Correct all subplot visuals
ax_all = findall(gcf, 'Type', 'axes');
for ax = ax_all'
    lg = get(ax, 'Legend');
    if ~isempty(lg)
        set(lg, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
    end
    set(get(ax, 'Title'),  'Color', 'black');
    set(get(ax, 'XLabel'), 'Color', 'black');
    set(get(ax, 'YLabel'), 'Color', 'black');
    set(ax, 'XColor', 'black', 'YColor', 'black');
end
set(findall(gcf, 'Type', 'text'), 'Color', 'black');

% =========================================================
% SECTION 12: Wait-for-Rod Analysis & Cycle Time
% =========================================================
phi_threshold_deg = 0.57;                    
phi_threshold     = phi_threshold_deg * pi/180;  
idx_traj_end = find(t_sim >= T(8), 1);

% Analysis A: Wait until rod settles within threshold
wait_settled   = false;
t_wait_start   = NaN;
t_rod_ready_A  = NaN;
consecutive_ok = 0;
min_consecutive = round(0.1 / dt);   

for k = idx_traj_end:N_sim
    if abs(phi_log(k)) <= phi_threshold
        consecutive_ok = consecutive_ok + 1;
        if ~wait_settled
            t_wait_start = t_sim(k);
            wait_settled = true;
        end
        if consecutive_ok >= min_consecutive
            t_rod_ready_A = t_wait_start;
            break;
        end
    else
        consecutive_ok = 0;
        wait_settled   = false;
    end
end

if ~isnan(t_rod_ready_A)
    t_wait_A = t_rod_ready_A - T(8);   
else
    t_wait_A = NaN;
end

% Analysis B: Monitor Rod angle mid-trajectory
idx_move_start = 1;
idx_move_end   = idx_traj_end;
phi_during_move    = phi_log(idx_move_start:idx_move_end);
n_exceed           = sum(abs(phi_during_move) > phi_threshold);
pct_exceed         = n_exceed / length(phi_during_move) * 100;
in_threshold_during = abs(phi_during_move) <= phi_threshold;
any_ok_during       = any(in_threshold_during);

% Cycle time approximations
t_pick  = 2.0;   
t_place = 2.0;   
n_piece = 4;

if ~isnan(t_wait_A)
    t_cycle_A = n_piece * (t_total + t_wait_A + t_pick + t_total + t_wait_A + t_place);
else
    t_cycle_A = Inf;
end

t_cycle_no_wait = n_piece * (t_total + t_pick + t_total + t_place);

fprintf('\n=== Rod Threshold Analysis ===\n');
fprintf('Threshold       : ±%.2f deg\n', phi_threshold_deg);
fprintf('Clearance check : L*sin(%.2f deg) = %.3f mm  (clearance = 1 mm)\n', ...
    phi_threshold_deg, L_rod*1000*sin(phi_threshold));

fprintf('\n--- Analysis A: Wait for rod to settle ---\n');
if isnan(t_rod_ready_A)
    fprintf('Rod does not settle within simulation time.\n');
else
    fprintf('Rod settles at t = %.4f s  (wait after target reached %.4f s)\n', ...
        t_rod_ready_A, t_wait_A);
end
fprintf('Cycle time (Type A, 4 pcs) = %.2f s  (req <= 35 s)  %s\n', ...
    t_cycle_A, pass_fail(t_cycle_A <= 35));

fprintf('\n--- Analysis B: Mid-trajectory monitoring ---\n');
fprintf('Rod exceeds threshold %.1f%% of move duration.\n', pct_exceed);
if any_ok_during
    fprintf('There are acceptable clearance windows to pick mid-trajectory.\n');
else
    fprintf('Rod exceeds threshold strictly through the trajectory.\n');
end

fprintf('\n--- Compare Cycle Times ---\n');
fprintf('Theoretical immediate pickup: %.2f s\n', t_cycle_no_wait);
fprintf('Awaiting rod settling       : %.2f s\n', t_cycle_A);
fprintf('Target limit                : <= 35 s\n');

% =========================================================
% SECTION 13: Plot Rod Analysis
% =========================================================
figure('Name', 'Rod Swing Analysis', ...
    'Position', [50 50 1200 800], 'Color', 'white');
set(gcf, 'DefaultTextColor',       'black');
set(gcf, 'DefaultAxesColor',       'white');
set(gcf, 'DefaultAxesXColor',      'black');
set(gcf, 'DefaultAxesYColor',      'black');
set(gcf, 'DefaultAxesGridColor',   [0.85 0.85 0.85]);

% Full Timeline Rod Angle Plot
subplot(2,1,1);
plot(t_sim, phi_log*180/pi, 'b', 'LineWidth', 1.2, 'DisplayName', 'Rod angle \phi');
hold on;
yline(+phi_threshold_deg, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('+%.2f deg threshold', phi_threshold_deg));
yline(-phi_threshold_deg, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('-%.2f deg threshold', phi_threshold_deg));
yline(+180, 'k:', 'LineWidth', 1.0, 'DisplayName', '+180 deg limit');
yline(-180, 'k:', 'LineWidth', 1.0, 'DisplayName', '-180 deg limit');
xline(T(8), 'g--', 'LineWidth', 1.5, 'DisplayName', 'Traj end');
if ~isnan(t_rod_ready_A)
    xline(t_rod_ready_A, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Rod ready (A)');
end
xlabel('Time (s)'); ylabel('\phi (deg)');
title(sprintf('Rod Swing — Max = %.2f deg  |  Rod ready after %.4f s', ...
    phi_max_deg, t_wait_A));
lg = legend('Location', 'northeast', 'NumColumns', 2);
set(lg, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
grid on;

% Zoom Plot: Post-Trajectory
subplot(2,1,2);
t_zoom  = t_sim(idx_traj_end:end);
phi_zoom = phi_log(idx_traj_end:end) * 180/pi;
plot(t_zoom, phi_zoom, 'b', 'LineWidth', 1.2, 'DisplayName', 'Rod angle \phi');
hold on;
yline(+phi_threshold_deg, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('±%.2f deg', phi_threshold_deg));
yline(-phi_threshold_deg, 'r--', 'LineWidth', 1.5);
if ~isnan(t_rod_ready_A)
    xline(t_rod_ready_A, 'm--', 'LineWidth', 1.5, 'DisplayName', sprintf('Rod ready t=%.3f s', t_rod_ready_A));
end

% Shade acceptable clearance regions
in_thr = abs(phi_zoom) <= phi_threshold_deg;
for k = 1:length(t_zoom)-1
    if in_thr(k)
        patch([t_zoom(k) t_zoom(k+1) t_zoom(k+1) t_zoom(k)], ...
              [-phi_threshold_deg -phi_threshold_deg ...
               phi_threshold_deg  phi_threshold_deg], ...
              [0.5 1 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
    end
end
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Zoom: Rod Swing post trajectory (Green = inside acceptable clearance)');
lg = legend('Location', 'northeast');
set(lg, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
grid on;

sgtitle('Rod Swing Analysis — ±0.57° Threshold', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', 'black');

% Correct visual properties for subplot
ax_all = findall(gcf, 'Type', 'axes');
for ax = ax_all'
    lg2 = get(ax, 'Legend');
    if ~isempty(lg2)
        set(lg2, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
    end
    set(get(ax, 'Title'),  'Color', 'black');
    set(get(ax, 'XLabel'), 'Color', 'black');
    set(get(ax, 'YLabel'), 'Color', 'black');
    set(ax, 'XColor', 'black', 'YColor', 'black');
end
set(findall(gcf, 'Type', 'text'), 'Color', 'black');

% =========================================================
% SECTION 14: Local Functions
% =========================================================

%% pass_fail
%  Evaluates a boolean condition and returns 'PASS' or 'FAIL' string
%  @param cond - Logical evaluation output
%  @returns string 'PASS' or 'FAIL'
function s = pass_fail(cond)
    if cond
        s = 'PASS'; 
    else
        s = 'FAIL'; 
    end
end