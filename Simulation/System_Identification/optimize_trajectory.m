% =========================================================
% optimize_trajectory.m
% Multi-objective optimization: Minimize Rod swing & Cycle time
% Variables: [v_max, a_max, j_max, wn_shaper, zeta_shaper]
% Constraints: cycle time <= 35 s, arm performance pass
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Fixed Plant Parameters
% =========================================================
params.R_m     = 1.45336;
params.L_m     = 0.00144802;
params.N_total = 70;
params.K_e     = 0.04165;
params.K_t     = 0.04065;
params.B_damp  = 0.19279;
params.J       = 0.72762;
params.eta     = 0.83607;
params.i_max   = 10.0;
params.Vin_max = 24.0;

% Rod parameters
params.m_rod   = 0.16408;
params.L_rod   = 0.10;
params.g       = 9.81;
params.r_arm   = 0.5;
params.l_cm    = 0.05;
params.I_pivot = (1/3) * params.m_rod * params.L_rod^2;
params.zeta_rod = 0.05;
params.wn_rod  = sqrt(params.m_rod * params.g * params.l_cm / params.I_pivot);

% PID gains (fixed)
params.Kp_vel = 20.0364; params.Ki_vel = 376.9228;
params.Kd_vel = 0.0325;  params.N_vel  = 100;
params.Kp_pos = 9.2670;  params.Ki_pos = 10.5;
params.Kd_pos = 0.08;    params.N_pos  = 20;
params.Kvff   = 3.034;   params.Kaff   = 0.4450;
params.Kaw    = 0.5;

% Constraints
params.cycle_time_limit  = 35.0;   % s
params.n_piece           = 4;
params.t_pick            = 2.0;    % s
params.t_place           = 2.0;    % s
params.phi_threshold_deg = 0.57;   % deg
params.overshoot_limit   = 1.0;    % %
params.settling_limit    = 0.5;    % s
params.dt                = 0.0001; % s

% =========================================================
% SECTION 2: Optimization Variables
% x = [v_max, a_max, j_max, wn_shaper, zeta_shaper]
% =========================================================

% Lower bounds
lb = [2.0,   5.0,   200,   5.0,  0.01];
% Upper bounds  
ub = [7.304, 27.49, 1400,  20.0, 0.20];

% Variable names for display
var_names = {'v_{max} (rad/s)', 'a_{max} (rad/s^2)', ...
             'j_{max} (rad/s^3)', '\omega_{n,shaper} (rad/s)', ...
             '\zeta_{shaper}'};

n_vars = length(lb);

% =========================================================
% SECTION 3: NSGA-II Options
% =========================================================
options = optimoptions('gamultiobj', ...
    'PopulationSize',       100, ...
    'MaxGenerations',       150, ...
    'CrossoverFraction',    0.8, ...
    'MutationFcn',          {@mutationadaptfeasible}, ...
    'ParetoFraction',       0.5, ...
    'FunctionTolerance',    1e-4, ...
    'Display',              'iter', ...
    'PlotFcn',              {@gaplotpareto}, ...
    'UseParallel',          false);

fprintf('=== Starting Multi-objective Optimization (NSGA-II) ===\n');
fprintf('Variables: v_max, a_max, j_max, wn_shaper, zeta_shaper\n');
fprintf('Objectives: [1] Rod swing (deg)  [2] Cycle time (s)\n');
fprintf('Population: 100,  Generations: 150\n\n');

tic;

% =========================================================
% SECTION 4: Run Optimization
% =========================================================
[x_pareto, f_pareto, exitflag, output] = gamultiobj( ...
    @(x) objective_fn(x, params), ...
    n_vars, [], [], [], [], lb, ub, ...
    @(x) constraint_fn(x, params), ...
    options);

t_elapsed = toc;
fprintf('\nOptimization complete in %.1f s\n', t_elapsed);
fprintf('Pareto front solutions: %d\n', size(x_pareto, 1));

% =========================================================
% SECTION 5: Analyze Pareto Front
% =========================================================
rod_swing_pareto  = f_pareto(:, 1);
cycle_time_pareto = f_pareto(:, 2);

% Find best compromise: minimize weighted sum (normalized)
rod_norm   = (rod_swing_pareto  - min(rod_swing_pareto))  / (max(rod_swing_pareto)  - min(rod_swing_pareto) + 1e-9);
cycle_norm = (cycle_time_pareto - min(cycle_time_pareto)) / (max(cycle_time_pareto) - min(cycle_time_pareto) + 1e-9);

% Equal weight compromise
w_rod   = 0.5;
w_cycle = 0.5;
score   = w_rod * rod_norm + w_cycle * cycle_norm;
[~, idx_best] = min(score);

x_best = x_pareto(idx_best, :);
f_best = f_pareto(idx_best, :);

fprintf('\n=== Best Compromise Solution (Equal Weight) ===\n');
for i = 1:n_vars
    fprintf('  %-30s = %.4f\n', var_names{i}, x_best(i));
end
fprintf('\n  Rod swing   = %.4f deg\n', f_best(1));
fprintf('  Cycle time  = %.4f s\n',   f_best(2));
fprintf('  Feasible?   = %s\n', ...
    pass_fail(f_best(2) <= params.cycle_time_limit && f_best(1) <= 180));

% Also find: minimum rod swing solution
[~, idx_min_rod] = min(rod_swing_pareto);
fprintf('\n=== Minimum Rod Swing Solution ===\n');
fprintf('  Rod swing   = %.4f deg\n', f_pareto(idx_min_rod, 1));
fprintf('  Cycle time  = %.4f s\n',   f_pareto(idx_min_rod, 2));

% Also find: minimum cycle time solution
[~, idx_min_cycle] = min(cycle_time_pareto);
fprintf('\n=== Minimum Cycle Time Solution ===\n');
fprintf('  Rod swing   = %.4f deg\n', f_pareto(idx_min_cycle, 1));
fprintf('  Cycle time  = %.4f s\n',   f_pareto(idx_min_cycle, 2));

% =========================================================
% SECTION 6: Plot Pareto Front
% =========================================================
figure('Name', 'Pareto Front', 'Position', [50 50 900 600], 'Color', 'white');
set(gcf, 'DefaultTextColor', 'black', 'DefaultAxesColor', 'white', ...
    'DefaultAxesXColor', 'black', 'DefaultAxesYColor', 'black');

scatter(cycle_time_pareto, rod_swing_pareto, 60, score, 'filled');
colorbar; colormap(parula);
hold on;

% Highlight key solutions
scatter(f_pareto(idx_best,     2), f_pareto(idx_best,     1), 150, 'r', 'filled', ...
    'DisplayName', 'Best compromise');
scatter(f_pareto(idx_min_rod,  2), f_pareto(idx_min_rod,  1), 150, 'g', 'filled', ...
    'DisplayName', 'Min rod swing');
scatter(f_pareto(idx_min_cycle,2), f_pareto(idx_min_cycle,1), 150, 'b', 'filled', ...
    'DisplayName', 'Min cycle time');

% Constraint lines
xline(params.cycle_time_limit, 'r--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Cycle limit = %g s', params.cycle_time_limit));
yline(180, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Rod limit = 180°');

xlabel('Cycle Time (s)');
ylabel('Max Rod Swing (deg)');
title('Pareto Front: Rod Swing vs Cycle Time');
lg = legend('Location', 'northeast');
set(lg, 'Color', 'white', 'TextColor', 'black', 'EdgeColor', [0.8 0.8 0.8]);
grid on;
set(findall(gcf, 'Type', 'text'), 'Color', 'black');

% =========================================================
% SECTION 7: Re-simulate Best Solution
% =========================================================
fprintf('\n=== Re-simulating Best Compromise Solution ===\n');
[f_check, info] = objective_fn(x_best, params);
fprintf('Rod swing   : %.4f deg\n', f_check(1));
fprintf('Cycle time  : %.4f s\n',   f_check(2));
fprintf('Arm settle  : %.4f s\n',   info.t_settle);
fprintf('Overshoot   : %.4f %%\n',  info.overshoot);
fprintf('Max current : %.4f A\n',   info.max_i);

% =========================================================
% SECTION 8: Objective Function
% =========================================================
function [f, varargout] = objective_fn(x, p)
    v_max      = x(1);
    a_max      = x(2);
    j_max      = x(3);
    wn_shaper  = x(4);
    zeta_shaper = x(5);

    % --- S-curve timing ---
    t_j = a_max / j_max;
    t_a = v_max / a_max - t_j;

    if t_a < 0
        % Triangle profile — recompute
        t_j = sqrt(v_max / j_max);
        t_a = 0;
        a_max = j_max * t_j;
    end

    v1 = 0.5 * j_max * t_j^2;
    a1 = j_max * t_j;
    d1 = j_max * t_j^3 / 6;
    d2 = v1 * t_a + 0.5 * a1 * t_a^2;
    v2 = v1 + a1 * t_a;
    d3 = v2 * t_j + 0.5 * a1 * t_j^2 - j_max * t_j^3 / 6;

    q_total = 2 * pi;
    d_accel = d1 + d2 + d3;

    if 2 * d_accel > q_total
        % ไม่พอระยะ -- penalty
        f = [1e6, 1e6];
        if nargout > 1; varargout{1} = struct('t_settle',NaN,'overshoot',NaN,'max_i',NaN); end
        return;
    end

    d_cruise = q_total - 2 * d_accel;
    t_v      = d_cruise / v_max;
    t_total  = 4 * t_j + 2 * t_a + t_v;

    % --- Phase end-times ---
    T    = zeros(1, 9);
    T(2) = t_j;
    T(3) = T(2) + t_a;
    T(4) = T(3) + t_j;
    T(5) = T(4) + t_v;
    T(6) = T(5) + t_j;
    T(7) = T(6) + t_a;
    T(8) = T(7) + t_j;

    % --- Discretize arm plant ---
    dt  = p.dt;
    A_c = [-p.R_m/p.L_m,                -p.K_e*p.N_total/p.L_m, 0;
            p.K_t*p.eta*p.N_total/p.J,  -p.B_damp/p.J,           0;
            0,                            1,                        0];
    B_c = [1/p.L_m; 0; 0];
    sys_d = c2d(ss(A_c, B_c, [0,1,0;0,0,1], [0;0]), dt, 'zoh');
    Ad = sys_d.A; Bd = sys_d.B;

    % --- Rod state space ---
    A_rod = [0, 1; -p.wn_rod^2, -2*p.zeta_rod*p.wn_rod];
    B_rod = [0; -p.m_rod*p.l_cm*p.r_arm/p.I_pivot];
    sys_rod_d = c2d(ss(A_rod, B_rod, eye(2), zeros(2,1)), dt, 'zoh');
    Ad_rod = sys_rod_d.A; Bd_rod = sys_rod_d.B;

    % --- ZV Shaper ---
    omega_d_sh = wn_shaper * sqrt(1 - zeta_shaper^2);
    K_zv       = exp(-zeta_shaper * pi / sqrt(1 - zeta_shaper^2));
    t2_zv      = pi / omega_d_sh;
    A1_zv      = 1 / (1 + K_zv);
    A2_zv      = K_zv / (1 + K_zv);

    % --- Pre-compute trajectory ---
    t_end = t_total + 2.0;
    t_sim = 0:dt:t_end;
    N_sim = length(t_sim);

    traj_p_raw = zeros(1, N_sim);
    traj_v_raw = zeros(1, N_sim);
    traj_a_raw = zeros(1, N_sim);

    for k = 1:N_sim
        tk = t_sim(k);
        if     tk<=T(2); j_n=+j_max; a0=0;    v0=0;      p0=0;                         t0=T(1);
        elseif tk<=T(3); j_n=0;      a0=+a1;  v0=v1;     p0=d1;                        t0=T(2);
        elseif tk<=T(4); j_n=-j_max; a0=+a1;  v0=v2;     p0=d1+d2;                     t0=T(3);
        elseif tk<=T(5); j_n=0;      a0=0;    v0=v_max;  p0=d_accel;                   t0=T(4);
        elseif tk<=T(6); j_n=-j_max; a0=0;    v0=v_max;  p0=d_accel+d_cruise;          t0=T(5);
        elseif tk<=T(7); j_n=0;      a0=-a1;  v0=v_max-v1; p0=d_accel+d_cruise+d1;     t0=T(6);
        elseif tk<=T(8); j_n=+j_max; a0=-a1;  v0=v_max-v1-a1*t_a; p0=d_accel+d_cruise+d1+d2; t0=T(7);
        else;            j_n=0;      a0=0;    v0=0;      p0=q_total;                   t0=T(8);
        end
        tau = tk - t0;
        traj_p_raw(k) = min(p0+v0*tau+0.5*a0*tau^2+j_n*tau^3/6, q_total);
        traj_v_raw(k) = v0+a0*tau+0.5*j_n*tau^2;
        traj_a_raw(k) = a0+j_n*tau;
    end

    % Apply ZV shaping
    delay_steps   = round(t2_zv / dt);
    traj_p_shaped = zeros(1, N_sim);
    traj_v_shaped = zeros(1, N_sim);
    traj_a_shaped = zeros(1, N_sim);
    for k = 1:N_sim
        kd = k - delay_steps;
        if kd > 0
            traj_p_shaped(k) = A1_zv*traj_p_raw(k) + A2_zv*traj_p_raw(kd);
            traj_v_shaped(k) = A1_zv*traj_v_raw(k) + A2_zv*traj_v_raw(kd);
            traj_a_shaped(k) = A1_zv*traj_a_raw(k) + A2_zv*traj_a_raw(kd);
        else
            traj_p_shaped(k) = A1_zv*traj_p_raw(k);
            traj_v_shaped(k) = A1_zv*traj_v_raw(k);
            traj_a_shaped(k) = A1_zv*traj_a_raw(k);
        end
    end

    % --- Simulate ---
    x_arm = zeros(3, N_sim);
    x_rod = zeros(2, N_sim);
    phi_log = zeros(1, N_sim);

    int_vel=0; int_pos=0; dfilt_vel=0; dfilt_pos=0;
    err_vel_prev=0; err_pos_prev=0; aw_vel=0; aw_pos=0;
    omega_f_prev=0; theta_f_prev=0; alpha_prev=0;

    wc_fb    = 2*pi*100;
    alpha_fb = wc_fb*dt/(1+wc_fb*dt);
    vref_max_local = v_max;

    for k = 1:N_sim-1
        traj_p_k = traj_p_shaped(k);
        traj_v_k = traj_v_shaped(k);
        traj_a_k = traj_a_shaped(k);

        alpha_now  = (x_arm(2,k) - alpha_prev) / dt;
        alpha_prev = x_arm(2,k);

        x_rod(:,k+1)  = Ad_rod*x_rod(:,k) + Bd_rod*alpha_now;
        phi_log(k)    = x_rod(1,k);

        omega_f      = alpha_fb*x_arm(2,k) + (1-alpha_fb)*omega_f_prev;
        theta_f      = alpha_fb*x_arm(3,k) + (1-alpha_fb)*theta_f_prev;
        omega_f_prev = omega_f;
        theta_f_prev = theta_f;

        err_pos      = traj_p_k - theta_f;
        dfilt_pos    = (1-p.N_pos*dt)*dfilt_pos + p.Kd_pos*p.N_pos*(err_pos-err_pos_prev);
        err_pos_prev = err_pos;
        int_pos      = int_pos + err_pos*dt + p.Kaw*aw_pos*dt;
        vref_pid     = p.Kp_pos*err_pos + p.Ki_pos*int_pos + dfilt_pos;
        vref_cmd     = max(-vref_max_local, min(vref_max_local, vref_pid+traj_v_k));
        aw_pos       = max(-vref_max_local, min(vref_max_local, vref_pid)) - vref_pid;

        err_vel      = vref_cmd - omega_f;
        dfilt_vel    = (1-p.N_vel*dt)*dfilt_vel + p.Kd_vel*p.N_vel*(err_vel-err_vel_prev);
        err_vel_prev = err_vel;
        int_vel      = int_vel + err_vel*dt + p.Kaw*aw_vel*dt;
        V_pid        = p.Kp_vel*err_vel + p.Ki_vel*int_vel + dfilt_vel;
        Vff          = p.Kvff*vref_cmd + p.Kaff*traj_a_k;
        Vin_raw      = V_pid + Vff;
        Vin          = max(-p.Vin_max, min(p.Vin_max, Vin_raw));
        aw_vel       = Vin - Vin_raw;

        i_now     = x_arm(1,k);
        omega_now = x_arm(2,k);
        V_bemf    = p.K_e*p.N_total*omega_now;
        Vin_max_i = p.L_m*(p.i_max-i_now)/dt + p.R_m*i_now + V_bemf;
        Vin_min_i = p.L_m*(-p.i_max-i_now)/dt + p.R_m*i_now + V_bemf;
        Vin_c     = max(-p.Vin_max, min(p.Vin_max, max(Vin_min_i, min(Vin_max_i, Vin))));

        x_arm(:,k+1) = Ad*x_arm(:,k) + Bd*Vin_c;
    end
    phi_log(end) = x_rod(1,end);

    % --- Metrics ---
    idx_end  = find(t_sim >= T(8), 1);
    band     = 0.01 * q_total;

    % Arm settling
    settled = false; t_settle_actual = NaN; t_settle_start = NaN;
    for k = idx_end:N_sim
        if abs(x_arm(3,k) - q_total) < band
            if ~settled; t_settle_start = t_sim(k); settled = true; end
            if (t_sim(k) - t_settle_start) > 0.05
                t_settle_actual = t_settle_start - T(8);
                break;
            end
        else
            settled = false;
        end
    end

    theta_max = max(x_arm(3, idx_end:end));
    overshoot = max(0, (theta_max - q_total)/q_total*100);
    max_i     = max(abs(x_arm(1,:)));
    phi_max   = max(abs(phi_log)) * 180/pi;

    % Rod settle time for cycle time calculation
    phi_thr   = p.phi_threshold_deg * pi/180;
    phi_after = phi_log(idx_end:end);
    t_after   = t_sim(idx_end:end);

    rod_settled = false; t_rod_ready = NaN;
    consecutive_ok = 0;
    min_consec = round(0.1/dt);
    t_wait_start_r = NaN;

    for k = 1:length(phi_after)
        if abs(phi_after(k)) <= phi_thr
            consecutive_ok = consecutive_ok + 1;
            if ~rod_settled; t_wait_start_r = t_after(k); rod_settled = true; end
            if consecutive_ok >= min_consec
                t_rod_ready = t_wait_start_r;
                break;
            end
        else
            consecutive_ok = 0; rod_settled = false;
        end
    end

    % Cycle time
    if ~isnan(t_rod_ready)
        t_wait = t_rod_ready - T(8);
        t_cycle = p.n_piece * (t_total + t_wait + p.t_pick + ...
                               t_total + t_wait + p.t_place);
    else
        t_cycle = 1e6;   % penalty ถ้า Rod ไม่นิ่ง
    end

    % Objectives
    f = [phi_max, t_cycle];

    if nargout > 1
        info.t_settle  = t_settle_actual;
        info.overshoot = overshoot;
        info.max_i     = max_i;
        info.t_cycle   = t_cycle;
        info.phi_max   = phi_max;
        varargout{1}   = info;
    end
end

% =========================================================
% SECTION 9: Constraint Function
% =========================================================
function [c, ceq] = constraint_fn(x, p)
    ceq = [];

    v_max = x(1);
    a_max = x(2);
    j_max = x(3);

    % S-curve feasibility
    t_j = a_max / j_max;
    t_a = v_max / a_max - t_j;

    if t_a < 0
        t_j = sqrt(v_max / j_max);
        t_a = 0;
        a_max = j_max * t_j;
    end

    v1 = 0.5*j_max*t_j^2;
    a1 = j_max*t_j;
    d1 = j_max*t_j^3/6;
    d2 = v1*t_a + 0.5*a1*t_a^2;
    v2 = v1 + a1*t_a;
    d3 = v2*t_j + 0.5*a1*t_j^2 - j_max*t_j^3/6;
    d_accel = d1+d2+d3;

    q_total = 2*pi;

    % c <= 0 คือ feasible
    c(1) = 2*d_accel - q_total;         % ระยะ accel ต้องไม่เกิน q_total
    c(2) = t_a - (v_max/a_max - t_j);   % t_a ต้องไม่ติดลบ
    c(3) = x(4) - 2*x(1)/p.L_rod;      % wn_shaper ไม่ควรเกิน physical limit
end

% =========================================================
function s = pass_fail(cond)
    if cond; s = 'PASS'; else; s = 'FAIL'; end
end