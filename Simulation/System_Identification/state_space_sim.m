% =============================================
% state_space_sim.m
% State Space Simulation — G6 Pick and Place Robot
% =============================================

clc; clear;

% =============================================
% 1. Parameters
% =============================================
R_m   = 1.5398;
L_m   = 0.001473;
K_e   = 0.04322;
K_t   = 0.04053;
B     = 0.19835;
J     = 0.6615;
eta   = 0.8495;
N     = 70;

Kp    = 66.6;
Ki    = 359.5;
Kd    = 2.22;

theta_ref = 2*pi;   % [rad] เป้าหมาย 1 รอบ

% =============================================
% 2. State Space Model
% =============================================
A = [-R_m/L_m,      -K_e*N/L_m;
      K_t*eta*N/J,  -B/J      ];

B_mat = [1/L_m; 0];
C_mat = [0, 1];
D_mat = 0;

sys_ss = ss(A, B_mat, C_mat, D_mat);

fprintf('=== State Space Model ===\n');
fprintf('A = [%.4f  %.4f]\n', A(1,1), A(1,2));
fprintf('    [%.4f  %.4f]\n', A(2,1), A(2,2));
fprintf('B = [%.4f]\n', B_mat(1));
fprintf('    [%.4f]\n', B_mat(2));

ev = eig(A);
fprintf('\n=== Open-Loop Eigenvalues ===\n');
for i = 1:length(ev)
    fprintf('  lambda_%d = %.4f + %.4fj\n', i, real(ev(i)), imag(ev(i)));
end
if all(real(ev) < 0)
    fprintf('System is Stable\n\n');
else
    fprintf('System is Unstable\n\n');
end

% =============================================
% 3. Closed-Loop System (Position Control)
% =============================================
% Plant: omega → integrate → theta
% Loop: PID(theta_error) → Vin → Motor → omega → integrate → theta

s       = tf('s');
sys_tf  = tf(sys_ss);           % Vin → omega
plant   = sys_tf * (1/s);       % Vin → theta (integrate omega)
PID_tf  = Kp + Ki/s + Kd*s/(0.005*s + 1);  % PID with filter N=200
sys_cl  = feedback(PID_tf * plant, 1);

fprintf('=== Closed-Loop Poles ===\n');
cl_poles = pole(sys_cl);
for i = 1:length(cl_poles)
    fprintf('  p_%d = %.4f + %.4fj\n', i, real(cl_poles(i)), imag(cl_poles(i)));
end

% =============================================
% 4. Step Response
% =============================================
t_sim = 0:0.001:5;
[theta_cl, t_cl] = step(theta_ref * sys_cl, t_sim);

% omega = derivative of theta
omega_cl = diff(theta_cl) ./ diff(t_cl);
t_omega  = t_cl(1:end-1);

% =============================================
% 5. Performance Metrics
% =============================================
steady_val  = theta_cl(end);
band        = 0.02 * steady_val;
settled_idx = find(abs(theta_cl - steady_val) > band, 1, 'last');

if ~isempty(settled_idx) && settled_idx < length(t_cl)
    t_settle = t_cl(settled_idx);
else
    t_settle = t_cl(end);
end

peak_val  = max(theta_cl);
overshoot = (peak_val - steady_val) / steady_val * 100;

fprintf('\n=== Performance Metrics ===\n');
fprintf('Target              = %.4f rad (%.1f deg)\n', theta_ref, rad2deg(theta_ref));
fprintf('Final Value         = %.4f rad\n', steady_val);
fprintf('Settling Time (2%%) = %.4f s  [Req: <= 0.5 s]\n', t_settle);
fprintf('Overshoot           = %.4f %%  [Req: <= 1%%]\n', overshoot);

if t_settle <= 0.5
    fprintf('Settling Time: PASS\n');
else
    fprintf('Settling Time: FAIL\n');
end
if overshoot <= 1.0
    fprintf('Overshoot    : PASS\n');
else
    fprintf('Overshoot    : FAIL\n');
end

% =============================================
% 6. Plot
% =============================================
figure('Name','Step Response — State Space + PID', ...
    'NumberTitle','off','Position',[100 100 1000 600]);

subplot(2,1,1);
plot(t_cl, theta_cl, 'b', 'LineWidth', 1.5, 'DisplayName', 'Simulated \theta');
hold on;
yline(theta_ref,      'r--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
yline(theta_ref*1.02, 'g:',  'LineWidth', 1.0, 'DisplayName', '+2% band');
yline(theta_ref*0.98, 'g:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
if t_settle < t_cl(end)
    xline(t_settle, 'm--', 'LineWidth', 1.2, ...
        'DisplayName', sprintf('t_{settle} = %.3f s', t_settle));
end
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Step Response — Closed-Loop Position Control (State Space + PID)');
legend('Location','southeast'); grid on;
xlim([0 5]);

subplot(2,1,2);
plot(t_omega, omega_cl, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Angular Velocity');
grid on; xlim([0 5]);

% =============================================
% 7. Save
% =============================================
save_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/state_space_result.mat';
save(save_path, 'sys_ss', 'sys_cl', 't_cl', 'theta_cl', 'omega_cl', ...
    'theta_ref', 't_settle', 'overshoot', 'Kp', 'Ki', 'Kd');
fprintf('\nSaved to: %s\n', save_path);