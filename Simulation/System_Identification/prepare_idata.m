% =============================================
% สร้าง iddata object สำหรับ System Identification
% =============================================

Ts = 0.001; % sample time

% ---- Estimation Data (ใช้ run1 และ run2) ----

% Stair Step
dd_ss1   = iddata(w_ss1,   v_ss1,   Ts, 'Name', 'StairStep_run1');
dd_ss2   = iddata(w_ss2,   v_ss2,   Ts, 'Name', 'StairStep_run2');

% Chirp 0.5 Hz
dd_c05_1 = iddata(w_c05_1, v_c05_1, Ts, 'Name', 'Chirp05_run1');
dd_c05_2 = iddata(w_c05_2, v_c05_2, Ts, 'Name', 'Chirp05_run2');

% Chirp 1 Hz (0-10s)
dd_c1_1  = iddata(w_c1_1,  v_c1_1,  Ts, 'Name', 'Chirp1_run1');
dd_c1_2  = iddata(w_c1_2,  v_c1_2,  Ts, 'Name', 'Chirp1_run2');

% ---- Validation Data (ใช้ run3) ----
dd_ss_val  = iddata(w_ss3,   v_ss3,   Ts, 'Name', 'StairStep_val');
dd_c05_val = iddata(w_c05_3, v_c05_3, Ts, 'Name', 'Chirp05_val');
dd_c1_val  = iddata(w_c1_3,  v_c1_3,  Ts, 'Name', 'Chirp1_val');

% ---- รวม Estimation Data ----
dd_est = merge(dd_ss1, dd_ss2, dd_c05_1, dd_c05_2, dd_c1_1, dd_c1_2);

% ---- แก้ Summary ----
fprintf('=== iddata Summary ===\n');
fprintf('Estimation datasets : %d\n', length(dd_est.ExperimentName));
fprintf('Output name         : %s\n', dd_est.OutputName{1});
fprintf('Input name          : %s\n', dd_est.InputName{1});
fprintf('Sample time         : %.3f s\n', dd_est.Ts);

% =============================================
% กำหนด Transfer Function structure ของ DC Motor
% G(s) = Km / (LJ*s^2 + (RJ+LB)*s + (RB+Ke*Km))
% ซึ่ง simplify เป็น 2nd order:
% G(s) = b1*s + b0 / (s^2 + a1*s + a0)
% =============================================

% แก้ sample time print
fprintf('Sample time : %.3f s\n', dd_est.Ts{1});

% =============================================
% Estimate Transfer Function
% G(s) = omega(s) / Vin(s)
% DC Motor = 2nd order, 1 zero
% =============================================

np = 2;  % poles
nz = 1;  % zeros

fprintf('\nEstimating Transfer Function...\n');
sys_est = tfest(dd_est, np, nz);

fprintf('\n=== Estimated Transfer Function ===\n');
sys_est

% ---- fit score ----
fprintf('\n=== Fit Score ===\n');

datasets = {dd_ss1, 'StairStep run1 (est)';
            dd_ss2, 'StairStep run2 (est)';
            dd_ss_val,  'StairStep run3 (val)';
            dd_c05_1,   'Chirp 0.5Hz run1 (est)';
            dd_c05_val, 'Chirp 0.5Hz run3 (val)'};

for i = 1:size(datasets,1)
    [~, fit] = compare(datasets{i,1}, sys_est);
    fprintf('%-25s : %.2f%%\n', datasets{i,2}, fit);
end

% ---- Plot compare ----
figure('Name', 'Model vs Data', 'NumberTitle', 'off');

subplot(2,1,1);
compare(dd_ss1, sys_est);
title('Stair Step run1 — Estimation');
grid on;

subplot(2,1,2);
compare(dd_ss_val, sys_est);
title('Stair Step run3 — Validation');
grid on;