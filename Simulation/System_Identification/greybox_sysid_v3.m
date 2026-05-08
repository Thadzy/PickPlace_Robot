% =============================================
% Grey-Box System Identification — Version 3
% G6 Circular Pick and Place Robot
% Estimate: J_total, B_arm, eta, k_e
% Fix:      k_t = 0.0382 (from rated torque/current)
%           R_m, L_m, N_total
% Training:   Step 24V (run1, run2, run3)
% Validation: Chirp 0.1-2 Hz + Stair Step
% =============================================

clear; clc; close all;

% ============================================================
% SECTION 0 — Fixed Parameters
% ============================================================
R_m     = 1.45;
L_m     = 1.47e-3;
N       = 70;

% k_t from rated calculation (fixed)
P_rated  = 60;
rpm_idle = 6000;
omega_m  = rpm_idle * (2*pi/60);
I_rated  = 2.5;
tau_m    = P_rated / omega_m;
k_t      = tau_m / I_rated;   % = 0.0382 N.m/A

fprintf('Fixed Parameters:\n');
fprintf('  R_m = %.4f Ohm\n', R_m);
fprintf('  L_m = %.4f mH\n',  L_m*1e3);
fprintf('  N   = %d\n',       N);
fprintf('  k_t = %.6f N.m/A (fixed)\n\n', k_t);

base_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';

% ============================================================
% SECTION 1 — Load Training Data (Step 24V)
% ============================================================
fprintf('Loading training data (Step 24V)...\n');

train_files = {'chirp_01_2_run1.mat', 'chirp_01_2_run2.mat', 'chirp_01_2_run3.mat'};
train_data  = [];

for r = 1:length(train_files)
    tmp      = load(fullfile(base_path, train_files{r}));
    d        = tmp.data;
    omega    = double(squeeze(d{1}.Values.Data));
    Vin      = double(squeeze(d{3}.Values.Data));
    t        = d{1}.Values.Time;
    Ts       = mean(diff(t));
    fc       = 5;
    [bf, af] = butter(2, fc*2*Ts, 'low');
    omega_f  = filtfilt(bf, af, omega);
    dat      = iddata(omega_f, Vin, Ts, ...
        'InputName', 'Vin', 'OutputName', 'omega_arm');
    if isempty(train_data)
        train_data = dat;
    else
        train_data = merge(train_data, dat);
    end
end
fprintf('Training data loaded: %d runs\n\n', length(train_files));

% ============================================================
% SECTION 2 — Build idgrey Model
% ============================================================
% Parameter order: [J_total, B_arm, eta, k_e]
% Initial guesses
p0 = [0.5764, 0.001, 0.85, 0.0416];
lb = [0.01,   1e-5,  0.3,  0.01  ];
ub = [5.0,    10.0,  1.0,  0.5   ];

odefun = @motor_grey_ode_v2;

% Pass fixed params as extra args: {R_m, L_m, N, k_t}
sys0 = idgrey(odefun, p0, 'c', {R_m, L_m, N, k_t});

sys0.Structure.Parameters.Minimum = lb;
sys0.Structure.Parameters.Maximum = ub;
sys0.Structure.Parameters.Free    = [true true true true];

% ============================================================
% SECTION 3 — Estimate
% ============================================================
fprintf('Running Grey-Box Identification...\n');

opt = greyestOptions;
opt.Display                     = 'on';
opt.SearchMethod                = 'lm';
opt.SearchOptions.MaxIterations = 300;
opt.SearchOptions.Tolerance     = 1e-10;
opt.InitialState                = 'estimate';

sys_est = greyest(train_data, sys0, opt);

% Extract
p_est   = sys_est.Structure.Parameters.Value;
J_est   = p_est(1);
B_est   = p_est(2);
eta_est = p_est(3);
Ke_est  = p_est(4);

% Uncertainty
try
    pstd    = sqrt(diag(getcov(sys_est)));
    J_std   = pstd(1);
    B_std   = pstd(2);
    eta_std = pstd(3);
    Ke_std  = pstd(4);
catch
    J_std = NaN; B_std = NaN; eta_std = NaN; Ke_std = NaN;
    fprintf('Warning: covariance not available\n');
end

fprintf('\n=== Estimated Parameters ===\n');
fprintf('J_total = %.6f +/- %.6f  kg.m2\n',    J_est,   J_std);
fprintf('B_arm   = %.6f +/- %.6f  N.m.s/rad\n', B_est,   B_std);
fprintf('eta     = %.6f +/- %.6f  -\n',          eta_est, eta_std);
fprintf('k_e     = %.6f +/- %.6f  V.s/rad\n',   Ke_est,  Ke_std);
fprintf('k_t     = %.6f            N.m/A  (fixed)\n', k_t);

% ============================================================
% SECTION 4 — Training Fit
% ============================================================
[~, fit_train, ~] = compare(train_data, sys_est);
fit_train = fit_train{1};
fprintf('\nTraining fit (Step 24V): %.2f%%\n', mean(fit_train));

% ============================================================
% SECTION 5 — Validation Data
% ============================================================
fprintf('\nLoading validation data...\n');

val_files_chirp = {'chirp_01_2_run1.mat', 'chirp_01_2_run2.mat', 'chirp_01_2_run3.mat'};
val_files_ss    = {'ss_run1.mat',          'ss_run2.mat',          'ss_run3.mat'};
val_chirp = []; val_ss = [];

for r = 1:3
    tmp = load(fullfile(base_path, val_files_chirp{r}));
    d = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));
    Vin   = double(squeeze(d{3}.Values.Data));
    t     = d{1}.Values.Time; Ts = mean(diff(t));
    fc = 5; [bf,af] = butter(2, fc*2*Ts, 'low');
    omega_f = filtfilt(bf, af, omega);
    dat = iddata(omega_f, Vin, Ts, 'InputName', 'Vin', 'OutputName', 'omega_arm');
    if isempty(val_chirp); val_chirp = dat; else; val_chirp = merge(val_chirp, dat); end

    tmp = load(fullfile(base_path, val_files_ss{r}));
    d = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));
    Vin   = double(squeeze(d{3}.Values.Data));
    t     = d{1}.Values.Time; Ts = mean(diff(t));
    omega_f = filtfilt(bf, af, omega);
    dat = iddata(omega_f, Vin, Ts, 'InputName', 'Vin', 'OutputName', 'omega_arm');
    if isempty(val_ss); val_ss = dat; else; val_ss = merge(val_ss, dat); end
end

[~, fit_chirp, ~] = compare(val_chirp, sys_est);
[~, fit_ss,    ~] = compare(val_ss,    sys_est);
fit_chirp = fit_chirp{1};
fit_ss    = fit_ss{1};

fprintf('Validation fit (Chirp 0.1-2 Hz): %.2f%%\n', mean(fit_chirp));
fprintf('Validation fit (Stair Step):      %.2f%%\n', mean(fit_ss));

% ============================================================
% SECTION 6 — Plots
% ============================================================
figure('Name', 'Training — Step 24V', 'Position', [100 100 1000 500]);
compare(train_data, sys_est);
title('Training Fit — Step 24V'); grid on;

figure('Name', 'Validation — Chirp', 'Position', [150 150 1000 500]);
compare(val_chirp, sys_est);
title('Validation Fit — Chirp 0.1-2 Hz'); grid on;

figure('Name', 'Validation — Stair Step', 'Position', [200 200 1000 500]);
compare(val_ss, sys_est);
title('Validation Fit — Stair Step'); grid on;

figure('Name', 'Residual Analysis', 'Position', [250 250 1000 500]);
resid(train_data, sys_est);
title('Residual Analysis — Training Data');

% ============================================================
% SECTION 7 — Summary
% ============================================================
fprintf('\n========================================\n');
fprintf('   SYSTEM IDENTIFICATION RESULTS\n');
fprintf('========================================\n');
fprintf('Fixed:\n');
fprintf('  R_m   = %.4f Ohm\n',    R_m);
fprintf('  L_m   = %.4f mH\n',     L_m*1e3);
fprintf('  N     = %d\n',          N);
fprintf('  k_t   = %.6f N.m/A\n', k_t);
fprintf('\nEstimated:\n');
fprintf('  J_total = %.6f +/- %.6f  kg.m2\n',    J_est,   J_std);
fprintf('  B_arm   = %.6f +/- %.6f  N.m.s/rad\n', B_est,   B_std);
fprintf('  eta     = %.6f +/- %.6f  -\n',          eta_est, eta_std);
fprintf('  k_e     = %.6f +/- %.6f  V.s/rad\n',   Ke_est,  Ke_std);
fprintf('\nFit Scores:\n');
fprintf('  Training  (Step 24V):       %.2f%%\n', mean(fit_train));
fprintf('  Validation (Chirp 0.1-2Hz): %.2f%%\n', mean(fit_chirp));
fprintf('  Validation (Stair Step):    %.2f%%\n', mean(fit_ss));
fprintf('========================================\n');

% Save
save(fullfile(base_path, 'sysid_v3_results.mat'), ...
    'J_est', 'B_est', 'eta_est', 'Ke_est', 'k_t', ...
    'J_std', 'B_std', 'eta_std', 'Ke_std', ...
    'sys_est', 'fit_train', 'fit_chirp', 'fit_ss');
fprintf('\nResults saved to sysid_v3_results.mat\n');