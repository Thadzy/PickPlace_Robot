% =============================================
% Grey-Box System Identification
% G6 Circular Pick and Place Robot
% Estimate: J, B, Kt, Ke, eta
% Training:   Chirp 0.1-2 Hz (run1, run2, run3)
% Validation: Step 24V + Stair Step (run1, run2, run3)
% =============================================

clear; clc; close all;

% ============================================================
% SECTION 0 — Known Parameters (Fixed, not estimated)
% ============================================================
R_m   = 1.45;       % Armature resistance [Ohm]
L_m   = 1.47e-3;    % Armature inductance [H]
N     = 70;         % Total gear ratio (gearbox x belt)

% eta*Kt appear as a product in torque eq: T = eta*Kt*N*i
% They cannot be identified separately from velocity data alone.
% Strategy: estimate KtEff = eta*Kt as a single parameter.
% eta is then computed after if Kt is known from datasheet.
% Parameter vector: [J, B, KtEff, Ke]  (4 params)

base_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';

% ============================================================
% SECTION 1 — Load and Prepare Training Data (Chirp 0.1-2 Hz)
% ============================================================
fprintf('Loading training data...\n');

% Use Step 24V as training — provides clear transient for J and B estimation
% Chirp moved to validation for frequency-domain verification
train_files = {'step_24_run1.mat', 'step_24_run2.mat', 'step_24_run3.mat'};

train_data = [];

for r = 1:length(train_files)
    tmp   = load(fullfile(base_path, train_files{r}));
    d     = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));   % rad/s at arm shaft
    Vin   = double(squeeze(d{3}.Values.Data));   % Volts
    t     = d{1}.Values.Time;

    Ts    = mean(diff(t));

    % Low-pass filter omega to reduce quantization noise
    % cutoff at 5 Hz, well above mechanical bandwidth
    fc    = 5;
    [bf, af] = butter(2, fc * 2 * Ts, 'low');
    omega_f  = filtfilt(bf, af, omega);

    % Create iddata object: input=Vin, output=omega_arm
    dat = iddata(omega_f, Vin, Ts, ...
        'InputName',  'Vin', ...
        'OutputName', 'omega_arm', ...
        'InputUnit',  'V', ...
        'OutputUnit', 'rad/s');

    if isempty(train_data)
        train_data = dat;
    else
        train_data = merge(train_data, dat);
    end
end

fprintf('Training data loaded: %d runs (Step 24V)\n', length(train_files));

% ============================================================
% SECTION 2 — Define Grey-Box ODE Model
% ============================================================
% State equations from Simulink model:
%
%   di/dt    = (1/Lm) * (Vin - Rm*i - Ke*N*omega_arm)
%   domega/dt = (1/J)  * (eta*Kt*N*i - B*omega_arm)
%
% States:  x = [i; omega_arm]
% Input:   u = Vin
% Output:  y = omega_arm = x(2)
%
% Parameters to estimate: [J, B, Kt, Ke, eta]
% ============================================================

% Initial guesses for parameters
% Combine eta*Kt into single KtEff to avoid identifiability issue
% Order: [J,    B,     KtEff,  Ke   ]
%         kg.m2 N.m.s  N.m/A   V.s/r
p0 = [0.5,  0.5,   0.08,   0.08];

% Lower and upper bounds
lb = [0.001, 0.001, 0.001, 0.001];
ub = [5.0,   50.0,  2.0,   2.0  ];

% ============================================================
% SECTION 3 — Build idgrey Model
% ============================================================
% idgrey requires a function that returns A, B, C, D, K
% as a function of the parameters

odefun = @motor_grey_ode;

sys0 = idgrey(odefun, p0, 'c', {R_m, L_m, N});

% Set bounds — Parameters is a single param.Continuous object with vector fields
sys0.Structure.Parameters.Minimum = lb;
sys0.Structure.Parameters.Maximum = ub;
sys0.Structure.Parameters.Free    = [true true true true];

% ============================================================
% SECTION 4 — Estimate Parameters
% ============================================================
fprintf('\nRunning Grey-Box Identification...\n');

opt = greyestOptions;
opt.Display          = 'on';
opt.SearchMethod     = 'lm';        % Levenberg-Marquardt
opt.SearchOptions.MaxIterations = 200;
opt.SearchOptions.Tolerance     = 1e-8;
opt.InitialState     = 'estimate';

sys_est = greyest(train_data, sys0, opt);

% Extract estimated parameters — Value is vector [J, B, KtEff, Ke]
p_est    = sys_est.Structure.Parameters.Value;
J_est    = p_est(1);
B_est    = p_est(2);
KtEff_est = p_est(3);   % KtEff = eta * Kt
Ke_est   = p_est(4);

% Parameter uncertainties (1-sigma)
try
    pstd  = sqrt(diag(getcov(sys_est)));
    J_std   = pstd(1);
    B_std   = pstd(2);
    KtEff_std = pstd(3);
    Ke_std    = pstd(4);
catch
    J_std = NaN; B_std = NaN; KtEff_std = NaN; Ke_std = NaN;
    fprintf('Warning: covariance not available\n');
end

fprintf('\n=== Estimated Parameters ===\n');
fprintf('J      = %.6f +/- %.6f  kg.m2\n',    J_est,     J_std);
fprintf('B      = %.6f +/- %.6f  N.m.s/rad\n', B_est,     B_std);
fprintf('KtEff  = %.6f +/- %.6f  N.m/A (eta*Kt)\n', KtEff_est, KtEff_std);
fprintf('Ke     = %.6f +/- %.6f  V.s/rad\n', Ke_est,    Ke_std);
fprintf('Note: KtEff = eta*Kt. Use Kt from datasheet to compute eta.\n');

% ============================================================
% SECTION 5 — Training Fit
% ============================================================
fprintf('\nComputing training fit...\n');

[~, fit_train, ~] = compare(train_data, sys_est);
fit_train = fit_train{1};
fprintf('Training fit (Step 24V): %.2f%%\n', mean(fit_train));

% ============================================================
% SECTION 6 — Load Validation Data
% ============================================================
fprintf('\nLoading validation data...\n');

val_files_step = {'chirp_01_2_run1.mat', 'chirp_01_2_run2.mat', 'chirp_01_2_run3.mat'};
val_files_ss   = {'ss_run1.mat',         'ss_run2.mat',         'ss_run3.mat'};

val_data_step = [];
val_data_ss   = [];

for r = 1:3
    % Step 24V
    tmp   = load(fullfile(base_path, val_files_step{r}));
    d     = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));
    Vin   = double(squeeze(d{3}.Values.Data));
    t     = d{1}.Values.Time;
    Ts    = mean(diff(t));
    fc    = 5;
    [bf, af] = butter(2, fc * 2 * Ts, 'low');
    omega_f  = filtfilt(bf, af, omega);
    dat = iddata(omega_f, Vin, Ts, 'InputName', 'Vin', 'OutputName', 'omega_arm');
    if isempty(val_data_step)
        val_data_step = dat;
    else
        val_data_step = merge(val_data_step, dat);
    end

    % Stair Step
    tmp   = load(fullfile(base_path, val_files_ss{r}));
    d     = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));
    Vin   = double(squeeze(d{3}.Values.Data));
    t     = d{1}.Values.Time;
    Ts    = mean(diff(t));
    omega_f  = filtfilt(bf, af, omega);
    dat = iddata(omega_f, Vin, Ts, 'InputName', 'Vin', 'OutputName', 'omega_arm');
    if isempty(val_data_ss)
        val_data_ss = dat;
    else
        val_data_ss = merge(val_data_ss, dat);
    end
end

% ============================================================
% SECTION 7 — Validation
% ============================================================
fprintf('Running validation...\n');

[~, fit_step, ~] = compare(val_data_step, sys_est);
fit_step = fit_step{1};
[~, fit_ss,   ~] = compare(val_data_ss,   sys_est);
fit_ss = fit_ss{1};

fprintf('Validation fit (Chirp 0.1-2 Hz): %.2f%%\n', mean(fit_step));
fprintf('Validation fit (Stair Step): %.2f%%\n', mean(fit_ss));

% ============================================================
% SECTION 8 — Plots
% ============================================================

% --- Plot 1: Training data compare ---
figure('Name', 'Training — Step 24V', 'Position', [100 100 1000 500]);
compare(train_data, sys_est);
title('Training Fit — Step 24V');
grid on;

% --- Plot 2: Validation Step 24V ---
figure('Name', 'Validation — Chirp 0.1-2 Hz', 'Position', [150 150 1000 500]);
compare(val_data_step, sys_est);
title('Validation Fit — Chirp 0.1-2 Hz');
grid on;

% --- Plot 3: Validation Stair Step ---
figure('Name', 'Validation — Stair Step', 'Position', [200 200 1000 500]);
compare(val_data_ss, sys_est);
title('Validation Fit — Stair Step');
grid on;

% --- Plot 4: Residual analysis on training data ---
figure('Name', 'Residual Analysis', 'Position', [250 250 1000 500]);
resid(train_data, sys_est);
title('Residual Analysis — Training Data');

% ============================================================
% SECTION 9 — Summary Table
% ============================================================
fprintf('\n========================================\n');
fprintf('   SYSTEM IDENTIFICATION RESULTS\n');
fprintf('========================================\n');
fprintf('Fixed Parameters:\n');
fprintf('  Rm  = %.4f Ohm\n', R_m);
fprintf('  Lm  = %.4f mH\n',  L_m * 1e3);
fprintf('  N   = %d\n',       N);
fprintf('\nEstimated Parameters:\n');
fprintf('  %-6s = %10.6f +/- %.6f\n', 'J',   J_est,   J_std);
fprintf('  %-6s = %10.6f +/- %.6f\n', 'B',   B_est,   B_std);
fprintf('  %-6s = %10.6f +/- %.6f  N.m/A (eta*Kt)\n', 'KtEff', KtEff_est, KtEff_std);
fprintf('  %-6s = %10.6f +/- %.6f  V.s/rad\n', 'Ke',    Ke_est,    Ke_std);
fprintf('\nFit Scores:\n');
fprintf('  Training  (Step 24V):       %.2f%%\n', mean(fit_train));
fprintf('  Validation (Chirp 0.1-2Hz): %.2f%%\n', mean(fit_step));
fprintf('  Validation (Stair Step):    %.2f%%\n', mean(fit_ss));
fprintf('========================================\n');

% Save results
save(fullfile(base_path, 'sysid_results.mat'), ...
    'J_est', 'B_est', 'KtEff_est', 'Ke_est', ...
    'J_std', 'B_std', 'KtEff_std', 'Ke_std', ...
    'sys_est', 'fit_train', 'fit_step', 'fit_ss');

fprintf('\nResults saved to sysid_results.mat\n');