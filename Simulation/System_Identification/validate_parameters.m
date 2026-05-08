% =========================================================
% validate_parameters.m
% Cross-validation ของ Parameter Estimation Results
% G6 Circular Pick and Place Robot
%
% Strategy:
%   1. In-group consistency: CV ของ parameter แต่ละกลุ่ม
%   2. Cross-signal validation: simulate กับ step/ss data
%   3. Fit Score และ NRMSE ทุก combination
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Fixed Parameters
% =========================================================
R_m     = 1.45336105;
L_m     = 0.00144802;
N_total = 70;

% =========================================================
% SECTION 2: Estimation Results จาก 9 runs
% =========================================================
% rows = [trial1, trial2, trial3]
% format: [J, B, K_e, K_t, eta]

results_chirp05 = [
    1.62100, 0.21393, 0.040909, 0.050415, 0.81220;
    1.01190, 0.20596, 0.044212, 0.039110, 0.82541;
    1.34450, 0.17306, 0.040947, 0.041988, 0.80881;
];

results_chirp1 = [
    0.74803, 0.20065, 0.041977, 0.039459, 0.83518;
    0.81083, 0.18381, 0.040288, 0.041642, 0.83091;
    0.62400, 0.19392, 0.042691, 0.040839, 0.84213;
];

results_chirp2 = [
    0.91785, 0.15994, 0.037031, 0.045497, 0.82573;
    0.57484, 0.18838, 0.042025, 0.040779, 0.84578;
    0.63090, 0.19365, 0.042539, 0.040916, 0.84066;
];

param_names = {'J', 'B', 'K_e', 'K_t', 'eta'};
param_units = {'kg.m^2', 'N.m.s/rad', 'V.s/rad', 'N.m/A', '-'};
group_names = {'chirp05', 'chirp1', 'chirp2'};
all_results = {results_chirp05, results_chirp1, results_chirp2};

% =========================================================
% SECTION 3: In-group Consistency Analysis
% =========================================================
fprintf('============================================================\n');
fprintf('SECTION 3: In-group Consistency (CV Analysis)\n');
fprintf('============================================================\n');
fprintf('CV = std/mean x 100%% -- เกณฑ์: CV < 10%% = สม่ำเสมอดี\n\n');

cv_table = zeros(3, 5);   % 3 groups x 5 parameters

for g = 1:3
    R = all_results{g};
    fprintf('--- %s ---\n', group_names{g});
    for p = 1:5
        mu  = mean(R(:, p));
        sig = std(R(:, p));
        cv  = (sig / mu) * 100;
        cv_table(g, p) = cv;
        flag = '';
        if cv > 10
            flag = '  <-- HIGH VARIANCE';
        end
        fprintf('  %-5s: mean=%.5f  std=%.5f  CV=%.1f%%%s\n', ...
            param_names{p}, mu, sig, cv, flag);
    end
    fprintf('\n');
end

% =========================================================
% SECTION 4: คำนวณค่าเฉลี่ยแต่ละกลุ่ม + รวมทุก run
% =========================================================

mean_chirp05 = mean(results_chirp05, 1);
mean_chirp1  = mean(results_chirp1,  1);
mean_chirp2  = mean(results_chirp2,  1);

% ตัด chirp05 ออกเพราะมี saturation -- ใช้ chirp1 + chirp2 เท่านั้น
all_valid = [results_chirp1; results_chirp2];
mean_all  = mean(all_valid, 1);

fprintf('============================================================\n');
fprintf('SECTION 4: Mean Parameters per Group\n');
fprintf('============================================================\n');
fprintf('%-10s  %-8s  %-8s  %-8s  %-8s  %-8s\n', ...
    'Group', 'J', 'B', 'K_e', 'K_t', 'eta');
fprintf('%-10s  %-8.5f  %-8.5f  %-8.5f  %-8.5f  %-8.5f\n', ...
    'chirp05', mean_chirp05);
fprintf('%-10s  %-8.5f  %-8.5f  %-8.5f  %-8.5f  %-8.5f\n', ...
    'chirp1',  mean_chirp1);
fprintf('%-10s  %-8.5f  %-8.5f  %-8.5f  %-8.5f  %-8.5f\n', ...
    'chirp2',  mean_chirp2);
fprintf('%-10s  %-8.5f  %-8.5f  %-8.5f  %-8.5f  %-8.5f\n', ...
    'ALL(1+2)', mean_all);
fprintf('\n');

% =========================================================
% SECTION 5: Cross-Signal Validation กับ Step/SS data
% =========================================================
fprintf('============================================================\n');
fprintf('SECTION 5: Cross-Signal Validation\n');
fprintf('============================================================\n');

% Path ของ raw step/ss data
raw_path   = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';
model_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';
model_name = 'Params_Estimate';

% Filter design (เหมือนกับ preprocess)
fs = 1000;
fc = 10;
N  = 4;
Wn = fc / (fs/2);
[b_f, a_f] = butter(N, Wn, 'low');

% รายชื่อ validation files
val_files = {
    'step_6_run1.mat',  'Step 6V  Run1';
    'step_6_run2.mat',  'Step 6V  Run2';
    'step_6_run3.mat',  'Step 6V  Run3';
    'step_12_run1.mat', 'Step 12V Run1';
    'step_12_run2.mat', 'Step 12V Run2';
    'step_12_run3.mat', 'Step 12V Run3';
    'step_24_run1.mat', 'Step 24V Run1';
    'step_24_run2.mat', 'Step 24V Run2';
    'step_24_run3.mat', 'Step 24V Run3';
    'ss_run1.mat',      'StairStep Run1';
    'ss_run2.mat',      'StairStep Run2';
    'ss_run3.mat',      'StairStep Run3';
};

% ชุด parameter ที่จะ validate
param_sets = {
    mean_chirp1,   'Mean chirp1';
    mean_chirp2,   'Mean chirp2';
    mean_all,      'Mean chirp1+2';
};

% โหลด model
if ~bdIsLoaded(model_name)
    load_system(model_path);
end

% เก็บผล
nV = size(val_files, 1);
nP = size(param_sets, 1);
fit_matrix  = zeros(nP, nV);
nrmse_matrix = zeros(nP, nV);

for v = 1:nV
    fname = val_files{v, 1};
    fpath = fullfile(raw_path, fname);

    % Load และ filter raw data
    loaded    = load(fpath);
    d         = loaded.data;
    t_v       = double(d{3}.Values.Time(:));
    Vin_v     = double(d{3}.Values.Data(:));
    omega_v   = double(d{1}.Values.Data(:));
    Vin_filt  = filtfilt(b_f, a_f, Vin_v);
    omega_filt = filtfilt(b_f, a_f, omega_v);

    % ส่ง Vin เข้า workspace สำหรับ Simulink
    Vin_ws   = [t_v, Vin_filt];
    omega_ws = omega_filt;
    t_in     = t_v;
    set_param(model_name, 'StopTime', num2str(t_v(end)));

    for p = 1:nP
        params = param_sets{p, 1};
        % ตั้งค่า parameter ใน workspace
        J   = params(1);
        B   = params(2);
        K_e = params(3);
        K_t = params(4);
        eta = params(5);

        % Simulate
        try
            out = sim(model_name);
            omega_sim    = double(out.yout{1}.Values.Data(:));
            t_sim        = double(out.yout{1}.Values.Time(:));
            omega_interp = interp1(t_sim, omega_sim, t_v, 'linear', 'extrap');

            % Fit Score (R²-based)
            ss_res   = sum((omega_filt - omega_interp).^2);
            ss_tot   = sum((omega_filt - mean(omega_filt)).^2);
            fit_score = (1 - ss_res/ss_tot) * 100;

            % NRMSE
            rmse  = sqrt(mean((omega_filt - omega_interp).^2));
            range = max(omega_filt) - min(omega_filt);
            nrmse = (rmse / range) * 100;

            fit_matrix(p, v)   = fit_score;
            nrmse_matrix(p, v) = nrmse;

        catch ME
            fprintf('  ERROR: %s | %s | %s\n', ...
                param_sets{p,2}, fname, ME.message);
            fit_matrix(p, v)   = NaN;
            nrmse_matrix(p, v) = NaN;
        end
    end
end

% =========================================================
% SECTION 6: แสดงผล Cross-Validation
% =========================================================
fprintf('\n--- Fit Score (%%) [เกณฑ์ > 80%%] ---\n');
fprintf('%-16s', 'ParamSet\File');
for v = 1:nV
    fprintf('  %-14s', val_files{v,2});
end
fprintf('\n');
for p = 1:nP
    fprintf('%-16s', param_sets{p,2});
    for v = 1:nV
        val = fit_matrix(p,v);
        if isnan(val)
            fprintf('  %-14s', 'ERROR');
        elseif val >= 80
            fprintf('  %-14s', sprintf('%.1f OK', val));
        else
            fprintf('  %-14s', sprintf('%.1f LOW', val));
        end
    end
    fprintf('\n');
end

fprintf('\n--- NRMSE (%%) [เกณฑ์ < 15%%] ---\n');
fprintf('%-16s', 'ParamSet\File');
for v = 1:nV
    fprintf('  %-14s', val_files{v,2});
end
fprintf('\n');
for p = 1:nP
    fprintf('%-16s', param_sets{p,2});
    for v = 1:nV
        val = nrmse_matrix(p,v);
        if isnan(val)
            fprintf('  %-14s', 'ERROR');
        elseif val <= 15
            fprintf('  %-14s', sprintf('%.1f OK', val));
        else
            fprintf('  %-14s', sprintf('%.1f HIGH', val));
        end
    end
    fprintf('\n');
end

% =========================================================
% SECTION 7: สรุปและเลือก Best Parameter Set
% =========================================================
fprintf('\n============================================================\n');
fprintf('SECTION 7: Summary - Best Parameter Set\n');
fprintf('============================================================\n');

mean_fit   = mean(fit_matrix,  2, 'omitnan');
mean_nrmse = mean(nrmse_matrix, 2, 'omitnan');

for p = 1:nP
    fprintf('%s: Mean Fit = %.1f%%  |  Mean NRMSE = %.1f%%\n', ...
        param_sets{p,2}, mean_fit(p), mean_nrmse(p));
end

[~, best_idx] = max(mean_fit);
fprintf('\n==> Best parameter set: %s\n', param_sets{best_idx, 2});
best_params = param_sets{best_idx, 1};
fprintf('    J   = %.5f kg.m^2\n',    best_params(1));
fprintf('    B   = %.5f N.m.s/rad\n', best_params(2));
fprintf('    K_e = %.5f V.s/rad\n',   best_params(3));
fprintf('    K_t = %.5f N.m/A\n',     best_params(4));
fprintf('    eta = %.5f\n',           best_params(5));

% =========================================================
% SECTION 8: Plot Consistency - J across all runs
% =========================================================
figure('Name', 'Parameter Consistency across runs', 'NumberTitle', 'off');

all_J = [results_chirp05(:,1); results_chirp1(:,1); results_chirp2(:,1)];
x_labels = {'c05-T1','c05-T2','c05-T3','c1-T1','c1-T2','c1-T3','c2-T1','c2-T2','c2-T3'};
colors = [repmat([0.8 0.2 0.2], 3, 1);
          repmat([0.2 0.6 0.2], 3, 1);
          repmat([0.2 0.2 0.8], 3, 1)];

bar_h = bar(all_J, 'FaceColor', 'flat');
bar_h.CData = colors;
hold on;
yline(mean_chirp1(1),  'g--', 'LineWidth', 1.5, 'DisplayName', 'Mean chirp1');
yline(mean_chirp2(1),  'b--', 'LineWidth', 1.5, 'DisplayName', 'Mean chirp2');
yline(mean_all(1),     'k-',  'LineWidth', 2.0, 'DisplayName', 'Mean chirp1+2');
set(gca, 'XTickLabel', x_labels);
ylabel('J (kg.m^2)');
title('J Estimate: Consistency across all runs');
legend('Location', 'best');
grid on;

% =========================================================
% SECTION 9: Plot Cross-Validation Fit Score
% =========================================================
figure('Name', 'Cross-Validation Fit Score', 'NumberTitle', 'off');
bar(fit_matrix');
set(gca, 'XTickLabel', val_files(:,2), 'XTickLabelRotation', 45);
ylabel('Fit Score (%)');
title('Cross-Validation: Fit Score per Parameter Set');
legend(param_sets(:,2), 'Location', 'best');
yline(80, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Threshold 80%');
grid on;

fprintf('\nDone.\n');