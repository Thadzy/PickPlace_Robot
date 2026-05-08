% =========================================================
% plot_cross_validation.m
% Plot ผลการ cross-validate ด้วย best parameters
% กับทุก data ที่เก็บได้ (21 files)
% แยกเป็น 3 figures: Chirp / Step / Stair-Step
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Best Parameters (Mean chirp1)
% =========================================================
J   = 0.72762;
B   = 0.19279;
K_e = 0.04165;
K_t = 0.04065;
eta = 0.83607;

% Fixed parameters
R_m     = 1.45336105;
L_m     = 0.00144802;
N_total = 70;

% =========================================================
% SECTION 2: Paths และ Filter Design
% =========================================================
chirp_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Preprocess_Data/';
raw_path   = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';
model_name = 'Params_Estimate';
model_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';

% Filter design (เหมือน preprocess)
fs = 1000;
fc = 10;
N  = 4;
Wn = fc / (fs/2);
[b_f, a_f] = butter(N, Wn, 'low');

% โหลด model
if ~bdIsLoaded(model_name)
    load_system(model_path);
end

% =========================================================
% SECTION 3: File Lists
% =========================================================

% --- Chirp: โหลดจาก Preprocess_Data (filtered แล้ว) ---
chirp_files = {
    'chirp05_run1.mat', 'chirp05_run2.mat', 'chirp05_run3.mat';
    'chirp1_run1.mat',  'chirp1_run2.mat',  'chirp1_run3.mat';
    'chirp2_run1.mat',  'chirp2_run2.mat',  'chirp2_run3.mat';
};
chirp_group_labels = {'Chirp 0.1-0.5 Hz', 'Chirp 0.1-1.0 Hz', 'Chirp 0.1-2.0 Hz'};

% --- Step: โหลดจาก raw แล้ว filter ---
step_files = {
    'step_6_run1.mat',  'step_6_run2.mat',  'step_6_run3.mat';
    'step_12_run1.mat', 'step_12_run2.mat', 'step_12_run3.mat';
    'step_24_run1.mat', 'step_24_run2.mat', 'step_24_run3.mat';
};
step_group_labels = {'Step 6V', 'Step 12V', 'Step 24V'};

% --- Stair-Step: โหลดจาก raw แล้ว filter ---
ss_files = {'ss_run1.mat', 'ss_run2.mat', 'ss_run3.mat'};

% =========================================================
% SECTION 4: Helper Function สำหรับ load และ simulate
% =========================================================

    function [t_meas, omega_meas, omega_sim_interp, fit_score, nrmse] = ...
            run_simulation(filepath, is_preprocessed, b_f, a_f, model_name)

        % Load data
        loaded = load(filepath);

        if is_preprocessed
            % Chirp: ตัวแปรชื่อ t, Vin, omega_f
            t_raw     = loaded.t(:);
            Vin_raw   = loaded.Vin(:);
            omega_raw = loaded.omega_f(:);
            % ข้อมูลนี้ filter แล้ว ไม่ต้อง filter ซ้ำ
            t_meas     = t_raw;
            Vin_filt   = Vin_raw;
            omega_meas = omega_raw;
        else
            % Step/SS: ตัวแปรชื่อ data (Simulink Dataset)
            d         = loaded.data;
            t_raw     = double(d{3}.Values.Time(:));   % index 3 = Vin
            Vin_raw   = double(d{3}.Values.Data(:));
            omega_raw = double(d{1}.Values.Data(:));   % index 1 = omega
            % Filter
            t_meas     = t_raw;
            Vin_filt   = filtfilt(b_f, a_f, Vin_raw);
            omega_meas = filtfilt(b_f, a_f, omega_raw);
        end

        % ตั้งค่า workspace สำหรับ Simulink
        assignin('base', 'Vin_ws',   [t_meas, Vin_filt]);
        assignin('base', 'omega_ws', omega_meas);
        assignin('base', 't_in',     t_meas);
        set_param(model_name, 'StopTime', num2str(t_meas(end)));

        % Simulate
        out          = sim(model_name);
        omega_sim    = double(out.yout{1}.Values.Data(:));
        t_sim        = double(out.yout{1}.Values.Time(:));
        omega_sim_interp = interp1(t_sim, omega_sim, t_meas, 'linear', 'extrap');

        % Fit Score
        ss_res     = sum((omega_meas - omega_sim_interp).^2);
        ss_tot     = sum((omega_meas - mean(omega_meas)).^2);
        fit_score  = (1 - ss_res/ss_tot) * 100;

        % NRMSE
        rmse  = sqrt(mean((omega_meas - omega_sim_interp).^2));
        range = max(omega_meas) - min(omega_meas);
        nrmse = (rmse / range) * 100;
    end

% =========================================================
% SECTION 5: FIGURE 1 — Chirp (9 subplots: 3 groups x 3 runs)
% =========================================================
fprintf('Processing Chirp data...\n');
fig1 = figure('Name', 'Cross-Validation: Chirp Signals', ...
              'NumberTitle', 'off', ...
              'Position', [50, 50, 1400, 900]);

for g = 1:3        % group: chirp05, chirp1, chirp2
    for r = 1:3    % run: 1, 2, 3
        subplot_idx = (g-1)*3 + r;
        subplot(3, 3, subplot_idx);

        fpath = fullfile(chirp_path, chirp_files{g, r});
        [t_m, om_m, om_s, fit, nrmse] = run_simulation(...
            fpath, true, b_f, a_f, model_name);

        plot(t_m, om_m, 'b', 'LineWidth', 1.0, 'DisplayName', 'Measured');
        hold on;
        plot(t_m, om_s, 'r--', 'LineWidth', 1.0, 'DisplayName', 'Simulated');

        xlabel('Time (s)');
        ylabel('\omega (rad/s)');
        title(sprintf('%s — Run %d\nFit = %.1f%%  |  NRMSE = %.1f%%', ...
            chirp_group_labels{g}, r, fit, nrmse), 'FontSize', 9);

        if fit >= 80
            title_color = [0.0, 0.5, 0.0];   % เขียว = ผ่าน
        else
            title_color = [0.8, 0.0, 0.0];   % แดง = ไม่ผ่าน
        end
        set(get(gca, 'Title'), 'Color', title_color);

        if subplot_idx == 1
            legend('Location', 'best', 'FontSize', 7);
        end
        grid on;

        fprintf('  Chirp %s Run%d: Fit=%.1f%%  NRMSE=%.1f%%\n', ...
            chirp_group_labels{g}, r, fit, nrmse);
    end
end

sgtitle('Cross-Validation: Chirp Signals (Best Parameters = Mean chirp1)', ...
    'FontSize', 13, 'FontWeight', 'bold');

% =========================================================
% SECTION 6: FIGURE 2 — Step (9 subplots: 3 groups x 3 runs)
% =========================================================
fprintf('\nProcessing Step data...\n');
fig2 = figure('Name', 'Cross-Validation: Step Signals', ...
              'NumberTitle', 'off', ...
              'Position', [100, 50, 1400, 900]);

for g = 1:3        % group: 6V, 12V, 24V
    for r = 1:3    % run: 1, 2, 3
        subplot_idx = (g-1)*3 + r;
        subplot(3, 3, subplot_idx);

        fpath = fullfile(raw_path, step_files{g, r});
        [t_m, om_m, om_s, fit, nrmse] = run_simulation(...
            fpath, false, b_f, a_f, model_name);

        plot(t_m, om_m, 'b', 'LineWidth', 1.2, 'DisplayName', 'Measured');
        hold on;
        plot(t_m, om_s, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Simulated');

        xlabel('Time (s)');
        ylabel('\omega (rad/s)');
        title(sprintf('%s — Run %d\nFit = %.1f%%  |  NRMSE = %.1f%%', ...
            step_group_labels{g}, r, fit, nrmse), 'FontSize', 9);

        if fit >= 80
            title_color = [0.0, 0.5, 0.0];
        else
            title_color = [0.8, 0.0, 0.0];
        end
        set(get(gca, 'Title'), 'Color', title_color);

        if subplot_idx == 1
            legend('Location', 'best', 'FontSize', 7);
        end
        grid on;

        fprintf('  Step %s Run%d: Fit=%.1f%%  NRMSE=%.1f%%\n', ...
            step_group_labels{g}, r, fit, nrmse);
    end
end

sgtitle('Cross-Validation: Step Signals (Best Parameters = Mean chirp1)', ...
    'FontSize', 13, 'FontWeight', 'bold');

% =========================================================
% SECTION 7: FIGURE 3 — Stair-Step (3 subplots: 1 row x 3 runs)
% =========================================================
fprintf('\nProcessing Stair-Step data...\n');
fig3 = figure('Name', 'Cross-Validation: Stair-Step Signals', ...
              'NumberTitle', 'off', ...
              'Position', [150, 50, 1400, 400]);

for r = 1:3
    subplot(1, 3, r);

    fpath = fullfile(raw_path, ss_files{r});
    [t_m, om_m, om_s, fit, nrmse] = run_simulation(...
        fpath, false, b_f, a_f, model_name);

    plot(t_m, om_m, 'b', 'LineWidth', 1.2, 'DisplayName', 'Measured');
    hold on;
    plot(t_m, om_s, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Simulated');

    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    title(sprintf('Stair-Step — Run %d\nFit = %.1f%%  |  NRMSE = %.1f%%', ...
        r, fit, nrmse), 'FontSize', 9);

    if fit >= 80
        title_color = [0.0, 0.5, 0.0];
    else
        title_color = [0.8, 0.0, 0.0];
    end
    set(get(gca, 'Title'), 'Color', title_color);

    if r == 1
        legend('Location', 'best', 'FontSize', 8);
    end
    grid on;

    fprintf('  StairStep Run%d: Fit=%.1f%%  NRMSE=%.1f%%\n', r, fit, nrmse);
end

sgtitle('Cross-Validation: Stair-Step Signals (Best Parameters = Mean chirp1)', ...
    'FontSize', 13, 'FontWeight', 'bold');

fprintf('\nDone. 3 figures generated.\n');