% =========================================================================
% Script Name: cross_validation_all.m
% Description: This script performs a cross-validation of a DC motor 
%              parameter estimation model. It loads 21 datasets (7 types of 
%              input signals, 3 runs each), simulates the system using 
%              estimated parameters, and calculates the fit percentage 
%              between measured and simulated angular velocities.
% Author:      Thadchai Suksaran (Thadzy)
% Date:        May 2026
% =========================================================================

model_name = 'Params_Estimate';
model_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';
raw_path   = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';

% --- Parameters for Simulink Workspace ---
R_m     = 1.4534;    % Armature Resistance (Ohm)
L_m     = 0.001448;  % Armature Inductance (H)
K_e     = 0.04165;   % Back-EMF Constant (V.s/rad)
K_t     = 0.04065;   % Torque Constant (N.m/A)
B       = 0.19279;   % Viscous Damping (N.m.s/rad)
J       = 0.72762;   % Moment of Inertia (kg.m^2)
eta     = 0.83607;   % Efficiency
N_total = 70;        % Total Gear Ratio
% =========================================================

fprintf('[cross_validation_all] Generating 21-subplot figure ...\n');

% --- Low-pass Filter Configuration ---
fs = 1000;      % Sampling frequency (Hz)
fc = 10;        % Cutoff frequency (Hz)
N_filt = 4;     % Filter order
[b_f, a_f] = butter(N_filt, fc/(fs/2), 'low');

% --- Load Simulink Model ---
if ~bdIsLoaded(model_name)
    load_system(model_path);
end

% --- Dataset Definition ---
% Format: {row_label, file_run1, file_run2, file_run3, voltage_label}
datasets = {
    'Chirp 0.1-0.5 Hz', 'chirp_01_05_run1.mat', 'chirp_01_05_run2.mat', 'chirp_01_05_run3.mat', '';
    'Chirp 0.1-1.0 Hz', 'chirp_01_1_run1.mat',  'chirp_01_1_run2.mat',  'chirp_01_1_run3.mat',  '';
    'Chirp 0.1-2.0 Hz', 'chirp_01_2_run1.mat',  'chirp_01_2_run2.mat',  'chirp_01_2_run3.mat',  '';
    'Step 6V',          'step_6_run1.mat',       'step_6_run2.mat',      'step_6_run3.mat',       '6V';
    'Step 12V',         'step_12_run1.mat',      'step_12_run2.mat',     'step_12_run3.mat',      '12V';
    'Step 24V',         'step_24_run1.mat',      'step_24_run2.mat',     'step_24_run3.mat',      '24V';
    'Stair-Step',       'ss_run1.mat',           'ss_run2.mat',          'ss_run3.mat',           '';
};

n_rows = size(datasets, 1);  % 7 signal types
n_cols = 3;                  % 3 runs per signal

% --- Figure Initialization ---
fig_all = figure( ...
    'Units',    'centimeters', ...
    'Position', [1 1 36 42], ...
    'Color',    'white');

% Base Theme Settings (Removed DefaultLegendTextColor to allow dynamic styling)
set(fig_all, 'DefaultAxesColor',          'white');
set(fig_all, 'DefaultAxesXColor',         'black');
set(fig_all, 'DefaultAxesYColor',         'black');
set(fig_all, 'DefaultAxesGridColor',      [0.85 0.85 0.85]);
set(fig_all, 'DefaultTextColor',          'black');

% Line Colors for each dataset row
row_colors = { ...
    [0.00 0.45 0.74], ...  % Chirp 0.5 Hz - blue
    [0.47 0.67 0.19], ...  % Chirp 1.0 Hz - green
    [0.85 0.33 0.10], ...  % Chirp 2.0 Hz - orange
    [0.49 0.18 0.56], ...  % Step 6V      - purple
    [0.93 0.69 0.13], ...  % Step 12V     - yellow
    [0.64 0.08 0.18], ...  % Step 24V     - dark red
    [0.30 0.75 0.93], ...  % Stair-Step   - cyan
};

fit_scores = zeros(n_rows, n_cols);

% --- Main Processing Loop ---
for row = 1:n_rows
    row_label = datasets{row, 1};
    c = row_colors{row}; % Extract specific color for this row
    
    for col = 1:n_cols
        fname = datasets{row, col + 1};
        fpath = fullfile(raw_path, fname);
        
        % --- Data Loading & Filtering ---
        ld    = load(fpath);
        d     = ld.data;
        t_v   = double(d{3}.Values.Time(:));
        Vin_v = filtfilt(b_f, a_f, double(d{3}.Values.Data(:)));
        om_v  = filtfilt(b_f, a_f, double(d{1}.Values.Data(:)));
        
        % --- Simulation Execution ---
        Vin_ws   = [t_v, Vin_v];
        omega_ws = om_v;
        t_in     = t_v;
        
        set_param(model_name, 'StopTime', num2str(t_v(end)));
        out_sim   = sim(model_name);
        
        om_sim    = double(out_sim.yout{1}.Values.Data(:));
        t_sim     = double(out_sim.yout{1}.Values.Time(:));
        om_interp = interp1(t_sim, om_sim, t_v, 'linear', 'extrap');
        
        % --- Fit Score Calculation (Normalized Root Mean Square Error) ---
        ss_res = sum((om_v - om_interp).^2);
        ss_tot = sum((om_v - mean(om_v)).^2);
        fit    = (1 - ss_res / ss_tot) * 100;
        fit_scores(row, col) = fit;
        
        % --- Subplot Generation ---
        subplot_idx = (row - 1) * n_cols + col;
        ax = subplot(n_rows, n_cols, subplot_idx);
        
        plot(t_v, om_v, 'Color', [0.6 0.6 0.6], ...
             'LineWidth', 1.0, 'DisplayName', 'Measured');
        hold on;
        plot(t_v, om_interp, '--', 'Color', c, ...
             'LineWidth', 1.4, 'DisplayName', 'Simulated');
             
        xlabel('Time (s)', 'FontSize', 7);
        ylabel('\omega_{arm} (rad/s)', 'FontSize', 7);
        title(sprintf('%s  Run %d   Fit = %.1f%%', row_label, col, fit), ...
              'FontSize', 7, 'FontWeight', 'bold');
              
        % --- Legend Styling Adjustments ---
        lgd = legend('Location', 'best', 'FontSize', 6);
        lgd.TextColor = c;       % Match legend text color to the simulated line color
        lgd.Box = 'off';         % Remove the legend border for a cleaner appearance
        
        grid on;
        ax.FontSize = 7;
    end
    fprintf('  Row %d (%s): Fit = %.1f, %.1f, %.1f %%\n', ...
        row, row_label, fit_scores(row,1), fit_scores(row,2), fit_scores(row,3));
end

% --- Overall Title Generation ---
sgtitle('Cross-Validation: All 21 Runs', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', 'black');

% --- Export to PNG ---
exportgraphics(fig_all, 'cross_validation_all.png', ...
    'Resolution', 200, ...
    'BackgroundColor', 'white');
fprintf('Done -> cross_validation_all.png\n');

% --- Summary Table Output to Console ---
fprintf('\n=== Fit Score Summary ===\n');
fprintf('%-20s  Run1    Run2    Run3    Mean\n', 'Dataset');
fprintf('%s\n', repmat('-', 1, 55));
row_labels = datasets(:, 1);
for row = 1:n_rows
    m = mean(fit_scores(row, :));
    fprintf('%-20s  %5.1f%%  %5.1f%%  %5.1f%%  %5.1f%%\n', ...
        row_labels{row}, ...
        fit_scores(row,1), fit_scores(row,2), fit_scores(row,3), m);
end