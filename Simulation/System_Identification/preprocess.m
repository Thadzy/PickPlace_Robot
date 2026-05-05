% =============================================
% Preprocess Script — G6 System ID Data
% =============================================

base_path    = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data/';
out_path     = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Preprocess_Data/';

if ~exist(out_path, 'dir')
    mkdir(out_path);
end

dt = 0.001;
fs = 1 / dt;

% --- Design Filters ---
[b_ss,  a_ss]  = butter(4, 17.5 / (fs/2), 'low');   % Stair Step
[b_c05, a_c05] = butter(4, 8.7  / (fs/2), 'low');   % Chirp 0.5 Hz
[b_c1,  a_c1]  = butter(4, 18.5 / (fs/2), 'low');   % Chirp 1 Hz

% --- Helper: load one .mat and extract signals ---
function [t, omega_f, Vin] = extract_filter(base_path, filename, b, a, t_start, t_end)
    tmp   = load(fullfile(base_path, filename));
    d     = tmp.data;
    t_raw = d.getElement(1).Values.Time;
    omega = double(squeeze(d.getElement(1).Values.Data));
    Vin   = double(squeeze(d.getElement(3).Values.Data));

    % Trim time window
    idx     = t_raw >= t_start & t_raw <= t_end;
    t       = t_raw(idx);
    omega   = omega(idx);
    Vin     = Vin(idx);

    % Zero-phase filter
    omega_f = filtfilt(b, a, omega);
end

% --- Define datasets ---
datasets = {
    % filename                      b      a      t_start  t_end   out_name
    'Ststep_run1.mat',              b_ss,  a_ss,  0, 60,   'ss_run1';
    'Ststep_run2.mat',              b_ss,  a_ss,  0, 60,   'ss_run2';
    'Ststep_run3.mat',              b_ss,  a_ss,  0, 60,   'ss_run3';
    'Chirp_0.1_30_0.5_run1.mat',   b_c05, a_c05, 0, 60,   'chirp05_run1';
    'Chirp_0.1_30_0.5_run2.mat',   b_c05, a_c05, 0, 60,   'chirp05_run2';
    'Chirp_0.1_30_0.5_run3.mat',   b_c05, a_c05, 0, 60,   'chirp05_run3';
    'Chirp_0.1_30_1_run1.mat',     b_c1,  a_c1,  0, 10,   'chirp1_run1';
    'Chirp_0.1_30_1_run2.mat',     b_c1,  a_c1,  0, 10,   'chirp1_run2';
    'Chirp_0.1_30_1_run3.mat',     b_c1,  a_c1,  0, 10,   'chirp1_run3';
};

% --- Process and Save ---
fprintf('\n%-20s  %-10s  %-10s\n', 'Dataset', 'Samples', 'Duration(s)');
fprintf('%s\n', repmat('-', 1, 45));

results = struct();

for i = 1:size(datasets, 1)
    fname    = datasets{i, 1};
    b        = datasets{i, 2};
    a        = datasets{i, 3};
    t_start  = datasets{i, 4};
    t_end    = datasets{i, 5};
    out_name = datasets{i, 6};

    [t, omega_f, Vin] = extract_filter(base_path, fname, b, a, t_start, t_end);

    % Store in results struct
    results.(out_name).t       = t;
    results.(out_name).omega_f = omega_f;
    results.(out_name).Vin     = Vin;

    % Save individual file
    save_path = fullfile(out_path, [out_name '.mat']);
    save(save_path, 't', 'omega_f', 'Vin');

    fprintf('%-20s  %-10d  %-10.1f\n', out_name, length(t), t(end)-t(1));
end

% Save all in one file
save(fullfile(out_path, 'all_preprocessed.mat'), 'results');
fprintf('\nSaved all to: %s\n', out_path);

% =============================================
% Plot ตรวจสอบ 1 run ต่อ group
% =============================================
check_keys = {'ss_run1', 'chirp05_run1', 'chirp1_run1'};
check_names = {'Stair Step run1', 'Chirp 0.5Hz run1', 'Chirp 1Hz run1 (0-10s)'};

figure('Name', 'Filtered Data Check', 'NumberTitle', 'off', 'Position', [100 100 1000 700]);
sgtitle('Filtered Data Check', 'FontSize', 13, 'FontWeight', 'bold');

for i = 1:length(check_keys)
    key  = check_keys{i};
    t_   = results.(key).t;
    v_   = results.(key).Vin;
    w_   = results.(key).omega_f;

    subplot(3, 2, 2*i - 1);
    plot(t_, v_, 'b', 'LineWidth', 1.0);
    xlabel('Time (s)'); ylabel('V_{in} (V)');
    title(['Input — ' check_names{i}]);
    xlim([t_(1) t_(end)]); grid on;

    subplot(3, 2, 2*i);
    plot(t_, w_, 'r', 'LineWidth', 1.0);
    xlabel('Time (s)'); ylabel('\omega (rad/s)');
    title(['\omega filtered — ' check_names{i}]);
    xlim([t_(1) t_(end)]); grid on;
end

fprintf('Preprocess complete.\n');