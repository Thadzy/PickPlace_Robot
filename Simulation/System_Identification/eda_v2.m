% =============================================
% EDA Script — System Identification Data V2
% G6 Circular Pick and Place Robot
% =============================================

base_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';

% --- Define all file groups ---
file_groups = {
    'Chirp 0.1-1 Hz',   {'chirp_01_1_run1.mat',  'chirp_01_1_run2.mat',  'chirp_01_1_run3.mat'};
    'Chirp 0.1-2 Hz',   {'chirp_01_2_run1.mat',  'chirp_01_2_run2.mat',  'chirp_01_2_run3.mat'};
    'Chirp 0.1-0.5 Hz', {'chirp_01_05_run1.mat', 'chirp_01_05_run2.mat', 'chirp_01_05_run3.mat'};
    'Stair Step',        {'ss_run1.mat',           'ss_run2.mat',          'ss_run3.mat'};
    'Step 6V',           {'step_6_run1.mat',       'step_6_run2.mat',      'step_6_run3.mat'};
    'Step 12V',          {'step_12_run1.mat',      'step_12_run2.mat',     'step_12_run3.mat'};
    'Step 24V',          {'step_24_run1.mat',      'step_24_run2.mat',     'step_24_run3.mat'};
};

% =============================================
% SECTION 0 — Helper Function
% =============================================
function [t, omega, Vin] = load_file(base_path, filename)
    tmp   = load(fullfile(base_path, filename));
    d     = tmp.data;
    omega = double(squeeze(d{1}.Values.Data));
    Vin   = double(squeeze(d{3}.Values.Data));
    t     = d{1}.Values.Time;
end

% =============================================
% SECTION 1 — Summary Table
% =============================================
fprintf('\n%-22s  %-4s  %-12s  %-10s  %-10s  %-12s  %-12s\n', ...
    'Group', 'Run', 'Duration(s)', 'Vin_min', 'Vin_max', 'omega_min', 'omega_max');
fprintf('%s\n', repmat('-', 1, 88));

for g = 1:size(file_groups, 1)
    gname = file_groups{g, 1};
    files = file_groups{g, 2};
    for r = 1:length(files)
        try
            [t, omega, Vin] = load_file(base_path, files{r});
            fprintf('%-22s  %-4d  %-12.2f  %-10.3f  %-10.3f  %-12.3f  %-12.3f\n', ...
                gname, r, t(end)-t(1), min(Vin), max(Vin), min(omega), max(omega));
        catch e
            fprintf('%-22s  %-4d  LOAD ERROR: %s\n', gname, r, e.message);
        end
    end
end

% =============================================
% SECTION 2 — Plot แต่ละ Group (3 runs ต่อ figure)
% =============================================
colors = {'b', 'r', [0.1 0.6 0.1]};  % run1=blue, run2=red, run3=green

for g = 1:size(file_groups, 1)
    gname = file_groups{g, 1};
    files = file_groups{g, 2};

    figure('Name', gname, 'NumberTitle', 'off', 'Position', [100, 100, 1100, 750]);
    sgtitle(gname, 'FontSize', 14, 'FontWeight', 'bold');

    for r = 1:length(files)
        try
            [t, omega, Vin] = load_file(base_path, files{r});

            % --- Vin plot ---
            subplot(3, 2, 2*r - 1);
            plot(t, Vin, 'Color', colors{r}, 'LineWidth', 1.2);
            xlabel('Time (s)');
            ylabel('V_{in} (V)');
            title(sprintf('Run %d — Input Voltage', r));
            xlim([t(1) t(end)]);
            grid on;
            box on;

            % --- omega plot ---
            subplot(3, 2, 2*r);
            plot(t, omega, 'Color', colors{r}, 'LineWidth', 1.2);
            xlabel('Time (s)');
            ylabel('\omega (rad/s)');
            title(sprintf('Run %d — Angular Velocity', r));
            xlim([t(1) t(end)]);
            grid on;
            box on;

        catch e
            subplot(3, 2, 2*r - 1);
            text(0.5, 0.5, sprintf('LOAD ERROR\n%s', e.message), ...
                'HorizontalAlignment', 'center', 'Color', 'r');
            axis off;
            subplot(3, 2, 2*r);
            text(0.5, 0.5, sprintf('LOAD ERROR\n%s', e.message), ...
                'HorizontalAlignment', 'center', 'Color', 'r');
            axis off;
        end
    end
end

% =============================================
% SECTION 3 — Overlay Plot (3 runs ใน plot เดียวต่อ group)
% =============================================
for g = 1:size(file_groups, 1)
    gname = file_groups{g, 1};
    files = file_groups{g, 2};

    figure('Name', sprintf('%s — Overlay', gname), 'NumberTitle', 'off', ...
        'Position', [150, 150, 1000, 450]);
    sgtitle(sprintf('%s — All Runs Overlay', gname), 'FontSize', 13, 'FontWeight', 'bold');

    ax1 = subplot(1, 2, 1); hold on;
    ax2 = subplot(1, 2, 2); hold on;

    legend_labels = {};

    for r = 1:length(files)
        try
            [t, omega, Vin] = load_file(base_path, files{r});

            plot(ax1, t, Vin,   'Color', colors{r}, 'LineWidth', 1.2);
            plot(ax2, t, omega, 'Color', colors{r}, 'LineWidth', 1.2);
            legend_labels{end+1} = sprintf('Run %d', r);
        catch
            % skip failed runs in overlay
        end
    end

    xlabel(ax1, 'Time (s)'); ylabel(ax1, 'V_{in} (V)');
    title(ax1, 'Input Voltage');
    legend(ax1, legend_labels, 'Location', 'best');
    grid(ax1, 'on'); box(ax1, 'on');

    xlabel(ax2, 'Time (s)'); ylabel(ax2, '\omega (rad/s)');
    title(ax2, 'Angular Velocity');
    legend(ax2, legend_labels, 'Location', 'best');
    grid(ax2, 'on'); box(ax2, 'on');
end

fprintf('\nEDA Complete — %d groups plotted\n', size(file_groups, 1));