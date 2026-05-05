% =============================================
% EDA Script — System Identification Data
% G6 Circular Pick and Place Robot
% =============================================

base_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data/';

% --- Define all files ---
file_groups = {
    'Ststep',            {'Ststep_run1.mat', 'Ststep_run2.mat', 'Ststep_run3.mat'};
    'Sin Amp6',          {'Sin_Amp6_run1.mat', 'Sin_Amp6_run2.mat', 'Sin_Amp6_run3.mat'};
    'Sin Amp9',          {'Sin_Amp9_run1.mat', 'Sin_Amp9_run2.mat', 'Sin_Amp9_run3.mat'};
    'Sin Amp12',         {'Sin_Amp12_run1.mat', 'Sin_Amp12_run2.mat', 'Sin_Amp12_run3.mat'};
    'Chirp 0.5Hz',       {'Chirp_0.1_30_0.5_run1.mat', 'Chirp_0.1_30_0.5_run2.mat', 'Chirp_0.1_30_0.5_run3.mat'};
    'Chirp 1Hz',         {'Chirp_0.1_30_1_run1.mat',   'Chirp_0.1_30_1_run2.mat',   'Chirp_0.1_30_1_run3.mat'};
    'Chirp 2Hz',         {'Chirp_0.1_30_2_run1.mat',   'Chirp_0.1_30_2_run2.mat',   'Chirp_0.1_30_2_run3.mat'};
};

% --- Helper function to load one file ---
function [t, omega, Vin] = load_file(base_path, filename)
    tmp   = load(fullfile(base_path, filename));
    d     = tmp.data;
    omega = double(squeeze(d.getElement(1).Values.Data));
    Vin   = double(squeeze(d.getElement(3).Values.Data));
    t     = d.getElement(1).Values.Time;
end

% =============================================
% SECTION 1 — Summary Table
% =============================================
fprintf('\n%-20s  %-4s  %-10s  %-8s  %-8s  %-10s  %-10s\n', ...
    'Group', 'Run', 'Duration(s)', 'Vin_min', 'Vin_max', 'omega_min', 'omega_max');
fprintf('%s\n', repmat('-', 1, 80));

for g = 1:size(file_groups, 1)
    gname = file_groups{g, 1};
    files = file_groups{g, 2};
    for r = 1:length(files)
        try
            [t, omega, Vin] = load_file(base_path, files{r});
            fprintf('%-20s  %-4d  %-10.2f  %-8.3f  %-8.3f  %-10.3f  %-10.3f\n', ...
                gname, r, t(end)-t(1), min(Vin), max(Vin), min(omega), max(omega));
        catch
            fprintf('%-20s  %-4d  LOAD ERROR\n', gname, r);
        end
    end
end

% =============================================
% SECTION 2 — Plot แต่ละ Group
% =============================================
for g = 1:size(file_groups, 1)
    gname = file_groups{g, 1};
    files = file_groups{g, 2};

    figure('Name', gname, 'NumberTitle', 'off', 'Position', [100, 100, 1000, 700]);
    sgtitle(gname, 'FontSize', 14, 'FontWeight', 'bold');

    for r = 1:length(files)
        try
            [t, omega, Vin] = load_file(base_path, files{r});

            % --- Vin plot ---
            subplot(3, 2, 2*r - 1);
            plot(t, Vin, 'b', 'LineWidth', 1.0);
            xlabel('Time (s)');
            ylabel('V_{in} (V)');
            title(sprintf('Run %d — Input Voltage', r));
            xlim([t(1) t(end)]);
            grid on;

            % --- omega plot ---
            subplot(3, 2, 2*r);
            plot(t, omega, 'r', 'LineWidth', 1.0);
            xlabel('Time (s)');
            ylabel('\omega (rad/s)');
            title(sprintf('Run %d — Angular Velocity', r));
            xlim([t(1) t(end)]);
            grid on;

        catch
            subplot(3, 2, 2*r - 1);
            text(0.5, 0.5, 'LOAD ERROR', 'HorizontalAlignment', 'center');
            subplot(3, 2, 2*r);
            text(0.5, 0.5, 'LOAD ERROR', 'HorizontalAlignment', 'center');
        end
    end
end

fprintf('\nEDA Complete — %d groups plotted\n', size(file_groups, 1));