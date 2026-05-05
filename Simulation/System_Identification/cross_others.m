% =============================================
% Cross Validation ทุก dataset
% =============================================

all_val = {
    dd_ss1,    'Stair Step run1 (est)';
    dd_ss2,    'Stair Step run2 (est)';
    dd_ss_val, 'Stair Step run3 (val)';
    dd_c05_1,  'Chirp 0.5Hz run1 (est)';
    dd_c05_2,  'Chirp 0.5Hz run2 (est)';
    dd_c05_val,'Chirp 0.5Hz run3 (val)';
    dd_c1_1,   'Chirp 1Hz run1 (est)';
    dd_c1_2,   'Chirp 1Hz run2 (est)';
    dd_c1_val, 'Chirp 1Hz run3 (val)';
    };

% ---- Print Fit Score ทั้งหมด ----
fprintf('=== Cross Validation — ทุก Dataset ===\n');
fprintf('%-30s  %-10s\n', 'Dataset', 'Fit (%)');
fprintf('%s\n', repmat('-',1,45));

fit_scores = zeros(size(all_val,1),1);
for i = 1:size(all_val,1)
    [~, fit] = compare(all_val{i,1}, sys_est);
    fit_scores(i) = fit;
    fprintf('%-30s  %.2f%%\n', all_val{i,2}, fit);
end

fprintf('%s\n', repmat('-',1,45));
fprintf('%-30s  %.2f%%\n', 'Average', mean(fit_scores));

% ---- Plot ทุก dataset แยก Figure ----

% Stair Step
figure('Name', 'Cross Val — Stair Step', 'NumberTitle', 'off');
for i = 1:3
    subplot(3,1,i);
    compare(all_val{i,1}, sys_est);
    title(all_val{i,2});
    grid on;
end

% Chirp 0.5Hz
figure('Name', 'Cross Val — Chirp 0.5Hz', 'NumberTitle', 'off');
for i = 4:6
    subplot(3,1,i-3);
    compare(all_val{i,1}, sys_est);
    title(all_val{i,2});
    grid on;
end

% Chirp 1Hz
figure('Name', 'Cross Val — Chirp 1Hz', 'NumberTitle', 'off');
for i = 7:9
    subplot(3,1,i-6);
    compare(all_val{i,1}, sys_est);
    title(all_val{i,2});
    grid on;
end