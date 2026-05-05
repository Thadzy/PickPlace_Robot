% =============================================
% ตัด Dead Zone ออก (Vin < 2V)
% =============================================

dead_zone_threshold = 2.0;  % V

function [t_cut, w_cut, v_cut] = remove_deadzone(t, omega, Vin, threshold)
    % ตัดช่วงที่ abs(Vin) < threshold ออก
    idx    = abs(Vin) >= threshold;
    t_cut  = t(idx);
    w_cut  = omega(idx);
    v_cut  = Vin(idx);
end

% ---- ตัด Dead Zone จาก Stair Step ทุก run ----
[t_ss1_cut, w_ss1_cut, v_ss1_cut] = remove_deadzone(t_ss1, w_ss1, v_ss1, dead_zone_threshold);
[t_ss2_cut, w_ss2_cut, v_ss2_cut] = remove_deadzone(t_ss2, w_ss2, v_ss2, dead_zone_threshold);
[t_ss3_cut, w_ss3_cut, v_ss3_cut] = remove_deadzone(t_ss3, w_ss3, v_ss3, dead_zone_threshold);

fprintf('=== Samples หลังตัด Dead Zone ===\n');
fprintf('%-20s  %-10s  %-10s\n', 'Dataset', 'Before', 'After');
fprintf('%s\n', repmat('-',1,45));
fprintf('%-20s  %-10d  %-10d\n', 'Stair Step run1', length(t_ss1), length(t_ss1_cut));
fprintf('%-20s  %-10d  %-10d\n', 'Stair Step run2', length(t_ss2), length(t_ss2_cut));
fprintf('%-20s  %-10d  %-10d\n', 'Stair Step run3', length(t_ss3), length(t_ss3_cut));

% ---- สร้าง iddata ใหม่ ----
Ts = 0.001;
dd_ss1_cut = iddata(w_ss1_cut, v_ss1_cut, Ts, 'Name', 'SS_run1_cut');
dd_ss2_cut = iddata(w_ss2_cut, v_ss2_cut, Ts, 'Name', 'SS_run2_cut');
dd_ss3_cut = iddata(w_ss3_cut, v_ss3_cut, Ts, 'Name', 'SS_run3_cut');

% ---- รวม Estimation Data ใหม่ ----
dd_est_new = merge(dd_ss1_cut, dd_ss2_cut, ...
                   dd_c05_1, dd_c05_2, ...
                   dd_c1_1, dd_c1_2);

% ---- Estimate ใหม่ ----
sys_est_new = tfest(dd_est_new, 2, 1);

fprintf('\n=== Transfer Function ใหม่ (ตัด Dead Zone แล้ว) ===\n');
sys_est_new

% ---- Cross Validation เปรียบเทียบ Old vs New ----
all_val = {
    dd_ss1_cut, 'Stair Step run1 cut';
    dd_ss2_cut, 'Stair Step run2 cut';
    dd_ss3_cut, 'Stair Step run3 cut (val)';
    dd_c05_1,   'Chirp 0.5Hz run1';
    dd_c05_val, 'Chirp 0.5Hz run3 (val)';
    dd_c1_1,    'Chirp 1Hz run1';
    dd_c1_val,  'Chirp 1Hz run3 (val)';
};

fprintf('\n=== Fit Score เปรียบเทียบ Old vs New ===\n');
fprintf('%-30s  %-12s  %-12s\n', 'Dataset', 'Old Model', 'New Model');
fprintf('%s\n', repmat('-',1,58));

for i = 1:size(all_val,1)
    [~, fit_old] = compare(all_val{i,1}, sys_est);
    [~, fit_new] = compare(all_val{i,1}, sys_est_new);
    fprintf('%-30s  %-12.2f  %-12.2f\n', all_val{i,2}, fit_old, fit_new);
end

% =============================================
% Plot เปรียบเทียบ Old vs New Model ทุก Dataset
% =============================================

for i = 1:size(all_val, 1)
    figure('Name', all_val{i,2}, 'NumberTitle', 'off');

    % Old model
    subplot(2,1,1);
    compare(all_val{i,1}, sys_est);
    [~, fit_old] = compare(all_val{i,1}, sys_est);
    title(sprintf('Old Model — %s (Fit: %.2f%%)', all_val{i,2}, fit_old));
    grid on;

    % New model
    subplot(2,1,2);
    compare(all_val{i,1}, sys_est_new);
    [~, fit_new] = compare(all_val{i,1}, sys_est_new);
    title(sprintf('New Model — %s (Fit: %.2f%%)', all_val{i,2}, fit_new));
    grid on;
end