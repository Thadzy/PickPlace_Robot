% ดึง theta และ phi ออกจาก Dataset
theta_ts = test2.getElement('theta');
phi_ts   = test2.getElement('phi');

% แปลงเป็น array ธรรมดา
t     = theta_ts.Values.Time;
theta = theta_ts.Values.Data;
phi   = phi_ts.Values.Data;

% ตรวจสอบว่าได้ข้อมูลมาถูกต้อง
fprintf('Time points : %d\n', length(t));
fprintf('theta range : %.3f to %.3f rad\n', min(theta), max(theta));
fprintf('phi range   : %.3f to %.3f rad\n', min(phi), max(phi));

% วัดผลเต็ม
mask1 = t >= 0 & t <= 8;
info1 = stepinfo(theta(mask1), t(mask1), 2*pi, ...
    'SettlingTimeThreshold', 0.02);

fprintf('=== ผลการวัด G6 ===\n');
fprintf('Settling Time : %.4f s\n', info1.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info1.Overshoot);
fprintf('Rise Time     : %.4f s\n', info1.RiseTime);

% วัด Cycle Time
idx_done = find(theta >= 4*2*pi - 0.2, 1);
if ~isempty(idx_done)
    fprintf('4 cycles done at : %.2f s\n', t(idx_done));
    fprintf('Avg cycle time   : %.2f s/cycle\n', t(idx_done)/4);
end

% วัด Rod Settling Time
% หาช่วงที่แขนนิ่ง cycle แรก (ประมาณ t=3 ถึง t=7)
mask_still = t >= 3 & t <= 7;
phi_still  = phi(mask_still);
t_still    = t(mask_still);
phi_band   = 0.02 * max(abs(phi_still));
idx_settle = find(abs(phi_still) <= phi_band, 1);
if ~isempty(idx_settle)
    fprintf('Rod settle time  : %.4f s after arm stops\n', ...
        t_still(idx_settle) - t_still(1));
end

% Plot
figure('Name','G6 Full Cycle Result','Position',[100 100 900 600]);

subplot(2,1,1);
plot(t, theta*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
yline(360, 'r--', '1 rotation = 360 deg');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Arm angle \theta — 4 cycles');
grid on;

subplot(2,1,2);
plot(t, phi*180/pi, 'y-', 'LineWidth', 1.5);
hold on;
yline(0, 'w--', 'Equilibrium');
yline(13.67,  'r:', '+\phi_0 calc');
yline(-13.67, 'r:', '-\phi_0 calc');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Rod swing angle \phi');
grid on;

sgtitle('G6 Pick and Place Robot — Simulation Results');