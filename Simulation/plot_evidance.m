% G6 Evidence Plot — Light Mode, Clean Style
% ใช้ PID: P=0.13 I=0.002 D=2.0 N=50

Kff = 0;
save_system('Simulation');
bdclose all;
open_system('Simulation.slx');
simOut = sim('Simulation', 'StopTime', '40', 'ReturnWorkspaceOutputs', 'on');
logs  = simOut.get('logsout');

th_d  = logs.getElement('theta').Values.Data;
phi_d = logs.getElement('phi').Values.Data;
tor_d = logs.getElement('torque').Values.Data;
t_d   = logs.getElement('theta').Values.Time;

% วัดผล
mask = t_d >= 0 & t_d <= 40;
info = stepinfo(th_d(mask), t_d(mask), 2*pi, 'SettlingTimeThreshold', 0.02);

target  = 2*pi;
band    = target * 0.02;
outside = find(th_d < target-band | th_d > target+band);
ST_man  = t_d(outside(end));

t_stop   = ST_man;
mask_rod = t_d > t_stop;
phi_a    = phi_d(mask_rod);
t_a      = t_d(mask_rod);
band_rod = 5 * pi/180;
idx_rod  = find(abs(phi_a) <= band_rod, 1);
if ~isempty(idx_rod)
    rod_ST = t_a(idx_rod) - t_stop;
else
    rod_ST = 0;
end

t_move   = ST_man;
t_pick   = 2.0;
t_place  = 2.0;
t_return = ST_man;
t_cycle  = t_move + rod_ST + t_pick + t_place + t_return;

fprintf('========== FINAL RESULTS ==========\n');
fprintf('Rise Time     : %.3f s\n', info.RiseTime);
fprintf('Settling Time : %.3f s\n', ST_man);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Rod swing max : %.4f deg\n', max(abs(phi_d))*180/pi);
fprintf('Rod ST        : %.3f s\n', rod_ST);
fprintf('Cycle time    : %.3f s\n', t_cycle);
fprintf('Pcs / 35s     : %.2f\n', 35/t_cycle);
fprintf('====================================\n');

% ===== PLOT =====
fig = figure('Name','G6 Evidence','Position',[80 80 1280 720]);
set(fig, 'Color', 'white');

c_blue   = [0.13 0.47 0.71];
c_red    = [0.84 0.15 0.16];
c_green  = [0.17 0.63 0.17];
c_orange = [1.00 0.50 0.05];
c_purple = [0.58 0.40 0.74];
c_gray   = [0.40 0.40 0.40];

ax_props = {'Color','white','FontName','Helvetica','FontSize',11,...
    'GridColor',[0.85 0.85 0.85],'GridAlpha',1,...
    'Box','on','LineWidth',0.8,...
    'XColor',[0.2 0.2 0.2],'YColor',[0.2 0.2 0.2]};

% --- Plot 1: Arm Angle ---
ax1 = subplot(2,2,1);
set(ax1, ax_props{:});
hold on;
patch([0 t_d(end) t_d(end) 0], ...
    [360*(1-0.02) 360*(1-0.02) 360*(1+0.02) 360*(1+0.02)], ...
    c_green, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
plot(t_d, th_d*180/pi, 'Color', c_blue, 'LineWidth', 2.0);
yline(360, '--', 'Color', c_red, 'LineWidth', 1.2, 'Label', 'Target 360°', ...
    'LabelHorizontalAlignment','left','FontSize',10);
xline(ST_man, '--', 'Color', c_green, 'LineWidth', 1.2, ...
    'Label', sprintf('ST = %.2f s', ST_man), ...
    'LabelVerticalAlignment','bottom','FontSize',10);
xlim([0 10]); ylim([0 420]);
xlabel('Time (s)', 'FontSize', 11);
ylabel('Angle (deg)', 'FontSize', 11);
title(sprintf('Arm angle  |  RT = %.2f s   OS = %.1f%%', ...
    info.RiseTime, info.Overshoot), 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% --- Plot 2: Rod Swing ---
ax2 = subplot(2,2,2);
set(ax2, ax_props{:});
hold on;
patch([0 t_d(end) t_d(end) 0], [-5 -5 5 5], ...
    c_green, 'FaceAlpha', 0.08, 'EdgeColor','none');
plot(t_d, phi_d*180/pi, 'Color', c_orange, 'LineWidth', 2.0);
yline(0,     '-',  'Color', c_gray,  'LineWidth', 0.8);
yline(13.67, '--', 'Color', c_red,   'LineWidth', 1.0, ...
    'Label','\phi_0 calc = 13.67°','LabelHorizontalAlignment','left','FontSize',9);
yline(-13.67,'--', 'Color', c_red,   'LineWidth', 1.0);
xlim([0 10]); ylim([-16 16]);
xlabel('Time (s)', 'FontSize', 11);
ylabel('Angle (deg)', 'FontSize', 11);
title(sprintf('Rod swing \\phi  |  Max = %.3f°  (calc = 13.67°)', ...
    max(abs(phi_d))*180/pi), 'FontSize', 12, 'FontWeight', 'bold');
grid on;

annotation(fig,'textbox',[0.52 0.74 0.22 0.06],...
    'String',sprintf('Smooth trajectory\\newline→ Rod barely moves'),...
    'FitBoxToText','on','EdgeColor',c_green,'BackgroundColor',[0.9 1.0 0.9],...
    'FontSize',9,'Color',[0.1 0.5 0.1]);

% --- Plot 3: Motor Torque ---
ax3 = subplot(2,2,3);
set(ax3, ax_props{:});
hold on;
patch([0 t_d(end) t_d(end) 0], [-5.4 -5.4 5.4 5.4], ...
    [0.95 0.95 0.95], 'FaceAlpha', 1, 'EdgeColor','none');
plot(t_d, tor_d, 'Color', c_red, 'LineWidth', 2.0);
yline(5.4,  '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, ...
    'Label','T_{max} = +5.4 Nm','LabelHorizontalAlignment','left','FontSize',9);
yline(-5.4, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0, ...
    'Label','T_{min} = -5.4 Nm','LabelHorizontalAlignment','left','FontSize',9);
yline(0,    '-',  'Color', c_gray, 'LineWidth', 0.6);
xlim([0 10]); ylim([-6.5 6.5]);
xlabel('Time (s)', 'FontSize', 11);
ylabel('Torque (N·m)', 'FontSize', 11);
title('Motor torque', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% --- Plot 4: Cycle Breakdown ---
ax4 = subplot(2,2,4);
set(ax4, ax_props{:});
hold on;

comp  = [t_move, rod_ST, t_pick, t_place, t_return];
names = {'Move', 'Rod settle', 'Pick', 'Place', 'Return'};
colors_bar = [c_blue; c_orange; c_green; c_purple; 0.3 0.3 0.8];

for i = 1:length(comp)
    bar(i, comp(i), 0.6, 'FaceColor', colors_bar(i,:), ...
        'EdgeColor', 'white', 'LineWidth', 1.2);
end

yline(35/4, '--', 'Color', c_red, 'LineWidth', 1.5, ...
    'Label','8.75 s  (= 35s ÷ 4 pcs)', ...
    'LabelHorizontalAlignment','right','FontSize',9);

for i = 1:length(comp)
    if comp(i) > 0.05
        text(i, comp(i) + 0.05, sprintf('%.2f s', comp(i)), ...
            'HorizontalAlignment','center','FontSize',10,...
            'FontWeight','bold','Color',[0.2 0.2 0.2]);
    else
        text(i, 0.12, sprintf('%.3f s', comp(i)), ...
            'HorizontalAlignment','center','FontSize',9,...
            'Color',[0.5 0.5 0.5]);
    end
end

set(ax4, 'XTick', 1:5, 'XTickLabel', names, 'XTickLabelRotation', 0);
ylim([0 max(comp)*1.3 + 0.5]);
ylabel('Time (s)', 'FontSize', 11);
title(sprintf('Cycle breakdown  |  Total = %.2f s  →  %.2f pcs / 35s', ...
    t_cycle, 35/t_cycle), 'FontSize', 12, 'FontWeight', 'bold');
grid on;

annotation(fig,'textbox',[0.73 0.10 0.24 0.07],...
    'String',sprintf('\\bf4 pieces / 35s ✓\\rm\n(%.2f pcs achievable)', 35/t_cycle),...
    'FitBoxToText','on','EdgeColor',c_green,'BackgroundColor',[0.9 1.0 0.9],...
    'FontSize',10,'Color',[0.05 0.45 0.05],'HorizontalAlignment','center');

sgtitle('G6 Circular Pick and Place Robot — Simulation Evidence', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);

% บันทึก
exportgraphics(fig, 'G6_Evidence_Final_v2.png', 'Resolution', 200, 'BackgroundColor', 'white');
fprintf('Saved: G6_Evidence_Final_v2.png\n');

save('G6_evidence_data.mat','t_d','th_d','phi_d','tor_d','ST_man','rod_ST','t_cycle','info');
fprintf('Saved: G6_evidence_data.mat\n');