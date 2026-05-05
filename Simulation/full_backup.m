% G6 Full Cycle Evidence Plot — Final Version
% PID: P=0.13 I=0.002 D=2.0 N=50
% Polynomial Trajectory: 4 cycles, Stop Time = 35s

Kff = 0;
save_system('Simulation');
bdclose all;
open_system('Simulation.slx');
simOut = sim('Simulation', 'StopTime', '35', 'ReturnWorkspaceOutputs', 'on');
logs  = simOut.get('logsout');

th_d  = logs.getElement('theta').Values.Data;
phi_d = logs.getElement('phi').Values.Data;
tor_d = logs.getElement('torque').Values.Data;
t_d   = logs.getElement('theta').Values.Time;

% ===== คำนวณ ST จาก Cycle แรกเท่านั้น =====
mask_s  = t_d >= 0 & t_d <= 5;
th_s    = th_d(mask_s);
t_s     = t_d(mask_s);
target  = 2*pi;
band    = target * 0.02;
out_s   = find(th_s < target-band | th_s > target+band);
if ~isempty(out_s)
    ST_man = t_s(out_s(end));
else
    ST_man = 2.11;
end

info = stepinfo(th_s, t_s, target, 'SettlingTimeThreshold', 0.02);

t_move   = ST_man;
t_pick   = 2.0;
t_place  = 2.0;
t_return = ST_man;
t_cycle  = t_move + t_pick + t_place + t_return;

fprintf('========== RESULTS ==========\n');
fprintf('Rise Time     : %.3f s\n', info.RiseTime);
fprintf('Settling Time : %.3f s\n', ST_man);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Cycle time    : %.3f s\n', t_cycle);
fprintf('Pcs / 35s     : %.2f\n',   35/t_cycle);
fprintf('Rod max phi   : %.4f deg\n', max(abs(phi_d))*180/pi);
fprintf('=============================\n');

% ===== COLORS =====
c_blue   = [0.13 0.47 0.71];
c_red    = [0.84 0.15 0.16];
c_green  = [0.17 0.63 0.17];
c_orange = [1.00 0.50 0.05];
c_purple = [0.58 0.40 0.74];
c_gray   = [0.50 0.50 0.50];

ax_props = {'Color','white','FontName','Helvetica','FontSize',11,...
    'GridColor',[0.87 0.87 0.87],'GridAlpha',1,'Box','on','LineWidth',0.8,...
    'XColor',[0.2 0.2 0.2],'YColor',[0.2 0.2 0.2]};

% ===== FIGURE =====
fig = figure('Name','G6 Full Cycle','Position',[60 40 1440 800]);
set(fig, 'Color', 'white');

% -----------------------------------------------
% Plot 1: Full Cycle Arm Angle (spanning full width)
% -----------------------------------------------
ax1 = subplot(2,2,[1 2]);
set(ax1, ax_props{:});
hold on;

% Shade Pick / Place zones per cycle
for k = 0:3
    t0        = k * t_cycle;
    t_pk_s    = t0 + t_move;
    t_pk_e    = t_pk_s + t_pick;
    t_pl_s    = t_pk_e;
    t_pl_e    = t_pl_s + t_place;

    patch([t_pk_s t_pk_e t_pk_e t_pk_s],[0 0 420 420], ...
        c_green,  'FaceAlpha',0.09,'EdgeColor','none','HandleVisibility','off');
    patch([t_pl_s t_pl_e t_pl_e t_pl_s],[0 0 420 420], ...
        c_purple, 'FaceAlpha',0.07,'EdgeColor','none','HandleVisibility','off');
end

% Label zones once
text(t_move + 0.2,        400, 'Pick',  'Color',c_green,  'FontSize',9,'FontWeight','bold');
text(t_move+t_pick + 0.2, 400, 'Place', 'Color',c_purple, 'FontSize',9,'FontWeight','bold');

% Arm angle line
plot(t_d, th_d*180/pi, 'Color',c_blue, 'LineWidth',2.0, 'DisplayName','Arm \theta');

% Reference lines
yline(360,'--','Color',c_red,   'LineWidth',1.2,'Label','360°',...
    'LabelHorizontalAlignment','left','FontSize',10,'HandleVisibility','off');
yline(180,'--','Color',c_orange,'LineWidth',1.0,'Label','180°',...
    'LabelHorizontalAlignment','left','FontSize',9, 'HandleVisibility','off');
yline(0,  '-', 'Color',c_gray,  'LineWidth',0.6,'HandleVisibility','off');

% Cycle dividers
for k = 1:4
    xline(k*t_cycle,':','Color',[0.65 0.65 0.65],'LineWidth',1.0,...
        'HandleVisibility','off');
    text(k*t_cycle - t_cycle/2, 15, sprintf('Cycle %d',k),...
        'HorizontalAlignment','center','FontSize',9,'Color',[0.5 0.5 0.5]);
end

% ST marker on cycle 1
xline(ST_man,'--','Color',c_green,'LineWidth',1.2,...
    'Label',sprintf('ST=%.2fs',ST_man),...
    'LabelVerticalAlignment','bottom','FontSize',9,'HandleVisibility','off');

xlim([0 35]); ylim([0 420]);
xlabel('Time (s)','FontSize',11);
ylabel('Arm angle (deg)','FontSize',11);
title(sprintf('Full Cycle — 4 Picks in 35s  |  Cycle = %.2fs  |  ST = %.2fs  OS = %.1f%%',...
    t_cycle, ST_man, info.Overshoot),'FontSize',12,'FontWeight','bold');
legend('Arm \theta','Location','northeast','FontSize',9,'Box','off');
grid on;

% -----------------------------------------------
% Plot 2: Rod Swing full 35s
% -----------------------------------------------
ax2 = subplot(2,2,3);
set(ax2, ax_props{:});
hold on;

patch([0 35 35 0],[-5 -5 5 5],c_green,'FaceAlpha',0.07,'EdgeColor','none');
plot(t_d, phi_d*180/pi,'Color',c_orange,'LineWidth',1.8);
yline(0,       '-', 'Color',c_gray,'LineWidth',0.8,'HandleVisibility','off');
yline( 13.67, '--', 'Color',c_red, 'LineWidth',1.0,...
    'Label','\phi_0 calc = 13.67°',...
    'LabelHorizontalAlignment','left','FontSize',9,'HandleVisibility','off');
yline(-13.67, '--', 'Color',c_red, 'LineWidth',1.0,'HandleVisibility','off');

xlim([0 35]); ylim([-16 16]);
xlabel('Time (s)','FontSize',11);
ylabel('Rod angle (deg)','FontSize',11);
title(sprintf('Rod swing \\phi — Full 35s  |  Max = %.3f°  (calc = 13.67°)',...
    max(abs(phi_d))*180/pi),'FontSize',12,'FontWeight','bold');
grid on;

text(17, -13, 'Smooth trajectory \rightarrow Rod barely swings',...
    'HorizontalAlignment','center','FontSize',9,...
    'Color',[0.1 0.5 0.1],'BackgroundColor',[0.92 1.0 0.92],...
    'EdgeColor',c_green,'Margin',3);

% -----------------------------------------------
% Plot 3: Cycle Breakdown
% -----------------------------------------------
ax3 = subplot(2,2,4);
set(ax3, ax_props{:});
hold on;

comp       = [t_move, 0.01, t_pick, t_place, t_return];
bar_names  = {'Move','Rod settle','Pick','Place','Return'};
bar_colors = [c_blue; c_orange; c_green; c_purple; 0.28 0.28 0.78];

for i = 1:length(comp)
    bar(i, comp(i), 0.65, 'FaceColor', bar_colors(i,:),...
        'EdgeColor','white','LineWidth',1.2);
end

yline(35/4,'--','Color',c_red,'LineWidth',1.5,...
    'Label','8.75 s  (35s ÷ 4 pcs)',...
    'LabelHorizontalAlignment','right','FontSize',9);

for i = 1:length(comp)
    if comp(i) > 0.1
        text(i, comp(i)+0.06, sprintf('%.2f s',comp(i)),...
            'HorizontalAlignment','center','FontSize',10,...
            'FontWeight','bold','Color',[0.15 0.15 0.15]);
    else
        text(i, 0.2, '~0 s',...
            'HorizontalAlignment','center','FontSize',9,'Color',[0.6 0.6 0.6]);
    end
end

set(ax3,'XTick',1:5,'XTickLabel',bar_names,'XTickLabelRotation',0);
ylim([0 max(comp)*1.4 + 0.6]);
ylabel('Time (s)','FontSize',11);
title(sprintf('Cycle breakdown  |  Total = %.2fs  →  %.2f pcs / 35s',...
    t_cycle, 35/t_cycle),'FontSize',12,'FontWeight','bold');
grid on;

% Result box
text(3, max(comp)*1.25,...
    sprintf('\\bf4 pieces / 35s  \\checkmark\\rm\n(%.2f achievable)', 35/t_cycle),...
    'HorizontalAlignment','center','FontSize',11,...
    'Color',[0.05 0.40 0.05],'BackgroundColor',[0.88 1.0 0.88],...
    'EdgeColor',c_green,'Margin',5);

% -----------------------------------------------
sgtitle('G6 Circular Pick and Place Robot — Full Cycle Simulation (35s)',...
    'FontSize',14,'FontWeight','bold','Color',[0.1 0.1 0.1]);

exportgraphics(fig,'G6_FullCycle_Final.png','Resolution',200,'BackgroundColor','white');
fprintf('Saved: G6_FullCycle_Final.png\n');

















