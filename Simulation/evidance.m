simOut = sim('Simulation', 'StopTime', '10', ...
    'ReturnWorkspaceOutputs', 'on');
logs  = simOut.get('logsout');

th_d  = logs.getElement('theta').Values.Data;
phi_d = logs.getElement('phi').Values.Data;
tor_d = logs.getElement('torque').Values.Data;
t_d   = logs.getElement('theta').Values.Time;

% วัด Arm
mask = t_d >= 0 & t_d <= 8;
info = stepinfo(th_d(mask), t_d(mask), 2*pi, ...
    'SettlingTimeThreshold', 0.02);

% วัด Rod Settling Time หลังแขนหยุด
t_stop    = info.SettlingTime;
mask_rod  = t_d > t_stop;
phi_after = phi_d(mask_rod);
t_after   = t_d(mask_rod);
band      = 5 * pi/180;
idx_rod   = find(abs(phi_after) <= band, 1);
rod_ST    = t_after(idx_rod) - t_stop;

% Cycle Time
t_move   = info.SettlingTime;
t_pick   = 2.0;
t_place  = 2.0;
t_return = info.SettlingTime;
t_cycle  = t_move + rod_ST + t_pick + t_place + t_return;

fprintf('=== ARM ===\n');
fprintf('Settling Time : %.3f s\n', info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('\n=== ROD ===\n');
fprintf('Max swing     : %.2f deg\n', max(abs(phi_d))*180/pi);
fprintf('Rod ST (±5deg): %.3f s\n',   rod_ST);
fprintf('\n=== CYCLE ===\n');
fprintf('Total/cycle   : %.3f s\n', t_cycle);
fprintf('Pieces in 35s : %.1f pcs\n', 35/t_cycle);

% Plot
figure('Name','G6 Evidence','Position',[50 50 1200 600]);

subplot(2,2,1);
plot(t_d, th_d*180/pi,'b-','LineWidth',2);
hold on;
yline(360,'r--','Target');
xline(info.SettlingTime,'g--', ...
    sprintf('ST=%.2fs',info.SettlingTime));
xlabel('Time (s)'); ylabel('deg');
title(sprintf('Arm | ST=%.2fs OS=%.0f%%', ...
    info.SettlingTime,info.Overshoot));
grid on;

subplot(2,2,2);
plot(t_d, phi_d*180/pi,'y-','LineWidth',2);
hold on;
yline(0,'w--');
yline(13.67,'r:','\phi_0 calc');
yline(-13.67,'r:');
xline(t_stop+rod_ST,'g--', ...
    sprintf('Rod ST=%.2fs',rod_ST));
xlabel('Time (s)'); ylabel('deg');
title(sprintf('Rod swing | Max=%.1f° ST=%.2fs', ...
    max(abs(phi_d))*180/pi, rod_ST));
grid on;

subplot(2,2,3);
plot(t_d, tor_d,'r-','LineWidth',2);
yline(5.4,'k--','T_{max}');
yline(-5.4,'k--');
xlabel('Time (s)'); ylabel('Nm');
title('Motor Torque');
grid on;

subplot(2,2,4);
comp  = [t_move, rod_ST, t_pick, t_place, t_return];
names = {'Move','Rod settle','Pick','Place','Return'};
b = bar(comp,'FaceColor','flat');
b.CData = [0 0.4 1;1 0.5 0;0 0.8 0;0.8 0 0.8;0.3 0.3 0.8];
set(gca,'XTickLabel',names);
yline(35/4,'r--','8.75s (4pcs/35s)');
ylabel('Time (s)');
title(sprintf('Cycle: %.1fs → %.1f pcs/35s', ...
    t_cycle, 35/t_cycle));
for i = 1:length(comp)
    text(i,comp(i)+0.05,sprintf('%.2fs',comp(i)), ...
        'HorizontalAlignment','center','FontSize',9);
end
grid on;

sgtitle('G6 Pick and Place Robot — Evidence for Negotiation', ...
    'FontSize',12,'FontWeight','bold');
saveas(gcf,'G6_Evidence.png');
fprintf('\nSaved: G6_Evidence.png\n');