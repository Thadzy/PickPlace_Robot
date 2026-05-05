% ใช้ PID เดิม แต่ลด Trajectory Time ให้สั้นลง
set_param('Simulation/PID Controller', 'P', '0.13');
set_param('Simulation/PID Controller', 'I', '0.002');
set_param('Simulation/PID Controller', 'D', '2.0');
set_param('Simulation/PID Controller', 'N', '50');

% เปลี่ยน Polynomial Trajectory ให้เร็วขึ้น
% t_move = 1.0s แทน 2.11s เพื่อให้ได้ Cycle = 1+2+2+1 = 6s → 5.8 pcs/35s
blk = 'Simulation/Polynomial Trajectory';
set_param(blk, 'Waypoints',  '[0, 2*pi, 2*pi, 0, 0, 2*pi, 2*pi, 0, 0, 2*pi, 2*pi, 0, 0, 2*pi, 2*pi, 0, 0]');
set_param(blk, 'TimePoints', '[0, 1.0, 3.0, 4.0, 6.0, 7.0, 9.0, 10.0, 12.0, 13.0, 15.0, 16.0, 18.0, 19.0, 21.0, 22.0, 24.0]');
save_system('Simulation');

bdclose all;
open_system('Simulation.slx');
simOut5 = sim('Simulation','StopTime','35','ReturnWorkspaceOutputs','on');
logs5   = simOut5.get('logsout');

th5  = logs5.getElement('theta').Values.Data;
phi5 = logs5.getElement('phi').Values.Data;
t5   = logs5.getElement('theta').Values.Time;

fprintf('theta max = %.2f deg\n', max(th5)*180/pi);
fprintf('theta min = %.2f deg\n', min(th5)*180/pi);
fprintf('Rod max   = %.4f deg\n', max(abs(phi5))*180/pi);

% Plot เปรียบเทียบ
figure('Color','white','Position',[60 60 1200 500]);

subplot(1,2,1);
plot(t5, th5*180/pi, 'r-', 'LineWidth', 2);
hold on;
yline(360,'--','Color',[0.2 0.6 0.2],'LineWidth',1.2,'Label','360°');
yline(0,  '--','Color',[0.5 0.5 0.5],'LineWidth',0.8);
xlim([0 35]); ylim([-50 500]);
xlabel('Time (s)'); ylabel('deg');
title('Case 5 pcs/35s — PID เดิม Trajectory เร็วขึ้น','FontWeight','bold');
grid on;
text(17, 450, sprintf('Rod max = %.2f°', max(abs(phi5))*180/pi),...
    'HorizontalAlignment','center','FontSize',10,...
    'Color',[0.8 0.2 0.2],'BackgroundColor',[1 0.9 0.9],'EdgeColor','r');

subplot(1,2,2);
plot(t5, phi5*180/pi, 'Color',[1 0.4 0], 'LineWidth', 2);
hold on;
yline(0,'k--','LineWidth',0.8);
yline(13.67,'r:','Label','\phi_0 calc','FontSize',9);
yline(-13.67,'r:');
xlim([0 35]); ylim([-20 20]);
xlabel('Time (s)'); ylabel('deg');
title('Rod swing \phi — 5 pcs case','FontWeight','bold');
grid on;

sgtitle('G6 — Case: 5 pieces/35s (same PID, faster trajectory)',...
    'FontWeight','bold','FontSize',13);

exportgraphics(gcf,'G6_Case5pcs.png','Resolution',200,'BackgroundColor','white');
fprintf('Saved: G6_Case5pcs.png\n');