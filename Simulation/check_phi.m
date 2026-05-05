bdclose all;
open_system('Simulation.slx');

simOut = sim('Simulation', 'StopTime', '20', ...
    'ReturnWorkspaceOutputs', 'on');
logs  = simOut.get('logsout');

phi_d = logs.getElement('phi').Values.Data;
th_d  = logs.getElement('theta').Values.Data;
t_d   = logs.getElement('phi').Values.Time;

fprintf('=== phi (Rod) ===\n');
fprintf('phi max : %.4f deg\n', max(phi_d)*180/pi);
fprintf('phi min : %.4f deg\n', min(phi_d)*180/pi);
fprintf('phi end : %.4f deg\n', phi_d(end)*180/pi);

fprintf('\n=== theta (Arm) ===\n');
fprintf('theta max : %.4f deg\n', max(th_d)*180/pi);
fprintf('theta end : %.4f deg\n', th_d(end)*180/pi);

figure;
subplot(2,1,1);
plot(t_d, th_d*180/pi, 'b-', 'LineWidth', 2);
yline(360,'r--','Target'); grid on;
xlabel('Time (s)'); ylabel('deg');
title('Arm angle theta');

subplot(2,1,2);
plot(t_d, phi_d*180/pi, 'y-', 'LineWidth', 2);
yline(0,'w--','Equilibrium'); grid on;
xlabel('Time (s)'); ylabel('deg');
title('Rod swing phi');