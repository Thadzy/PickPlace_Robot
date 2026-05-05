% คืน PID กลับเป็นค่า PID Tuner
set_param('Simulation/PID Controller', 'P', '0.094054');
set_param('Simulation/PID Controller', 'I', '0.0012872');
set_param('Simulation/PID Controller', 'D', '1.1688');
set_param('Simulation/PID Controller', 'N', '228.3587');
save_system('Simulation');

bdclose all;
open_system('Simulation.slx');

simOut = sim('Simulation','StopTime','40','ReturnWorkspaceOutputs','on');
logs  = simOut.get('logsout');
th_d  = logs.getElement('theta').Values.Data;
phi_d = logs.getElement('phi').Values.Data;
t_d   = logs.getElement('theta').Values.Time;

mask = t_d >= 0 & t_d <= 10;
info = stepinfo(th_d(mask),t_d(mask),2*pi,'SettlingTimeThreshold',0.02);

fprintf('========== FINAL RESULTS ==========\n');
fprintf('Settling Time : %.3f s\n', info.SettlingTime);
fprintf('Overshoot     : %.2f %%\n', info.Overshoot);
fprintf('Rise Time     : %.3f s\n', info.RiseTime);
fprintf('Theta final   : %.2f deg\n', th_d(end)*180/pi);
fprintf('Rod max swing : %.4f deg\n', max(abs(phi_d))*180/pi);
fprintf('===================================\n');