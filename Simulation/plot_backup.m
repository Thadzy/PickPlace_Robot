% รัน Simulation สุดท้าย
set_param('Simulation/PID Controller', 'P', '0.094054');
set_param('Simulation/PID Controller', 'I', '0.0012872');
set_param('Simulation/PID Controller', 'D', '1.1688');
set_param('Simulation/PID Controller', 'N', '228.3587');
save_system('Simulation');

bdclose all;
open_system('Simulation.slx');
simOut = sim('Simulation','StopTime','10','ReturnWorkspaceOutputs','on');
logs  = simOut.get('logsout');

th_d  = logs.getElement('theta').Values.Data;
phi_d = logs.getElement('phi').Values.Data;
tor_d = logs.getElement('torque').Values.Data;
t_d   = logs.getElement('theta').Values.Time;

% วัดผล
mask  = t_d >= 0 & t_d <= 10;
info  = stepinfo(th_d(mask), t_d(mask), 2*pi, ...
    'SettlingTimeThreshold', 0.02);

% ST manual
target  = 2*pi;
band    = target * 0.02;
outside = find(th_d < target-band | th_d > target+band);
ST_man  = t_d(outside(end));

% Rod ST
t_stop   = 4.0;
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

% Cycle Time
t_move   = ST_man;
t_pick   = 2.0;
t_place  = 2.0;
t_return = ST_man;
t_cycle  = t_move + rod_ST + t_pick + t_place + t_return;

fprintf('Rise Time  : %.3f s\n', info.RiseTime);
fprintf('ST manual  : %.3f s\n', ST_man);
fprintf('Overshoot  : %.2f %%\n', info.Overshoot);
fprintf('Rod swing  : %.4f deg\n', max(abs(phi_d))*180/pi);
fprintf('Rod ST     : %.3f s\n', rod_ST);
fprintf('Cycle time : %.3f s\n', t_cycle);
fprintf('Pcs/35s    : %.2f\n', 35/t_cycle);

% Save data สำหรับ Claude
save('G6_evidence_data.mat', 't_d','th_d','phi_d','tor_d', ...
    'ST_man','rod_ST','t_cycle','info');
fprintf('Saved: G6_evidence_data.mat\n');