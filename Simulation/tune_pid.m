% ตัด Feedforward โดยตั้ง Kff = 0 ถาวร
Kff = 0;

% ใช้ค่า PID ที่ดีที่สุด P=0.13 I=0.002 D=2.0 ST=2.11s
set_param('Simulation/PID Controller', ...
    'P', '0.13', ...
    'I', '0.002', ...
    'D', '2.0', ...
    'N', '50');
save_system('Simulation');

fprintf('P=0.13 I=0.002 D=2.0 Kff=0\n');
fprintf('Expected: ST~2.1s OS~2%% theta_end~365 deg\n');
fprintf('\nCycle time estimate:\n');
t_move   = 2.11;
t_pick   = 2.0;
t_place  = 2.0;
t_return = 2.11;
t_cycle  = t_move + t_pick + t_place + t_return;
fprintf('Move+Return: %.2f s\n', t_move + t_return);
fprintf('Pick+Place : %.2f s\n', t_pick + t_place);
fprintf('Total/cycle: %.2f s\n', t_cycle);
fprintf('Pcs/35s    : %.2f\n', 35/t_cycle);