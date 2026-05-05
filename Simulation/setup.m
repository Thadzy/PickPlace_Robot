% =========================================================
% G6 Motor Model Setup Script
% สร้าง DC Motor Subsystem ใน Simulink
% Input:  V_input (Voltage)
% Output: theta_arm (rad)
% =========================================================
% วิธีใช้:
%   1. รัน Script นี้ใน Command Window
%   2. Simulink จะสร้าง Model ใหม่ชื่อ G6_MotorModel.slx
%   3. นำ Subsystem ไป Copy ใส่ใน Simulation.slx

% --- Motor Parameters (ติดเป็นตัวแปร ใส่จาก System ID ภายหลัง) ---
% ถ้ายังไม่มีค่าจริง ใช้ค่า Assume เหล่านี้ก่อน
R    = 1.0;       % Armature resistance [Ohm]          *** ใส่จาก System ID ***
L    = 0.005;     % Armature inductance [H]            *** ใส่จาก System ID ***
Kt   = 0.05;      % Torque constant [N*m/A]            *** ใส่จาก System ID ***
Ke   = 0.05;      % Back-EMF constant [V*s/rad]        *** ใส่จาก System ID ***
Jm   = 0.0001;    % Motor rotor inertia [kg*m^2]       *** ใส่จาก System ID ***
Bm   = 0.001;     % Motor viscous friction [N*m*s/rad] *** ใส่จาก System ID ***

% --- Transmission Parameters (รู้แล้วจาก Design) ---
i_gear   = 24;          % Gear ratio
eta_gear = 0.85;        % Gear efficiency
i_belt   = 70/24;       % Belt ratio (Driven/Driver teeth)
eta_belt = 0.95;        % Belt efficiency
N_total  = i_gear * i_belt;   % Total ratio = 70

% --- Arm Inertia (ที่ Motor Shaft ก่อนทด) ---
J_arm    = 0.5764;      % Arm inertia at output [kg*m^2]
J_arm_m  = J_arm / (N_total^2);  % Reflected to motor shaft

J_total  = Jm + J_arm_m;  % Total inertia at motor shaft

fprintf('=== Motor Model Parameters ===\n');
fprintf('R    = %.4f Ohm\n', R);
fprintf('L    = %.5f H\n', L);
fprintf('Kt   = %.4f N*m/A\n', Kt);
fprintf('Ke   = %.4f V*s/rad\n', Ke);
fprintf('Jm   = %.6f kg*m^2\n', Jm);
fprintf('Bm   = %.5f N*m*s/rad\n', Bm);
fprintf('N_total = %.2f\n', N_total);
fprintf('J_arm reflected = %.6f kg*m^2\n', J_arm_m);
fprintf('J_total at motor = %.6f kg*m^2\n', J_total);

% --- Steady state omega_arm ที่ V=24V (No-load estimate) ---
omega_m_noload  = 24 / Ke;              % rad/s at motor shaft
omega_arm_noload = omega_m_noload / N_total;
fprintf('\nomega_motor (no-load, 24V) = %.2f rad/s\n', omega_m_noload);
fprintf('omega_arm   (no-load, 24V) = %.2f rad/s\n', omega_arm_noload);

% ===== สร้าง Simulink Model =====
mdl = 'G6_MotorModel';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% --- สร้าง Subsystem ---
sub = [mdl '/DC Motor with Transmission'];
add_block('simulink/Ports & Subsystems/Subsystem', sub);

% ลบ Block เดิมใน Subsystem
delete_block([sub '/In1']);
delete_block([sub '/Out1']);
delete_block([sub '/Sum']);  % ถ้ามี

% --- Port: V_input ---
add_block('simulink/Sources/In1', [sub '/V_input'], ...
    'Position', [30 120 60 140]);

% --- Saturation: จำกัด Voltage ±24V ---
add_block('simulink/Discontinuities/Saturation', [sub '/V_sat'], ...
    'UpperLimit', '24', 'LowerLimit', '-24', ...
    'Position', [100 115 140 145]);

% --- Sum: V - Back_EMF ---
add_block('simulink/Math Operations/Sum', [sub '/Sum_V'], ...
    'Inputs', '+-', ...
    'Position', [190 115 220 145]);

% --- Gain: 1/L ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_1overL'], ...
    'Gain', num2str(1/L), ...
    'Position', [260 115 300 145]);

% --- Integrator: i (current) ---
add_block('simulink/Continuous/Integrator', [sub '/Integrator_i'], ...
    'InitialCondition', '0', ...
    'Position', [340 115 380 145]);

% --- Gain: Kt (Torque) ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_Kt'], ...
    'Gain', num2str(Kt), ...
    'Position', [420 115 460 145]);

% --- Sum: T_motor - T_friction ---
add_block('simulink/Math Operations/Sum', [sub '/Sum_T'], ...
    'Inputs', '+-', ...
    'Position', [510 115 540 145]);

% --- Gain: 1/J_total ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_1overJ'], ...
    'Gain', num2str(1/J_total), ...
    'Position', [570 115 610 145]);

% --- Integrator: omega_motor ---
add_block('simulink/Continuous/Integrator', [sub '/Integrator_w'], ...
    'InitialCondition', '0', ...
    'Position', [650 115 690 145]);

% --- Gain: Back-EMF (Ke) ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_Ke'], ...
    'Gain', num2str(Ke), ...
    'Orientation', 'left', ...
    'Position', [340 200 380 230]);

% --- Gain: Bm (Friction) ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_Bm'], ...
    'Gain', num2str(Bm), ...
    'Orientation', 'left', ...
    'Position', [570 200 610 230]);

% --- Gain: Gearbox (1/i_gear * eta_gear) ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_Gear'], ...
    'Gain', num2str(eta_gear / i_gear), ...
    'Position', [730 115 780 145]);

% --- Gain: Belt (1/i_belt * eta_belt) ---
add_block('simulink/Math Operations/Gain', [sub '/Gain_Belt'], ...
    'Gain', num2str(eta_belt / i_belt), ...
    'Position', [810 115 860 145]);

% --- Integrator: theta_arm ---
add_block('simulink/Continuous/Integrator', [sub '/Integrator_th'], ...
    'InitialCondition', '0', ...
    'Position', [890 115 930 145]);

% --- Port: theta_arm output ---
add_block('simulink/Sinks/Out1', [sub '/theta_arm'], ...
    'Position', [970 120 1000 140]);

% --- เชื่อม Feedforward path ---
add_line(sub, 'V_input/1',       'V_sat/1');
add_line(sub, 'V_sat/1',         'Sum_V/1');
add_line(sub, 'Sum_V/1',         'Gain_1overL/1');
add_line(sub, 'Gain_1overL/1',   'Integrator_i/1');
add_line(sub, 'Integrator_i/1',  'Gain_Kt/1');
add_line(sub, 'Gain_Kt/1',       'Sum_T/1');
add_line(sub, 'Sum_T/1',         'Gain_1overJ/1');
add_line(sub, 'Gain_1overJ/1',   'Integrator_w/1');
add_line(sub, 'Integrator_w/1',  'Gain_Gear/1');
add_line(sub, 'Gain_Gear/1',     'Gain_Belt/1');
add_line(sub, 'Gain_Belt/1',     'Integrator_th/1');
add_line(sub, 'Integrator_th/1', 'theta_arm/1');

% --- เชื่อม Back-EMF feedback ---
add_line(sub, 'Integrator_w/1', 'Gain_Ke/1');
add_line(sub, 'Gain_Ke/1',      'Sum_V/2');

% --- เชื่อม Friction feedback ---
add_line(sub, 'Integrator_w/1', 'Gain_Bm/1');
add_line(sub, 'Gain_Bm/1',      'Sum_T/2');

% --- จัด Layout ---
Simulink.BlockDiagram.arrangeSystem(sub);

% --- บันทึก ---
save_system(mdl, [mdl '.slx']);
fprintf('\nSaved: %s.slx\n', mdl);
fprintf('\nวิธีใช้ต่อ:\n');
fprintf('1. เปิด G6_MotorModel.slx\n');
fprintf('2. Copy block "DC Motor with Transmission"\n');
fprintf('3. วางใน Simulation.slx แทน Saturation Block เดิม\n');
fprintf('4. ต่อ PID output -> V_input\n');
fprintf('5. ต่อ theta_arm -> Sum (feedback)\n');
fprintf('6. พอมีค่าจาก System ID แก้ตัวแปร R,L,Kt,Ke,Jm,Bm ด้านบน\n');