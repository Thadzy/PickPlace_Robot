fprintf('========================================\n');
fprintf('G6 Simulation Full Config Check\n');
fprintf('========================================\n');

% --- PID ---
pid = 'Simulation/PID Controller';
fprintf('\n=== PID Controller ===\n');
fprintf('P = %s\n', get_param(pid,'P'));
fprintf('I = %s\n', get_param(pid,'I'));
fprintf('D = %s\n', get_param(pid,'D'));
fprintf('N = %s\n', get_param(pid,'N'));

% % --- Saturation ---
% sat = 'Simulation/Saturation';
% fprintf('\n=== Saturation ===\n');
% fprintf('Upper = %s Nm\n', get_param(sat,'UpperLimit'));
% fprintf('Lower = %s Nm\n', get_param(sat,'LowerLimit'));

% --- Joints ---
arm_j = 'Simulation/Simscape/Arm Pivot';
rod_j = 'Simulation/Simscape/Joint Pivot';

fprintf('\n=== Arm Pivot Joint ===\n');
fprintf('TorqueActuation  : %s\n', get_param(arm_j,'TorqueActuationMode'));
fprintf('MotionActuation  : %s\n', get_param(arm_j,'MotionActuationMode'));
fprintf('SensePosition    : %s\n', get_param(arm_j,'SensePosition'));
fprintf('SenseVelocity    : %s\n', get_param(arm_j,'SenseVelocity'));
fprintf('Damping          : %s %s\n', get_param(arm_j,'DampingCoefficient'), get_param(arm_j,'DampingCoefficientUnits'));

fprintf('\n=== Rod Joint Pivot ===\n');
fprintf('TorqueActuation  : %s\n', get_param(rod_j,'TorqueActuationMode'));
fprintf('MotionActuation  : %s\n', get_param(rod_j,'MotionActuationMode'));
fprintf('SensePosition    : %s\n', get_param(rod_j,'SensePosition'));
fprintf('PositionTarget   : %s\n', get_param(rod_j,'PositionTargetSpecify'));
fprintf('Damping          : %s %s\n', get_param(rod_j,'DampingCoefficient'), get_param(rod_j,'DampingCoefficientUnits'));
fprintf('UpperLimit       : %s\n', get_param(rod_j,'UpperLimitSpecify'));
fprintf('LowerLimit       : %s\n', get_param(rod_j,'LowerLimitSpecify'));

% --- Rigid Transforms ---
rts = {
    'RT1 Base Center',  'Simulation/Simscape/RT1 Base Center';
    'RT2 Arm Pivot',    'Simulation/Simscape/RT2 Arm Pivot';
    'RT3 Arm COM',      'Simulation/Simscape/RT3 Arm COM';
    'RT4 Arm Tip',      'Simulation/Simscape/RT4 Arm Tip';
    'RT5 Rod COM',      'Simulation/Simscape/RT5 Rod COM';
    'RT6 Joint Axis',   'Simulation/Simscape/RT6 Joint Axis'};

fprintf('\n=== Rigid Transforms ===\n');
for i = 1:size(rts,1)
    try
        blk = rts{i,2};
        rot = get_param(blk,'RotationMethod');
        ang = get_param(blk,'RotationAngle');
        ax  = get_param(blk,'RotationStandardAxis');
        tr  = get_param(blk,'TranslationCartesianOffset');
        fprintf('%s:\n', rts{i,1});
        fprintf('  Rotation : %s %s %s deg\n', rot, ax, ang);
        fprintf('  Translation: %s m\n', tr);
    catch
        fprintf('%s: cannot read\n', rts{i,1});
    end
end

% --- Bodies ---
bodies = {
    'Base',           'Simulation/Simscape/Base';
    'Shaft',          'Simulation/Simscape/Shaft';
    'Arm',            'Simulation/Simscape/Arm';
    'Gripper',        'Simulation/Simscape/Gripper';
    'Connection Rod', 'Simulation/Simscape/Connection Rod'};

fprintf('\n=== Body Densities ===\n');
for i = 1:size(bodies,1)
    try
        den = get_param(bodies{i,2},'Density');
        fprintf('%s: %s kg/m^3\n', bodies{i,1}, den);
    catch
        fprintf('%s: cannot read\n', bodies{i,1});
    end
end

% --- Gravity ---
mech = 'Simulation/Simscape/Mechanism Configuration';
fprintf('\n=== Mechanism Config ===\n');
try
    grav = get_param(mech,'UniformGravity');
    fprintf('Gravity: %s\n', grav);
catch
    fprintf('Gravity: cannot read\n');
end

% --- Solver ---
fprintf('\n=== Solver ===\n');
fprintf('Solver   : %s\n', get_param('Simulation','Solver'));
fprintf('StopTime : %s\n', get_param('Simulation','StopTime'));
fprintf('MaxStep  : %s\n', get_param('Simulation','MaxStep'));

fprintf('\n========================================\n');
fprintf('Config Check Complete\n');
fprintf('========================================\n');