% /**
%  * @file debug_rod_phi.m
%  * @description Analyzes the angular motion of the rod (phi) exported
%  * from Simscape as a timeseries via To Workspace block.
%  * Checks whether the rod actually moves, oscillates, or tracks
%  * the reference trajectory correctly.
%  */

% =========================================================
% 1. Load and Validate phi Data
% =========================================================
if ~exist('out', 'var') || ~isprop(out, 'phi')
    error('out.phi not found. Please run the simulation first.');
end

time_phi = out.phi.Time;
phi_data = out.phi.Data;

% Convert from rad to deg for easier interpretation
phi_deg = rad2deg(phi_data);

% =========================================================
% 2. Load theta_ref for Comparison (if available)
% =========================================================
has_ref = isprop(out, 'theta_ref');
if has_ref
    time_ref  = out.theta_ref.Time;
    theta_ref = out.theta_ref.Data;
    theta_ref_deg = rad2deg(theta_ref);
end

% =========================================================
% 3. Compute Rod Motion Statistics
% =========================================================
phi_range   = max(phi_deg) - min(phi_deg);   % Total angular range swept
phi_start   = phi_deg(1);                     % Starting angle
phi_end     = phi_deg(end);                   % Final angle
phi_max     = max(phi_deg);
phi_min     = min(phi_deg);

% Angular velocity of rod (numerical differentiation)
dphi = diff(phi_data) ./ diff(time_phi);      % rad/s
time_dphi = time_phi(1:end-1);

dphi_max = max(abs(dphi));
dphi_deg = rad2deg(dphi);

% Detect if rod is essentially stationary
MOTION_THRESHOLD_DEG = 1.0;                   % deg — below this = not moving
is_stationary = phi_range < MOTION_THRESHOLD_DEG;

% =========================================================
% 4. Oscillation Detection at End of Motion
% =========================================================
% Look at last 10% of simulation time
settle_idx  = round(0.90 * length(phi_deg));
phi_tail    = phi_deg(settle_idx:end);
phi_tail_range = max(phi_tail) - min(phi_tail);

OSCILLATION_THRESHOLD_DEG = 0.5;             % deg — above this = oscillating
is_oscillating = phi_tail_range > OSCILLATION_THRESHOLD_DEG;

% =========================================================
% 5. Plotting
% =========================================================
figure('Name', 'Rod (phi) Motion Debug', 'Position', [100, 100, 1200, 700]);

% Plot 1: phi angle over time
subplot(2, 2, 1);
plot(time_phi, phi_deg, 'b-', 'LineWidth', 1.8);
if has_ref
    hold on;
    plot(time_ref, theta_ref_deg, 'r--', 'LineWidth', 1.5);
    legend('phi actual (deg)', 'theta ref (deg)', 'Location', 'best');
end
grid on;
title('Rod Angle (phi) Over Time');
xlabel('Time (s)');
ylabel('Angle (deg)');

% Plot 2: Angular velocity of rod
subplot(2, 2, 2);
plot(time_dphi, dphi_deg, 'k-', 'LineWidth', 1.5);
grid on;
title('Rod Angular Velocity (d\phi/dt)');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

% Plot 3: Tail section — check for oscillation at settling
subplot(2, 2, 3);
time_tail = time_phi(settle_idx:end);
plot(time_tail, phi_tail, 'm-', 'LineWidth', 1.8);
grid on;
title('Rod Angle — Last 10% of Simulation (Settling Check)');
xlabel('Time (s)');
ylabel('Angle (deg)');

% Plot 4: phi vs theta_ref error (if ref available)
if has_ref
    subplot(2, 2, 4);
    % Interpolate ref onto phi time axis for fair comparison
    theta_ref_interp = interp1(time_ref, theta_ref, time_phi, 'linear', 'extrap');
    rod_error_deg    = rad2deg(theta_ref_interp - phi_data);
    plot(time_phi, rod_error_deg, 'r-', 'LineWidth', 1.5);
    grid on;
    title('Tracking Error: theta\_ref - phi');
    xlabel('Time (s)');
    ylabel('Error (deg)');
end

% =========================================================
% 6. Console Report
% =========================================================
fprintf('\n=================================================================\n');
fprintf('                   ROD (phi) MOTION REPORT                     \n');
fprintf('=================================================================\n');
fprintf('1. Angular Range:\n');
fprintf('   - Start angle     : %10.4f deg\n', phi_start);
fprintf('   - End angle       : %10.4f deg\n', phi_end);
fprintf('   - Max angle       : %10.4f deg\n', phi_max);
fprintf('   - Min angle       : %10.4f deg\n', phi_min);
fprintf('   - Total range     : %10.4f deg\n', phi_range);
fprintf('-----------------------------------------------------------------\n');
fprintf('2. Motion Check:\n');
if is_stationary
    fprintf('   [FAIL] Rod is STATIONARY — total motion < %.1f deg\n', MOTION_THRESHOLD_DEG);
    fprintf('          Possible causes:\n');
    fprintf('          - Torque still too low (check N_total gain)\n');
    fprintf('          - Joint locked or Computed Motion still active\n');
    fprintf('          - Gravity overpowering torque\n');
else
    fprintf('   [PASS] Rod is MOVING — range = %.4f deg\n', phi_range);
end
fprintf('-----------------------------------------------------------------\n');
fprintf('3. Oscillation Check (last 10%% of simulation):\n');
if is_oscillating
    fprintf('   [WARN] Rod is OSCILLATING at end — tail range = %.4f deg\n', phi_tail_range);
    fprintf('          Suggestion: Increase Kd or reduce Kp after System ID\n');
else
    fprintf('   [PASS] Rod is SETTLED — tail range = %.4f deg\n', phi_tail_range);
end
fprintf('-----------------------------------------------------------------\n');
fprintf('4. Angular Velocity:\n');
fprintf('   - Peak angular velocity : %10.4f rad/s\n', dphi_max);
fprintf('                           : %10.4f deg/s\n', rad2deg(dphi_max));
fprintf('=================================================================\n\n');