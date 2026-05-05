% /**
%  * @file debug_simulink_scurve_v2.m
%  * @description Analyzes the timeseries data exported from the Simulink model.
%  * This script retrieves simulation data, generates diagnostic plots, 
%  * and prints a statistical performance report to the console.
%  * It supports the 'Single simulation output' format (out object).
%  */

% =========================================================
% 1. Data Initialization and Verification
% =========================================================
has_out_struct = exist('out', 'var') == 1;

if has_out_struct && isprop(out, 'theta_ref')
    % Extract data packed inside the 'out' simulation output object
    time_data = out.theta_ref.Time;
    pos_ref   = out.theta_ref.Data;
    pos_act   = out.theta_actual.Data;
    voltage   = out.V_in.Data;
    tau       = out.Torque.Data;
    
elseif exist('theta_ref', 'var') == 1
    % Extract data exported directly to the base workspace
    time_data = theta_ref.Time;
    pos_ref   = theta_ref.Data;
    pos_act   = theta_actual.Data;
    voltage   = V_in.Data;
    tau       = Torque.Data;
    
else
    % Terminate execution if required data is missing
    error('Simulation data not found in base workspace or "out" object. Please verify the "To Workspace" block names.');
end

% Calculate position tracking error array
pos_error = pos_ref - pos_act;

% =========================================================
% 2. Data Visualization (Plotting)
% =========================================================
% Create a figure window for system diagnostic plots
figure('Name', 'Simulink System Debug Analysis', 'Position', [100, 100, 1200, 800]);

% Subplot 1: Position Tracking (Reference vs Actual)
subplot(2, 2, 1);
plot(time_data, pos_ref, 'b--', 'LineWidth', 2);
hold on;
plot(time_data, pos_act, 'r-', 'LineWidth', 1.5);
grid on;
title('Position Tracking: Reference vs Actual');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Reference', 'Actual', 'Location', 'best');

% Subplot 2: Position Tracking Error
subplot(2, 2, 2);
plot(time_data, pos_error, 'k-', 'LineWidth', 1.5);
grid on;
title('Position Tracking Error');
xlabel('Time (s)');
ylabel('Error (rad)');

% Subplot 3: Motor Input Voltage
subplot(2, 2, 3);
plot(time_data, voltage, 'm-', 'LineWidth', 1.5);
grid on;
title('Motor Input Voltage (V_{in})');
xlabel('Time (s)');
ylabel('Voltage (V)');

% Subplot 4: Motor Torque Output
subplot(2, 2, 4);
plot(time_data, tau, 'g-', 'LineWidth', 1.5);
grid on;
title('Motor Torque Output');
xlabel('Time (s)');
ylabel('Torque (N.m)');

% =========================================================
% 3. Statistical Analysis
% =========================================================
% Calculate Maximum Absolute Error and locate its timestamp
[max_abs_error, max_err_idx] = max(abs(pos_error));
time_max_error = time_data(max_err_idx);

% Calculate Root Mean Square (RMS) Error for overall tracking performance
rms_error = sqrt(mean(pos_error.^2));

% Calculate Steady-State Error using the last 5 percent of the simulation data
steady_state_idx = round(0.95 * length(pos_error));
ss_error = mean(abs(pos_error(steady_state_idx:end)));

% Calculate peak Voltage limits and their respective timestamps
[max_voltage, max_v_idx] = max(voltage);
time_max_voltage = time_data(max_v_idx);
[min_voltage, min_v_idx] = min(voltage);
time_min_voltage = time_data(min_v_idx);

% Calculate peak Torque limits and their respective timestamps
[max_torque, max_t_idx] = max(tau);
time_max_torque = time_data(max_t_idx);
[min_torque, min_t_idx] = min(tau);
time_min_torque = time_data(min_t_idx);

% =========================================================
% 4. Console Output (Reporting)
% =========================================================
% Print the calculated metrics in a formatted table to the Command Window
fprintf('\n=================================================================\n');
fprintf('                 SIMULATION PERFORMANCE REPORT                 \n');
fprintf('=================================================================\n');
fprintf('1. Tracking Error Metrics:\n');
fprintf('   - Max Absolute Error : %10.4f rad  (at t = %6.3f s)\n', max_abs_error, time_max_error);
fprintf('   - RMS Error          : %10.4f rad\n', rms_error);
fprintf('   - Steady-State Error : %10.4f rad\n', ss_error);
fprintf('-----------------------------------------------------------------\n');
fprintf('2. Electrical and Mechanical Limits:\n');
fprintf('   - Max Voltage (+ve)  : %10.4f V    (at t = %6.3f s)\n', max_voltage, time_max_voltage);
fprintf('   - Min Voltage (-ve)  : %10.4f V    (at t = %6.3f s)\n', min_voltage, time_min_voltage);
fprintf('   - Max Torque (+ve)   : %10.4f N.m  (at t = %6.3f s)\n', max_torque, time_max_torque);
fprintf('   - Min Torque (-ve)   : %10.4f N.m  (at t = %6.3f s)\n', min_torque, time_min_torque);
fprintf('=================================================================\n\n');