% /**
%  * @file setup_simulink_parameters.m
%  * @description Initializes physical and control parameters for the robotic arm 
%  * simulation in Simulink. Includes motor electrical limits, gearbox 
%  * transmission ratio, and calculated feedforward gains.
%  */

% =========================================================
% 1. Motor Electrical and Mechanical Parameters
% =========================================================
V_max    = 24;          % [V] Maximum power supply voltage
P_rated  = 60;          % [W] Rated motor power
R_m      = 1.53982909;         % [Ohm] Motor internal terminal resistance
I_rated  = 2.5;         % [A] Rated continuous current (60W / 24V)
rpm_idle = 6000;        % [rpm] Motor internal idle speed
omega_m  = rpm_idle * (2*pi/60); % [rad/s] Motor internal speed
L_m = 0.00147261;

% Calculated motor constants
tau_m = P_rated / omega_m;                  % [N.m] Motor rated torque (approx 0.0955)
k_t   = tau_m / I_rated;                    % [N.m/A] Torque constant (approx 0.0382)
k_e   = (V_max - (I_rated * R_m)) / omega_m; % [V/(rad/s)] Back-EMF constant (approx 0.0330)
B_arm = 0.001;      % [N.m/(rad/s)] Damping at Arm Pivot (initial guess)
eta   = 0.85;       % [-] Motor efficiency (initial guess, typical 0.7-0.9)

% =========================================================
% 2. Mechanical Load and Transmission Parameters
% =========================================================
N_gear   = 24;          % Motor internal gearbox ratio
N_belt   = 70/24;       % Belt and pulley transmission ratio
N_total  = N_gear * N_belt; % Total transmission ratio (approx 70)
J_total  = 0.5764;      % [kg.m^2] Total equivalent inertia at the arm joint

J_scale = 1.0;  % initial guess = ไม่ scale

% =========================================================
% 3. Trajectory Parameters (S-Curve Profile)
% =========================================================
q_init   = 0;           % [rad] Initial position
q_final  = 2*pi;        % [rad] Final target position
v_max    = 8.97597901;           % [rad/s] Maximum allowable arm velocity
a_max    = 9.364072394;          % [rad/s^2] Maximum allowable arm acceleration
j_max    = 93.64072394;          % [rad/s^3] Maximum allowable arm jerk

% =========================================================
% 4. Feedforward Controller Gains
% =========================================================
% Velocity feedforward component to overcome back-EMF based on arm speed
% K_vff = k_e * N_total;  % Approx 2.31
K_vff = 0

% Acceleration feedforward component to overcome load inertia
% K_aff = (J_total * R_m) / (N_total * k_t); % Approx 0.2802
K_aff = 0

% Print confirmation to console
fprintf('System Setup Complete. K_vff = %.4f, K_aff = %.4f\n', K_vff, K_aff);