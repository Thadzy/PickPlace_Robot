function [A, B, C, D, K] = motor_grey_ode_v2(p, Ts, R_m, L_m, N, k_t)
% MOTOR_GREY_ODE_V2
% Parameters p (4 params):
%   p(1) = J_total [kg.m2]
%   p(2) = B_arm   [N.m.s/rad]
%   p(3) = eta     [-]         motor efficiency
%   p(4) = k_e     [V.s/rad]
%
% Fixed: k_t from datasheet calculation

J     = p(1);
B_val = p(2);
eta   = p(3);
k_e   = p(4);

% Effective torque = eta * k_t
KtEff = eta * k_t;

A = [-R_m/L_m,       -k_e*N/L_m;
    KtEff*N/J,     -B_val/J  ];

B = [1/L_m; 0];
C = [0, 1];
D = 0;
K = zeros(2, 1);
end