function [A, B, C, D, K] = motor_grey_ode(p, Ts, R_m, L_m, N)
% MOTOR_GREY_ODE — State-space matrices for DC motor grey-box model
%
% Parameters p (4 params):
%   p(1) = J     [kg.m2]   total inertia at arm shaft
%   p(2) = B     [N.m.s]   viscous damping at arm shaft
%   p(3) = KtEff [N.m/A]   effective torque constant = eta * Kt
%   p(4) = Ke    [V.s/rad] back-EMF constant
%
% Note: eta and Kt cannot be separated from velocity data alone.
%       Estimate KtEff = eta*Kt. If Kt is known from datasheet,
%       compute eta = KtEff / Kt afterward.

J     = p(1);
B_val = p(2);
KtEff = p(3);
Ke    = p(4);

A = [-R_m/L_m,      -Ke*N/L_m;
      KtEff*N/J,    -B_val/J  ];

B = [1/L_m; 0];

C = [0, 1];

D = 0;

K = zeros(2, 1);

end