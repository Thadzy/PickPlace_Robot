% ---- Poles and Zeros ----
fprintf('=== Poles and Zeros ===\n');
fprintf('Poles : \n'); disp(pole(sys_est));
fprintf('Zeros : \n'); disp(zero(sys_est));
fprintf('DC Gain    : %.4f rad/s/V\n', dcgain(sys_est));
fprintf('Wn         : %.4f rad/s\n',   sqrt(74.13));
fprintf('Zeta       : %.4f\n',         10.65/(2*sqrt(74.13)));

% ---- Cross Validation กับ Chirp ----
figure('Name', 'Cross Validation — Chirp', 'NumberTitle', 'off');

subplot(2,1,1);
compare(dd_c05_1, sys_est);
title('Chirp 0.5Hz run1 — Cross Validation');
grid on;

subplot(2,1,2);
compare(dd_c05_val, sys_est);
title('Chirp 0.5Hz run3 — Cross Validation');
grid on;

% ---- Bode Plot ----
figure('Name', 'Bode Plot', 'NumberTitle', 'off');
bode(sys_est);
grid on;
title('Bode Plot of Estimated Model');