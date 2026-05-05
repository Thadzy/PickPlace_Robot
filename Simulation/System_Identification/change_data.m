% เปลี่ยนเป็น Chirp 0.5Hz run2
tmp      = load('/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Preprocess_Data/ss_run3.mat');
t_in     = tmp.t;
Vin_in   = tmp.Vin;
omega_ws = tmp.omega_f;
Vin_ws   = [t_in, Vin_in];

fprintf('Duration: %.1f s, Samples: %d\n', t_in(end), length(t_in));