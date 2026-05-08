% =========================================================
% preprocess_chirp.m
% Preprocess raw chirp data from Simulink Dataset
% - Extract Vin and omega signals
% - Apply zero-phase Butterworth low-pass filter
% - Save each run as separate .mat file for Parameter Estimation
%
% Naming convention (from PDF):
%   Input folder : Parameters_Estimator2/
%   Output folder: Preprocess_Data/
%   Output name  : chirp05_run1.mat, chirp1_run1.mat, chirp2_run1.mat
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: USER SETTINGS - เปลี่ยนตรงนี้เท่านั้น
% =========================================================

% Path ของโฟลเดอร์ raw data
raw_data_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';

% Path สำหรับบันทึกผล (Preprocess_Data folder)
output_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Preprocess_Data/';

% สร้าง output folder ถ้ายังไม่มี
if ~exist(output_path, 'dir')
    mkdir(output_path);
    fprintf('Created output folder: %s\n', output_path);
end

% =========================================================
% SECTION 2: FILTER DESIGN
% =========================================================

fs   = 1000;   % Sampling frequency [Hz] - จาก time step 0.001 s
fc   = 10;     % Cutoff frequency [Hz]   - 5x chirp max freq (2 Hz)
N    = 4;      % Filter order            - Butterworth, -80 dB at 10x fc
Wn   = fc / (fs/2);  % Normalized cutoff (0 to 1, where 1 = Nyquist)

% ออกแบบ Butterworth low-pass filter
[b, a] = butter(N, Wn, 'low');

% แสดง filter spec ให้ verify
fprintf('=== Filter Design ===\n');
fprintf('Sampling rate    : %d Hz\n', fs);
fprintf('Chirp max freq   : 2 Hz\n');
fprintf('Cutoff freq (fc) : %.1f Hz  (= 5 x 2 Hz)\n', fc);
fprintf('Filter type      : Butterworth, Order %d\n', N);
fprintf('Normalized Wn    : %.4f\n', Wn);
fprintf('Attenuation at 100 Hz: %.1f dB\n', -20*N*log10(100/fc));
fprintf('=====================\n\n');

% =========================================================
% SECTION 3: FILE LIST
% =========================================================
% {raw filename,  output filename}
file_list = {
    'chirp_01_05_run1.mat', 'chirp05_run1.mat';
    'chirp_01_05_run2.mat', 'chirp05_run2.mat';
    'chirp_01_05_run3.mat', 'chirp05_run3.mat';
    'chirp_01_1_run1.mat',  'chirp1_run1.mat';
    'chirp_01_1_run2.mat',  'chirp1_run2.mat';
    'chirp_01_1_run3.mat',  'chirp1_run3.mat';
    'chirp_01_2_run1.mat',  'chirp2_run1.mat';
    'chirp_01_2_run2.mat',  'chirp2_run2.mat';
    'chirp_01_2_run3.mat',  'chirp2_run3.mat';
};

% =========================================================
% SECTION 4: PROCESS EACH FILE
% =========================================================

num_files = size(file_list, 1);

for i = 1:num_files

    raw_name    = file_list{i, 1};
    output_name = file_list{i, 2};
    raw_file    = fullfile(raw_data_path, raw_name);
    out_file    = fullfile(output_path, output_name);

    fprintf('[%d/%d] Processing: %s\n', i, num_files, raw_name);

    % --- Load file ---
    loaded = load(raw_file);
    data   = loaded.data;  % Simulink.SimulationData.Dataset

    % --- Extract time vector ---
    t_raw   = double(data{3}.Values.Time);   % time จาก Vin
    t_omega = double(data{1}.Values.Time);   % time จาก omega

    % --- Extract raw signals แล้วแปลงเป็น double ทันที ---
    Vin_raw   = double(data{3}.Values.Data);
    omega_raw = double(data{1}.Values.Data);

    % --- ตรวจสอบว่า time vector ตรงกัน ---
    if ~isequal(t_raw, t_omega)
        warning('Time vectors of Vin and omega do not match in %s.', raw_name);
    end

    % reshape เป็น column vector ป้องกัน dimension ผิด
    t_in      = t_raw(:);
    Vin_raw   = Vin_raw(:);
    omega_raw = omega_raw(:);

    % 
    % % กำหนด t_in เป็น time หลัก
    % t_in = t_in(:);
    % Vin_raw   = Vin_raw(:);
    % omega_raw = omega_raw(:);

    % --- Apply zero-phase Butterworth filter ---
    % filtfilt: pass forward แล้ว backward เพื่อให้ phase shift = 0
    Vin_filt   = filtfilt(b, a, Vin_raw);
    omega_filt = filtfilt(b, a, omega_raw);

    % --- บันทึกตัวแปรสำหรับ Parameter Estimation ---
    % ชื่อตัวแปร t_in, Vin_ws, omega_ws ตาม setup_params_est.m ใน PDF
    t       = t_in;
    Vin     = Vin_filt;
    omega_f = omega_filt;
    
    save(out_file, 't', 'Vin', 'omega_f');

    fprintf('    Saved -> %s\n', out_file);

    % --- Plot เปรียบเทียบ raw vs filtered ---
    figure('Name', sprintf('Preprocess: %s', output_name), 'NumberTitle', 'off');

    subplot(2,1,1);
    plot(t_in, Vin_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_in, Vin_filt, 'b', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('Vin (V)');
    title(sprintf('Vin - %s', output_name));
    legend('Raw', 'Filtered');
    grid on;

    subplot(2,1,2);
    plot(t_in, omega_raw, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold on;
    plot(t_in, omega_filt, 'r', 'LineWidth', 1.2);
    xlabel('Time (s)'); ylabel('\omega (rad/s)');
    title(sprintf('omega - %s', output_name));
    legend('Raw', 'Filtered');
    grid on;

end

% =========================================================
% SECTION 5: VERIFY FILTER - Frequency Response Plot
% =========================================================

figure('Name', 'Filter Frequency Response', 'NumberTitle', 'off');
freqz(b, a, 4096, fs);
title(sprintf('Butterworth LPF: Order %d, fc = %.1f Hz, fs = %d Hz', N, fc, fs));

fprintf('\nDone. All %d files processed.\n', num_files);