% =============================================
% หา Signal Bandwidth ของแต่ละสัญญาณ
% =============================================

signals = {
    ststep1,    'Stair Step';
    chirp_05_1, 'Chirp 0.5Hz';
    chirp_1_1,  'Chirp 1Hz';
    chirp_2_1,  'Chirp 2Hz';
    sin6_1,     'Sine 6V';
    sin9_1,     'Sine 9V';
    sin12_1,    'Sine 12V';
    };

dt = 0.001;
fs = 1/dt;

fprintf('%-15s  %-12s  %-12s  %-12s\n', 'Signal', 'f_signal(Hz)', 'f_noise(Hz)', 'fc_suggest(Hz)');
fprintf('%s\n', repmat('-', 1, 60));

for i = 1:size(signals, 1)
    d     = signals{i,1};
    name  = signals{i,2};
    omega = double(squeeze(d{1}.Values.Data));
    t     = d{1}.Values.Time;

    % หา frequency content ด้วย FFT
    N       = length(omega);
    f_axis  = (0:N-1) * (fs/N);
    Y       = abs(fft(omega)) / N;
    Y_half  = Y(1:floor(N/2));
    f_half  = f_axis(1:floor(N/2));

    % หา f ที่มี power สะสม 95% (signal bandwidth)
    power_cum = cumsum(Y_half.^2) / sum(Y_half.^2);
    idx_95    = find(power_cum >= 0.95, 1);
    f_signal  = f_half(idx_95);

    % quantization noise frequency = Nyquist
    f_noise = fs / 2;

    % แนะนำ fc = 10x f_signal แต่ไม่เกิน f_noise/10
    fc_suggest = min(f_signal * 10, f_noise / 10);

    fprintf('%-15s  %-12.3f  %-12.1f  %-12.1f\n', name, f_signal, f_noise, fc_suggest);
end