% =========================================================
% export_report_figures.m
% สร้างและ export รูปทุกอันสำหรับ Report
% Output: /Users/thadzy/.../Report/images/
% Run ครั้งเดียว ได้ครบ 8 ไฟล์
% =========================================================

clc; clear; close all;

% Light theme — Academic style
set(groot, ...
    'defaultFigureColor',        [1 1 1], ...
    'defaultAxesColor',          [1 1 1], ...
    'defaultAxesFontSize',       10, ...
    'defaultAxesFontName',       'Helvetica', ...
    'defaultAxesLineWidth',      0.8, ...
    'defaultAxesXColor',         [0.15 0.15 0.15], ...
    'defaultAxesYColor',         [0.15 0.15 0.15], ...
    'defaultAxesGridColor',      [0.5 0.5 0.5], ...
    'defaultAxesGridAlpha',      0.25, ...
    'defaultAxesGridLineStyle',  '--', ...
    'defaultAxesBox',            'off', ...
    'defaultTextColor',          [0.15 0.15 0.15], ...
    'defaultLegendColor',        [1 1 1], ...
    'defaultLegendEdgeColor',    [0.8 0.8 0.8]);

% =========================================================
% PATHS
% =========================================================
img_path        = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Report/images/';
preprocess_path = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Preprocess_Data/';
raw_path        = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Data_V2/Parameters_Estimator2/';
model_path      = '/Users/thadzy/Documents/01_Projects/PickPlace_Robot/Simulation/System_Identification/Params_Estimate.slx';
model_name      = 'Params_Estimate';

if ~exist(img_path, 'dir'); mkdir(img_path); end

dpi = 300;
export_fig = @(fig, name) print(fig, fullfile(img_path, name), '-dpng', sprintf('-r%d', dpi));

fprintf('=== Exporting Report Figures ===\n\n');

% =========================================================
% SHARED PARAMETERS
% =========================================================
R_m     = 1.45336;
L_m     = 0.00144802;
N_total = 70;
K_e     = 0.04165;
K_t     = 0.04065;
B  = 0.19279;
J       = 0.72762;
eta     = 0.83607;
i_max   = 10.0;

% Filter (shared)
fs = 1000; fc = 10; N_ord = 4;
[b_f, a_f] = butter(N_ord, fc/(fs/2), 'low');

% S-curve parameters (shared)
v_max_sc = 7.3044; a_max_sc = 27.4912; j_max_sc = 1400.40;
q_total  = 2*pi;
t_j_sc   = a_max_sc / j_max_sc;
t_a_sc   = v_max_sc / a_max_sc - t_j_sc;
d1_sc    = j_max_sc * t_j_sc^3 / 6;
v1_sc    = 0.5 * j_max_sc * t_j_sc^2;
a1_sc    = j_max_sc * t_j_sc;
d2_sc    = v1_sc*t_a_sc + 0.5*a1_sc*t_a_sc^2;
v2_sc    = v1_sc + a1_sc*t_a_sc;
d3_sc    = v2_sc*t_j_sc + 0.5*a1_sc*t_j_sc^2 - j_max_sc*t_j_sc^3/6;
d_acc    = d1_sc + d2_sc + d3_sc;
d_cru    = q_total - 2*d_acc;
t_v_sc   = d_cru / v_max_sc;
t_tot_sc = 4*t_j_sc + 2*t_a_sc + t_v_sc;
T_sc     = [0, t_j_sc, t_j_sc+t_a_sc, 2*t_j_sc+t_a_sc, ...
            2*t_j_sc+t_a_sc+t_v_sc, 3*t_j_sc+t_a_sc+t_v_sc, ...
            3*t_j_sc+2*t_a_sc+t_v_sc, 4*t_j_sc+2*t_a_sc+t_v_sc];

% =========================================================
% [1/8] chirp_raw_filtered.png
% =========================================================
fprintf('[1/8] chirp_raw_filtered.png ... ');

loaded    = load(fullfile(raw_path, 'chirp_01_1_run1.mat'));
d         = loaded.data;
t_r       = double(d{3}.Values.Time(:));
Vin_r     = double(d{3}.Values.Data(:));
omega_r   = double(d{1}.Values.Data(:));
Vin_f     = filtfilt(b_f, a_f, Vin_r);
omega_f_r = filtfilt(b_f, a_f, omega_r);

fig1 = figure('Units','centimeters','Position',[1 1 22 14],'Color','white');
subplot(2,2,1);
plot(t_r, Vin_r,   'Color',[0.6 0.6 0.6],'LineWidth',0.8);
xlabel('Time (s)'); ylabel('V_{in} (V)');
title('V_{in} — Raw'); grid on; xlim([0 60]);

subplot(2,2,2);
plot(t_r, Vin_f,   'b','LineWidth',1.2);
xlabel('Time (s)'); ylabel('V_{in} (V)');
title('V_{in} — Filtered  (Butterworth, f_c=10 Hz, Order 4)');
grid on; xlim([0 60]);

subplot(2,2,3);
plot(t_r, omega_r, 'Color',[0.6 0.6 0.6],'LineWidth',0.8);
xlabel('Time (s)'); ylabel('\omega_{arm} (rad/s)');
title('\omega_{arm} — Raw'); grid on; xlim([0 60]);

subplot(2,2,4);
plot(t_r, omega_f_r,'r','LineWidth',1.2);
xlabel('Time (s)'); ylabel('\omega_{arm} (rad/s)');
title('\omega_{arm} — Filtered'); grid on; xlim([0 60]);

sgtitle('Chirp Signal Raw vs Filtered — chirp1\_run1','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig1, 'chirp_raw_filtered.png');
fprintf('Done\n');

% =========================================================
% [2/8] cross_validation.png  (Stair-Step: measured vs simulated)
% =========================================================
fprintf('[2/8] cross_validation.png ... ');

J   = 0.72762; B = 0.19279;
K_e = 0.04165; K_t    = 0.04065; eta = 0.83607;

if ~bdIsLoaded(model_name); load_system(model_path); end

ss_files = {'ss_run1.mat','ss_run2.mat','ss_run3.mat'};

fig2 = figure('Units','centimeters','Position',[1 1 24 9],'Color','white');

for r = 1:3
    ld2    = load(fullfile(raw_path, ss_files{r}));
    d2     = ld2.data;
    t_v2   = double(d2{3}.Values.Time(:));
    Vin_v2 = filtfilt(b_f, a_f, double(d2{3}.Values.Data(:)));
    om_v2  = filtfilt(b_f, a_f, double(d2{1}.Values.Data(:)));

    Vin_ws = [t_v2, Vin_v2]; omega_ws = om_v2; t_in = t_v2;
    set_param(model_name, 'StopTime', num2str(t_v2(end)));
    out2        = sim(model_name);
    om_sim2     = double(out2.yout{1}.Values.Data(:));
    t_sim2      = double(out2.yout{1}.Values.Time(:));
    om_interp2  = interp1(t_sim2, om_sim2, t_v2, 'linear', 'extrap');
    ss_res2     = sum((om_v2 - om_interp2).^2);
    ss_tot2     = sum((om_v2 - mean(om_v2)).^2);
    fit2        = (1 - ss_res2/ss_tot2) * 100;

    subplot(1,3,r);
    plot(t_v2, om_v2,      'b',  'LineWidth',1.2,'DisplayName','Measured'); hold on;
    plot(t_v2, om_interp2, 'r--','LineWidth',1.2,'DisplayName','Simulated');
    xlabel('Time (s)'); ylabel('\omega_{arm} (rad/s)');
    title(sprintf('Stair-Step Run %d   Fit = %.1f%%', r, fit2));
    legend('Location','best','FontSize',8); grid on;
end
sgtitle('Cross-Validation: Stair-Step Data','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig2, 'cross_validation.png');
fprintf('Done\n');

% =========================================================
% [3/8] cross_val_bar.png
% =========================================================
fprintf('[3/8] cross_val_bar.png ... ');

fit_scores = [45.5,33.6,59.8, 81.9,76.7,90.8, 95.4,91.5,84.7, ...
              95.2,96.0,95.6, 88.3,97.1,92.7, 93.9,97.8,98.1, 81.8,92.2,95.8];
x_labels = {'S6-1','S6-2','S6-3','S12-1','S12-2','S12-3','S24-1','S24-2','S24-3', ...
             'SS-1','SS-2','SS-3','C05-1','C05-2','C05-3','C1-1','C1-2','C1-3','C2-1','C2-2','C2-3'};
gc = [repmat([0.85 0.33 0.10],3,1); repmat([0.93 0.69 0.13],3,1);
      repmat([0.47 0.67 0.19],3,1); repmat([0.30 0.75 0.93],3,1);
      repmat([0.64 0.08 0.18],3,1); repmat([0.00 0.45 0.74],3,1);
      repmat([0.49 0.18 0.56],3,1)];

fig3 = figure('Units','centimeters','Position',[1 1 30 10],'Color','white');
bh = bar(fit_scores,'FaceColor','flat'); bh.CData = gc;
hold on;
yline(80,'r--','LineWidth',1.5);
set(gca,'XTickLabel',x_labels,'XTickLabelRotation',45,'FontSize',8);
ylabel('Fit Score (%)'); ylim([0 105]);
title('Cross-Validation Fit Score — All 21 Runs','FontSize',11,'FontWeight','bold','Color','black');

leg_patches = gobjects(8,1);
leg_names   = {'Step 6V','Step 12V','Step 24V','Stair-Step','Chirp 0.5Hz','Chirp 1.0Hz','Chirp 2.0Hz','Threshold 80%'};
gc_leg = [0.85 0.33 0.10; 0.93 0.69 0.13; 0.47 0.67 0.19; 0.30 0.75 0.93;
          0.64 0.08 0.18; 0.00 0.45 0.74; 0.49 0.18 0.56; 1 0 0];
for ii=1:7; leg_patches(ii)=patch(NaN,NaN,gc_leg(ii,:)); end
leg_patches(8) = plot(NaN,NaN,'r--','LineWidth',1.5);
legend(leg_patches,leg_names,'Location','southwest','FontSize',8);
grid on;
% export_fig(fig3, 'cross_val_bar.png');
% fprintf('Done\n');

% =========================================================
% [4/8] bode_plant.png
% =========================================================
fprintf('[4/8] bode_plant.png ... ');

K_num = K_t*eta*N_total / (L_m*J);
a1_tf = R_m/L_m + B/J;
a0_tf = (R_m*B + K_e*K_t*N_total^2) / (L_m*J);
G_vel = tf(K_num,[1,a1_tf,a0_tf]);
G_pos = tf(K_num,[1,a1_tf,a0_tf,0]);

omega_r2 = logspace(-2,4,5000);
[mv,phv] = bode(G_vel,omega_r2); mv=squeeze(mv); phv=squeeze(phv);
[mp,php] = bode(G_pos,omega_r2); mp=squeeze(mp); php=squeeze(php);

fig4 = figure('Units','centimeters','Position',[1 1 22 14],'Color','white');
subplot(2,2,1); semilogx(omega_r2,20*log10(mv),'b','LineWidth',1.5);
hold on; yline(0,'r--','0 dB');
xlabel('\omega (rad/s)'); ylabel('Magnitude (dB)');
title('|G_{vel}(j\omega)|  [V_{in} \rightarrow \omega_{arm}]');
grid on; xlim([1e-2 1e4]);

subplot(2,2,3); semilogx(omega_r2,phv,'b','LineWidth',1.5);
hold on; yline(-180,'r--','-180°');
xlabel('\omega (rad/s)'); ylabel('Phase (deg)');
title('\angle G_{vel}(j\omega)'); grid on; xlim([1e-2 1e4]);

subplot(2,2,2); semilogx(omega_r2,20*log10(mp),'r','LineWidth',1.5);
hold on; yline(0,'b--','0 dB');
xlabel('\omega (rad/s)'); ylabel('Magnitude (dB)');
title('|G_{pos}(j\omega)|  [V_{in} \rightarrow \theta_{arm}]');
grid on; xlim([1e-2 1e4]);

subplot(2,2,4); semilogx(omega_r2,php,'r','LineWidth',1.5);
hold on; yline(-180,'b--','-180°'); yline(-135,'g--','-135° (PM=45°)');
xlabel('\omega (rad/s)'); ylabel('Phase (deg)');
title('\angle G_{pos}(j\omega)'); grid on; xlim([1e-2 1e4]);

sgtitle('Plant Frequency Response','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig4,'bode_plant.png');
fprintf('Done\n');

% =========================================================
% [5/8] bode_openloop.png
% =========================================================
fprintf('[5/8] bode_openloop.png ... ');

opts_vel = pidtuneOptions('CrossoverFrequency',47,'PhaseMargin',70);
opts_pos = pidtuneOptions('CrossoverFrequency',9.41,'PhaseMargin',80);
C_vel_d  = pidtune(G_vel,'PID',opts_vel);
C_pos_d  = pidtune(tf(1,[1,0]),'PID',opts_pos);
L_vel    = C_vel_d * G_vel;
L_pos    = C_pos_d * tf(1,[1,0]);

[mlv,plv] = bode(L_vel,omega_r2); mlv=squeeze(mlv); plv=squeeze(plv);
[mlp,plp] = bode(L_pos,omega_r2); mlp=squeeze(mlp); plp=squeeze(plp);
[~,igv]   = min(abs(mlv-1)); [~,igp] = min(abs(mlp-1));
pmv = 180+plv(igv); pmp = 180+plp(igp);

fig5 = figure('Units','centimeters','Position',[1 1 22 14],'Color','white');
subplot(2,2,1); semilogx(omega_r2,20*log10(mlv),'b','LineWidth',1.5);
hold on; yline(0,'r--','0 dB');
xline(omega_r2(igv),'k--',sprintf('\\omega_c=%.1f',omega_r2(igv)));
xlabel('\omega (rad/s)'); ylabel('Magnitude (dB)');
title('|L_{vel}|  Velocity Open-loop'); grid on; xlim([1e-1 1e4]);

subplot(2,2,3); semilogx(omega_r2,plv,'b','LineWidth',1.5);
hold on; yline(-180,'r--','-180°');
xline(omega_r2(igv),'k--');
yline(-180+pmv,'g--',sprintf('PM=%.1f°',pmv));
xlabel('\omega (rad/s)'); ylabel('Phase (deg)');
title('\angle L_{vel}'); grid on; xlim([1e-1 1e4]);

subplot(2,2,2); semilogx(omega_r2,20*log10(mlp),'r','LineWidth',1.5);
hold on; yline(0,'b--','0 dB');
xline(omega_r2(igp),'k--',sprintf('\\omega_c=%.1f',omega_r2(igp)));
xlabel('\omega (rad/s)'); ylabel('Magnitude (dB)');
title('|L_{pos}|  Position Open-loop'); grid on; xlim([1e-1 1e4]);

subplot(2,2,4); semilogx(omega_r2,plp,'r','LineWidth',1.5);
hold on; yline(-180,'b--','-180°');
xline(omega_r2(igp),'k--');
yline(-180+pmp,'g--',sprintf('PM=%.1f°',pmp));
xlabel('\omega (rad/s)'); ylabel('Phase (deg)');
title('\angle L_{pos}'); grid on; xlim([1e-1 1e4]);

sgtitle('Open-loop Bode after PID Design','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig5,'bode_openloop.png');
fprintf('Done\n');

% =========================================================
% [6/8] scurve_360.png
% =========================================================
fprintf('[6/8] scurve_360.png ... ');

dt_sc = 0.0001;
t_sc  = 0:dt_sc:t_tot_sc+0.02;
n_sc  = length(t_sc);
jk=zeros(1,n_sc); ak=zeros(1,n_sc); vk=zeros(1,n_sc); pk=zeros(1,n_sc);

for k=1:n_sc
    tk=t_sc(k);
    if     tk<=T_sc(2); jn=+j_max_sc; a0=0;     v0=0;       p0=0;           t0=T_sc(1);
    elseif tk<=T_sc(3); jn=0;         a0=+a1_sc; v0=v1_sc;   p0=d1_sc;       t0=T_sc(2);
    elseif tk<=T_sc(4); jn=-j_max_sc; a0=+a1_sc; v0=v2_sc;   p0=d1_sc+d2_sc; t0=T_sc(3);
    elseif tk<=T_sc(5); jn=0;         a0=0;      v0=v_max_sc; p0=d_acc;       t0=T_sc(4);
    elseif tk<=T_sc(6); jn=-j_max_sc; a0=0;      v0=v_max_sc; p0=d_acc+d_cru; t0=T_sc(5);
    elseif tk<=T_sc(7); jn=0;         a0=-a1_sc; v0=v_max_sc-v1_sc; p0=d_acc+d_cru+d1_sc; t0=T_sc(6);
    elseif tk<=T_sc(8); jn=+j_max_sc; a0=-a1_sc; v0=v_max_sc-v1_sc-a1_sc*t_a_sc; p0=d_acc+d_cru+d1_sc+d2_sc; t0=T_sc(7);
    else;               jn=0;         a0=0;      v0=0;        p0=q_total;     t0=T_sc(8);
    end
    tau=tk-t0; jk(k)=jn; ak(k)=a0+jn*tau;
    vk(k)=v0+a0*tau+0.5*jn*tau^2;
    pk(k)=p0+v0*tau+0.5*a0*tau^2+jn*tau^3/6;
end

fig6 = figure('Units','centimeters','Position',[1 1 16 20],'Color','white');
subplot(4,1,1); plot(t_sc,jk,'k','LineWidth',1.2);
ylabel('Jerk (rad/s^3)');
title('S-Curve Motion Profile — 360°'); grid on; xlim([0 t_tot_sc+0.05]);

subplot(4,1,2); plot(t_sc,ak,'b','LineWidth',1.2); hold on;
yline(+a_max_sc,'r--','a_{max}'); yline(-a_max_sc,'r--');
ylabel('Accel (rad/s^2)'); grid on; xlim([0 t_tot_sc+0.05]);

subplot(4,1,3); plot(t_sc,vk,'g','LineWidth',1.2); hold on;
yline(v_max_sc,'r--','v_{max}');
ylabel('Velocity (rad/s)'); grid on; xlim([0 t_tot_sc+0.05]);

subplot(4,1,4); plot(t_sc,pk*180/pi,'m','LineWidth',1.2); hold on;
yline(360,'r--','360°');
xlabel('Time (s)'); ylabel('Position (deg)'); grid on; xlim([0 t_tot_sc+0.05]);

export_fig(fig6,'scurve_360.png');
fprintf('Done\n');

% =========================================================
% [7/8] + [8/8] sim_result.png + sim_current.png
% =========================================================

fprintf('[7/8] Running simulation (may take ~30s) ... ');

A_ss = [-R_m/L_m,          -K_e*N_total/L_m,  0;
         K_t*eta*N_total/J, -B/J,         0;
         0,                  1,                 0];
B_ss_s = [1/L_m;0;0];
C_ss   = [0,1,0;0,0,1]; D_ss = [0;0];

dt_sim = 0.0001;
sys_c  = ss(A_ss,B_ss_s,C_ss,D_ss);
sys_d  = c2d(sys_c,dt_sim,'zoh');
Ad=sys_d.A; Bd=sys_d.B;

Kp_vel=20.0364; Ki_vel=376.9228; Kd_vel=0.0325; N_vel=100;
Kp_pos=9.2670;  Ki_pos=10.50;   Kd_pos=0.08;   N_pos=20;
Kvff=3.034; Kaff=0.4450;
Vin_max_s=24.0; vref_max=7.3044; Kaw=0.5;
alpha_fb_s = (2*pi*100)*dt_sim / (1+(2*pi*100)*dt_sim);

t_end_s=t_tot_sc+2.0; t_sv=0:dt_sim:t_end_s; N_sv=length(t_sv);
xs=zeros(3,N_sv); trl=zeros(1,N_sv); vrl=zeros(1,N_sv);
arl=zeros(1,N_sv); Vl=zeros(1,N_sv); ofl=zeros(1,N_sv);
icl=false(1,N_sv);
iv=0; ip=0; dfv=0; dfp=0; epv=0; epp=0; awv=0; awp=0; ofp=0; tfp=0;

for k=1:N_sv-1
    tk=t_sv(k);
    if     tk<=T_sc(2); jn=+j_max_sc; a0=0;     v0=0;        p0=0;            t0=T_sc(1);
    elseif tk<=T_sc(3); jn=0;         a0=+a1_sc; v0=v1_sc;    p0=d1_sc;        t0=T_sc(2);
    elseif tk<=T_sc(4); jn=-j_max_sc; a0=+a1_sc; v0=v2_sc;    p0=d1_sc+d2_sc; t0=T_sc(3);
    elseif tk<=T_sc(5); jn=0;         a0=0;      v0=v_max_sc; p0=d_acc;        t0=T_sc(4);
    elseif tk<=T_sc(6); jn=-j_max_sc; a0=0;      v0=v_max_sc; p0=d_acc+d_cru; t0=T_sc(5);
    elseif tk<=T_sc(7); jn=0;         a0=-a1_sc; v0=v_max_sc-v1_sc; p0=d_acc+d_cru+d1_sc; t0=T_sc(6);
    elseif tk<=T_sc(8); jn=+j_max_sc; a0=-a1_sc; v0=v_max_sc-v1_sc-a1_sc*t_a_sc; p0=d_acc+d_cru+d1_sc+d2_sc; t0=T_sc(7);
    else;               jn=0;         a0=0;      v0=0;        p0=q_total;      t0=T_sc(8);
    end
    tau=tk-t0;
    tp=min(p0+v0*tau+0.5*a0*tau^2+jn*tau^3/6, q_total);
    tv_r=v0+a0*tau+0.5*jn*tau^2; ta_r=a0+jn*tau;
    trl(k)=tp; arl(k)=ta_r;

    of_k=alpha_fb_s*xs(2,k)+(1-alpha_fb_s)*ofp;
    tf_k=alpha_fb_s*xs(3,k)+(1-alpha_fb_s)*tfp;
    ofp=of_k; tfp=tf_k; ofl(k)=of_k;

    ep=tp-tf_k;
    dfp=(1-N_pos*dt_sim)*dfp+Kd_pos*N_pos*(ep-epp); epp=ep;
    ip=ip+ep*dt_sim+Kaw*awp*dt_sim;
    vc=max(-vref_max,min(vref_max,Kp_pos*ep+Ki_pos*ip+dfp+tv_r));
    awp=max(-vref_max,min(vref_max,Kp_pos*ep+Ki_pos*ip+dfp))-( Kp_pos*ep+Ki_pos*ip+dfp);
    vrl(k)=vc;

    ev=vc-of_k;
    dfv=(1-N_vel*dt_sim)*dfv+Kd_vel*N_vel*(ev-epv); epv=ev;
    iv=iv+ev*dt_sim+Kaw*awv*dt_sim;
    Vr=Kp_vel*ev+Ki_vel*iv+dfv+Kvff*vc+Kaff*ta_r;
    Vo=max(-Vin_max_s,min(Vin_max_s,Vr)); awv=Vo-Vr;

    in_k=xs(1,k); Vb=K_e*N_total*xs(2,k);
    Vmax_i=L_m*(i_max-in_k)/dt_sim+R_m*in_k+Vb;
    Vmin_i=L_m*(-i_max-in_k)/dt_sim+R_m*in_k+Vb;
    Vc_f=max(-Vin_max_s,min(Vin_max_s,max(Vmin_i,min(Vmax_i,Vo))));
    icl(k)=(abs(Vc_f)<abs(Vo)-0.01); Vl(k)=Vc_f;
    xs(:,k+1)=Ad*xs(:,k)+Bd*Vc_f;
end
trl(end)=q_total;
fprintf('Done\n');

fprintf('[7/8] sim_result.png ... ');
fig7=figure('Units','centimeters','Position',[1 1 26 16],'Color','white');
subplot(3,2,1);
plot(t_sv,trl*180/pi,'b--','LineWidth',1.2,'DisplayName','Reference'); hold on;
plot(t_sv,xs(3,:)*180/pi,'r','LineWidth',1.2,'DisplayName','Actual');
xline(T_sc(8),'k--','Traj end','LineWidth',1); yline(360,'m:','360°','LineWidth',1);
ylabel('\theta (deg)'); title('Position'); legend('Location','se'); grid on;

subplot(3,2,2);
plot(t_sv,(trl-xs(3,:))*180/pi,'r','LineWidth',1.2);
yline(+0.02*360,'b--','+2% band'); yline(-0.02*360,'b--','-2% band');
xline(T_sc(8),'k--'); ylabel('Error (deg)'); title('Position Error'); grid on;

subplot(3,2,3);
plot(t_sv,vrl,'b--','LineWidth',1.2,'DisplayName','v_{ref}'); hold on;
plot(t_sv,xs(2,:),'r','LineWidth',1.2,'DisplayName','\omega actual');
plot(t_sv,ofl,'g:','LineWidth',1.0,'DisplayName','\omega filtered');
yline(+vref_max,'k--','v_{max}'); yline(-vref_max,'k--');
ylabel('\omega (rad/s)'); title('Velocity'); legend('Location','best'); grid on;

subplot(3,2,4);
plot(t_sv,vrl-ofl,'r','LineWidth',1.2);
ylabel('Error (rad/s)'); title('Velocity Error'); grid on;

subplot(3,2,5);
plot(t_sv,Vl,'b','LineWidth',1.2);
yline(+Vin_max_s,'r--','+24V'); yline(-Vin_max_s,'r--','-24V');
xlabel('Time (s)'); ylabel('V_{in} (V)'); title('Motor Voltage'); grid on;

subplot(3,2,6);
plot(t_sv,xs(1,:),'b','LineWidth',1.2);
yline(+i_max,'r--','+10A','LineWidth',1.5); yline(-i_max,'r--','-10A','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Current (A)'); title('Motor Current'); grid on;

sgtitle('Cascade PID Simulation — S-Curve 360°','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig7,'sim_result.png');
fprintf('Done\n');

fprintf('[8/8] sim_current.png ... ');
i_cut_pct=sum(icl)/N_sv*100;
fig8=figure('Units','centimeters','Position',[1 1 20 12],'Color','white');
subplot(2,1,1);
plot(t_sv,xs(1,:),'b','LineWidth',1.2,'DisplayName','Current'); hold on;
area(t_sv,double(icl)*i_max,'FaceColor',[1 0.5 0.5],'FaceAlpha',0.4,'EdgeColor','none','DisplayName','Clamp zone');
yline(+i_max,'r--','+10A','LineWidth',1.5); yline(-i_max,'r--','-10A','LineWidth',1.5);
xline(T_sc(8),'k--','Traj end','LineWidth',1.5);
ylabel('Current (A)');
title(sprintf('Motor Current  (max=%.2f A,  clamp=%.1f%% of time)',max(abs(xs(1,:))),i_cut_pct));
legend('Location','best'); grid on;

subplot(2,1,2);
plot(t_sv,Vl,'b','LineWidth',1.0); hold on;
xline(T_sc(8),'k--','Traj end','LineWidth',1.5);
yline(+Vin_max_s,'r--','+24V'); yline(-Vin_max_s,'r--','-24V');
xlabel('Time (s)'); ylabel('V_{in} (V)'); title('Motor Voltage'); grid on;

sgtitle('Current Limit and Voltage Clamp','FontSize',12,'FontWeight','bold','Color','black');
export_fig(fig8,'sim_current.png');
fprintf('Done\n');

% =========================================================
fprintf('\n=== All 8 figures saved to: ===\n%s\n', img_path);