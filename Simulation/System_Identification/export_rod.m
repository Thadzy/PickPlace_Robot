% =========================================================
% export_rod_figures.m
% สร้างกราฟ 3 อัน สำหรับใส่รายงาน
% 1. rod_no_shaping.png   -- Rod swing 198 deg (ไม่มี shaping)
% 2. rod_zv_compare.png   -- เปรียบเทียบ before/after ZV shaping
% 3. rod_best_compromise.png -- Best compromise simulation
% =========================================================

clc; clear; close all;

% =========================================================
% SECTION 1: Fixed Parameters
% =========================================================
R_m=1.45336; L_m=0.00144802; N_total=70; K_e=0.04165; K_t=0.04065;
B_damp=0.19279; J=0.72762; eta=0.83607; i_max=10.0; Vin_max=24.0;

m_rod=0.16408; L_rod=0.10; g=9.81; r_arm=0.5;
l_cm=L_rod/2; I_pivot=(1/3)*m_rod*L_rod^2;
wn_rod=sqrt(m_rod*g*l_cm/I_pivot); zeta_rod=0.05;

Kp_vel=20.0364; Ki_vel=376.9228; Kd_vel=0.0325; N_vel=100;
Kp_pos=9.2670;  Ki_pos=10.5;    Kd_pos=0.08;   N_pos=20;
Kvff=3.034; Kaff=0.4450; Kaw=0.5;
dt=0.0001; fc_fb=100; wc_fb=2*pi*fc_fb;

% =========================================================
% SECTION 2: สามชุดพารามิเตอร์
% =========================================================
% Case 1: ไม่มี Shaping (original S-curve)
c1.v_max=7.3044; c1.a_max=27.4912; c1.j_max=1400.40;
c1.use_shaping=false; c1.label='No Input Shaping';

% Case 2: ZV Shaping พารามิเตอร์เดิม
c2.v_max=7.3044; c2.a_max=27.4912; c2.j_max=1400.40;
c2.use_shaping=true;
c2.wn_sh=wn_rod; c2.zeta_sh=0.05;
c2.label='ZV Input Shaping (Original Params)';

% Case 3: Best Compromise จาก NSGA-II
c3.v_max=6.985; c3.a_max=9.433; c3.j_max=1068.4;
c3.use_shaping=true;
c3.wn_sh=12.374; c3.zeta_sh=0.0404;
c3.label='NSGA-II Best Compromise';

cases = {c1, c2, c3};

% =========================================================
% SECTION 3: Simulate ทุก Case
% =========================================================
results = cell(1,3);
for ci = 1:3
    results{ci} = run_simulation(cases{ci}, ...
        R_m,L_m,N_total,K_e,K_t,B_damp,J,eta,i_max,Vin_max, ...
        m_rod,l_cm,I_pivot,wn_rod,zeta_rod,r_arm, ...
        Kp_vel,Ki_vel,Kd_vel,N_vel,Kp_pos,Ki_pos,Kd_pos,N_pos, ...
        Kvff,Kaff,Kaw,dt,wc_fb);
    fprintf('Case %d (%s): Max phi = %.2f deg\n', ...
        ci, cases{ci}.label, results{ci}.phi_max_deg);
end

phi_threshold_deg = 0.57;

% =========================================================
% FIGURE 1: Rod Swing ไม่มี Shaping
% =========================================================
r1 = results{1};
fig1 = figure('Position',[50 50 1000 500],'Color','white');
set(fig1,'DefaultTextColor','black','DefaultAxesColor','white',...
    'DefaultAxesXColor','black','DefaultAxesYColor','black',...
    'DefaultAxesGridColor',[0.85 0.85 0.85]);

subplot(1,2,1);
plot(r1.t_sim, r1.x_arm(3,:)*180/pi,'r','LineWidth',1.2,...
    'DisplayName','Actual');
hold on;
plot(r1.t_sim, r1.theta_r_log*180/pi,'b--','LineWidth',1.2,...
    'DisplayName','Reference');
xline(r1.T8,'k--','Traj end','LineWidth',1.2);
yline(360,'m:','360°','LineWidth',1.0);
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Arm Position');
lg=legend('Location','southeast');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
grid on;

subplot(1,2,2);
plot(r1.t_sim, r1.phi_log*180/pi,'b','LineWidth',1.2);
hold on;
yline(+phi_threshold_deg,'r--','LineWidth',1.5,...
    'DisplayName',sprintf('±%.2f° threshold',phi_threshold_deg));
yline(-phi_threshold_deg,'r--','LineWidth',1.5,'HandleVisibility','off');
yline(+180,'k:','LineWidth',1.2,'DisplayName','+180° limit');
yline(-180,'k:','LineWidth',1.2,'HandleVisibility','off');
xline(r1.T8,'k--','Traj end','LineWidth',1.2);
xlabel('Time (s)'); ylabel('\phi (deg)');
title(sprintf('Rod Swing — Max = %.1f°', r1.phi_max_deg));
lg=legend('Location','northeast');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
grid on;

sgtitle('Case 1: No Input Shaping', ...
    'FontSize',12,'FontWeight','bold','Color','black');
fix_colors(fig1);
exportgraphics(fig1,'rod_no_shaping.png','Resolution',200,...
    'BackgroundColor','white');
fprintf('Saved: rod_no_shaping.png\n');

% =========================================================
% FIGURE 2: เปรียบเทียบ Before/After ZV Shaping
% =========================================================
r2 = results{2};
fig2 = figure('Position',[50 50 1000 700],'Color','white');
set(fig2,'DefaultTextColor','black','DefaultAxesColor','white',...
    'DefaultAxesXColor','black','DefaultAxesYColor','black',...
    'DefaultAxesGridColor',[0.85 0.85 0.85]);

% Rod Swing เปรียบเทียบ
subplot(2,1,1);
plot(r1.t_sim, r1.phi_log*180/pi,'r','LineWidth',1.2,...
    'DisplayName',sprintf('No Shaping (max=%.1f°)',r1.phi_max_deg));
hold on;
plot(r2.t_sim, r2.phi_log*180/pi,'b','LineWidth',1.2,...
    'DisplayName',sprintf('ZV Shaping (max=%.1f°)',r2.phi_max_deg));
yline(+phi_threshold_deg,'k--','LineWidth',1.5,...
    'DisplayName',sprintf('±%.2f° threshold',phi_threshold_deg));
yline(-phi_threshold_deg,'k--','LineWidth',1.5,'HandleVisibility','off');
yline(+180,'m:','LineWidth',1.0,'DisplayName','Physical limit ±180°');
yline(-180,'m:','LineWidth',1.0,'HandleVisibility','off');
xline(r1.T8,'k--','Traj end','LineWidth',1.0);
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Rod Swing: No Shaping vs ZV Shaping');
lg=legend('Location','northeast','NumColumns',2);
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
grid on;

% Trajectory Reference เปรียบเทียบ
subplot(2,1,2);
plot(r1.t_sim, r1.traj_v_log,'r','LineWidth',1.2,...
    'DisplayName','No Shaping');
hold on;
plot(r2.t_sim, r2.traj_v_log,'b','LineWidth',1.2,...
    'DisplayName','ZV Shaped');
xlabel('Time (s)'); ylabel('\omega_{ref} (rad/s)');
title('Velocity Reference: Effect of ZV Shaping');
lg=legend('Location','best');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
grid on;

sgtitle('Case 2: ZV Input Shaping — Before vs After', ...
    'FontSize',12,'FontWeight','bold','Color','black');
fix_colors(fig2);
exportgraphics(fig2,'rod_zv_compare.png','Resolution',200,...
    'BackgroundColor','white');
fprintf('Saved: rod_zv_compare.png\n');

% =========================================================
% FIGURE 3: Best Compromise Simulation
% =========================================================
r3 = results{3};
fig3 = figure('Position',[50 50 1200 800],'Color','white');
set(fig3,'DefaultTextColor','black','DefaultAxesColor','white',...
    'DefaultAxesXColor','black','DefaultAxesYColor','black',...
    'DefaultAxesGridColor',[0.85 0.85 0.85]);

% Arm Position
subplot(2,3,1);
plot(r3.t_sim, r3.theta_r_log*180/pi,'b--','LineWidth',1.2,...
    'DisplayName','Reference');
hold on;
plot(r3.t_sim, r3.x_arm(3,:)*180/pi,'r','LineWidth',1.2,...
    'DisplayName','Actual');
xline(r3.T8,'k--','Traj end');
yline(360,'m:','360°');
ylabel('\theta (deg)'); title('Arm Position'); grid on;
lg=legend('Location','southeast');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);

% Position Error
subplot(2,3,2);
plot(r3.t_sim,(r3.theta_r_log-r3.x_arm(3,:))*180/pi,'r','LineWidth',1.2);
yline(+0.01*360,'b--','+1% band');
yline(-0.01*360,'b--','-1% band');
xline(r3.T8,'k--');
ylabel('Error (deg)'); title('Position Error'); grid on;

% Rod Swing
subplot(2,3,3);
plot(r3.t_sim, r3.phi_log*180/pi,'b','LineWidth',1.2);
hold on;
yline(+phi_threshold_deg,'r--','LineWidth',1.5,...
    'DisplayName',sprintf('±%.2f° threshold',phi_threshold_deg));
yline(-phi_threshold_deg,'r--','LineWidth',1.5,'HandleVisibility','off');
xline(r3.T8,'k--','Traj end');
xlabel('Time (s)'); ylabel('\phi (deg)');
title(sprintf('Rod Swing — Max = %.1f°', r3.phi_max_deg));
lg=legend('Location','northeast');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
grid on;

% Velocity
subplot(2,3,4);
plot(r3.t_sim, r3.vref_log,'b--','LineWidth',1.2,'DisplayName','v_{ref}');
hold on;
plot(r3.t_sim, r3.x_arm(2,:),'r','LineWidth',1.2,'DisplayName','\omega actual');
ylabel('\omega (rad/s)'); title('Arm Velocity'); grid on;
lg=legend('Location','best');
set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);

% Motor Voltage
subplot(2,3,5);
plot(r3.t_sim, r3.Vin_log,'b','LineWidth',1.2);
yline(+Vin_max,'r--','+24V'); yline(-Vin_max,'r--','-24V');
xlabel('Time (s)'); ylabel('V_{in} (V)'); title('Motor Voltage'); grid on;

% Motor Current
subplot(2,3,6);
plot(r3.t_sim, r3.x_arm(1,:),'b','LineWidth',1.2);
yline(+i_max,'r--','+10A'); yline(-i_max,'r--','-10A');
xlabel('Time (s)'); ylabel('Current (A)');
title(sprintf('Motor Current (max=%.2f A)',max(abs(r3.x_arm(1,:)))));
grid on;

sgtitle(sprintf('Case 3: NSGA-II Best Compromise — Rod Swing=%.1f°, Cycle=%.1f s',...
    r3.phi_max_deg, r3.cycle_time), ...
    'FontSize',11,'FontWeight','bold','Color','black');
fix_colors(fig3);
exportgraphics(fig3,'rod_best_compromise.png','Resolution',200,...
    'BackgroundColor','white');
fprintf('Saved: rod_best_compromise.png\n');

fprintf('\nAll figures exported.\n');

% =========================================================
% Helper: run_simulation
% =========================================================
function res = run_simulation(c, ...
    R_m,L_m,N_total,K_e,K_t,B_damp,J,eta,i_max,Vin_max, ...
    m_rod,l_cm,I_pivot,wn_rod,zeta_rod,r_arm, ...
    Kp_vel,Ki_vel,Kd_vel,N_vel,Kp_pos,Ki_pos,Kd_pos,N_pos, ...
    Kvff,Kaff,Kaw,dt,wc_fb)

    v_max=c.v_max; a_max=c.a_max; j_max=c.j_max;
    q_total=2*pi;

    % S-curve timing
    t_j=a_max/j_max; t_a=v_max/a_max-t_j;
    if t_a<0; t_j=sqrt(v_max/j_max); t_a=0; a_max=j_max*t_j; end

    v1=0.5*j_max*t_j^2; a1=j_max*t_j;
    d1=j_max*t_j^3/6;
    d2=v1*t_a+0.5*a1*t_a^2;
    v2=v1+a1*t_a;
    d3=v2*t_j+0.5*a1*t_j^2-j_max*t_j^3/6;
    d_accel=d1+d2+d3; d_cruise=q_total-2*d_accel;
    t_v=d_cruise/v_max; t_total=4*t_j+2*t_a+t_v;

    T=zeros(1,9);
    T(2)=t_j; T(3)=T(2)+t_a; T(4)=T(3)+t_j; T(5)=T(4)+t_v;
    T(6)=T(5)+t_j; T(7)=T(6)+t_a; T(8)=T(7)+t_j;

    t_end=t_total+3.0; t_sim=0:dt:t_end; N_sim=length(t_sim);

    % Discrete plant
    A_c=[-R_m/L_m,-K_e*N_total/L_m,0;
          K_t*eta*N_total/J,-B_damp/J,0; 0,1,0];
    B_c=[1/L_m;0;0];
    sys_d=c2d(ss(A_c,B_c,[0,1,0;0,0,1],[0;0]),dt,'zoh');
    Ad=sys_d.A; Bd=sys_d.B;

    % Rod discrete
    A_rod=[0,1;-wn_rod^2,-2*zeta_rod*wn_rod];
    B_rod=[0;-m_rod*l_cm*r_arm/I_pivot];
    sys_rod_d=c2d(ss(A_rod,B_rod,eye(2),zeros(2,1)),dt,'zoh');
    Ad_rod=sys_rod_d.A; Bd_rod=sys_rod_d.B;

    % Raw trajectory
    traj_p_raw=zeros(1,N_sim); traj_v_raw=zeros(1,N_sim);
    traj_a_raw=zeros(1,N_sim);
    for k=1:N_sim
        tk=t_sim(k);
        if     tk<=T(2); j_n=+j_max;a0=0;   v0=0;     p0=0;                       t0=T(1);
        elseif tk<=T(3); j_n=0;     a0=+a1; v0=v1;    p0=d1;                      t0=T(2);
        elseif tk<=T(4); j_n=-j_max;a0=+a1; v0=v2;    p0=d1+d2;                   t0=T(3);
        elseif tk<=T(5); j_n=0;     a0=0;   v0=v_max; p0=d_accel;                 t0=T(4);
        elseif tk<=T(6); j_n=-j_max;a0=0;   v0=v_max; p0=d_accel+d_cruise;        t0=T(5);
        elseif tk<=T(7); j_n=0;     a0=-a1; v0=v_max-v1; p0=d_accel+d_cruise+d1;  t0=T(6);
        elseif tk<=T(8); j_n=+j_max;a0=-a1; v0=v_max-v1-a1*t_a; p0=d_accel+d_cruise+d1+d2; t0=T(7);
        else;            j_n=0;     a0=0;   v0=0;     p0=q_total;                 t0=T(8);
        end
        tau=tk-t0;
        traj_p_raw(k)=min(p0+v0*tau+0.5*a0*tau^2+j_n*tau^3/6,q_total);
        traj_v_raw(k)=v0+a0*tau+0.5*j_n*tau^2;
        traj_a_raw(k)=a0+j_n*tau;
    end

    % Apply ZV shaping (ถ้ามี)
    if c.use_shaping
        omega_d_sh=c.wn_sh*sqrt(1-c.zeta_sh^2);
        K_zv=exp(-c.zeta_sh*pi/sqrt(1-c.zeta_sh^2));
        t2_zv=pi/omega_d_sh;
        A1_zv=1/(1+K_zv); A2_zv=K_zv/(1+K_zv);
        delay_steps=round(t2_zv/dt);
        traj_p_s=zeros(1,N_sim); traj_v_s=zeros(1,N_sim);
        traj_a_s=zeros(1,N_sim);
        for k=1:N_sim
            kd=k-delay_steps;
            if kd>0
                traj_p_s(k)=A1_zv*traj_p_raw(k)+A2_zv*traj_p_raw(kd);
                traj_v_s(k)=A1_zv*traj_v_raw(k)+A2_zv*traj_v_raw(kd);
                traj_a_s(k)=A1_zv*traj_a_raw(k)+A2_zv*traj_a_raw(kd);
            else
                traj_p_s(k)=A1_zv*traj_p_raw(k);
                traj_v_s(k)=A1_zv*traj_v_raw(k);
                traj_a_s(k)=A1_zv*traj_a_raw(k);
            end
        end
    else
        traj_p_s=traj_p_raw; traj_v_s=traj_v_raw; traj_a_s=traj_a_raw;
    end

    % Simulation loop
    x_arm=zeros(3,N_sim); x_rod=zeros(2,N_sim);
    phi_log=zeros(1,N_sim); theta_r_log=zeros(1,N_sim);
    vref_log=zeros(1,N_sim); Vin_log=zeros(1,N_sim);
    traj_v_log=zeros(1,N_sim);

    int_vel=0;int_pos=0;dfilt_vel=0;dfilt_pos=0;
    err_vel_prev=0;err_pos_prev=0;aw_vel=0;aw_pos=0;
    omega_f_prev=0;theta_f_prev=0;alpha_prev=0;
    alpha_fb=wc_fb*dt/(1+wc_fb*dt);

    for k=1:N_sim-1
        traj_p_k=traj_p_s(k); traj_v_k=traj_v_s(k); traj_a_k=traj_a_s(k);
        theta_r_log(k)=traj_p_k; traj_v_log(k)=traj_v_k;

        alpha_now=(x_arm(2,k)-alpha_prev)/dt; alpha_prev=x_arm(2,k);
        x_rod(:,k+1)=Ad_rod*x_rod(:,k)+Bd_rod*alpha_now;
        phi_log(k)=x_rod(1,k);

        omega_f=alpha_fb*x_arm(2,k)+(1-alpha_fb)*omega_f_prev;
        theta_f=alpha_fb*x_arm(3,k)+(1-alpha_fb)*theta_f_prev;
        omega_f_prev=omega_f; theta_f_prev=theta_f;

        err_pos=traj_p_k-theta_f;
        dfilt_pos=(1-N_pos*dt)*dfilt_pos+Kd_pos*N_pos*(err_pos-err_pos_prev);
        err_pos_prev=err_pos;
        int_pos=int_pos+err_pos*dt+Kaw*aw_pos*dt;
        vref_pid=Kp_pos*err_pos+Ki_pos*int_pos+dfilt_pos;
        vref_cmd=max(-v_max,min(v_max,vref_pid+traj_v_k));
        aw_pos=max(-v_max,min(v_max,vref_pid))-vref_pid;
        vref_log(k)=vref_cmd;

        err_vel=vref_cmd-omega_f;
        dfilt_vel=(1-N_vel*dt)*dfilt_vel+Kd_vel*N_vel*(err_vel-err_vel_prev);
        err_vel_prev=err_vel;
        int_vel=int_vel+err_vel*dt+Kaw*aw_vel*dt;
        V_pid=Kp_vel*err_vel+Ki_vel*int_vel+dfilt_vel;
        Vff=Kvff*vref_cmd+Kaff*traj_a_k;
        Vin_raw=V_pid+Vff;
        Vin=max(-Vin_max,min(Vin_max,Vin_raw));
        aw_vel=Vin-Vin_raw;

        i_now=x_arm(1,k); omega_now=x_arm(2,k);
        V_bemf=K_e*N_total*omega_now;
        Vin_max_i=L_m*(i_max-i_now)/dt+R_m*i_now+V_bemf;
        Vin_min_i=L_m*(-i_max-i_now)/dt+R_m*i_now+V_bemf;
        Vin_c=max(-Vin_max,min(Vin_max,max(Vin_min_i,min(Vin_max_i,Vin))));
        Vin_log(k)=Vin_c;

        x_arm(:,k+1)=Ad*x_arm(:,k)+Bd*Vin_c;
    end
    phi_log(end)=x_rod(1,end);
    theta_r_log(end)=q_total;

    % Metrics
    phi_max_deg=max(abs(phi_log))*180/pi;
    idx_end=find(t_sim>=T(8),1);

    % Cycle time (rod settle)
    phi_thr=0.57*pi/180;
    phi_after=phi_log(idx_end:end); t_after=t_sim(idx_end:end);
    rod_settled=false; t_rod_ready=NaN; consec=0;
    min_c=round(0.1/dt); t_wait_start=NaN;
    for k=1:length(phi_after)
        if abs(phi_after(k))<=phi_thr
            consec=consec+1;
            if ~rod_settled; t_wait_start=t_after(k); rod_settled=true; end
            if consec>=min_c; t_rod_ready=t_wait_start; break; end
        else; consec=0; rod_settled=false; end
    end
    if ~isnan(t_rod_ready)
        t_wait=t_rod_ready-T(8);
        cycle_time=4*(t_total+t_wait+2.0+t_total+t_wait+2.0);
    else
        cycle_time=Inf;
    end

    res.t_sim=t_sim; res.x_arm=x_arm; res.phi_log=phi_log;
    res.theta_r_log=theta_r_log; res.vref_log=vref_log;
    res.Vin_log=Vin_log; res.traj_v_log=traj_v_log;
    res.phi_max_deg=phi_max_deg; res.T8=T(8);
    res.t_total=t_total; res.cycle_time=cycle_time;
end

% =========================================================
% Helper: fix_colors
% =========================================================
function fix_colors(fig)
    ax_all=findall(fig,'Type','axes');
    for ax=ax_all'% =========================================================
% export_pareto_front.m
% Export Pareto Front figure จาก x_pareto และ f_pareto
% ที่ได้จาก gamultiobj
% =========================================================

% ตรวจสอบว่ามีข้อมูลใน workspace
if ~exist('f_pareto','var') || ~exist('x_pareto','var')
    error('ไม่พบ f_pareto หรือ x_pareto ใน workspace กรุณารัน optimize_trajectory.m ก่อน');
end

rod_swing_pareto  = f_pareto(:, 1);
cycle_time_pareto = f_pareto(:, 2);

% Normalize สำหรับ color
rod_norm   = (rod_swing_pareto  - min(rod_swing_pareto))  / ...
             (max(rod_swing_pareto)  - min(rod_swing_pareto) + 1e-9);
cycle_norm = (cycle_time_pareto - min(cycle_time_pareto)) / ...
             (max(cycle_time_pareto) - min(cycle_time_pareto) + 1e-9);
score      = 0.5*rod_norm + 0.5*cycle_norm;

[~, idx_best]      = min(score);
[~, idx_min_rod]   = min(rod_swing_pareto);
[~, idx_min_cycle] = min(cycle_time_pareto);

% =========================================================
% Plot
% =========================================================
fig = figure('Position',[50 50 900 600],'Color','white');
set(fig,'DefaultTextColor','black','DefaultAxesColor','white',...
    'DefaultAxesXColor','black','DefaultAxesYColor','black',...
    'DefaultAxesGridColor',[0.85 0.85 0.85]);

scatter(cycle_time_pareto, rod_swing_pareto, 80, score, ...
    'filled','DisplayName','Pareto solutions');
colormap(parula);
cb = colorbar;
cb.Label.String = 'Normalized score (lower = better)';
cb.Color = 'black';
hold on;

% Key solutions
scatter(f_pareto(idx_best,     2), f_pareto(idx_best,     1), ...
    180, 'r', 'filled', 'DisplayName', ...
    sprintf('Best compromise (%.1f°, %.1f s)', ...
    f_pareto(idx_best,1), f_pareto(idx_best,2)));

scatter(f_pareto(idx_min_rod,  2), f_pareto(idx_min_rod,  1), ...
    180, 'g', 'filled', 'DisplayName', ...
    sprintf('Min rod swing (%.1f°, %.1f s)', ...
    f_pareto(idx_min_rod,1), f_pareto(idx_min_rod,2)));

scatter(f_pareto(idx_min_cycle,2), f_pareto(idx_min_cycle,1), ...
    180, 'b', 'filled', 'DisplayName', ...
    sprintf('Min cycle time (%.1f°, %.1f s)', ...
    f_pareto(idx_min_cycle,1), f_pareto(idx_min_cycle,2)));

% Constraint lines
xline(35, 'r--', 'LineWidth', 1.8, ...
    'DisplayName', 'Cycle limit = 35 s');
yline(180, 'k--', 'LineWidth', 1.2, ...
    'DisplayName', 'Rod limit = 180°');

% Feasible region shading
patch([30 35 35 30], [0 0 180 180], ...
    [0.9 1 0.9], 'FaceAlpha', 0.15, ...
    'EdgeColor', 'none', 'DisplayName', 'Feasible region');

xlabel('Cycle Time (s)', 'FontSize', 12);
ylabel('Max Rod Swing (deg)', 'FontSize', 12);
title('Pareto Front: Rod Swing vs Cycle Time (NSGA-II)', ...
    'FontSize', 13, 'FontWeight', 'bold');

lg = legend('Location', 'northeast', 'NumColumns', 1);
set(lg, 'Color', 'white', 'TextColor', 'black', ...
    'EdgeColor', [0.8 0.8 0.8], 'FontSize', 9);

grid on;
xlim([min(cycle_time_pareto)-0.5, max(cycle_time_pareto)+0.5]);
ylim([0, max(rod_swing_pareto)+5]);

% Fix colors
ax = gca;
set(ax, 'XColor','black','YColor','black','FontSize',11);
set(get(ax,'Title'), 'Color','black');
set(get(ax,'XLabel'),'Color','black');
set(get(ax,'YLabel'),'Color','black');
set(findall(fig,'Type','text'),'Color','black');

% Export
exportgraphics(fig, 'pareto_front.png', ...
    'Resolution', 200, 'BackgroundColor', 'white');
fprintf('Saved: pareto_front.png\n');
        lg=get(ax,'Legend');
        if ~isempty(lg)
            set(lg,'Color','white','TextColor','black','EdgeColor',[0.8 0.8 0.8]);
        end
        set(get(ax,'Title'),'Color','black');
        set(get(ax,'XLabel'),'Color','black');
        set(get(ax,'YLabel'),'Color','black');
        set(ax,'XColor','black','YColor','black');
    end
    set(findall(fig,'Type','text'),'Color','black');
end