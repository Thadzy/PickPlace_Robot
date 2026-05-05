function theta_ref = scurve_test(t)

theta_total = 2 * pi;
w_max       = 8.976;
a_max       = 9.364;
t_j         = 0.1;

t_a = w_max / a_max;
t_v = (theta_total - w_max * (t_a + t_j)) / w_max;
if t_v < 0, t_v = 0; end

t1 = t_j;
t2 = t1 + t_a;
t3 = t2 + t_j;
t4 = t3 + t_v;
t7 = t4 + t_j + t_a + t_j;

% คำนวณ waypoints ของ accel half
w1 = 0.5 * a_max * t_j;
p1 = (1/6) * (a_max/t_j) * t_j^3;
p2 = p1 + w1*t_a + 0.5*a_max*t_a^2;
w2 = w1 + a_max*t_a;
p3 = p2 + w2*t_j - (1/6)*(a_max/t_j)*t_j^3;
p4 = p3 + w_max*t_v;

if t <= 0
    theta_ref = 0;

elseif t >= t7
    theta_ref = theta_total;

elseif t <= t1
    theta_ref = (1/6)*(a_max/t_j)*t^3;

elseif t <= t2
    dt        = t - t1;
    theta_ref = p1 + w1*dt + 0.5*a_max*dt^2;

elseif t <= t3
    dt        = t - t2;
    theta_ref = p2 + w2*dt - (1/6)*(a_max/t_j)*dt^3;

elseif t <= t4
    dt        = t - t3;
    theta_ref = p3 + w_max*dt;

else
    % Decel half: symmetry กับ accel half
    % mirror t กลับจาก t7
    tm = t7 - t;

    if tm <= 0
        pos_mirror = 0;
    elseif tm <= t1
        pos_mirror = (1/6)*(a_max/t_j)*tm^3;
    elseif tm <= t2
        dt         = tm - t1;
        pos_mirror = p1 + w1*dt + 0.5*a_max*dt^2;
    elseif tm <= t3
        dt         = tm - t2;
        pos_mirror = p2 + w2*dt - (1/6)*(a_max/t_j)*dt^3;
    else
        dt         = tm - t3;
        pos_mirror = p3 + w_max*dt;
    end

    theta_ref = theta_total - pos_mirror;
end