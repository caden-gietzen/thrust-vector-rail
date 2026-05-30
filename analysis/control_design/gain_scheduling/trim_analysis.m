% trim_analysis.m
%
% LQR gain computation for the 1D thrust-vector rail vehicle.
% Runs all three operating-point strategies in sequence:
%
%   Tier 1 — Fixed-gain LQR : single linearization at theta*=0, T*=T_r_nom
%   Tier 2 — Angle-scheduled: sweep theta* grid at nominal thrust
%   Tier 3 — Pair-scheduled  : sweep (theta*, T*) grid
%
% States:  x = [p; v; theta; T]      (m, m/s, rad, N)
% Inputs:  u = [theta_cmd_rad; T_cmd_N]
%
% Output:  gain_schedule_tier{1,2,3}.mat

%% Identified Parameters
m          = 0.4536;  % kg
tau_servo  = 0.0244;  % s
tau_thrust = 0.0781;  % s
mu_c       = 0.8158;  % N  — Coulomb friction (global symmetric fit; ~42% directional asymmetry)
T_max_phys = 3.5;     % N  — conservative motor limit (static sweep plateau ~4.17 N)
T_r_nom    = 2.574;   % N  — nominal thrust at 1700 µs (deg-4 polynomial; see thrust identification results)

%% Bryson LQR Weights
% Bryson's rule: Q_ii = 1/(max acceptable x_i)^2,  R_jj = 1/(max acceptable u_j)^2.
% Smaller max → larger weight → harder penalty → more aggressive correction.
%
% State weights (Q) — tolerable deviation before the controller works hard to correct it
% Input weights (R) — how large a command the LQR is allowed to issue
%   Smaller max → larger R → more conservative commands (softer, less likely to saturate)

% Tier 1 — Fixed-Gain LQR
t1_p_max        = 0.1;          % m    position error
t1_v_max        = 0.5;           % m/s  velocity
t1_theta_max    = deg2rad(60);   % rad  servo angle deviation from trim
t1_T_max_dev    = 0.25;          % N    thrust deviation from trim
t1_du_theta_max = deg2rad(180);  % rad  max servo command (hardware limit)
t1_dT_max       = 0.25;          % N    max thrust command deviation

Q1 = diag([1/t1_p_max^2, 1/t1_v_max^2, 1/t1_theta_max^2, 1/t1_T_max_dev^2]);
R1 = diag([1/t1_du_theta_max^2, 1/t1_dT_max^2]);

% Tier 2 — Angle-Scheduled LQR
t2_p_max        = 0.1;          % m
t2_v_max        = 0.5;           % m/s
t2_theta_max    = deg2rad(60);   % rad  — tightened so Q_theta ≈ Q_p; forces theta to zero at same rate as position
t2_T_max_dev    = 0.25;          % N
t2_du_theta_max = deg2rad(180);  % rad
t2_dT_max       = 0.25;          % N

Q2 = diag([1/t2_p_max^2, 1/t2_v_max^2, 1/t2_theta_max^2, 1/t2_T_max_dev^2]);
R2 = diag([1/t2_du_theta_max^2, 1/t2_dT_max^2]);

% Tier 3 — Pair-Scheduled LQR
t3_p_max        = 0.1;           % m
t3_v_max        = 0.5;           % m/s
t3_theta_max    = deg2rad(60);   % rad
t3_T_max_dev    = 0.25;          % N
t3_du_theta_max = deg2rad(180);  % rad
t3_dT_max       = 0.25;          % N

Q3 = diag([1/t3_p_max^2, 1/t3_v_max^2, 1/t3_theta_max^2, 1/t3_T_max_dev^2]);
R3 = diag([1/t3_du_theta_max^2, 1/t3_dT_max^2]);

%% Constant Linearized Matrices
% B is independent of trim point
B = [0,            0;
     0,            0;
     1/tau_servo,  0;
     0,            1/tau_thrust];

% A skeleton — only A(2,3) and A(2,4) change with trim
A_skel = [0,  1,            0,             0;
          0,  0,            0,             0;
          0,  0, -1/tau_servo,             0;
          0,  0,            0, -1/tau_thrust];

%% Nonlinear EOM (for trim residual check)
f = @(x, u) [
    x(2);
    (x(4)*sin(x(3)) - mu_c*sign(x(2))) / m;
    (u(1) - x(3)) / tau_servo;
    (u(2) - x(4)) / tau_thrust;
];

here = fileparts(mfilename('fullpath'));

%% ── TIER 1: Fixed-Gain LQR ───────────────────────────────────────────────

mat_path = fullfile(here, 'gain_schedule_tier1.mat');

% Trim: theta*=0, v*=0.  At theta*=0, sin(theta*)=0 so T has no lateral
% effect in the linearization — T* is chosen as the nominal operating thrust.
theta_star = 0;
v_star     = 0;
T_star     = T_r_nom;

x_star = [0; v_star; theta_star; T_star];
u_star = [theta_star; T_star];

A = A_skel;
A(2,3) = T_star * cos(theta_star) / m;   % = T_r_nom/m; servo is sole lateral actuator
A(2,4) = sin(theta_star) / m;            % = 0

K_fixed = lqr(A, B, Q1, R1);
poles   = sort(eig(A - B*K_fixed), 'ComparisonMethod', 'real');

fprintf('\nTier 1 — Fixed-Gain LQR\n');
fprintf('Trim: theta*=0 deg,  v*=0 m/s,  T*=%.4f N (1700 µs nominal)\n', T_star);
fprintf('K_servo:  [%s]\n', sprintf('%7.3f ', K_fixed(1,:)));
fprintf('K_thrust: [%s]\n', sprintf('%7.3f ', K_fixed(2,:)));
pole_str = '';
for k = 1:numel(poles)
    pk = poles(k);
    if abs(imag(pk)) > 1e-6
        pole_str = [pole_str, sprintf('%+.2f%+.2fi ', real(pk), imag(pk))]; %#ok<AGROW>
    else
        pole_str = [pole_str, sprintf('%+.2f ', real(pk))]; %#ok<AGROW>
    end
end
fprintf('Poles: %s\n', strtrim(pole_str));

save(mat_path, 'K_fixed', 'x_star', 'u_star', 'A', 'B', 'Q1', 'R1');
fprintf('\nGain schedule saved: %s\n', mat_path);

%% ── TIER 2: Angle-Scheduled LQR ─────────────────────────────────────────

mat_path = fullfile(here, 'gain_schedule_tier2.mat');

% Sweep theta* ∈ [-60°, -5°] ∪ [5°, 60°].
% T* = mu_c / sin(theta*): sign works out naturally (positive theta → positive thrust,
% negative theta → negative sin → negative T*, filtered below).
% Only |sign(v*)| matters for trim, not the magnitude — so no v loop is needed.
theta_trim_vec = deg2rad([-(60:-5:5), 5:5:60]);

gain_schedule = struct('theta', {}, 'T', {}, 'A', {}, 'K', {}, 'poles', {});
idx = 1;

for theta_star = theta_trim_vec
    T_star = mu_c / sin(theta_star);   % negative for theta*<0; filtered next line
    if T_star < 0 || T_star > T_max_phys; continue; end

    A = A_skel;
    A(2,3) = T_star * cos(theta_star) / m;
    A(2,4) = sin(theta_star) / m;

    K     = lqr(A, B, Q2, R2);
    poles = sort(eig(A - B*K), 'ComparisonMethod', 'real');

    gain_schedule(idx).theta = theta_star;
    gain_schedule(idx).T     = T_star;
    gain_schedule(idx).A     = A;
    gain_schedule(idx).K     = K;
    gain_schedule(idx).poles = poles;
    idx = idx + 1;
end

% theta=0 anchor: v*=0, T*=T_r_nom (nominal thrust, no friction to balance).
% Ensures interp1 converges to the correct no-friction equilibrium near zero
% rather than blending the opposite-sign trim points at ±5°.
A0 = A_skel;
A0(2,3) = T_r_nom / m;   % T_r_nom*cos(0)/m
A0(2,4) = 0;              % sin(0)/m
K0 = lqr(A0, B, Q2, R2);
gain_schedule(idx).theta = 0;
gain_schedule(idx).T     = T_r_nom;
gain_schedule(idx).A     = A0;
gain_schedule(idx).K     = K0;
gain_schedule(idx).poles = sort(eig(A0 - B*K0), 'ComparisonMethod', 'real');

[~, ord]  = sort([gain_schedule.theta]);
gs_sorted = gain_schedule(ord);

fprintf('\n%d gain-schedule entries (theta* sweep + theta=0 anchor).\n', numel(gs_sorted));
fprintf('LQR weights — Q diag: [%.1f  %.1f  %.1f  %.1f]  R diag: [%.1f  %.1f]\n\n', ...
    diag(Q2)', diag(R2)');

hdr = '  theta*    T*(N)   K_servo [Kp      Kv      Kth     KT  ]   K_thrust[Kp      Kv      Kth     KT  ]   poles';
sep = repmat('-', 1, length(hdr));
fprintf('%s\n%s\n', hdr, sep);
for i = 1:numel(gs_sorted)
    e  = gs_sorted(i);
    K1 = e.K(1,:);
    K2 = e.K(2,:);
    pole_str = '';
    for k = 1:numel(e.poles)
        pk = e.poles(k);
        if abs(imag(pk)) > 1e-6
            pole_str = [pole_str, sprintf('%+.2f%+.2fi ', real(pk), imag(pk))]; %#ok<AGROW>
        else
            pole_str = [pole_str, sprintf('%+.2f ', real(pk))]; %#ok<AGROW>
        end
    end
    fprintf('  %5.0f deg  %6.4f  [%7.3f %7.3f %7.3f %7.3f]   [%7.3f %7.3f %7.3f %7.3f]   %s\n', ...
        rad2deg(e.theta), e.T, K1(1), K1(2), K1(3), K1(4), K2(1), K2(2), K2(3), K2(4), strtrim(pole_str));
end
fprintf('\n');

theta_sched = [gs_sorted.theta];
K_sched     = cat(3, gs_sorted.K);

save(mat_path, 'theta_sched', 'K_sched');
fprintf('Gain schedule saved: %s\n  %d points over [%.0f, %.0f] deg\n', ...
    mat_path, numel(theta_sched), rad2deg(theta_sched(1)), rad2deg(theta_sched(end)));

%% ── TIER 3: Pair-Scheduled LQR ──────────────────────────────────────────

mat_path = fullfile(here, 'gain_schedule_tier3.mat');

% Sweep full (theta*, T*) grid — no friction equilibrium constraint.
% B_theta = (T/m)cos(theta) depends on both; capturing T variation corrects
% servo authority scaling that Tier 2 misses when T deviates from the trim value.
theta_grid = deg2rad(-60:5:60);                        % 25 points, includes 0°
T_grid     = [0.5, 1.0, 1.5, 2.0, 2.574, 3.0, 3.5];  % 7 points, includes T_r_nom

N_th = numel(theta_grid);
N_T  = numel(T_grid);
K_sched = zeros(2, 4, N_th, N_T);

for i = 1:N_th
    for j = 1:N_T
        theta_star = theta_grid(i);
        T_star     = T_grid(j);

        A = A_skel;
        A(2,3) = T_star * cos(theta_star) / m;
        A(2,4) = sin(theta_star) / m;

        K_sched(:,:,i,j) = lqr(A, B, Q3, R3);
    end
end

theta_sched = theta_grid;
T_sched     = T_grid;

fprintf('\nTier 3 — Pair-Scheduled LQR\n');
fprintf('%d theta points × %d T points = %d gain matrices.\n', N_th, N_T, N_th*N_T);
fprintf('theta range: [%.0f, %.0f] deg\n', rad2deg(theta_sched(1)), rad2deg(theta_sched(end)));
fprintf('T range:     [%.2f, %.2f] N\n', T_sched(1), T_sched(end));
fprintf('LQR weights — Q diag: [%.1f  %.1f  %.1f  %.1f]  R diag: [%.1f  %.1f]\n\n', ...
    diag(Q3)', diag(R3)');

% Print K at a representative slice: theta=0, all T levels
idx0 = find(theta_sched == 0);
if ~isempty(idx0)
    fprintf('K at theta*=0 deg across T* grid:\n');
    hdr = '  T*(N)   K_servo [Kp      Kv      Kth     KT  ]   K_thrust[Kp      Kv      Kth     KT  ]';
    fprintf('%s\n%s\n', hdr, repmat('-', 1, length(hdr)));
    for j = 1:N_T
        K1 = squeeze(K_sched(1,:,idx0,j));
        K2 = squeeze(K_sched(2,:,idx0,j));
        fprintf('  %5.3f   [%7.3f %7.3f %7.3f %7.3f]   [%7.3f %7.3f %7.3f %7.3f]\n', ...
            T_sched(j), K1(1), K1(2), K1(3), K1(4), K2(1), K2(2), K2(3), K2(4));
    end
    fprintf('\n');
end

save(mat_path, 'theta_sched', 'T_sched', 'K_sched');
fprintf('Gain schedule saved: %s\n  theta: %d pts over [%.0f, %.0f] deg\n  T: %d pts over [%.2f, %.2f] N\n', ...
    mat_path, N_th, rad2deg(theta_sched(1)), rad2deg(theta_sched(end)), N_T, T_sched(1), T_sched(end));
