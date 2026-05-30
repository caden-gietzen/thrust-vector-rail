% analyze_friction_sweep.m
%
% Residual-based friction identification from fixed-angle, stepped-ESC runs.
%
% Method:
%   For each run, the servo and thrust dynamic models are applied via lsim()
%   to predict the rail-direction force. Friction is the residual between that
%   predicted force and M*xddot measured from the encoder.
%
%       F_friction(t) = F_thrust(t) * sin(theta(t)) - M * xddot(t)
%
%   Stiction is identified from runs where the encoder shows no motion
%   (stiction_halt phase) after thrust is applied: the breakaway force
%   brackets the static friction threshold.
%
% Models used:
%   Servo:  G_servo(s) = 0.001556 / (1 + 0.0244*s) * exp(-0.0288*s)  [rad/µs]
%   Thrust: G_thrust(s) = 0.00414 / (1 + 0.0781*s) * exp(-0.0252*s)  [N/µs]
%   Source: experiments/servo_identification/results.md
%           experiments/thrust_identification/results.md
%
% Encoder calibration: 64810.4 counts/m (experiments/encoder_calibration/results.md)
%
% Input data:
%   data/raw/system_identification/friction_identification/friction_sweep_log/accepted/
%   (or candidate/ if not yet accepted)
%
% Output:
%   Friction model: F_friction = b*xdot + mu*sign(xdot)   [N·s/m, N]
%   Stiction:       F_stiction = F_thrust_breakaway * sin(theta_fixed)   [N]

close all; clear; clc

%% User-Editable Parameters

SAVE_FIGURES = true;

% --- Vehicle mass ---
M_kg       = 0.4536;    % 1 lb nominal — re-weigh before test and update
M_sigma_kg = 0.136;     % ±0.3 lb uncertainty

% --- Servo dynamic model (experiments/servo_identification/results.md) ---
K_servo   = 0.001556;   % rad/µs  (static gain magnitude; sign handled below)
tau_servo = 0.0244;     % s
L_servo   = 0.0288;     % s
u_neutral = 1431;       % µs (neutral servo command → 0° angle)

% --- Thrust dynamic model (experiments/thrust_identification/results.md) ---
% Global first-order-plus-delay, 1100–1950 µs
K_T       = 0.00414;    % N/µs
tau_T     = 0.0781;     % s
L_T       = 0.0252;     % s
u_T_min   = 1075;       % µs — below this the motor is in its dead zone

% --- Encoder calibration (experiments/encoder_calibration/results.md) ---
C_m = 64810.4;          % counts/m (measured; do not use nominal 60000)

% --- Signal processing ---
SG_WINDOW_S = 0.20;     % Savitzky-Golay window length (s); widen if xddot is too noisy
SG_POLY     = 3;        % SG polynomial order
V_MIN_MPS   = 0.01;     % exclude samples where |xdot| < this (sign ambiguity near zero)

% --- Model selection ---
SIMPLICITY_TOLERANCE = 0.10;  % accept ≤10% worse RMSE to prefer simpler model

%% Setup

scriptPath = mfilename('fullpath');
repoRoot   = findRepoRoot(fileparts(scriptPath));
plotDir    = getMirroredPlotDir(scriptPath);

figNum = 0;

%% Load CSV Files

% Try accepted first; fall back to candidate.
dataDir = getMirroredRawDataDir(scriptPath, 'accepted');
if ~isfolder(dataDir) || isempty(dir(fullfile(dataDir, '*.csv')))
    fprintf('No accepted data found. Trying candidate...\n');
    dataDir = getMirroredRawDataDir(scriptPath, 'candidate');
end

csvFiles = dir(fullfile(dataDir, '*.csv'));
if isempty(csvFiles)
    error('No CSV files found in %s', dataDir);
end

fprintf('Loading %d CSV file(s) from:\n  %s\n\n', numel(csvFiles), dataDir);

%% Parse Runs
% Each CSV corresponds to one segment (one ESC command level, one angle direction).
% Extract key metadata from the constant-value columns during the 'run' phase.

runs = struct();
nRuns = 0;

for k = 1:numel(csvFiles)
    fpath = fullfile(dataDir, csvFiles(k).name);
    T = readtable(fpath, 'TextType', 'string');

    if ~all(ismember({'t_s','phase','servo_us','esc_pwm_us','count','count_from_start'}, T.Properties.VariableNames))
        fprintf('Skipping %s — unexpected columns.\n', csvFiles(k).name);
        continue;
    end

    nRuns = nRuns + 1;
    runs(nRuns).filename = csvFiles(k).name;
    runs(nRuns).T        = T;

    % Metadata from run-phase rows
    runMask = strcmp(T.phase, 'run');
    if any(runMask)
        runs(nRuns).servo_us   = median(T.servo_us(runMask));
        runs(nRuns).esc_us     = median(T.esc_pwm_us(runMask));
    else
        runs(nRuns).servo_us   = median(T.servo_us);
        runs(nRuns).esc_us     = median(T.esc_pwm_us);
    end

    % Servo angle from command (using static gain)
    runs(nRuns).theta_cmd_deg = -(runs(nRuns).servo_us - u_neutral) * 0.091092;
    runs(nRuns).theta_cmd_rad = runs(nRuns).theta_cmd_deg * pi / 180;

    % Direction (sign of angle)
    runs(nRuns).angle_sign = sign(runs(nRuns).theta_cmd_deg);
    if runs(nRuns).angle_sign == 0; runs(nRuns).angle_sign = 1; end

    % Halt classification
    hasStiction  = any(strcmp(T.phase, 'stiction_halt'));
    hasEndstop   = any(strcmp(T.phase, 'endstop_halt'));
    hasTimeout   = any(strcmp(T.phase, 'timeout_end'));
    runs(nRuns).is_stiction = hasStiction && ~hasEndstop && ~hasTimeout;
    runs(nRuns).is_motion   = hasEndstop || hasTimeout || ...
                              (any(runMask) && max(abs(T.count_from_start(runMask))) >= 15);

    fprintf('Run %2d: servo=%4d us  esc=%4d us  angle=%+.1f deg  halt=%s\n', ...
        nRuns, round(runs(nRuns).servo_us), round(runs(nRuns).esc_us), ...
        runs(nRuns).theta_cmd_deg, ...
        haltLabel(hasStiction, hasEndstop, hasTimeout));
end

if nRuns == 0
    error('No valid runs loaded.');
end
fprintf('\nTotal runs: %d\n\n', nRuns);

%% Build Transfer Function Models

% Servo: G_servo(s) = K_servo / (tau_servo*s + 1) * exp(-L_servo*s)
% Note: the static gain is negative (increasing u → decreasing angle),
% but we use the deviation from neutral (u - u_neutral) as input, so the
% sign of K_servo here is positive and the angle polarity is handled by the
% static map in finalize_config on the Pico.
[n_servo_pad, d_servo_pad] = pade(L_servo, 3);
G_servo = tf(K_servo, [tau_servo, 1]) * tf(n_servo_pad, d_servo_pad);

% Thrust: G_thrust(s) = K_T / (tau_T*s + 1) * exp(-L_T*s)
[n_thrust_pad, d_thrust_pad] = pade(L_T, 3);
G_thrust = tf(K_T, [tau_T, 1]) * tf(n_thrust_pad, d_thrust_pad);

%% Stiction Analysis

fprintf('=== Stiction Analysis ===\n');
fprintf('%-8s  %-8s  %-14s  %-14s  %-10s\n', ...
    'angle', 'ESC(µs)', 'F_thrust_ss(N)', 'F_rail_ss(N)', 'motion?');

for k = 1:nRuns
    % Steady-state thrust at this ESC command (static approximation for stiction):
    % At stiction threshold, dynamics have had time to settle → use K_T * delta_u
    delta_u = max(0, runs(k).esc_us - u_T_min);
    F_thrust_ss = K_T * delta_u;
    F_rail_ss   = F_thrust_ss * sin(runs(k).theta_cmd_rad);

    runs(k).F_thrust_ss = F_thrust_ss;
    runs(k).F_rail_ss   = F_rail_ss;

    fprintf('%+7.1f°  %7d   %13.4f   %13.4f   %s\n', ...
        runs(k).theta_cmd_deg, round(runs(k).esc_us), ...
        F_thrust_ss, F_rail_ss, ...
        motionLabel(runs(k).is_motion));
end

% For each unique angle value, find the stiction bracket.
% Stiction breakaway is angle-dependent (F_rail = F_thrust * sin(theta)), so
% runs must be grouped by angle, not just by direction sign.
angle_degs_rounded = arrayfun(@(r) round(r.theta_cmd_deg), runs(1:nRuns));
unique_angles = unique(angle_degs_rounded);

for ua = unique_angles(:)'
    angleRuns = find(angle_degs_rounded == ua);
    if isempty(angleRuns); continue; end

    stiction_idxs = angleRuns(arrayfun(@(i) runs(i).is_stiction, angleRuns));
    motion_idxs   = angleRuns(arrayfun(@(i) runs(i).is_motion,   angleRuns));

    if ~isempty(stiction_idxs)
        [~, im] = max(arrayfun(@(i) runs(i).esc_us, stiction_idxs));
        r_max_stiction = runs(stiction_idxs(im));
        fprintf('\n  Angle %+.0f°: stiction confirmed at ESC ≤ %.0f µs\n', ...
            ua, r_max_stiction.esc_us);
        fprintf('    → F_thrust_ss ≤ %.4f N, F_rail ≤ %.4f N (below stiction threshold)\n', ...
            r_max_stiction.F_thrust_ss, r_max_stiction.F_rail_ss);
    end

    if ~isempty(motion_idxs)
        [~, im] = min(arrayfun(@(i) runs(i).esc_us, motion_idxs));
        r_min_motion = runs(motion_idxs(im));
        fprintf('  Angle %+.0f°: first motion at ESC = %.0f µs\n', ...
            ua, r_min_motion.esc_us);
        fprintf('    → F_stiction ≈ %.4f N (F_rail at breakaway)\n', ...
            r_min_motion.F_rail_ss);
    end
end
fprintf('\n');

%% Process Motion Runs

allXdot    = [];
allFfric   = [];
allEsc     = [];
allDirSign = [];   % +1 / -1 matching runs(k).angle_sign

for k = 1:nRuns
    if ~runs(k).is_motion
        continue;
    end

    T = runs(k).T;

    % Use run phase data only
    mask = strcmp(T.phase, 'run') | strcmp(T.phase, 'endstop_halt') | ...
           strcmp(T.phase, 'timeout_end');
    if sum(mask) < 20
        fprintf('Run %d (%s): too few motion samples, skipping.\n', k, runs(k).filename);
        continue;
    end

    t_s   = T.t_s(mask);
    cnt   = T.count_from_start(mask);
    sv_us = T.servo_us(mask);
    ec_us = T.esc_pwm_us(mask);

    % Convert to position (m)
    x_m = double(cnt) / C_m;

    % Uniform time vector for lsim and SG filter
    % (raw timestamps have jitter; lsim requires exactly uniform spacing)
    dt = median(diff(t_s));
    if dt <= 0 || dt > 0.1
        fprintf('Run %d: unexpected dt = %.4f s, skipping.\n', k, dt);
        continue;
    end
    N_samp  = length(t_s);
    t_lsim  = (0:N_samp-1)' * dt;   % uniform grid anchored at 0

    % Savitzky-Golay filter then differentiate
    win_n = round(SG_WINDOW_S / dt);
    if mod(win_n, 2) == 0; win_n = win_n + 1; end
    win_n = max(win_n, SG_POLY + 2);

    x_filt = sgolayfilt(x_m, SG_POLY, win_n);
    xdot   = gradient(x_filt, dt);      % m/s
    xddot  = gradient(xdot,   dt);      % m/s²

    % Predict servo angle via lsim
    u_servo_dev = double(sv_us) - u_neutral;   % deviation from neutral (µs)
    theta_rad   = lsim(G_servo, u_servo_dev, t_lsim);

    % Predict thrust via lsim (zero below dead zone)
    u_thrust_dev = max(0, double(ec_us) - u_T_min);
    F_thrust     = lsim(G_thrust, u_thrust_dev, t_lsim);
    F_thrust     = max(0, F_thrust);   % clamp negative Padé artefacts

    % Rail-direction force and friction residual
    F_rail = F_thrust .* sin(theta_rad);
    F_fric = F_rail - M_kg .* xddot;

    % Trim transient edges and near-zero velocity
    n_trim = round(0.5 / dt);
    idx_valid = (abs(xdot) >= V_MIN_MPS);
    idx_valid(1:min(n_trim, end)) = false;
    idx_valid(max(1,end-n_trim):end) = false;

    if sum(idx_valid) < 5
        fprintf('Run %d: too few valid samples after trimming, skipping.\n', k);
        continue;
    end

    runs(k).xdot_valid  = xdot(idx_valid);
    runs(k).Ffric_valid = F_fric(idx_valid);
    runs(k).x_m         = x_m;
    runs(k).xdot        = xdot;
    runs(k).xddot       = xddot;
    runs(k).F_rail      = F_rail;
    runs(k).F_fric      = F_fric;
    runs(k).t_s         = t_s;
    runs(k).theta_rad   = theta_rad;
    runs(k).F_thrust    = F_thrust;
    runs(k).idx_valid   = idx_valid;

    allXdot    = [allXdot;    xdot(idx_valid)];
    allFfric   = [allFfric;   F_fric(idx_valid)];
    allEsc     = [allEsc;     ones(sum(idx_valid),1) * runs(k).esc_us];
    allDirSign = [allDirSign; ones(sum(idx_valid),1) * runs(k).angle_sign];
end

if isempty(allXdot)
    error('No valid motion data found across all runs. Check data quality and thresholds.');
end

fprintf('Pooled %d valid samples from motion runs.\n\n', numel(allXdot));

%% Fit Friction Model Candidates

sgn_v = sign(allXdot);

% 1. Viscous only: F = b * v
b_visc  = allXdot \ allFfric;
Fpred_visc = b_visc * allXdot;
rmse_visc  = rms(Fpred_visc - allFfric);

% 2. Coulomb only: F = mu * sign(v)
mu_coul     = mean(allFfric .* sgn_v);
Fpred_coul  = mu_coul * sgn_v;
rmse_coul   = rms(Fpred_coul - allFfric);

% 3. Viscous + Coulomb: F = b*v + mu*sign(v)
A_vc         = [allXdot, sgn_v];
params_vc    = A_vc \ allFfric;
b_vc         = params_vc(1);
mu_vc        = params_vc(2);
Fpred_vc     = b_vc * allXdot + mu_vc * sgn_v;
rmse_vc      = rms(Fpred_vc - allFfric);

%% Model Selection

rmse_best_1param = min(rmse_visc, rmse_coul);

if rmse_vc < (1 - SIMPLICITY_TOLERANCE) * rmse_best_1param
    selected_model = 'viscous_coulomb';
    b_sel  = b_vc;
    mu_sel = mu_vc;
    rmse_sel = rmse_vc;
elseif rmse_visc <= rmse_coul
    selected_model = 'viscous';
    b_sel  = b_visc;
    mu_sel = 0;
    rmse_sel = rmse_visc;
else
    selected_model = 'coulomb';
    b_sel  = 0;
    mu_sel = mu_coul;
    rmse_sel = rmse_coul;
end

%% Mass Uncertainty Propagation

M_lo = M_kg - M_sigma_kg;
M_hi = M_kg + M_sigma_kg;

% Refit viscous+Coulomb at mass bounds
F_fric_lo = allFfric + (M_kg - M_lo) .* arrayfun(@(i) 0, 1:numel(allFfric))';
% Actually: F_fric depends on M through M*xddot, so we need the xddot values.
% Recompute F_fric at mass bounds by collecting all xddot values.
allXddot = [];
for k = 1:nRuns
    if isfield(runs(k), 'idx_valid') && ~isempty(runs(k).idx_valid)
        allXddot = [allXddot; runs(k).xddot(runs(k).idx_valid)];
    end
end

Ffric_lo = allFfric + (M_kg - M_lo) * allXddot;   % F_fric at lower mass
Ffric_hi = allFfric + (M_kg - M_hi) * allXddot;   % F_fric at upper mass

p_lo = [allXdot, sgn_v] \ Ffric_lo;
p_hi = [allXdot, sgn_v] \ Ffric_hi;

b_lo  = min(p_lo(1), p_hi(1));  b_hi  = max(p_lo(1), p_hi(1));
mu_lo = min(p_lo(2), p_hi(2));  mu_hi = max(p_lo(2), p_hi(2));

%% Print Summary

fprintf('============================================================\n');
fprintf('Friction Model Comparison\n');
fprintf('  %-20s  RMSE = %.4f N\n', 'viscous', rmse_visc);
fprintf('  %-20s  RMSE = %.4f N\n', 'coulomb', rmse_coul);
fprintf('  %-20s  RMSE = %.4f N\n', 'viscous_coulomb', rmse_vc);
fprintf('\n');
fprintf('=== Selected model: %s ===\n', selected_model);
if b_sel ~= 0
    fprintf('  b   = %.4f N·s/m   (viscous coefficient)\n', b_sel);
end
if mu_sel ~= 0
    fprintf('  mu  = %.4f N       (Coulomb friction force)\n', mu_sel);
end
fprintf('  RMSE = %.4f N\n', rmse_sel);
fprintf('\n');
fprintf('=== Mass sensitivity (M = %.3f ± %.3f kg) ===\n', M_kg, M_sigma_kg);
fprintf('  b  ∈ [%.4f, %.4f] N·s/m\n', b_lo, b_hi);
fprintf('  mu ∈ [%.4f, %.4f] N\n', mu_lo, mu_hi);
fprintf('============================================================\n');

%% Directional Asymmetry
% Note: for this test design, positive angle_sign → xdot > 0 and
% negative angle_sign → xdot < 0, so direction and motion sign are
% confounded.  The comparison still reveals whether friction magnitude
% differs between the two traversal directions.

dir_signs  = [1, -1];
dir_labels = {'positive dir', 'negative dir'};
b_dir      = nan(1,2);
mu_dir     = nan(1,2);
rmse_dir_v = nan(1,2);
n_dir      = zeros(1,2);
asym_flag  = false;

fprintf('=== Directional Asymmetry ===\n');
for di = 1:2
    mask_d   = (allDirSign == dir_signs(di));
    n_dir(di) = sum(mask_d);
    if n_dir(di) < 10
        fprintf('  %s: too few samples (%d), skipping.\n', dir_labels{di}, n_dir(di));
        continue;
    end
    xd = allXdot(mask_d);
    fd = allFfric(mask_d);
    p  = [xd, sign(xd)] \ fd;
    b_dir(di)      = p(1);
    mu_dir(di)     = p(2);
    rmse_dir_v(di) = rms(fd - (p(1)*xd + p(2)*sign(xd)));
    fprintf('  %s:  b = %.4f N*s/m,  mu = %.4f N,  RMSE = %.4f N  (n=%d)\n', ...
        dir_labels{di}, p(1), p(2), rmse_dir_v(di), n_dir(di));
end

if ~any(isnan(b_dir))
    b_asym  = abs(b_dir(1)  - b_dir(2))  / mean(abs(b_dir))  * 100;
    mu_asym = abs(mu_dir(1) - mu_dir(2)) / mean(abs(mu_dir)) * 100;
    fprintf('  Delta_b = %.1f%%,  Delta_mu = %.1f%%\n', b_asym, mu_asym);
    asym_flag = max(b_asym, mu_asym) > 20;
    if asym_flag
        fprintf('  *** Significant asymmetry (>20%%) — direction-dependent model recommended. ***\n');
    else
        fprintf('  Asymmetry within 20%% — pooled model is adequate.\n');
    end
end
fprintf('\n');

%% Figure 1 — Representative Run

% Find the motion run with the highest ESC command for the +20° direction.
motionIdx = find(arrayfun(@(r) r.is_motion && isfield(r,'xdot') && r.angle_sign > 0, runs));
if ~isempty(motionIdx)
    [~, im] = max(arrayfun(@(i) runs(i).esc_us, motionIdx));
    rr = runs(motionIdx(im));

    figNum = figNum + 1;
    figure(figNum);
    tiledlayout(3,1,'TileSpacing','compact');

    nexttile;
    plot(rr.t_s, rr.x_m * 1000);
    ylabel('Position (mm)'); grid on;
    title(sprintf('Representative run: ESC = %d µs, angle = %+.0f°', ...
        round(rr.esc_us), rr.theta_cmd_deg));

    nexttile;
    plot(rr.t_s, rr.xdot);
    ylabel('Velocity (m/s)'); grid on;

    nexttile;
    plot(rr.t_s, rr.xddot);
    ylabel('Accel (m/s²)'); xlabel('Time (s)'); grid on;
end

%% Figure 2 — Stiction Boundary

figNum = figNum + 1;
figure(figNum);
hold on;

esc_vals    = arrayfun(@(r) r.esc_us,     runs(1:nRuns));
is_motion_v = arrayfun(@(r) r.is_motion,  runs(1:nRuns));
angle_signs = arrayfun(@(r) r.angle_sign, runs(1:nRuns));

for k = 1:nRuns
    clr = [0.2 0.6 0.2];   % green = motion
    if ~runs(k).is_motion
        clr = [0.8 0.2 0.2];  % red = stiction
    end
    bar(k, runs(k).esc_us, 'FaceColor', clr);
end

set(gca, 'XTick', 1:nRuns, 'XTickLabel', ...
    arrayfun(@(k) sprintf('%+.0f°/%d', runs(k).theta_cmd_deg, round(runs(k).esc_us)), ...
    1:nRuns, 'UniformOutput', false), 'XTickLabelRotation', 45);
ylabel('ESC command (µs)');
title('Halt classification by run (green = motion, red = stiction)');
grid on;

%% Figure 3 — Friction Scatter vs Velocity

figNum = figNum + 1;
figure(figNum);
hold on;

esc_all_unique = unique(allEsc);
cmap = parula(numel(esc_all_unique));

for ek = 1:numel(esc_all_unique)
    mask = allEsc == esc_all_unique(ek);
    scatter(allXdot(mask), allFfric(mask), 6, cmap(ek,:), 'filled', 'DisplayName', ...
        sprintf('%d µs', round(esc_all_unique(ek))));
end

% Overlay model curves
v_range = linspace(min(allXdot), max(allXdot), 200);
sgn_range = sign(v_range);
plot(v_range, b_visc  * v_range,                    'b--',  'LineWidth', 1.5, 'DisplayName', ...
    sprintf('Viscous (b=%.4f)', b_visc));
plot(v_range, mu_coul * sgn_range,                  'r--',  'LineWidth', 1.5, 'DisplayName', ...
    sprintf('Coulomb (μ=%.4f)', mu_coul));
plot(v_range, b_vc * v_range + mu_vc * sgn_range,   'k-',   'LineWidth', 2,   'DisplayName', ...
    sprintf('Viscous+Coulomb (b=%.4f, μ=%.4f)', b_vc, mu_vc));

xlabel('Velocity (m/s)');
ylabel('F_{friction} (N)');
title('Friction residual vs velocity — all motion runs');
legend('show', 'Location', 'best');
grid on;
colorbar; clim([min(esc_all_unique), max(esc_all_unique)]);

%% Figure 4 — Residual Histogram

switch selected_model
    case 'viscous_coulomb'; resid = allFfric - (b_vc * allXdot + mu_vc * sgn_v);
    case 'viscous';         resid = allFfric - b_visc * allXdot;
    case 'coulomb';         resid = allFfric - mu_coul * sgn_v;
end

figNum = figNum + 1;
figure(figNum);
histogram(resid, 40);
xlabel('Residual (N)');
ylabel('Count');
title(sprintf('Residual histogram — %s model  (RMSE = %.4f N)', selected_model, rmse_sel));
grid on;

%% Figure 5 — Per-Direction Friction Scatter

figNum = figNum + 1;
figure(figNum);
tiledlayout(1,2,'TileSpacing','compact');

for di = 1:2
    sgn_dir = [1, -1];
    sgn_d = sgn_dir(di);

    nexttile;
    hold on;

    for k = 1:nRuns
        if ~runs(k).is_motion || runs(k).angle_sign ~= sgn_d; continue; end
        if ~isfield(runs(k), 'xdot_valid'); continue; end
        scatter(runs(k).xdot_valid, runs(k).Ffric_valid, 6, 'filled', 'DisplayName', ...
            sprintf('%d µs', round(runs(k).esc_us)));
    end

    plot(v_range, b_vc * v_range + mu_vc * sgn_range, 'k-', 'LineWidth', 2, 'DisplayName', 'pooled fit');
    xlabel('Velocity (m/s)'); ylabel('F_{friction} (N)');
    title(sprintf('Direction: %s', dir_labels{di}));
    legend('show', 'Location', 'best');
    grid on;
end

%% Figure 6 — Mass Sensitivity Bands

figNum = figNum + 1;
figure(figNum);
hold on;

scatter(allXdot, allFfric, 4, [0.7 0.7 0.7], 'filled');
plot(v_range, b_vc * v_range + mu_vc * sgn_range, 'k-',  'LineWidth', 2, 'DisplayName', 'nominal fit');
plot(v_range, b_lo * v_range + mu_lo * sgn_range, 'b--', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('M - σ (%.3f kg)', M_lo));
plot(v_range, b_hi * v_range + mu_hi * sgn_range, 'r--', 'LineWidth', 1.5, 'DisplayName', ...
    sprintf('M + σ (%.3f kg)', M_hi));

xlabel('Velocity (m/s)'); ylabel('F_{friction} (N)');
title('Mass sensitivity: friction model bands over ±0.3 lb uncertainty');
legend('show', 'Location', 'best');
grid on;

%% Figure 7 — Per-Direction Model Comparison

if ~any(isnan(b_dir))
    figNum = figNum + 1;
    figure(figNum);
    hold on;

    dir_colors = {[0 0.9 1], [1 0.65 0]};   % cyan (+), orange (-)
    v_sweep = linspace(min(allXdot), max(allXdot), 300);
    sgn_sweep = sign(v_sweep);

    for di = 1:2
        mask_d = (allDirSign == dir_signs(di));
        scatter(allXdot(mask_d), allFfric(mask_d), 4, dir_colors{di}, 'filled', ...
            'DisplayName', sprintf('%s data', dir_labels{di}));
    end
    for di = 1:2
        plot(v_sweep, b_dir(di)*v_sweep + mu_dir(di)*sgn_sweep, '-', ...
            'Color', dir_colors{di}, 'LineWidth', 2, ...
            'DisplayName', sprintf('%s fit  b=%.4f  mu=%.4f', dir_labels{di}, b_dir(di), mu_dir(di)));
    end
    plot(v_sweep, b_vc*v_sweep + mu_vc*sgn_sweep, '--', ...
        'Color', [0.75 0.75 0.75], 'LineWidth', 1.5, 'DisplayName', 'pooled fit');

    xlabel('Velocity (m/s)'); ylabel('F_{friction} (N)');
    title('Per-direction friction model comparison');
    legend('show', 'Location', 'best');
    grid on;
end

%% Save Figures

if SAVE_FIGURES
    saveAllFiguresIfEnabled(true, plotDir);
end

%% Write Analysis Report

reportPath = fullfile(plotDir, 'analyze_friction_sweep.report.md');
writeAnalysisReport(reportPath, nRuns, selected_model, b_sel, mu_sel, rmse_sel, ...
    b_dir, mu_dir, rmse_dir_v, n_dir, dir_labels, ...
    b_lo, b_hi, mu_lo, mu_hi, M_kg, M_sigma_kg, dataDir, asym_flag);
fprintf('Report written to:\n  %s\n', reportPath);

%% Local Functions

function label = motionLabel(isMotion)
    if isMotion
        label = 'YES';
    else
        label = 'no (stiction)';
    end
end

function label = haltLabel(hasStiction, hasEndstop, hasTimeout)
    if hasStiction
        label = 'stiction';
    elseif hasEndstop
        label = 'endstop';
    elseif hasTimeout
        label = 'timeout';
    else
        label = 'unclear';
    end
end

function writeAnalysisReport(path, nRuns, model, b, mu, rmse, ...
    b_dir, mu_dir, rmse_dir, n_dir, dir_labels, ...
    b_lo, b_hi, mu_lo, mu_hi, M, M_sig, dataDir, asym_flag)

    fid = fopen(path, 'w');
    fprintf(fid, '# Friction Sweep Analysis Report\n\n');
    fprintf(fid, '**Generated:** %s  \n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '**Dataset:** `%s`  \n', strrep(dataDir, '\', '/'));
    fprintf(fid, '**Runs loaded:** %d\n\n', nRuns);

    fprintf(fid, '## Selected Friction Model: `%s`\n\n', model);
    if b ~= 0
        fprintf(fid, '- **b** (viscous)  = %.4f N·s/m\n', b);
    end
    if mu ~= 0
        fprintf(fid, '- **mu** (Coulomb) = %.4f N\n', mu);
    end
    fprintf(fid, '- **RMSE** = %.4f N\n\n', rmse);

    fprintf(fid, '## Mass Sensitivity (M = %.3f +/- %.3f kg)\n\n', M, M_sig);
    fprintf(fid, '- b  in [%.4f, %.4f] N·s/m\n', b_lo, b_hi);
    fprintf(fid, '- mu in [%.4f, %.4f] N\n\n', mu_lo, mu_hi);

    fprintf(fid, '## Directional Asymmetry\n\n');
    fprintf(fid, '| Direction | b (N·s/m) | mu (N) | RMSE (N) | n |\n');
    fprintf(fid, '|-----------|-----------|--------|----------|---|\n');
    for di = 1:2
        if isnan(b_dir(di))
            fprintf(fid, '| %s | — | — | — | %d |\n', dir_labels{di}, n_dir(di));
        else
            fprintf(fid, '| %s | %.4f | %.4f | %.4f | %d |\n', ...
                dir_labels{di}, b_dir(di), mu_dir(di), rmse_dir(di), n_dir(di));
        end
    end
    fprintf(fid, '\n');

    if ~any(isnan(b_dir))
        b_asym  = abs(b_dir(1) - b_dir(2)) / mean(abs(b_dir))  * 100;
        mu_asym = abs(mu_dir(1) - mu_dir(2)) / mean(abs(mu_dir)) * 100;
        fprintf(fid, '**Delta_b = %.1f%%**, **Delta_mu = %.1f%%**\n\n', b_asym, mu_asym);
        if asym_flag
            fprintf(fid, '**WARNING: Significant directional asymmetry (>20%%).**\n');
            fprintf(fid, 'Direction-dependent friction model recommended.\n\n');
        else
            fprintf(fid, 'Asymmetry within 20%% — pooled model adequate.\n\n');
        end
        fprintf(fid, '_Note: for this test design, positive servo direction produces positive_\n');
        fprintf(fid, '_velocity and negative servo direction produces negative velocity, so_\n');
        fprintf(fid, '_servo direction and motion direction are confounded in this comparison._\n\n');
    end

    fclose(fid);
end
