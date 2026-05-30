% analyze_lqr_tier1.m
%
% Closed-loop performance analysis for the fixed-gain LQR tier 1 controller.
%
% Loads the latest CSV from candidate/ (or accepted/ if promoted), then plots:
%   Fig 1 — Position tracking: p vs p_ref, and position error over time
%   Fig 2 — Servo command: theta_cmd_rad with saturation limits, and servo_us
%   Fig 3 — State estimators: theta_hat_rad (with bang-bang ss level), T_hat_N
%   Fig 4 — Phase portrait: error vs velocity (coloured by time)
%   Fig 5 — Per-waypoint error traces aligned to each transition
%
% Per-waypoint statistics (RMS error, convergence time, saturation fraction)
% are printed to the console and written to analyze_lqr_tier1.report.md.
%
% Controller design:
%   K_FIXED = [10.472, 3.268, 0.704, 0; 0, 0, 0, 0.414]
%   Dominant poles ≈ −6.3 ± 1.8i rad/s  (bandwidth ≈ 1 Hz)
%   See: analysis/control_design/gain_scheduling/gain_schedule_tier1.mat

close all; clear; clc

%% User-Editable Parameters

SAVE_FIGURES = true;

% Controller / hardware constants (must match firmware lqr_tier1.py)
THETA_SAT_RAD   = pi / 3;       % ±60° servo saturation limit
THETA_HAT_SS    = 0.7272;       % |theta_hat| at bang-bang steady state (rad)
                                 %   = (b_srv/(1+a_srv)) * THETA_SAT_RAD
                                 %   where a_srv = 1 - dt/tau = 0.1803, b_srv = 0.8197
T_NOM_N         = 2.574;        % nominal thrust at 1700 µs (N)

% Convergence threshold for "settling time" computation
CONVERGE_M      = 0.010;        % ±10 mm band around reference

%% Setup

scriptPath = mfilename('fullpath');
plotDir    = getMirroredPlotDir(scriptPath);

figNum = 0;

%% Load CSV

dataDir = getMirroredRawDataDir(scriptPath, 'accepted');
if ~isfolder(dataDir) || isempty(dir(fullfile(dataDir, '*.csv')))
    fprintf('No accepted data — falling back to candidate/\n');
    dataDir = getMirroredRawDataDir(scriptPath, 'candidate');
end

csvFiles = dir(fullfile(dataDir, '*.csv'));
if isempty(csvFiles)
    error('No CSV files found in %s', dataDir);
end

latestFile = selectLatestIndexedCsv(csvFiles);
fpath      = fullfile(dataDir, latestFile);

fprintf('Loading: %s\n\n', fpath);
T = readtable(fpath, 'TextType', 'string');

required = {'t_s','p_m','p_ref_m','v_mps','theta_hat_rad','T_hat_N', ...
            'theta_cmd_rad','T_cmd_N','servo_us','esc_us'};
missing = required(~ismember(required, T.Properties.VariableNames));
if ~isempty(missing)
    error('CSV missing columns: %s', strjoin(missing, ', '));
end

t           = T.t_s;
p           = T.p_m;
p_ref       = T.p_ref_m;
v           = T.v_mps;
theta_hat   = T.theta_hat_rad;
T_hat       = T.T_hat_N;
theta_cmd   = T.theta_cmd_rad;
T_cmd       = T.T_cmd_N;
servo_us    = T.servo_us;
esc_us      = T.esc_us;
err         = p - p_ref;
N           = numel(t);

%% Detect Waypoint Transitions

ref_change = [false; abs(diff(p_ref)) > 1e-6];
trans_idx  = find(ref_change);                % row indices where p_ref steps
trans_t    = t(trans_idx);

% Build per-waypoint segments: each segment runs from one transition to the next
seg_starts = [1; trans_idx];
seg_ends   = [trans_idx - 1; N];
n_segs     = numel(seg_starts);

%% Per-Waypoint Statistics

fprintf('%-4s  %-8s  %-8s  %-10s  %-14s  %-12s\n', ...
    'Seg', 'p_ref(m)', 'Dur(s)', 'RMS err(mm)', 'Settle(s)', 'Sat frac(%)');
fprintf('%s\n', repmat('-', 1, 70));

stats = struct();
for k = 1:n_segs
    si = seg_starts(k);
    ei = seg_ends(k);
    mask = si:ei;

    ref_k   = p_ref(si);                          % reference for this segment
    err_k   = err(mask);
    t_k     = t(mask);
    sat_k   = abs(theta_cmd(mask)) >= THETA_SAT_RAD * 0.99;

    rms_err_mm = rms(err_k) * 1e3;
    sat_frac   = mean(sat_k) * 100;
    dur_s      = t_k(end) - t_k(1);

    % Settling time: first time |error| stays within CONVERGE_M for rest of segment
    settled_idx = find(abs(err_k) < CONVERGE_M);
    settle_s = NaN;
    if ~isempty(settled_idx)
        % require it stays converged for at least 0.5 s
        min_samples = round(0.5 / median(diff(t)));
        for si2 = 1:numel(settled_idx)
            run_end = min(settled_idx(si2) + min_samples - 1, numel(err_k));
            if all(abs(err_k(settled_idx(si2):run_end)) < CONVERGE_M)
                settle_s = t_k(settled_idx(si2)) - t_k(1);
                break;
            end
        end
    end

    stats(k).ref_m     = ref_k;
    stats(k).rms_mm    = rms_err_mm;
    stats(k).settle_s  = settle_s;
    stats(k).sat_frac  = sat_frac;
    stats(k).dur_s     = dur_s;
    stats(k).mask      = mask;
    stats(k).t_k       = t_k;
    stats(k).err_k     = err_k;

    if isnan(settle_s)
        settle_str = '  never';
    else
        settle_str = sprintf('%6.2f s', settle_s);
    end

    fprintf('  %d   %+7.3f   %6.2f    %7.2f       %s        %6.1f\n', ...
        k, ref_k, dur_s, rms_err_mm, settle_str, sat_frac);
end

total_rms_mm    = rms(err) * 1e3;
total_sat_frac  = mean(abs(theta_cmd) >= THETA_SAT_RAD * 0.99) * 100;
fprintf('%s\n', repmat('-', 1, 70));
fprintf('TOTAL: RMS err = %.2f mm,  servo saturation = %.1f%% of all samples\n\n', ...
    total_rms_mm, total_sat_frac);

%% Colours (dark-mode palette — avoid black/navy)

C_REF   = [0.00, 0.90, 1.00];   % cyan     — reference signal
C_ACT   = [1.00, 0.65, 0.00];   % orange   — actual / measured
C_ERR   = [0.45, 1.00, 0.45];   % lime     — error
C_SAT   = [1.00, 0.40, 0.40];   % salmon   — saturation limit lines
C_TH    = [0.00, 0.90, 1.00];   % cyan     — theta signals
C_T     = [1.00, 0.65, 0.00];   % orange   — thrust signals
C_TRANS = [0.80, 0.80, 0.80];   % light grey — transition markers

%% Figure 1 — Position Tracking

figNum = figNum + 1;
figure(figNum);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(t, p_ref * 1e3, '-',  'Color', C_REF, 'LineWidth', 1.5, 'DisplayName', 'p_{ref}');
plot(t, p    * 1e3, '-',  'Color', C_ACT, 'LineWidth', 1.2, 'DisplayName', 'p');
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('Position (mm)');
legend('Location', 'best');
title(sprintf('LQR Tier 1 — Position Tracking   [%s]', latestFile), 'Interpreter', 'none');
grid on;

nexttile;
hold on;
plot(t, err * 1e3, '-', 'Color', C_ERR, 'LineWidth', 1.2, 'DisplayName', 'error');
yline( CONVERGE_M * 1e3, '--', 'Color', C_SAT, 'Alpha', 0.8, 'HandleVisibility', 'off');
yline(-CONVERGE_M * 1e3, '--', 'Color', C_SAT, 'Alpha', 0.8, 'HandleVisibility', 'off');
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('Error (mm)');
xlabel('Time (s)');
yline(0, 'Color', [0.6 0.6 0.6], 'Alpha', 0.5, 'HandleVisibility', 'off');
legend('Location', 'best');
grid on;

%% Figure 2 — Servo Command

figNum = figNum + 1;
figure(figNum);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(t, theta_cmd, '-', 'Color', C_ACT, 'LineWidth', 1.2, 'DisplayName', '\theta_{cmd}');
yline( THETA_SAT_RAD, '--', 'Color', C_SAT, 'Alpha', 0.9, 'DisplayName', '±60° limit');
yline(-THETA_SAT_RAD, '--', 'Color', C_SAT, 'Alpha', 0.9, 'HandleVisibility', 'off');
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('\theta_{cmd} (rad)');
ylim([-THETA_SAT_RAD * 1.2, THETA_SAT_RAD * 1.2]);
legend('Location', 'best');
title('Servo Command vs Saturation Limits');
grid on;

nexttile;
hold on;
plot(t, servo_us, '-', 'Color', C_REF, 'LineWidth', 1.2, 'DisplayName', 'servo_us');
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('Servo command (µs)');
xlabel('Time (s)');
legend('Location', 'best');
grid on;

%% Figure 3 — State Estimators

figNum = figNum + 1;
figure(figNum);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(t, theta_hat, '-', 'Color', C_TH, 'LineWidth', 1.2, 'DisplayName', '\theta_{hat}');
yline( THETA_HAT_SS, '--', 'Color', C_SAT, 'Alpha', 0.8, ...
    'DisplayName', sprintf('bang-bang ss = ±%.4f rad', THETA_HAT_SS));
yline(-THETA_HAT_SS, '--', 'Color', C_SAT, 'Alpha', 0.8, 'HandleVisibility', 'off');
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('\theta_{hat} (rad)');
legend('Location', 'best');
title('State Estimator Signals');
grid on;

nexttile;
hold on;
plot(t, T_hat, '-',  'Color', C_T,   'LineWidth', 1.2, 'DisplayName', 'T_{hat}');
plot(t, T_cmd, '--', 'Color', C_REF, 'LineWidth', 1.0, 'DisplayName', 'T_{cmd}');
yline(T_NOM_N, ':', 'Color', C_TRANS, 'Alpha', 0.7, 'DisplayName', sprintf('T_{nom} = %.3f N', T_NOM_N));
for k = 1:numel(trans_t)
    xline(trans_t(k), '--', 'Color', C_TRANS, 'Alpha', 0.6, 'HandleVisibility', 'off');
end
ylabel('Thrust (N)');
xlabel('Time (s)');
legend('Location', 'best');
grid on;

%% Figure 4 — Phase Portrait (error vs velocity)

figNum = figNum + 1;
figure(figNum);
hold on;

cmap = parula(N);
for i = 1:N-1
    plot(err(i:i+1) * 1e3, v(i:i+1), '-', 'Color', cmap(i,:), 'LineWidth', 1.2);
end

% Mark waypoint transition points
for k = 1:numel(trans_idx)
    scatter(err(trans_idx(k)) * 1e3, v(trans_idx(k)), 60, C_TRANS, 'o', 'filled', ...
        'HandleVisibility', 'off');
end

xlabel('Position error (mm)');
ylabel('Velocity (m/s)');
title('Phase portrait: error vs velocity (colour = time, early→late)');
colormap(parula); cb = colorbar;
cb.Label.String = 'Time (s)';
clim([t(1), t(end)]);
cb.Ticks       = linspace(t(1), t(end), 5);
cb.TickLabels  = arrayfun(@(x) sprintf('%.0f s', x), cb.Ticks, 'UniformOutput', false);
xline(0, 'Color', [0.6 0.6 0.6], 'Alpha', 0.5);
yline(0, 'Color', [0.6 0.6 0.6], 'Alpha', 0.5);
grid on;

%% Figure 5 — Per-Waypoint Error Traces

figNum = figNum + 1;
figure(figNum);
tl = tiledlayout(n_segs, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Per-Waypoint Error Traces');

seg_colors = {C_ERR, C_ACT, C_REF, [0.75 0.50 1.00]};   % cycle through 4 colours

for k = 1:n_segs
    nexttile;
    hold on;
    t_rel = stats(k).t_k - stats(k).t_k(1);
    clr   = seg_colors{mod(k-1, numel(seg_colors)) + 1};
    plot(t_rel, stats(k).err_k * 1e3, '-', 'Color', clr, 'LineWidth', 1.2);
    yline( CONVERGE_M * 1e3, '--', 'Color', C_SAT, 'Alpha', 0.7);
    yline(-CONVERGE_M * 1e3, '--', 'Color', C_SAT, 'Alpha', 0.7);
    yline(0, 'Color', [0.6 0.6 0.6], 'Alpha', 0.4);

    rms_str = sprintf('RMS = %.1f mm', stats(k).rms_mm);
    if isnan(stats(k).settle_s)
        settle_str = 'settle: never';
    else
        settle_str = sprintf('settle: %.2f s', stats(k).settle_s);
    end
    sat_str = sprintf('sat: %.0f%%', stats(k).sat_frac);
    title(sprintf('Seg %d  p_{ref}=%+.0f mm    %s    %s    %s', ...
        k, stats(k).ref_m * 1e3, rms_str, settle_str, sat_str));
    ylabel('err (mm)');
    if k == n_segs
        xlabel('Time within segment (s)');
    end
    grid on;
end

%% Save Figures

if SAVE_FIGURES
    saveAllFiguresIfEnabled(true, plotDir);
end

%% Write Report

reportPath = fullfile(plotDir, 'analyze_lqr_tier1.report.md');
writeReport(reportPath, latestFile, dataDir, t, n_segs, stats, ...
    total_rms_mm, total_sat_frac, THETA_HAT_SS, THETA_SAT_RAD, CONVERGE_M);
fprintf('Report written to:\n  %s\n', reportPath);

%% Local Functions

function writeReport(path, filename, dataDir, t, n_segs, stats, ...
        total_rms_mm, total_sat_frac, theta_hat_ss, theta_sat, converge_m)

    dur_s = t(end) - t(1);
    fid = fopen(path, 'w');

    fprintf(fid, '# LQR Tier 1 Analysis Report\n\n');
    fprintf(fid, '**Generated:** %s  \n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '**Dataset:** `%s`  \n', strrep(dataDir, '\', '/'));
    fprintf(fid, '**File:** `%s`  \n', filename);
    fprintf(fid, '**Duration:** %.1f s  (%d samples)\n\n', dur_s, numel(t));

    fprintf(fid, '## Overall\n\n');
    fprintf(fid, '- Total RMS error: **%.2f mm**\n', total_rms_mm);
    fprintf(fid, '- Servo saturation fraction: **%.1f%%** (theta_cmd = +/-%.0f deg)\n\n', ...
        total_sat_frac, rad2deg(theta_sat));

    fprintf(fid, '## Per-Waypoint Statistics\n\n');
    fprintf(fid, '| Seg | p_ref (mm) | Dur (s) | RMS err (mm) | Settle (s) | Sat (%%) |\n');
    fprintf(fid, '|-----|------------|---------|--------------|------------|----------|\n');
    for k = 1:n_segs
        if isnan(stats(k).settle_s)
            settle_str = 'never';
        else
            settle_str = sprintf('%.2f', stats(k).settle_s);
        end
        fprintf(fid, '| %d | %+.0f | %.2f | %.2f | %s | %.1f |\n', ...
            k, stats(k).ref_m * 1e3, stats(k).dur_s, stats(k).rms_mm, ...
            settle_str, stats(k).sat_frac);
    end

    fprintf(fid, '\n## Diagnostics\n\n');
    fprintf(fid, '- Bang-bang steady-state |theta_hat| = %.4f rad (%.1f deg)\n', ...
        theta_hat_ss, rad2deg(theta_hat_ss));
    fprintf(fid, '  - If servo permanently saturates, theta_hat locks to this level\n');
    fprintf(fid, '  - K_theta * theta_hat_ss = 0.704 * %.4f = %.4f rad ', ...
        theta_hat_ss, 0.704 * theta_hat_ss);
    if 0.704 * theta_hat_ss < theta_sat
        fprintf(fid, '(< %.3f rad sat limit — controller CAN exit bang-bang)\n', theta_sat);
    else
        fprintf(fid, '(>= %.3f rad sat limit — PERMANENT bang-bang expected)\n', theta_sat);
    end
    fprintf(fid, '- Convergence band: +/-%.0f mm\n', converge_m * 1e3);

    fclose(fid);
end
