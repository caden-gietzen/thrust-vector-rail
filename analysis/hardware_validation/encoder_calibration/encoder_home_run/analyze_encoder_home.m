% analyze_encoder_home.m
%
% Quick analysis of a powered-homing log (encoder_home_run.csv).
%
% Two products, both of which feed friction identification:
%
%   1. Rail length — inferred from the per-traverse encoder count span and the
%      trusted spec-derived scale (counts/mm). The scale is NOT re-measured here;
%      rail length = mean(span_counts) / scale.
%
%   2. Breakaway force at the homing angle — the axial thrust force at which the
%      cart first broke away from rest on each leg. Converted from the breakaway
%      ESC command via the identified static thrust map:
%
%          F_breakaway = F_ss(u_breakaway) * sin(|theta|)        [N]
%
%      This is a direct static-friction (stiction) estimate per direction. The
%      true threshold is bracketed by the ramp quantization:
%          F_ss(u_breakaway - step) ... F_ss(u_breakaway), both * sin|theta|.
%      Compare against the Coulomb mu from analyze_friction_sweep.
%
% Static thrust map (experiments/thrust_identification/results.md):
%   F_ss(u) = polyval(p, (u - 1525.0)/256.2),  dead zone below 1075 us.
%
% Input data (one row per driven leg):
%   data/raw/hardware_validation/encoder_calibration/encoder_home_run/candidate/
%   columns: leg_index,leg_name,direction,angle_deg,status,breakaway_us,
%            start_count,end_count,span_counts,scale_counts_per_mm,ramp_step_us

close all; clear; clc

%% User-Editable Parameters

SAVE_FIGURES = true;

% --- Static thrust map (experiments/thrust_identification/results.md) ---
THRUST_POLY = [1.828e-5, 0.053686, 0.17600, 1.06699, 1.74620];  % deg-4, normalized
THRUST_MU   = [1525.0, 256.2];   % normalization [mean, std] (us)
U_ARM_US    = 1075;              % motor dead zone; below this F_ss = 0

%% Setup

scriptPath = mfilename('fullpath');
repoRoot   = findRepoRoot(fileparts(scriptPath));
plotDir    = getMirroredPlotDir(scriptPath);
reportDir  = char(strrep(string(plotDir), ...
    string(fullfile(repoRoot, 'plots')), string(fullfile(repoRoot, 'reports'))));

figNum = 0;

%% Load latest CSV (candidate first — this is fresh per-run calibration data)

dataDir = getMirroredRawDataDir(scriptPath, 'candidate');
if ~isfolder(dataDir) || isempty(dir(fullfile(dataDir, '*.csv')))
    fprintf('No candidate data found. Trying accepted...\n');
    dataDir = getMirroredRawDataDir(scriptPath, 'accepted');
end

csvFiles = dir(fullfile(dataDir, '*.csv'));
if isempty(csvFiles)
    error('No CSV files found in %s', dataDir);
end

dataFile = selectLatestIndexedCsv(csvFiles);
fpath    = fullfile(dataDir, dataFile);
fprintf('Loading homing log:\n  %s\n\n', fpath);

T = readtable(fpath, 'TextType', 'string');

required = {'leg_name','direction','angle_deg','status','breakaway_us', ...
            'span_counts','scale_counts_per_mm','ramp_step_us'};
if ~all(ismember(required, T.Properties.VariableNames))
    error('Unexpected CSV columns in %s', dataFile);
end

scale_counts_per_mm = T.scale_counts_per_mm(1);
ramp_step_us        = T.ramp_step_us(1);
nLegs               = height(T);

fprintf('Legs: %d   scale: %.3f counts/mm   ramp step: %d us\n\n', ...
    nLegs, scale_counts_per_mm, ramp_step_us);

%% Rail Length (from traverse spans)

% Use only completed traverse legs (status endstop, span present).
isTraverse = strcmp(T.status, "endstop") & ~isnan(T.span_counts);
spans      = T.span_counts(isTraverse);

if isempty(spans)
    error('No completed traverse legs with a span — cannot infer rail length.');
end

rail_mm_per_leg = spans / scale_counts_per_mm;
rail_mm_mean    = mean(rail_mm_per_leg);
rail_mm_std     = std(rail_mm_per_leg);
rail_spread_pct = 100 * (max(spans) - min(spans)) / mean(spans);

fprintf('=== Rail Length ===\n');
fprintf('  traverses used : %d\n', numel(spans));
fprintf('  span (counts)  : mean %.1f, std %.1f\n', mean(spans), std(spans));
fprintf('  rail length    : %.2f mm  (std %.2f mm)\n', rail_mm_mean, rail_mm_std);
fprintf('  repeatability  : %.2f%% (max-min / mean span)\n\n', rail_spread_pct);

%% Breakaway Force (per leg, per direction)

% Static map helper (handles vectors; zeros the dead zone).
thrustN = @(u) max(0, polyval(THRUST_POLY, (u - THRUST_MU(1)) / THRUST_MU(2))) ...
              .* (u >= U_ARM_US);

hasBrk   = ~isnan(T.breakaway_us);
brk_us   = T.breakaway_us(hasBrk);
brk_dir  = T.direction(hasBrk);
brk_ang  = abs(T.angle_deg(hasBrk));        % magnitude; sin uses |theta|
brk_name = T.leg_name(hasBrk);

% Breakaway axial force and its quantization bracket (one ramp step wide).
sin_th     = sind(brk_ang);
F_brk      = thrustN(brk_us)                .* sin_th;   % upper edge of bracket
F_brk_lo   = thrustN(brk_us - ramp_step_us) .* sin_th;   % lower edge

fprintf('=== Breakaway Force (static-friction estimate) ===\n');
if isempty(brk_us)
    fprintf('  No breakaway captured on any leg (cart pinned at stops?).\n\n');
else
    fprintf('  %-12s %-6s %8s %10s   %s\n', ...
        'leg', 'dir', 'u_brk', '|theta|', 'F_breakaway [N] (bracket)');
    for i = 1:numel(brk_us)
        fprintf('  %-12s %-6s %6d us %8.1f deg   %.4f  [%.4f, %.4f]\n', ...
            brk_name(i), brk_dir(i), round(brk_us(i)), brk_ang(i), ...
            F_brk(i), F_brk_lo(i), F_brk(i));
    end
    fprintf('\n');
end

% Group by direction.
dirNames  = ["home", "far"];
dirStats  = struct('name', {}, 'n', {}, 'u_mean', {}, ...
                   'F_mean', {}, 'F_lo', {}, 'F_hi', {});
for d = 1:numel(dirNames)
    m = strcmp(brk_dir, dirNames(d));
    if ~any(m); continue; end
    s.name   = dirNames(d);
    s.n      = sum(m);
    s.u_mean = mean(brk_us(m));
    s.F_mean = mean(F_brk(m));
    s.F_lo   = mean(F_brk_lo(m));      % bracket lower (mean over legs)
    s.F_hi   = mean(F_brk(m));         % bracket upper
    dirStats(end+1) = s; %#ok<SAGROW>
    fprintf('  %-5s dir: n=%d, u_brk mean=%.0f us, F_breakaway ~ %.4f N  [%.4f, %.4f]\n', ...
        s.name, s.n, s.u_mean, s.F_mean, s.F_lo, s.F_hi);
end

% Directional asymmetry (the friction-ID-relevant number).
asym_pct = NaN;
if numel(dirStats) == 2
    fa = dirStats(1).F_mean; fb = dirStats(2).F_mean;
    asym_pct = 100 * abs(fa - fb) / mean([fa, fb]);
    fprintf('\n  Directional asymmetry in breakaway force: %.1f%%\n', asym_pct);
    if asym_pct > 20
        fprintf('  *** >20%% — direction-dependent static friction; carry into friction ID. ***\n');
    end
end
fprintf('\n');

%% Figure 1 — Rail length per traverse

figNum = figNum + 1;
figure(figNum); hold on;

travDir = T.direction(isTraverse);
for i = 1:numel(rail_mm_per_leg)
    if travDir(i) == "far"
        clr = [0 0.9 1];      % cyan
    else
        clr = [1 0.65 0];     % orange
    end
    bar(i, rail_mm_per_leg(i), 'FaceColor', clr, 'EdgeColor', [0.9 0.9 0.9]);
end
yline(rail_mm_mean, '--', sprintf('mean %.1f mm', rail_mm_mean), ...
    'Color', [0.45 1 0.45], 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
xlabel('Traverse #'); ylabel('Inferred rail length (mm)');
title(sprintf('Rail length per traverse  (repeatability %.2f%%)', rail_spread_pct));
grid on;

%% Figure 2 — Breakaway force per leg with quantization bracket

if ~isempty(brk_us)
    figNum = figNum + 1;
    figure(figNum); hold on;

    for i = 1:numel(brk_us)
        if brk_dir(i) == "far"
            clr = [0 0.9 1];
        else
            clr = [1 0.65 0];
        end
        bar(i, F_brk(i), 'FaceColor', clr, 'EdgeColor', [0.9 0.9 0.9]);
        % Bracket: lower edge (one ramp step down) up to the captured value.
        errorbar(i, F_brk(i), F_brk(i) - F_brk_lo(i), 0, ...
            'Color', [0.95 0.95 0.95], 'LineWidth', 1.5, 'CapSize', 8);
    end
    set(gca, 'XTick', 1:numel(brk_us), ...
        'XTickLabel', cellstr(brk_name), 'XTickLabelRotation', 30);
    ylabel('Breakaway axial force (N)');
    title('Breakaway (static-friction) force per leg — bracket = ramp quantization');
    grid on;
end

%% Figure 3 — Static thrust map with breakaway operating points

figNum = figNum + 1;
figure(figNum); hold on;

u_curve = linspace(1000, 2000, 400);
plot(u_curve, thrustN(u_curve), '-', 'Color', [0.45 1 0.45], ...
    'LineWidth', 2, 'DisplayName', 'static thrust map F_{ss}(u)');

if ~isempty(brk_us)
    for d = 1:numel(dirNames)
        m = strcmp(brk_dir, dirNames(d));
        if ~any(m); continue; end
        if dirNames(d) == "far"
            clr = [0 0.9 1];
        else
            clr = [1 0.65 0];
        end
        scatter(brk_us(m), thrustN(brk_us(m)), 60, clr, 'filled', ...
            'DisplayName', sprintf('%s breakaway', dirNames(d)));
    end
end
xline(U_ARM_US, ':', 'arm threshold', 'Color', [0.8 0.8 0.8]);
xlabel('ESC command (us)'); ylabel('Thrust F_{ss} (N)');
title('Breakaway commands on the identified static thrust map');
legend('show', 'Location', 'northwest'); grid on;

%% Save Figures

if SAVE_FIGURES
    saveAllFiguresIfEnabled(true, plotDir);
end

%% Write Analysis Report

if ~exist(reportDir, 'dir')
    mkdir(reportDir);
end

reportPath = fullfile(reportDir, 'analyze_encoder_home.report.md');
writeReport(reportPath, fpath, scale_counts_per_mm, ramp_step_us, ...
    spans, rail_mm_mean, rail_mm_std, rail_spread_pct, ...
    brk_name, brk_dir, brk_us, brk_ang, F_brk_lo, F_brk, dirStats, asym_pct);
fprintf('Report written to:\n  %s\n', reportPath);

%% Local Functions

function writeReport(path, fpath, scale, step, spans, railMean, railStd, ...
        railSpread, brkName, brkDir, brkUs, brkAng, Flo, Fhi, dirStats, asymPct)

    fid = fopen(path, 'w');
    fprintf(fid, '# Encoder Homing Analysis Report\n\n');
    fprintf(fid, '**Generated:** %s  \n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '**Dataset:** `%s`  \n', strrep(fpath, '\', '/'));
    fprintf(fid, '**Scale:** %.3f counts/mm (spec-derived, fixed)  \n', scale);
    fprintf(fid, '**Ramp step:** %d us\n\n', step);

    fprintf(fid, '## Rail Length\n\n');
    fprintf(fid, '- Traverses used: %d\n', numel(spans));
    fprintf(fid, '- Mean span: %.1f counts (std %.1f)\n', mean(spans), std(spans));
    fprintf(fid, '- **Rail length: %.2f mm** (std %.2f mm)\n', railMean, railStd);
    fprintf(fid, '- Repeatability: %.2f%% (max-min / mean span)\n\n', railSpread);

    fprintf(fid, '## Breakaway Force (static-friction estimate)\n\n');
    if isempty(brkUs)
        fprintf(fid, '_No breakaway captured — cart was pinned at the stops._\n\n');
    else
        fprintf(fid, 'F_breakaway = F_ss(u_breakaway) * sin|theta|. ');
        fprintf(fid, 'Bracket = one ramp step of command quantization.\n\n');
        fprintf(fid, '| Leg | Dir | u_brk (us) | \\|theta\\| (deg) | F_breakaway (N) | Bracket (N) |\n');
        fprintf(fid, '|-----|-----|-----------|------------|-----------------|-------------|\n');
        for i = 1:numel(brkUs)
            fprintf(fid, '| %s | %s | %d | %.1f | %.4f | [%.4f, %.4f] |\n', ...
                brkName(i), brkDir(i), round(brkUs(i)), brkAng(i), ...
                Fhi(i), Flo(i), Fhi(i));
        end
        fprintf(fid, '\n');

        fprintf(fid, '### Per-direction summary\n\n');
        fprintf(fid, '| Direction | n | u_brk mean (us) | F_breakaway (N) | Bracket (N) |\n');
        fprintf(fid, '|-----------|---|-----------------|-----------------|-------------|\n');
        for d = 1:numel(dirStats)
            s = dirStats(d);
            fprintf(fid, '| %s | %d | %.0f | %.4f | [%.4f, %.4f] |\n', ...
                s.name, s.n, s.u_mean, s.F_mean, s.F_lo, s.F_hi);
        end
        fprintf(fid, '\n');

        if ~isnan(asymPct)
            fprintf(fid, '**Directional asymmetry: %.1f%%**\n\n', asymPct);
            if asymPct > 20
                fprintf(fid, '> **>20%% asymmetry** — static friction is direction-dependent; ');
                fprintf(fid, 'carry a per-direction Coulomb term into friction identification.\n\n');
            else
                fprintf(fid, 'Within 20%% — a symmetric static-friction term is adequate.\n\n');
            end
        end
    end

    fprintf(fid, '## How this feeds friction identification\n\n');
    fprintf(fid, '- The breakaway axial force is a direct **stiction threshold** sample, ');
    fprintf(fid, 'independent of the residual-based Coulomb mu from `analyze_friction_sweep`.\n');
    fprintf(fid, '- Caveats: (1) the threshold is quantized to one ramp step; ');
    fprintf(fid, '(2) any rail tilt adds a gravity bias not separated here; ');
    fprintf(fid, '(3) thrust is taken at the static-map steady state (valid because each ramp ');
    fprintf(fid, 'step dwells >> the thrust time constant).\n\n');

    fclose(fid);
end
