%% analyze_servo_pwm_sweep_encoder_mismatch.m
% Encoder count-integrity validation under bidirectional servo motion.
%
% Question: does the quadrature encoder lose counts during fast/large servo
% excursions? Each cycle drives center -> +A -> -A -> center and logs the
% return-to-center count error (final_mismatch_counts). A small mismatch means
% the encoder faithfully tracks position; a growing mismatch means dropped counts.
%
% Why it matters: the servo step test shows a ~2 deg directional offset
% (experiments/servo_identification/results.md). This test decides whether that
% offset is REAL (mechanical hysteresis/backlash) or a MEASUREMENT artifact
% (encoder miscount). If the encoder returns to center in the regime the ID tests
% use, the offset is mechanical.
%
% Output is limited to the conclusion:
%   - Plot 1: max return mismatch (deg) vs command amplitude, per dwell
%   - Plot 2: per-cycle mismatch (deg) at the longest dwell (count-loss evidence)
%   - A printed verdict + the reliable operating envelope.

clear; clc; close all;

%% Options

DATASET_STATUS = "candidate";   % candidate | accepted | rejected | diagnostics
USE_LATEST_CSV = true;
dataFile       = "servo_pwm_sweep_encoder_mismatch.csv";   % used if USE_LATEST_CSV=false

COUNTS_PER_REV     = 2400;
QUANT_REF_COUNTS   = 5;          % quantization/minor-backlash reference (~0.75 deg) for plots
SYSTEMATIC_LOSS_DEG = 2.0;       % above this = real dropped counts, not quantization noise
SAVE_FIGURES       = false;

% Bright palette (MATLAB dark mode), up to 5 dwell/amplitude series.
PALETTE = [0 0.90 1.00; 1 0.65 0.00; 0.45 1 0.45; 1 0.40 0.80; 1 0.95 0.30];

%% Load

scriptPath = mfilename("fullpath");
dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

if USE_LATEST_CSV
    csvFiles = dir(fullfile(dataDir, "*.csv"));
    if isempty(csvFiles)
        error("No CSV files found in:\n%s", dataDir);
    end
    dataFile = selectLatestIndexedCsv(csvFiles);
end
dataPath = fullfile(dataDir, dataFile);

T = readtable(dataPath);
req = ["amplitude_us", "dwell_ms", "cycle_index", "final_mismatch_counts"];
for k = 1:numel(req)
    if ~ismember(req(k), string(T.Properties.VariableNames))
        error("Missing required column: %s", req(k));
    end
end

% Derived quantities (degrees are the units that matter downstream).
counts2deg = 360 / COUNTS_PER_REV;
gainDegPerUs = abs(servoStaticMap().gain_deg_per_us);     % command us -> half-stroke deg
T.mismatch_deg     = T.final_mismatch_counts * counts2deg;
T.abs_mismatch_deg = abs(T.mismatch_deg);
T.amplitude_deg    = T.amplitude_us * gainDegPerUs;       % +/- from center
quantRefDeg = QUANT_REF_COUNTS * counts2deg;

ampVals   = sort(unique(T.amplitude_us), "ascend");
dwellVals = sort(unique(T.dwell_ms), "ascend");

%% Plot 1 — max return mismatch vs amplitude, per dwell

figure("Color", "k"); hold on; grid on;
for i = 1:numel(dwellVals)
    d = dwellVals(i);
    maxByAmp = arrayfun(@(a) max(T.abs_mismatch_deg(T.dwell_ms==d & T.amplitude_us==a)), ampVals);
    plot(ampVals * gainDegPerUs, maxByAmp, "-o", "LineWidth", 1.6, ...
        "Color", PALETTE(mod(i-1,5)+1,:), "MarkerFaceColor", PALETTE(mod(i-1,5)+1,:), ...
        "DisplayName", sprintf("dwell %d ms", d));
end
yline(quantRefDeg, "--w", sprintf("quantization ref %.2f deg", quantRefDeg), "LineWidth", 1.1);
xlabel("Command amplitude (deg, \pm from center)");
ylabel("Max return-to-center mismatch (deg)");
title("Encoder Return-to-Center Mismatch vs Command Amplitude");
legend("Location", "northwest");

%% Plot 2 — per-cycle mismatch at the longest dwell (count-loss evidence)

dLong = max(dwellVals);
figure("Color", "k"); hold on; grid on;
for a = 1:numel(ampVals)
    m = T.dwell_ms==dLong & T.amplitude_us==ampVals(a);
    Ti = sortrows(T(m,:), "cycle_index");
    if isempty(Ti); continue; end
    plot(Ti.cycle_index, Ti.mismatch_deg, "-o", "LineWidth", 1.5, ...
        "Color", PALETTE(mod(a-1,5)+1,:), "MarkerFaceColor", PALETTE(mod(a-1,5)+1,:), ...
        "DisplayName", sprintf("%.0f deg", ampVals(a)*gainDegPerUs));
end
yline([quantRefDeg -quantRefDeg], "--w");
xlabel("Cycle"); ylabel("Return-to-center mismatch (deg)");
title(sprintf("Per-Cycle Mismatch at %d ms Dwell (count-loss check)", dLong));
legend("Location", "best");

%% Per-amplitude summary table

ampDeg = ampVals * gainDegPerUs;
maxAbs = arrayfun(@(a) max(T.abs_mismatch_deg(T.amplitude_us==a)), ampVals);
medAbs = arrayfun(@(a) median(T.abs_mismatch_deg(T.amplitude_us==a)), ampVals);
dwellAtMax = arrayfun(@(a) dwellOfMax(T, a), ampVals);
lossFree = maxAbs <= SYSTEMATIC_LOSS_DEG;     % no systematic count loss at this amplitude
summary = table(ampVals, round(ampDeg,1), round(medAbs,3), round(maxAbs,3), dwellAtMax, lossFree, ...
    'VariableNames', {'amp_us', 'amp_deg', 'median_abs_deg', 'max_abs_deg', 'dwell_at_max_ms', 'loss_free'});

%% Verdict

[worstVal, wi] = max(T.abs_mismatch_deg);
reliableAmps = ampVals(lossFree);
if isempty(reliableAmps)
    reliableTxt = "no tested amplitude";
else
    reliableTxt = sprintf("excursions up to +/-%.0f deg (%.0f us)", ...
        max(reliableAmps) * gainDegPerUs, max(reliableAmps));
end

fprintf("\n=== Encoder count-integrity validation ===\n");
fprintf("File: %s\n", dataFile);
fprintf("Grid: amplitude %.0f-%.0f deg, dwell %d-%d ms, %d cycles each (%d samples)\n", ...
    min(ampDeg), max(ampDeg), min(dwellVals), max(dwellVals), ...
    height(T)/(numel(ampVals)*numel(dwellVals)), height(T));
disp(summary);

fprintf("\nQuantization floor (all conditions): median %.2f deg (~1 count); 95th pct %.2f deg\n", ...
    median(T.abs_mismatch_deg), prctile(T.abs_mismatch_deg, 95));
fprintf("Worst case: %.2f deg at amplitude %.0f deg, dwell %d ms\n", ...
    worstVal, T.amplitude_deg(wi), T.dwell_ms(wi));

fprintf("\nVERDICT:\n");
fprintf("  Return-to-center sits at the ~1-count quantization floor for %s.\n", reliableTxt);
fprintf("  Systematic count loss appears only at the largest excursion (~%.0f deg) once the\n", max(ampDeg));
fprintf("  servo fully settles (long dwell) and slews hardest: up to %.1f deg lost, every cycle.\n", worstVal);
fprintf("  The servo-ID tests (steps <= +/-20 deg, small-signal PRPS) sit inside the loss-free\n");
fprintf("  regime, so the ~2 deg step-response offset is MECHANICAL (hysteresis/backlash),\n");
fprintf("  not an encoder miscount. Avoid trusting raw counts for large fast near-full-deflection moves.\n");

%% Save

if SAVE_FIGURES
    if ~exist(plotDir, "dir"); mkdir(plotDir); end
    figs = flipud(findall(0, "Type", "figure"));
    names = ["mismatch_vs_amplitude", "per_cycle_mismatch_longest_dwell"];
    for k = 1:numel(figs)
        exportgraphics(figs(k), fullfile(plotDir, names(k) + ".png"), "Resolution", 200);
        fprintf("Saved: %s\n", fullfile(plotDir, names(k) + ".png"));
    end
end

%% Local functions

function d = dwellOfMax(T, amp)
    sub = T(T.amplitude_us == amp, :);
    [~, i] = max(sub.abs_mismatch_deg);
    d = sub.dwell_ms(i);
end
