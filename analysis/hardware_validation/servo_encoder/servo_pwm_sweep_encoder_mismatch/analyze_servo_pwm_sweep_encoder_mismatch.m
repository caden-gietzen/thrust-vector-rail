%% analyze_servo_pwm_sweep_encoder_mismatch.m
% Analyze bidirectional servo PWM sweep encoder mismatch data.
%
% Expected CSV from:
%   servo_pwm_sweep_encoder_mismatch.py
%
% Expected mirrored data location:
%   data/raw/system_identification/servo_identification/servo_pwm_sweep_encoder_mismatch/candidate
%
% Main outputs:
%   1. Average final count mismatch vs amplitude and dwell time
%   2. RMS mismatch vs amplitude and dwell time
%   3. Max absolute mismatch vs amplitude and dwell time
%   4. Direction/motion stroke deltas
%   5. Pass/fail table based on count mismatch threshold

clear; clc; close all;

%% ============================================================
% User options
% ============================================================

USE_LATEST_CSV = true;

% Used only when USE_LATEST_CSV = false
dataFile = "servo_pwm_sweep_encoder_mismatch.csv";

COUNTS_PER_REV = 2400;

% Define acceptable final return-to-center mismatch.
ACCEPTABLE_MISMATCH_COUNTS = 5;

SAVE_FIGURES = false;

%% ============================================================
% Locate project paths
% ============================================================

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, "candidate");
plotDir = getMirroredPlotDir(scriptPath);

if USE_LATEST_CSV
    csvFiles = dir(fullfile(dataDir, "*.csv"));

    if isempty(csvFiles)
        error("No CSV files found in mirrored data folder:\n%s", dataDir);
    end

    dataFile = selectLatestIndexedCsv(csvFiles);
end

dataPath = fullfile(dataDir, dataFile);

fprintf("\nUsing data file:\n");
fprintf("  %s\n", dataPath);

%% ============================================================
% Load data
% ============================================================

T = readtable(dataPath);

requiredVars = [
    "amplitude_us"
    "dwell_ms"
    "cycle_index"
    "count_pos"
    "count_neg"
    "final_count"
    "positive_delta_counts"
    "negative_delta_counts"
    "return_delta_counts"
    "final_mismatch_counts"
    "final_mismatch_revs"
];

for k = 1:numel(requiredVars)
    if ~ismember(requiredVars(k), string(T.Properties.VariableNames))
        error("Missing required CSV column: %s", requiredVars(k));
    end
end

T.final_mismatch_abs_counts = abs(T.final_mismatch_counts);
T.final_mismatch_deg = T.final_mismatch_revs * 360;
T.final_mismatch_abs_deg = abs(T.final_mismatch_deg);

if ismember("approx_command_cycle_hz", string(T.Properties.VariableNames))
    hasApproxFrequency = true;
else
    hasApproxFrequency = false;
end

%% ============================================================
% Summary table
% ============================================================

G = groupsummary( ...
    T, ...
    ["amplitude_us", "dwell_ms"], ...
    {"mean", "std", "max", "min"}, ...
    "final_mismatch_counts" ...
);

[groupId, groupAmp, groupDwell] = findgroups(T.amplitude_us, T.dwell_ms);

meanAbsMismatch = splitapply( ...
    @(x) mean(abs(x), "omitnan"), ...
    T.final_mismatch_counts, ...
    groupId ...
);

maxAbsMismatch = splitapply( ...
    @(x) max(abs(x)), ...
    T.final_mismatch_counts, ...
    groupId ...
);

rmsAbsMismatch = splitapply( ...
    @(x) sqrt(mean(x.^2, "omitnan")), ...
    T.final_mismatch_counts, ...
    groupId ...
);

G.mean_abs_mismatch_counts = meanAbsMismatch;
G.max_abs_mismatch_counts = maxAbsMismatch;
G.rms_abs_mismatch_counts = rmsAbsMismatch;

G.mean_abs_mismatch_deg = G.mean_abs_mismatch_counts / COUNTS_PER_REV * 360;
G.max_abs_mismatch_deg = G.max_abs_mismatch_counts / COUNTS_PER_REV * 360;
G.rms_abs_mismatch_deg = G.rms_abs_mismatch_counts / COUNTS_PER_REV * 360;

G.pass = G.max_abs_mismatch_counts <= ACCEPTABLE_MISMATCH_COUNTS;

disp(" ");
disp("Grouped mismatch summary:");
disp(G);

%% ============================================================
% Pivot matrices
% ============================================================

ampVals = sort(unique(T.amplitude_us), "ascend");
dwellVals = sort(unique(T.dwell_ms), "descend");

meanMismatch = makePivot(G, ampVals, dwellVals, "mean_final_mismatch_counts");
meanAbsMismatch = makePivot(G, ampVals, dwellVals, "mean_abs_mismatch_counts");
rmsMismatch = makePivot(G, ampVals, dwellVals, "rms_abs_mismatch_counts");
maxAbsMismatch = makePivot(G, ampVals, dwellVals, "max_abs_mismatch_counts");

%% ============================================================
% Plots
% ============================================================

figure("Name", "Mean Final Count Mismatch");
heatmap(ampVals, dwellVals, meanMismatch);
xlabel("Amplitude (\mus)");
ylabel("Dwell Time (ms)");
title("Mean Final Count Mismatch (counts)");

figure("Name", "Mean Absolute Final Count Mismatch");
heatmap(ampVals, dwellVals, meanAbsMismatch);
xlabel("Amplitude (\mus)");
ylabel("Dwell Time (ms)");
title("Mean Absolute Final Count Mismatch (counts)");

figure("Name", "RMS Final Count Mismatch");
heatmap(ampVals, dwellVals, rmsMismatch);
xlabel("Amplitude (\mus)");
ylabel("Dwell Time (ms)");
title("RMS Final Count Mismatch (counts)");

figure("Name", "Max Absolute Final Count Mismatch");
heatmap(ampVals, dwellVals, maxAbsMismatch);
xlabel("Amplitude (\mus)");
ylabel("Dwell Time (ms)");
title("Max Absolute Final Count Mismatch (counts)");

figure("Name", "Mean Absolute Mismatch vs Dwell");
hold on; grid on;

for i = 1:numel(ampVals)
    amp = ampVals(i);
    idx = G.amplitude_us == amp;
    Gi = sortrows(G(idx, :), "dwell_ms", "ascend");

    plot( ...
        Gi.dwell_ms, ...
        Gi.mean_abs_mismatch_counts, ...
        "-o", ...
        "DisplayName", sprintf("A = %d us", amp) ...
    );
end

yline(ACCEPTABLE_MISMATCH_COUNTS, "--", ...
    sprintf("threshold = %d counts", ACCEPTABLE_MISMATCH_COUNTS), ...
    "LabelHorizontalAlignment", "left");

xlabel("Dwell Time (ms)");
ylabel("Mean Absolute Final Mismatch (counts)");
title("Mean Absolute Return-to-Center Mismatch vs Dwell Time");
legend("Location", "best");

figure("Name", "Mean Absolute Mismatch vs Amplitude");
hold on; grid on;

for i = 1:numel(dwellVals)
    dwell = dwellVals(i);
    idx = G.dwell_ms == dwell;
    Gi = sortrows(G(idx, :), "amplitude_us", "ascend");

    plot( ...
        Gi.amplitude_us, ...
        Gi.mean_abs_mismatch_counts, ...
        "-o", ...
        "DisplayName", sprintf("dwell = %d ms", dwell) ...
    );
end

yline(ACCEPTABLE_MISMATCH_COUNTS, "--", ...
    sprintf("threshold = %d counts", ACCEPTABLE_MISMATCH_COUNTS), ...
    "LabelHorizontalAlignment", "left");

xlabel("Command Amplitude (\mus)");
ylabel("Mean Absolute Final Mismatch (counts)");
title("Mean Absolute Return-to-Center Mismatch vs Command Amplitude");
legend("Location", "best");

figure("Name", "Cycle-by-Cycle Final Mismatch");
tiledlayout("flow");

for d = 1:numel(dwellVals)
    dwell = dwellVals(d);

    for a = 1:numel(ampVals)
        amp = ampVals(a);

        idx = T.dwell_ms == dwell & T.amplitude_us == amp;
        Ti = sortrows(T(idx, :), "cycle_index");

        if isempty(Ti)
            continue;
        end

        nexttile;
        plot(Ti.cycle_index, Ti.final_mismatch_counts, "-o");
        grid on;
        yline(0, "-");
        yline(ACCEPTABLE_MISMATCH_COUNTS, "--");
        yline(-ACCEPTABLE_MISMATCH_COUNTS, "--");
        title(sprintf("A=%d us, dwell=%d ms", amp, dwell));
        xlabel("Cycle");
        ylabel("Mismatch (counts)");
    end
end

%% ============================================================
% Stroke delta plots
% ============================================================

strokeSummary = groupsummary( ...
    T, ...
    ["amplitude_us", "dwell_ms"], ...
    "mean", ...
    ["positive_delta_counts", "negative_delta_counts", "return_delta_counts"] ...
);

figure("Name", "Stroke Delta Counts");
tiledlayout(3, 1);

nexttile;
hold on; grid on;
for i = 1:numel(dwellVals)
    dwell = dwellVals(i);
    S = sortrows(strokeSummary(strokeSummary.dwell_ms == dwell, :), "amplitude_us");
    plot(S.amplitude_us, S.mean_positive_delta_counts, "-o", ...
        "DisplayName", sprintf("dwell = %d ms", dwell));
end
xlabel("Amplitude (\mus)");
ylabel("+ Stroke Delta (counts)");
title("Mean Positive Stroke Delta");
legend("Location", "best");

nexttile;
hold on; grid on;
for i = 1:numel(dwellVals)
    dwell = dwellVals(i);
    S = sortrows(strokeSummary(strokeSummary.dwell_ms == dwell, :), "amplitude_us");
    plot(S.amplitude_us, S.mean_negative_delta_counts, "-o", ...
        "DisplayName", sprintf("dwell = %d ms", dwell));
end
xlabel("Amplitude (\mus)");
ylabel("- Stroke Delta (counts)");
title("Mean Negative Stroke Delta");
legend("Location", "best");

nexttile;
hold on; grid on;
for i = 1:numel(dwellVals)
    dwell = dwellVals(i);
    S = sortrows(strokeSummary(strokeSummary.dwell_ms == dwell, :), "amplitude_us");
    plot(S.amplitude_us, S.mean_return_delta_counts, "-o", ...
        "DisplayName", sprintf("dwell = %d ms", dwell));
end
xlabel("Amplitude (\mus)");
ylabel("Return Stroke Delta (counts)");
title("Mean Return-to-Center Stroke Delta");
legend("Location", "best");

%% ============================================================
% Pass/fail map
% ============================================================

passMap = maxAbsMismatch <= ACCEPTABLE_MISMATCH_COUNTS;

figure("Name", "Encoder Reliability Pass-Fail Map");
heatmap(ampVals, dwellVals, double(passMap));
xlabel("Amplitude (\mus)");
ylabel("Dwell Time (ms)");
title(sprintf("Pass/Fail Map: Max Abs Mismatch <= %d counts", ACCEPTABLE_MISMATCH_COUNTS));

%% ============================================================
% Optional approximate command-cycle plot
% ============================================================

if hasApproxFrequency
    figure("Name", "Mismatch vs Approx Command Cycle Frequency");
    hold on; grid on;

    for i = 1:numel(ampVals)
        amp = ampVals(i);
        idx = T.amplitude_us == amp;
        Ti = T(idx, :);

        scatter( ...
            Ti.approx_command_cycle_hz, ...
            abs(Ti.final_mismatch_counts), ...
            "DisplayName", sprintf("A = %d us", amp) ...
        );
    end

    xlabel("Approx Command Cycle Frequency (Hz)");
    ylabel("Absolute Final Mismatch (counts)");
    title("Mismatch vs Approx Command Cycle Frequency");
    legend("Location", "best");
end

%% ============================================================
% Print summary
% ============================================================

Gw = sortrows(G, "max_abs_mismatch_counts", "descend");

fprintf("\n============================================================\n");
fprintf("Servo PWM sweep encoder mismatch summary\n");
fprintf("============================================================\n");

fprintf("\nData file:\n");
fprintf("  %s\n", dataPath);

fprintf("\nSamples: %d\n", height(T));
fprintf("Amplitude range: %.1f to %.1f us\n", min(T.amplitude_us), max(T.amplitude_us));
fprintf("Dwell time range: %.1f to %.1f ms\n", min(T.dwell_ms), max(T.dwell_ms));

fprintf("\nWorst case:\n");
fprintf("  Amplitude: %.1f us\n", Gw.amplitude_us(1));
fprintf("  Dwell: %.1f ms\n", Gw.dwell_ms(1));
fprintf("  Max abs mismatch: %.2f counts\n", Gw.max_abs_mismatch_counts(1));
fprintf("  Equivalent angle: %.4f deg\n", Gw.max_abs_mismatch_deg(1));

fprintf("\nPass/fail threshold:\n");
fprintf("  Max abs mismatch <= %.1f counts\n", ACCEPTABLE_MISMATCH_COUNTS);

disp(" ");
disp("Worst cases by max absolute mismatch:");
disp(Gw(:, [
    "amplitude_us"
    "dwell_ms"
    "GroupCount"
    "mean_final_mismatch_counts"
    "std_final_mismatch_counts"
    "mean_abs_mismatch_counts"
    "rms_abs_mismatch_counts"
    "max_abs_mismatch_counts"
    "max_abs_mismatch_deg"
    "pass"
]));

fprintf("\nPractical interpretation:\n");
fprintf("  If low dwell-time cases show poor endpoint motion but low final mismatch,\n");
fprintf("  that likely indicates servo attenuation rather than encoder failure.\n");
fprintf("  Treat very short dwell times as actuator bandwidth tests, not clean encoder tests.\n");

%% ============================================================
% Save figures
% ============================================================

if SAVE_FIGURES
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    figs = findall(0, "Type", "figure");

    for k = 1:numel(figs)
        fig = figs(k);
        figure(fig);

        name = fig.Name;
        if name == ""
            name = sprintf("figure_%02d", k);
        end

        name = regexprep(name, "[^\w\d_-]", "_");
        exportgraphics(fig, fullfile(plotDir, name + ".png"), "Resolution", 200);
    end

    fprintf("\nSaved figures to:\n  %s\n", plotDir);
end

%% ============================================================
% Local functions
% ============================================================

function M = makePivot(G, ampVals, dwellVals, metricName)
    M = nan(numel(dwellVals), numel(ampVals));

    for i = 1:numel(dwellVals)
        for j = 1:numel(ampVals)
            idx = G.dwell_ms == dwellVals(i) & G.amplitude_us == ampVals(j);

            if any(idx)
                M(i, j) = G.(metricName)(find(idx, 1));
            end
        end
    end
end