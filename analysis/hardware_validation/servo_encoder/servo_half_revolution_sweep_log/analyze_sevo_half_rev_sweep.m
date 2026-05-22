%% analyze_servo_half_rev_sweep.m
% Analyze servo half-revolution sweep data from Pico CSV log.
%
% Expected CSV columns from servo_450_to_2450_half_rev_sweep.csv:
%   t_ms,t_s,sweep_idx,direction,servo_us,count,
%   count_delta_from_start,expected_deg_from_command,
%   estimated_counts_per_rev
%
% Goal:
%   Check whether 450 us -> 2450 us produces approximately 1200 encoder counts.
%
% Interpretation:
%   If 180 deg produces about 1200 counts, then:
%
%       counts_per_rev ≈ 1200 / (180/360) = 2400 counts/rev
%
%   This supports the assumption:
%
%       600 PPR encoder * 4 quadrature edges = 2400 counts/rev
%
%   where:
%       PPR = pulses per revolution

clear; clc; close all;

%% ============================================================
% User options
% ============================================================

USE_LATEST_CSV = true;

% Used only when USE_LATEST_CSV = false
dataFile = "servo_450_to_2450_half_rev_sweep.csv";

EXPECTED_HALF_REV_COUNTS = 1200;
EXPECTED_COUNTS_PER_REV = 2400;

SERVO_0_DEG_US = 450;
SERVO_180_DEG_US = 2450;

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
    "t_s"
    "sweep_idx"
    "direction"
    "servo_us"
    "count"
    "count_delta_from_start"
    "expected_deg_from_command"
    "estimated_counts_per_rev"
];

for k = 1:numel(requiredVars)
    if ~ismember(requiredVars(k), string(T.Properties.VariableNames))
        error("Missing required CSV column: %s", requiredVars(k));
    end
end

t = T.t_s;
servo_us = T.servo_us;
count = T.count;
count_delta = T.count_delta_from_start;
expected_deg = T.expected_deg_from_command;
estimated_cpr = T.estimated_counts_per_rev;
direction = string(T.direction);

%% ============================================================
% Endpoint extraction
% ============================================================

idx_180 = direction == "up_endpoint_settled";
idx_0_return = direction == "down_endpoint_settled";

if ~any(idx_180)
    warning("No up_endpoint_settled rows found. Using max servo command rows instead.");
    idx_180 = servo_us == max(servo_us);
end

if ~any(idx_0_return)
    warning("No down_endpoint_settled rows found. Using min servo command rows instead.");
    idx_0_return = servo_us == min(servo_us);
end

endpoint180 = T(idx_180, :);
endpoint0Return = T(idx_0_return, :);

delta180 = endpoint180.count_delta_from_start;
cpr180 = endpoint180.estimated_counts_per_rev;

delta0Return = endpoint0Return.count_delta_from_start;

absDelta180 = abs(delta180);
meanHalfRevCounts = mean(absDelta180, "omitnan");
stdHalfRevCounts = std(absDelta180, "omitnan");

meanCountsPerRev = mean(cpr180, "omitnan");
stdCountsPerRev = std(cpr180, "omitnan");

halfRevErrorCounts = meanHalfRevCounts - EXPECTED_HALF_REV_COUNTS;
halfRevErrorPct = 100 * halfRevErrorCounts / EXPECTED_HALF_REV_COUNTS;

cprErrorCounts = meanCountsPerRev - EXPECTED_COUNTS_PER_REV;
cprErrorPct = 100 * cprErrorCounts / EXPECTED_COUNTS_PER_REV;

meanReturnErrorCounts = mean(abs(delta0Return), "omitnan");

%% ============================================================
% Minimal plots
% ============================================================

% Plot 1: Command and encoder count over time
figure;
yyaxis left;
plot(t, servo_us, "LineWidth", 1.3);
ylabel("Servo Command (\mus)");

yyaxis right;
plot(t, count_delta, "LineWidth", 1.3);
ylabel("Encoder Count Delta");

grid on;
xlabel("Time (s)");
title("Servo Half-Revolution Sweep: Command and Encoder Count");

% Plot 2: Static relationship, command vs count
figure;
plot(servo_us, count_delta, "o-", "LineWidth", 1.2);
hold on;
yline(EXPECTED_HALF_REV_COUNTS, "--", "Expected +1200 counts");
yline(-EXPECTED_HALF_REV_COUNTS, "--", "Expected -1200 counts");
grid on;
xlabel("Servo Command (\mus)");
ylabel("Encoder Count Delta from Start");
title("Command vs Encoder Count Delta");

% Plot 3: Estimated counts/rev vs command angle
validCpr = isfinite(estimated_cpr) & estimated_cpr > 0;

figure;
plot(expected_deg(validCpr), estimated_cpr(validCpr), "o", "LineWidth", 1.1);
hold on;
yline(EXPECTED_COUNTS_PER_REV, "--", "Expected 2400 counts/rev");
grid on;
xlabel("Expected Commanded Angle (deg)");
ylabel("Estimated Counts per Revolution");
title("Estimated Encoder Counts per Revolution");

%% ============================================================
% Optional fit over full sweep
% ============================================================

validFit = isfinite(expected_deg) & isfinite(count_delta);

p_deg_to_count = polyfit(expected_deg(validFit), count_delta(validFit), 1);

counts_per_deg_fit = p_deg_to_count(1);
counts_per_rev_fit = abs(counts_per_deg_fit) * 360;

fitErrorCounts = counts_per_rev_fit - EXPECTED_COUNTS_PER_REV;
fitErrorPct = 100 * fitErrorCounts / EXPECTED_COUNTS_PER_REV;

%% ============================================================
% Print summary
% ============================================================

fprintf("\n============================================================\n");
fprintf("Servo half-revolution encoder calibration summary\n");
fprintf("============================================================\n");

fprintf("\nData file:\n");
fprintf("  %s\n", dataPath);

fprintf("\nSamples: %d\n", height(T));
fprintf("Servo command range: %.1f to %.1f us\n", min(servo_us), max(servo_us));
fprintf("Encoder count range: %.1f to %.1f counts\n", min(count_delta), max(count_delta));

fprintf("\nEndpoint-based result:\n");
fprintf("  Expected half-rev count change: %.1f counts\n", EXPECTED_HALF_REV_COUNTS);
fprintf("  Measured mean half-rev count change: %.2f counts\n", meanHalfRevCounts);
fprintf("  Measured half-rev count std: %.2f counts\n", stdHalfRevCounts);
fprintf("  Half-rev error: %.2f counts (%.2f%%)\n", halfRevErrorCounts, halfRevErrorPct);

fprintf("\nCounts-per-revolution estimate from endpoint:\n");
fprintf("  Expected counts/rev: %.1f\n", EXPECTED_COUNTS_PER_REV);
fprintf("  Estimated mean counts/rev: %.2f\n", meanCountsPerRev);
fprintf("  Estimated counts/rev std: %.2f\n", stdCountsPerRev);
fprintf("  Counts/rev error: %.2f counts (%.2f%%)\n", cprErrorCounts, cprErrorPct);

fprintf("\nReturn-to-zero check:\n");
fprintf("  Mean absolute return error: %.2f counts\n", meanReturnErrorCounts);
fprintf("  Equivalent degrees using 2400 counts/rev: %.3f deg\n", ...
    meanReturnErrorCounts / EXPECTED_COUNTS_PER_REV * 360);

fprintf("\nFull-sweep linear fit:\n");
fprintf("  count_delta = %.6f * expected_deg + %.6f\n", ...
    p_deg_to_count(1), p_deg_to_count(2));
fprintf("  Fit-implied counts/rev: %.2f\n", counts_per_rev_fit);
fprintf("  Fit-implied counts/rev error: %.2f counts (%.2f%%)\n", ...
    fitErrorCounts, fitErrorPct);

fprintf("\nPractical interpretation:\n");

if abs(halfRevErrorPct) <= 5
    fprintf("  PASS: Endpoint result is within 5%% of 1200 counts.\n");
    fprintf("  It is reasonable to use 2400 counts/rev for servo angle conversion.\n");
elseif abs(halfRevErrorPct) <= 12.5
    fprintf("  MARGINAL: Endpoint result is in the rough usable range, but not tight.\n");
    fprintf("  Check belt tension, endpoint assumptions, and servo settling.\n");
else
    fprintf("  FAIL / INVESTIGATE: Endpoint result is far from 1200 counts.\n");
    fprintf("  Check encoder counts/rev, quadrature mode, belt ratio, slip, or servo range.\n");
end

fprintf("\nAngle conversion if accepted:\n");
fprintf("  theta_deg = count_delta / %.1f * 360\n", EXPECTED_COUNTS_PER_REV);
fprintf("  theta_rad = count_delta / %.1f * 2*pi\n", EXPECTED_COUNTS_PER_REV);

%% ============================================================
% Save figures
% ============================================================

if SAVE_FIGURES
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    cd(plotDir);
    saveas(1, "servo_half_rev_command_and_count.png");
    saveas(2, "servo_half_rev_command_vs_count.png");
    saveas(3, "servo_half_rev_counts_per_rev.png");

    fprintf("\nSaved figures to:\n  %s\n", plotDir);
end