%% validate_servo_prps_data.m
% Validate servo PRPS encoder data before frequency-domain identification.
%
% Purpose:
%   Catch bad encoder / servo logs before running:
%       analyze_servo_prps_frequency_fit.m
%
% Checks:
%   1. Required columns exist
%   2. Sample time is reasonable
%   3. Encoder angle does not drift to unreasonable values
%   4. Center-command behavior is reasonable
%   5. Baseline / settle segments stay near zero
%   6. Command and measured angle are finite
%
% Expected mirrored data folder:
%   .../candidate

clear; clc; close all;

%% User options

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"

USE_LATEST_CSV = true;

% Used only when USE_LATEST_CSV = false
dataFile = "servo_prps_log_amp100_seed4001_set01.csv";

SAVE_FIGURES = false;

COUNTS_PER_REV = encoderAngleScale().counts_per_rev;   % spec-derived (1:1 GT2 16T)
DEFAULT_SERVO_CENTER_US = 1450;

% Validation thresholds
MAX_REASONABLE_ABS_THETA_DEG = 120;       % hard sanity bound
MAX_BASELINE_ABS_THETA_DEG   = 5;         % during baseline/settle/center
MAX_CENTER_CMD_ERROR_US      = 2;         % consider PWM near center
MAX_CENTER_ABS_THETA_DEG     = 5;         % measured angle when command near center
MAX_SAMPLE_DT_JITTER_RATIO   = 0.35;      % std(dt)/median(dt)
EXPECTED_DT_S                = 0.010;     % 10 ms command/update period
MAX_DT_ERROR_RATIO           = 0.25;      % median dt can be +/-25%

%% Locate mirrored folders

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No CSV files found in:\n%s", dataDir);
end

[~, sortIdx] = sort(string({csvFiles.name}));
csvFiles = csvFiles(sortIdx);

if USE_LATEST_CSV
    dataFile = selectLatestIndexedCsv(csvFiles);
end

dataPath = fullfile(dataDir, dataFile);

fprintf("\nValidating servo PRPS file:\n");
fprintf("  %s\n", dataPath);

%% Load and normalize

T = readtable(dataPath);

T = normalizeServoPrpsValidationTable(T, COUNTS_PER_REV, DEFAULT_SERVO_CENTER_US);

requiredVars = [
    "t_s"
    "servo_us"
    "command_delta_us"
    "count_delta"
    "theta_rad"
    "theta_deg"
    "segment"
];

for k = 1:numel(requiredVars)
    if ~ismember(requiredVars(k), string(T.Properties.VariableNames))
        error("Missing required column after normalization: %s", requiredVars(k));
    end
end

%% Basic signal extraction

t = T.t_s;
servo_us = T.servo_us;
command_delta_us = T.command_delta_us;
count_delta = T.count_delta;
theta_rad = T.theta_rad;
theta_deg = T.theta_deg;
segment = string(T.segment);

validFinite = isfinite(t) & ...
              isfinite(servo_us) & ...
              isfinite(command_delta_us) & ...
              isfinite(count_delta) & ...
              isfinite(theta_rad) & ...
              isfinite(theta_deg);

Tvalid = T(validFinite, :);

if height(Tvalid) < 10
    error("Too few finite samples after filtering.");
end

t = Tvalid.t_s;
servo_us = Tvalid.servo_us;
command_delta_us = Tvalid.command_delta_us;
count_delta = Tvalid.count_delta;
theta_rad = Tvalid.theta_rad;
theta_deg = Tvalid.theta_deg;
segment = string(Tvalid.segment);

%% Check 1: sample timing

dt = diff(t);
dt = dt(isfinite(dt) & dt > 0);

medianDt = median(dt, "omitnan");
meanDt = mean(dt, "omitnan");
stdDt = std(dt, "omitnan");
maxDt = max(dt, [], "omitnan");

dtJitterRatio = stdDt / medianDt;
dtErrorRatio = abs(medianDt - EXPECTED_DT_S) / EXPECTED_DT_S;

passTiming = dtJitterRatio <= MAX_SAMPLE_DT_JITTER_RATIO && ...
             dtErrorRatio <= MAX_DT_ERROR_RATIO;

%% Check 2: global angle sanity

maxAbsThetaDeg = max(abs(theta_deg), [], "omitnan");
thetaRangeDeg = max(theta_deg, [], "omitnan") - min(theta_deg, [], "omitnan");

passAngleBound = maxAbsThetaDeg <= MAX_REASONABLE_ABS_THETA_DEG;

%% Check 3: baseline / settle behavior

baselineMask = contains(lower(segment), "baseline") | ...
               contains(lower(segment), "settle");

if any(baselineMask)
    baselineAbsThetaDeg = abs(theta_deg(baselineMask));
    maxBaselineAbsThetaDeg = max(baselineAbsThetaDeg, [], "omitnan");
    meanBaselineAbsThetaDeg = mean(baselineAbsThetaDeg, "omitnan");
else
    maxBaselineAbsThetaDeg = NaN;
    meanBaselineAbsThetaDeg = NaN;
end

passBaseline = ~any(baselineMask) || ...
               maxBaselineAbsThetaDeg <= MAX_BASELINE_ABS_THETA_DEG;

%% Check 4: center-command behavior

centerMask = abs(servo_us - DEFAULT_SERVO_CENTER_US) <= MAX_CENTER_CMD_ERROR_US;

if any(centerMask)
    centerAbsThetaDeg = abs(theta_deg(centerMask));
    maxCenterAbsThetaDeg = max(centerAbsThetaDeg, [], "omitnan");
    meanCenterAbsThetaDeg = mean(centerAbsThetaDeg, "omitnan");
    stdCenterThetaDeg = std(theta_deg(centerMask), "omitnan");
else
    maxCenterAbsThetaDeg = NaN;
    meanCenterAbsThetaDeg = NaN;
    stdCenterThetaDeg = NaN;
end

passCenter = ~any(centerMask) || ...
             maxCenterAbsThetaDeg <= MAX_CENTER_ABS_THETA_DEG;

%% Check 5: command sanity

minServoUs = min(servo_us, [], "omitnan");
maxServoUs = max(servo_us, [], "omitnan");
maxAbsCommandDeltaUs = max(abs(command_delta_us), [], "omitnan");

passCommandFinite = all(isfinite(servo_us)) && all(isfinite(command_delta_us));

%% Optional per-period drift check

hasPeriodIndex = ismember("period_index", string(Tvalid.Properties.VariableNames));

periodDriftTable = table();

if hasPeriodIndex
    periodIds = unique(Tvalid.period_index, "stable");
    periodIds = periodIds(periodIds >= 0);

    period_id = [];
    theta_start_deg = [];
    theta_end_deg = [];
    theta_drift_deg = [];

    for k = 1:numel(periodIds)
        idx = Tvalid.period_index == periodIds(k);

        if nnz(idx) < 2
            continue;
        end

        thetaThis = Tvalid.theta_deg(idx);

        period_id(end+1, 1) = periodIds(k); %#ok<SAGROW>
        theta_start_deg(end+1, 1) = thetaThis(1); %#ok<SAGROW>
        theta_end_deg(end+1, 1) = thetaThis(end); %#ok<SAGROW>
        theta_drift_deg(end+1, 1) = thetaThis(end) - thetaThis(1); %#ok<SAGROW>
    end

    periodDriftTable = table( ...
        period_id, ...
        theta_start_deg, ...
        theta_end_deg, ...
        theta_drift_deg ...
    );
end

%% Print validation summary

overallPass = passTiming && ...
              passAngleBound && ...
              passBaseline && ...
              passCenter && ...
              passCommandFinite;

fprintf("\n============================================================\n");
fprintf("Servo PRPS validation summary\n");
fprintf("============================================================\n");

fprintf("\nData file:\n");
fprintf("  %s\n", dataPath);

fprintf("\nSamples:\n");
fprintf("  Raw samples: %d\n", height(T));
fprintf("  Finite valid samples: %d\n", height(Tvalid));

fprintf("\nSample timing:\n");
fprintf("  Expected dt: %.5f s\n", EXPECTED_DT_S);
fprintf("  Median dt:   %.5f s\n", medianDt);
fprintf("  Mean dt:     %.5f s\n", meanDt);
fprintf("  Std dt:      %.5f s\n", stdDt);
fprintf("  Max dt:      %.5f s\n", maxDt);
fprintf("  Jitter ratio std/median: %.3f\n", dtJitterRatio);
fprintf("  Timing check: %s\n", passFail(passTiming));

fprintf("\nCommand sanity:\n");
fprintf("  Servo range: %.1f to %.1f us\n", minServoUs, maxServoUs);
fprintf("  Max abs command delta: %.1f us\n", maxAbsCommandDeltaUs);
fprintf("  Command finite check: %s\n", passFail(passCommandFinite));

fprintf("\nAngle sanity:\n");
fprintf("  Theta range: %.3f deg\n", thetaRangeDeg);
fprintf("  Max abs theta: %.3f deg\n", maxAbsThetaDeg);
fprintf("  Angle bound check: %s\n", passFail(passAngleBound));

fprintf("\nBaseline / settle check:\n");
if any(baselineMask)
    fprintf("  Max abs baseline theta: %.3f deg\n", maxBaselineAbsThetaDeg);
    fprintf("  Mean abs baseline theta: %.3f deg\n", meanBaselineAbsThetaDeg);
else
    fprintf("  No baseline/settle rows found.\n");
end
fprintf("  Baseline check: %s\n", passFail(passBaseline));

fprintf("\nCenter-command check:\n");
if any(centerMask)
    fprintf("  Center-command samples: %d\n", nnz(centerMask));
    fprintf("  Max abs theta at center command: %.3f deg\n", maxCenterAbsThetaDeg);
    fprintf("  Mean abs theta at center command: %.3f deg\n", meanCenterAbsThetaDeg);
    fprintf("  Std theta at center command: %.3f deg\n", stdCenterThetaDeg);
else
    fprintf("  No samples near center command %.1f us.\n", DEFAULT_SERVO_CENTER_US);
end
fprintf("  Center-command check: %s\n", passFail(passCenter));

if ~isempty(periodDriftTable)
    fprintf("\nPeriod drift summary:\n");
    disp(periodDriftTable);

    fprintf("  Max abs period drift: %.3f deg\n", ...
        max(abs(periodDriftTable.theta_drift_deg), [], "omitnan"));
end

fprintf("\nOVERALL VALIDATION: %s\n", passFail(overallPass));

if ~overallPass
    fprintf("\nInterpretation:\n");
    fprintf("  Do not blindly trust the frequency fit yet.\n");
    fprintf("  Inspect the time plots and center-command behavior first.\n");
else
    fprintf("\nInterpretation:\n");
    fprintf("  Data is reasonable enough to proceed to frequency-domain fitting.\n");
end

%% Plots

figure("Name", "Servo PRPS Validation - Command and Angle");
yyaxis left;
plot(t, theta_deg, "LineWidth", 1.1);
ylabel("Measured Servo Angle (deg)");

yyaxis right;
plot(t, servo_us, "LineWidth", 1.0);
ylabel("Servo PWM Command (\mus)");

grid on;
xlabel("Time (s)");
title("Servo PRPS Validation: Command and Measured Angle");

figure("Name", "Servo PRPS Validation - Count Delta");
plot(t, count_delta, "LineWidth", 1.1);
grid on;
xlabel("Time (s)");
ylabel("Encoder Count Delta");
title("Encoder Count Delta Over Time");

figure("Name", "Servo PRPS Validation - Center Command Samples");
hold on; grid on;

plot(t, theta_deg, "LineWidth", 1.0, "DisplayName", "All samples");

if any(centerMask)
    scatter(t(centerMask), theta_deg(centerMask), 20, "filled", ...
        "DisplayName", "Samples where PWM near center");
end

yline(MAX_CENTER_ABS_THETA_DEG, "--", "+ center threshold");
yline(-MAX_CENTER_ABS_THETA_DEG, "--", "- center threshold");

xlabel("Time (s)");
ylabel("Measured Servo Angle (deg)");
title("Theta When Servo Command Is Near Center");
legend("Location", "best");

figure("Name", "Servo PRPS Validation - Sample Time");
plot(t(2:end), diff(t), "LineWidth", 1.1);
grid on;
xlabel("Time (s)");
ylabel("Sample Interval \Deltat (s)");
title("Sample Timing Diagnostic");

if ~isempty(periodDriftTable)
    figure("Name", "Servo PRPS Validation - Period Drift");
    bar(periodDriftTable.period_id, periodDriftTable.theta_drift_deg);
    grid on;
    xlabel("Period Index");
    ylabel("End - Start Angle Drift (deg)");
    title("Per-Period Angle Drift");
end

%% Save figures

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
            name = sprintf("servo_prps_validation_%02d", k);
        end

        name = regexprep(name, "[^\w\d_-]", "_");
        exportgraphics(fig, fullfile(plotDir, name + ".png"), "Resolution", 200);
    end

    fprintf("\nSaved validation figures to:\n  %s\n", plotDir);
end

%% Local functions

function T = normalizeServoPrpsValidationTable(T, countsPerRev, defaultServoCenterUs)
    vars = string(T.Properties.VariableNames);

    if ~ismember("t_s", vars)
        if ismember("t_ms", vars)
            T.t_ms = forceNumericLocal(T.t_ms);
            T.t_s = T.t_ms / 1000;
        else
            error("CSV must contain either t_s or t_ms.");
        end
    else
        T.t_s = forceNumericLocal(T.t_s);
    end

    if ~ismember("servo_us", vars)
        error("CSV must contain servo_us.");
    end

    T.servo_us = forceNumericLocal(T.servo_us);

    if ~ismember("command_delta_us", vars)
        T.command_delta_us = T.servo_us - defaultServoCenterUs;
    else
        T.command_delta_us = forceNumericLocal(T.command_delta_us);
    end

    if ~ismember("count_delta", vars)
        if ismember("count", vars) && ismember("count_zero", vars)
            T.count = forceNumericLocal(T.count);
            T.count_zero = forceNumericLocal(T.count_zero);
            T.count_delta = T.count - T.count_zero;
        else
            error("CSV must contain count_delta or both count and count_zero.");
        end
    else
        T.count_delta = forceNumericLocal(T.count_delta);
    end

    if ~ismember("theta_rad", vars)
        T.theta_rad = T.count_delta / countsPerRev * 2*pi;
    else
        T.theta_rad = forceNumericLocal(T.theta_rad);
    end

    if ~ismember("theta_deg", vars)
        T.theta_deg = T.theta_rad * 180/pi;
    else
        T.theta_deg = forceNumericLocal(T.theta_deg);
    end

    if ~ismember("segment", vars)
        T.segment = repmat("prps", height(T), 1);
    else
        T.segment = string(T.segment);
    end

    if ismember("period_index", vars)
        T.period_index = forceNumericLocal(T.period_index);
    end

    if ismember("period_sample_index", vars)
        T.period_sample_index = forceNumericLocal(T.period_sample_index);
    end
end

function x = forceNumericLocal(x)
    if isnumeric(x)
        x = double(x);
        return;
    end

    if iscell(x)
        x = string(x);
    end

    if isstring(x) || ischar(x) || iscategorical(x)
        x = str2double(string(x));
        return;
    end

    x = double(x);
end

function s = passFail(tf)
    if tf
        s = "PASS";
    else
        s = "FAIL";
    end
end