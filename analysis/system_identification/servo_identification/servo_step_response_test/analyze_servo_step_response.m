%% analyze_servo_step_response.m
% Analyze servo PWM step-response data from Pico CSV logs.
%
% Goal:
%   Identify command-to-angle servo dynamics from discrete PWM steps:
%
%       servo_us(t) -> theta_deg(t)
%
% using the encoder count-to-angle conversion validated in the
% half-revolution calibration:
%
%       theta_deg = count_delta / COUNTS_PER_REV * 360
%
% Expected minimum CSV columns:
%   t_s, servo_us, count
%
% Preferred CSV columns produced by servo_step_response_test.py:
%   t_ms, t_s, trial_idx, case_idx, case_label, rep_idx, phase, segment,
%   step_start_us, step_end_us, step_delta_us, servo_us,
%   theta_cmd_deg, theta_cmd_rad,
%   count, count_zero, count_delta, theta_deg, theta_rad,
%   time_since_step_ms, time_since_step_s
%
% Main analysis outputs:
%   - Whole-file command and angle time histories
%   - Per-case step response overlays
%   - Per-case normalized response overlays
%   - Summary table with delay, rise time, settling time, overshoot,
%     measured gain, and static-map gain consistency

clear; clc; close all;

%% User Options

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"
ANALYZE_MODE = "all";           % "latest", "single", or "all"

% Used only when ANALYZE_MODE = "single"
dataFile = "servo_step_response_test.csv";

COUNTS_PER_REV = 2400.0;

% Static map from the static servo sweep.
% theta_deg = STATIC_GAIN_DEG_PER_US * servo_us + STATIC_INTERCEPT_DEG
STATIC_GAIN_DEG_PER_US = -0.091092;
STATIC_INTERCEPT_DEG = 130.329728;

SERVO_CENTER_US = 1430.75;

% Step metric settings.
PRE_STEP_TAIL_FRACTION = 0.40;       % last 40% of pre-step hold estimates theta_initial
POST_STEP_TAIL_FRACTION = 0.25;      % last 25% of step response estimates theta_final
MIN_STEP_RESPONSE_SAMPLES = 5;
SETTLING_BAND_FRACTION_2 = 0.02;
SETTLING_BAND_FRACTION_5 = 0.05;
DELAY_THRESHOLD_FRACTION = 0.02;
PLOT_EACH_REP = false;

SAVE_FIGURES = false;
SAVE_SUMMARY_TABLE = false;

%% Locate Project Paths

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

fprintf("\nServo step-response analysis\n");
fprintf("Dataset status folder: %s\n", DATASET_STATUS);
fprintf("Data directory:\n  %s\n", dataDir);

%% Select Data Files

csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No CSV files found in %s folder:\n%s", DATASET_STATUS, dataDir);
end

switch ANALYZE_MODE
    case "latest"
        dataFiles = string(selectLatestIndexedCsv(csvFiles));

    case "single"
        dataFiles = string(dataFile);

    case "all"
        dataFiles = string({csvFiles.name});

    otherwise
        error("Unknown ANALYZE_MODE: %s. Use 'latest', 'single', or 'all'.", ANALYZE_MODE);
end

fprintf("Analysis mode: %s\n", ANALYZE_MODE);
fprintf("Files selected:\n");

for k = 1:numel(dataFiles)
    fprintf("  %s\n", dataFiles(k));
end

%% Initialize Summary Storage

summaryRows = table();
allStepRows = table();

%% Analyze Selected Files

for fileIdx = 1:numel(dataFiles)

    dataPath = fullfile(dataDir, dataFiles(fileIdx));

    fprintf("\n============================================================\n");
    fprintf("Analyzing file %d of %d\n", fileIdx, numel(dataFiles));
    fprintf("============================================================\n");
    fprintf("Data file:\n  %s\n", dataPath);

    opts = detectImportOptions(dataPath, ...
        "FileType", "text", ...
        "Delimiter", ",", ...
        "VariableNamingRule", "preserve");

    T = readtable(dataPath, opts);

    % Clean and standardize variable names.
    rawNames = string(T.Properties.VariableNames);
    cleanNames = strtrim(rawNames);
    cleanNames = erase(cleanNames, char(65279));   % remove UTF-8 BOM if present
    cleanNames = matlab.lang.makeValidName(cleanNames);

    T.Properties.VariableNames = cellstr(cleanNames);

    disp("Imported variable names:");
    disp(string(T.Properties.VariableNames)');

    requiredVars = [
        "t_s"
        "servo_us"
        "count"
    ];

    for k = 1:numel(requiredVars)
        if ~ismember(requiredVars(k), string(T.Properties.VariableNames))
            error("Missing required CSV column in %s: %s", dataFiles(fileIdx), requiredVars(k));
        end
    end

    %% Standardize Signals

    t = T.t_s;
    servo_us = T.servo_us;
    count = T.count;

    if ismember("count_delta", string(T.Properties.VariableNames))
        count_delta = T.count_delta;
    elseif ismember("count_zero", string(T.Properties.VariableNames))
        count_delta = T.count - T.count_zero;
    else
        count_delta = T.count - T.count(1);
        warning("No count_delta or count_zero column found in %s. Using count - first count.", dataFiles(fileIdx));
    end

    theta_deg = count_delta / COUNTS_PER_REV * 360.0;
    theta_rad = count_delta / COUNTS_PER_REV * 2.0 * pi;

    if ismember("theta_deg", string(T.Properties.VariableNames))
        theta_deg = T.theta_deg;
    end

    if ismember("theta_rad", string(T.Properties.VariableNames))
        theta_rad = T.theta_rad;
    end

    if ismember("theta_cmd_deg", string(T.Properties.VariableNames))
        theta_cmd_deg = T.theta_cmd_deg;
    else
        theta_cmd_deg = STATIC_GAIN_DEG_PER_US .* servo_us + STATIC_INTERCEPT_DEG;
    end

    if ismember("theta_cmd_rad", string(T.Properties.VariableNames))
        theta_cmd_rad = T.theta_cmd_rad;
    else
        theta_cmd_rad = theta_cmd_deg * pi / 180.0;
    end

    if ismember("phase", string(T.Properties.VariableNames))
        phase = string(T.phase);
        phase(ismissing(phase) | strlength(strtrim(phase)) == 0) = "unknown";
    else
        phase = strings(height(T), 1);
        phase(:) = "unknown";
    end

    if ismember("segment", string(T.Properties.VariableNames))
        segment = string(T.segment);
        segment(ismissing(segment) | strlength(strtrim(segment)) == 0) = phase(ismissing(segment) | strlength(strtrim(segment)) == 0);
    else
        segment = phase;
    end

    if ismember("case_idx", string(T.Properties.VariableNames))
        case_idx = T.case_idx;
    else
        case_idx = inferCaseIndexFromSteps(T);
    end

    if ismember("case_label", string(T.Properties.VariableNames))
        case_label = string(T.case_label);
        case_label(ismissing(case_label) | strlength(strtrim(case_label)) == 0) = "unknown_case";
    else
        case_label = "case_" + string(case_idx);
    end

    if ismember("rep_idx", string(T.Properties.VariableNames))
        rep_idx = T.rep_idx;
    else
        rep_idx = ones(height(T), 1);
    end

    if ismember("step_start_us", string(T.Properties.VariableNames))
        step_start_us = T.step_start_us;
    else
        step_start_us = NaN(height(T), 1);
    end

    if ismember("step_end_us", string(T.Properties.VariableNames))
        step_end_us = T.step_end_us;
    else
        step_end_us = NaN(height(T), 1);
    end

    if ismember("step_delta_us", string(T.Properties.VariableNames))
        step_delta_us = T.step_delta_us;
    else
        step_delta_us = step_end_us - step_start_us;
    end

    if ismember("time_since_step_s", string(T.Properties.VariableNames))
        time_since_step_s = T.time_since_step_s;
    elseif ismember("time_since_step_ms", string(T.Properties.VariableNames))
        time_since_step_s = T.time_since_step_ms / 1000.0;
    else
        time_since_step_s = inferTimeSinceStep(t, phase, case_idx, rep_idx);
    end

    %% Build Standardized Table for File

    D = table( ...
        t, servo_us, theta_cmd_deg, theta_cmd_rad, count, count_delta, theta_deg, theta_rad, ...
        phase, segment, case_idx, case_label, rep_idx, ...
        step_start_us, step_end_us, step_delta_us, time_since_step_s, ...
        'VariableNames', { ...
            't_s', 'servo_us', 'theta_cmd_deg', 'theta_cmd_rad', 'count', 'count_delta', 'theta_deg', 'theta_rad', ...
            'phase', 'segment', 'case_idx', 'case_label', 'rep_idx', ...
            'step_start_us', 'step_end_us', 'step_delta_us', 'time_since_step_s' ...
        });

    %% Print File-Level Summary

    fprintf("\nSamples: %d\n", height(D));
    fprintf("Servo command range: %.1f to %.1f us\n", min(D.servo_us), max(D.servo_us));
    fprintf("Measured angle range: %.3f to %.3f deg\n", min(D.theta_deg), max(D.theta_deg));
    fprintf("Commanded angle range from static map: %.3f to %.3f deg\n", min(D.theta_cmd_deg), max(D.theta_cmd_deg));

    validStep = D.phase == "step_response" & isfinite(D.time_since_step_s);
    fprintf("Step-response samples: %d\n", nnz(validStep));

    %% Plot Whole-File Diagnostics

    figure;
    plot(D.t_s, D.servo_us, "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("Servo Command (\mus)");
    title("Servo Step Response: Command vs Time");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    figure;
    plot(D.t_s, D.theta_deg, "LineWidth", 1.3);
    hold on;
    plot(D.t_s, D.theta_cmd_deg, "--", "LineWidth", 1.1);
    grid on;
    xlabel("Time (s)");
    ylabel("Servo Angle (deg)");
    title("Servo Step Response: Measured Angle vs Static-Map Command Angle");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Measured angle", "Static-map command angle", "Location", "best");

    %% Analyze Each Case/Repetition

    groupKeys = unique(D(:, {'case_idx', 'case_label', 'rep_idx'}), "rows", "stable");

    fileStepRows = table();

    for g = 1:height(groupKeys)

        thisCaseIdx = groupKeys.case_idx(g);
        thisCaseLabel = groupKeys.case_label(g);
        thisRepIdx = groupKeys.rep_idx(g);

        idxGroup = D.case_idx == thisCaseIdx & ...
                   D.case_label == thisCaseLabel & ...
                   D.rep_idx == thisRepIdx;

        G = D(idxGroup, :);

        idxStep = G.phase == "step_response" & isfinite(G.time_since_step_s);

        if nnz(idxStep) < MIN_STEP_RESPONSE_SAMPLES
            continue;
        end

        stepData = G(idxStep, :);
        stepData = sortrows(stepData, "time_since_step_s");

        idxPre = G.phase == "pre_step_hold";

        if any(idxPre)
            preData = sortrows(G(idxPre, :), "t_s");
            theta_initial_measured_deg = tailMedian(preData.theta_deg, PRE_STEP_TAIL_FRACTION);
            theta_initial_cmd_deg = tailMedian(preData.theta_cmd_deg, PRE_STEP_TAIL_FRACTION);
        else
            theta_initial_measured_deg = stepData.theta_deg(1);
            theta_initial_cmd_deg = STATIC_GAIN_DEG_PER_US * stepData.step_start_us(1) + STATIC_INTERCEPT_DEG;
        end

        theta_final_measured_deg = tailMedian(stepData.theta_deg, POST_STEP_TAIL_FRACTION);
        theta_final_cmd_deg = STATIC_GAIN_DEG_PER_US * stepData.step_end_us(1) + STATIC_INTERCEPT_DEG;

        measured_delta_deg = theta_final_measured_deg - theta_initial_measured_deg;
        commanded_delta_deg = theta_final_cmd_deg - theta_initial_cmd_deg;

        if abs(measured_delta_deg) < 1e-9
            y_norm = NaN(size(stepData.theta_deg));
        else
            y_norm = (stepData.theta_deg - theta_initial_measured_deg) ./ measured_delta_deg;
        end

        if abs(commanded_delta_deg) < 1e-9
            y_cmd_norm = NaN(size(stepData.theta_cmd_deg));
        else
            y_cmd_norm = (stepData.theta_cmd_deg - theta_initial_cmd_deg) ./ commanded_delta_deg;
        end

        metrics = estimateStepMetrics( ...
            stepData.time_since_step_s, ...
            stepData.theta_deg, ...
            theta_initial_measured_deg, ...
            theta_final_measured_deg, ...
            DELAY_THRESHOLD_FRACTION, ...
            SETTLING_BAND_FRACTION_2, ...
            SETTLING_BAND_FRACTION_5);

        effective_gain_deg_per_us = measured_delta_deg / stepData.step_delta_us(1);
        static_gain_from_step_deg_per_us = commanded_delta_deg / stepData.step_delta_us(1);
        gain_error_deg_per_us = effective_gain_deg_per_us - static_gain_from_step_deg_per_us;

        newStepRow = table( ...
            dataFiles(fileIdx), ...
            thisCaseIdx, ...
            thisCaseLabel, ...
            thisRepIdx, ...
            stepData.step_start_us(1), ...
            stepData.step_end_us(1), ...
            stepData.step_delta_us(1), ...
            theta_initial_cmd_deg, ...
            theta_final_cmd_deg, ...
            commanded_delta_deg, ...
            theta_initial_measured_deg, ...
            theta_final_measured_deg, ...
            measured_delta_deg, ...
            effective_gain_deg_per_us, ...
            static_gain_from_step_deg_per_us, ...
            gain_error_deg_per_us, ...
            metrics.delay_s, ...
            metrics.t10_s, ...
            metrics.t50_s, ...
            metrics.t90_s, ...
            metrics.rise_time_10_90_s, ...
            metrics.settling_time_2pct_s, ...
            metrics.settling_time_5pct_s, ...
            metrics.overshoot_pct, ...
            metrics.final_error_vs_command_deg, ...
            height(stepData), ...
            'VariableNames', { ...
                'file', ...
                'case_idx', ...
                'case_label', ...
                'rep_idx', ...
                'step_start_us', ...
                'step_end_us', ...
                'step_delta_us', ...
                'theta_initial_cmd_deg', ...
                'theta_final_cmd_deg', ...
                'commanded_delta_deg', ...
                'theta_initial_measured_deg', ...
                'theta_final_measured_deg', ...
                'measured_delta_deg', ...
                'effective_gain_deg_per_us', ...
                'static_gain_from_step_deg_per_us', ...
                'gain_error_deg_per_us', ...
                'delay_s', ...
                't10_s', ...
                't50_s', ...
                't90_s', ...
                'rise_time_10_90_s', ...
                'settling_time_2pct_s', ...
                'settling_time_5pct_s', ...
                'overshoot_pct', ...
                'final_error_vs_command_deg', ...
                'num_step_samples' ...
            });

        fileStepRows = [fileStepRows; newStepRow]; %#ok<AGROW>

        % Store normalized trace rows for overlays.
        traceRows = table( ...
            repmat(dataFiles(fileIdx), height(stepData), 1), ...
            repmat(thisCaseIdx, height(stepData), 1), ...
            repmat(thisCaseLabel, height(stepData), 1), ...
            repmat(thisRepIdx, height(stepData), 1), ...
            stepData.time_since_step_s, ...
            stepData.servo_us, ...
            stepData.theta_cmd_deg, ...
            stepData.theta_deg, ...
            y_norm, ...
            y_cmd_norm, ...
            'VariableNames', { ...
                'file', 'case_idx', 'case_label', 'rep_idx', ...
                'time_since_step_s', 'servo_us', 'theta_cmd_deg', ...
                'theta_deg', 'theta_norm', 'theta_cmd_norm' ...
            });

        allStepRows = [allStepRows; traceRows]; %#ok<AGROW>

        if PLOT_EACH_REP
            figure;
            plot(stepData.time_since_step_s, stepData.theta_deg, "LineWidth", 1.4);
            hold on;
            yline(theta_initial_measured_deg, "--", "Initial measured");
            yline(theta_final_measured_deg, "--", "Final measured");
            yline(theta_final_cmd_deg, ":", "Final command");
            grid on;
            xlabel("Time Since Step (s)");
            ylabel("Measured Servo Angle (deg)");
            title("Servo Step Response");
            subtitle(sprintf("%s | %s | rep %d", dataFiles(fileIdx), thisCaseLabel, thisRepIdx), "Interpreter", "none");
        end
    end

    summaryRows = [summaryRows; fileStepRows]; %#ok<AGROW>

    %% Print File Step Summary

    if isempty(fileStepRows)
        warning("No valid step-response groups found in %s.", dataFiles(fileIdx));
    else
        fprintf("\nStep-response metric summary for file:\n");
        disp(fileStepRows(:, { ...
            'case_label', ...
            'rep_idx', ...
            'step_start_us', ...
            'step_end_us', ...
            'commanded_delta_deg', ...
            'measured_delta_deg', ...
            'delay_s', ...
            'rise_time_10_90_s', ...
            'settling_time_2pct_s', ...
            'overshoot_pct' ...
        }));
    end

    %% Per-File Case Overlay Plots

    if ~isempty(fileStepRows)
        fileTraceRows = allStepRows(allStepRows.file == dataFiles(fileIdx), :);
        uniqueCases = unique(fileTraceRows(:, {'case_idx', 'case_label'}), "rows", "stable");

        for c = 1:height(uniqueCases)
            thisCaseIdx = uniqueCases.case_idx(c);
            thisCaseLabel = uniqueCases.case_label(c);

            idxCase = fileTraceRows.case_idx == thisCaseIdx & fileTraceRows.case_label == thisCaseLabel;
            C = fileTraceRows(idxCase, :);

            figure;
            hold on;
            reps = unique(C.rep_idx, "stable");

            for r = 1:numel(reps)
                idxRep = C.rep_idx == reps(r);
                R = sortrows(C(idxRep, :), "time_since_step_s");
                plot(R.time_since_step_s, R.theta_deg, "LineWidth", 1.2);
            end

            grid on;
            xlabel("Time Since Step (s)");
            ylabel("Measured Servo Angle (deg)");
            title("Servo Step Response Overlay");
            subtitle(sprintf("%s | %s", dataFiles(fileIdx), thisCaseLabel), "Interpreter", "none");
            legend("rep " + string(reps), "Location", "best");

            figure;
            hold on;

            for r = 1:numel(reps)
                idxRep = C.rep_idx == reps(r);
                R = sortrows(C(idxRep, :), "time_since_step_s");
                plot(R.time_since_step_s, R.theta_norm, "LineWidth", 1.2);
            end

            yline(0, "--");
            yline(1, "--");
            yline(0.1, ":");
            yline(0.9, ":");
            grid on;
            xlabel("Time Since Step (s)");
            ylabel("Normalized Response");
            title("Servo Normalized Step Response Overlay");
            subtitle(sprintf("%s | %s", dataFiles(fileIdx), thisCaseLabel), "Interpreter", "none");
            legend("rep " + string(reps), "Location", "best");
        end
    end

    %% Save Individual Figures

    if SAVE_FIGURES
        if ~exist(plotDir, "dir")
            mkdir(plotDir);
        end

        fileStem = erase(dataFiles(fileIdx), ".csv");
        fileStem = sanitizeFileStem(fileStem);

        figHandles = findall(0, "Type", "figure");
        figHandles = sort(figHandles);

        fprintf("\nSaving currently open figures for %s:\n", dataFiles(fileIdx));

        for h = 1:numel(figHandles)
            figName = sprintf("%s_fig_%02d", fileStem, h);
            savedPaths = saveFigurePair(figHandles(h), plotDir, figName);

            for p = 1:numel(savedPaths)
                fprintf("  %s\n", savedPaths(p));
            end
        end
    end

end

%% Print Combined Summary

fprintf("\n============================================================\n");
fprintf("Combined servo step-response summary\n");
fprintf("============================================================\n");

if isempty(summaryRows)
    warning("No step-response metrics were generated.");
else
    disp(summaryRows);

    fprintf("\nGrouped averages by case label:\n");

    groupedSummary = groupsummary(summaryRows, "case_label", "mean", { ...
        'commanded_delta_deg', ...
        'measured_delta_deg', ...
        'effective_gain_deg_per_us', ...
        'delay_s', ...
        'rise_time_10_90_s', ...
        'settling_time_2pct_s', ...
        'settling_time_5pct_s', ...
        'overshoot_pct', ...
        'final_error_vs_command_deg' ...
    });

    disp(groupedSummary);

    fprintf("\nStep-response interpretation:\n");
    fprintf("  Use center-out ±10 deg steps to estimate local small-signal servo dynamics.\n");
    fprintf("  Use center-out ±20 deg steps to check amplitude dependence.\n");
    fprintf("  Use cross-center ±20 deg reversal steps to expose asymmetry, backlash, or hysteresis.\n");
    fprintf("  Compare effective_gain_deg_per_us to the static-map gain %.6f deg/us.\n", STATIC_GAIN_DEG_PER_US);
end

%% Save Summary Table

if SAVE_SUMMARY_TABLE && ~isempty(summaryRows)
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    summaryPath = fullfile(plotDir, "servo_step_response_summary_table.csv");
    writetable(summaryRows, summaryPath);

    groupedSummaryPath = fullfile(plotDir, "servo_step_response_grouped_summary_table.csv");
    writetable(groupedSummary, groupedSummaryPath);

    fprintf("\nSaved summary tables to:\n");
    fprintf("  %s\n", summaryPath);
    fprintf("  %s\n", groupedSummaryPath);
end

%% Local Function: Infer Case Index

function case_idx = inferCaseIndexFromSteps(T)

    n = height(T);
    case_idx = ones(n, 1);

    if all(ismember(["step_start_us", "step_end_us"], string(T.Properties.VariableNames)))
        stepPairs = [T.step_start_us, T.step_end_us];
        [~, ~, case_idx] = unique(stepPairs, "rows", "stable");
    end
end

%% Local Function: Infer Time Since Step

function time_since_step_s = inferTimeSinceStep(t, phase, case_idx, rep_idx)

    time_since_step_s = NaN(size(t));

    groups = unique([case_idx, rep_idx], "rows", "stable");

    for g = 1:size(groups, 1)
        idxGroup = case_idx == groups(g, 1) & rep_idx == groups(g, 2);
        idxStep = idxGroup & phase == "step_response";

        if any(idxStep)
            t0 = t(find(idxStep, 1, "first"));
            time_since_step_s(idxStep) = t(idxStep) - t0;
        end
    end
end

%% Local Function: Tail Median

function yTail = tailMedian(y, tailFraction)

    y = y(isfinite(y));

    if isempty(y)
        yTail = NaN;
        return;
    end

    n = numel(y);
    nTail = max(1, round(tailFraction * n));
    yTail = median(y(end - nTail + 1:end), "omitnan");
end

%% Local Function: Step Metrics

function metrics = estimateStepMetrics( ...
    t, y, y0, yf, delayThresholdFraction, settlingBandFraction2, settlingBandFraction5)

    t = t(:);
    y = y(:);

    valid = isfinite(t) & isfinite(y);

    t = t(valid);
    y = y(valid);

    if numel(t) < 2 || ~isfinite(y0) || ~isfinite(yf) || abs(yf - y0) < 1e-9
        metrics = emptyMetrics();
        return;
    end

    [t, order] = sort(t);
    y = y(order);

    dy = yf - y0;
    yNorm = (y - y0) ./ dy;

    metrics.delay_s = firstCrossingTime(t, yNorm, delayThresholdFraction);
    metrics.t10_s = firstCrossingTime(t, yNorm, 0.10);
    metrics.t50_s = firstCrossingTime(t, yNorm, 0.50);
    metrics.t90_s = firstCrossingTime(t, yNorm, 0.90);

    if isfinite(metrics.t10_s) && isfinite(metrics.t90_s)
        metrics.rise_time_10_90_s = metrics.t90_s - metrics.t10_s;
    else
        metrics.rise_time_10_90_s = NaN;
    end

    metrics.settling_time_2pct_s = settlingTime(t, y, yf, abs(dy) * settlingBandFraction2);
    metrics.settling_time_5pct_s = settlingTime(t, y, yf, abs(dy) * settlingBandFraction5);

    if dy > 0
        peak = max(y);
        metrics.overshoot_pct = max(0, (peak - yf) / abs(dy) * 100.0);
    else
        valley = min(y);
        metrics.overshoot_pct = max(0, (yf - valley) / abs(dy) * 100.0);
    end

    % This value is filled outside if command final is available. Kept here as NaN.
    metrics.final_error_vs_command_deg = NaN;
end

%% Local Function: Empty Metrics

function metrics = emptyMetrics()

    metrics.delay_s = NaN;
    metrics.t10_s = NaN;
    metrics.t50_s = NaN;
    metrics.t90_s = NaN;
    metrics.rise_time_10_90_s = NaN;
    metrics.settling_time_2pct_s = NaN;
    metrics.settling_time_5pct_s = NaN;
    metrics.overshoot_pct = NaN;
    metrics.final_error_vs_command_deg = NaN;
end

%% Local Function: First Crossing Time

function tCross = firstCrossingTime(t, yNorm, threshold)

    idx = find(yNorm >= threshold, 1, "first");

    if isempty(idx)
        tCross = NaN;
        return;
    end

    if idx == 1
        tCross = t(1);
        return;
    end

    t1 = t(idx - 1);
    t2 = t(idx);
    y1 = yNorm(idx - 1);
    y2 = yNorm(idx);

    if abs(y2 - y1) < eps
        tCross = t2;
    else
        tCross = t1 + (threshold - y1) * (t2 - t1) / (y2 - y1);
    end
end

%% Local Function: Settling Time

function ts = settlingTime(t, y, yf, bandAbs)

    inside = abs(y - yf) <= bandAbs;

    ts = NaN;

    for k = 1:numel(t)
        if all(inside(k:end))
            ts = t(k);
            return;
        end
    end
end

%% Local Function: Save Figure Pair

function savedPaths = saveFigurePair(figHandle, plotDir, fileStem)

    pngPath = fullfile(plotDir, fileStem + ".png");
    figPath = fullfile(plotDir, fileStem + ".fig");

    exportgraphics(figHandle, pngPath, "Resolution", 300);
    savefig(figHandle, figPath);

    savedPaths = [string(pngPath); string(figPath)];
end

%% Local Function: Sanitize File Stem

function cleanStem = sanitizeFileStem(rawStem)

    cleanStem = string(rawStem);
    cleanStem = regexprep(cleanStem, "[^\w\-]", "_");
    cleanStem = regexprep(cleanStem, "_+", "_");
    cleanStem = strip(cleanStem, "_");
end
