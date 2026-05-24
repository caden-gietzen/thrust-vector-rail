%% analyze_servo_static_sweep.m
% Analyze static servo PWM sweep data from Pico CSV logs.
%
% Goal:
%   Identify the steady-state static map:
%
%       servo_us -> theta_deg
%
% using the encoder count-to-angle conversion validated in the
% half-revolution calibration:
%
%       theta_deg = count_delta / COUNTS_PER_REV * 360
%
% Expected minimum CSV columns:
%   t_s, servo_us, count
%
% Optional CSV columns supported:
%   sweep_idx, direction, count_zero, count_delta, theta_deg, theta_rad

clear; clc; close all;

%% User Options

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"
ANALYZE_MODE = "all";           % "latest", "single", or "all"

% Used only when ANALYZE_MODE = "single"
dataFile = "servo_static_pwm_to_angle_sweep.csv";

COUNTS_PER_REV = 2400.0;

SERVO_CENTER_US = 1450;
CENTER_FIT_WINDOW_US = 300;

SAVE_FIGURES = false;
SAVE_SUMMARY_TABLE = false;

%% Locate Project Paths

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

fprintf("\nServo static command-to-angle sweep analysis\n");
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

allServoUs = [];
allThetaDeg = [];

%% Analyze Selected Files

for fileIdx = 1:numel(dataFiles)

    dataPath = fullfile(dataDir, dataFiles(fileIdx));

    fprintf("\n============================================================\n");
    fprintf("Analyzing file %d of %d\n", fileIdx, numel(dataFiles));
    fprintf("============================================================\n");
    fprintf("Data file:\n  %s\n", dataPath);

    T = readtable(dataPath);

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

    t = T.t_s;
    servo_us = T.servo_us;
    count = T.count;

    %% Build Count and Angle Signals

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

    if ismember("sweep_idx", string(T.Properties.VariableNames))
        sweep_idx = T.sweep_idx;
    else
        sweep_idx = ones(height(T), 1);
    end

    if ismember("direction", string(T.Properties.VariableNames))
        direction = string(T.direction);
    else
        direction = inferSweepDirection(servo_us);
    end

    allServoUs = [allServoUs; servo_us(:)];
    allThetaDeg = [allThetaDeg; theta_deg(:)];

    %% Fit Static Command-to-Angle Map

    validFit = isfinite(servo_us) & isfinite(theta_deg);

    p_all = polyfit(servo_us(validFit), theta_deg(validFit), 1);

    static_gain_deg_per_us = p_all(1);
    static_intercept_deg = p_all(2);

    theta_fit_deg = polyval(p_all, servo_us);

    if abs(static_gain_deg_per_us) > eps
        neutral_pwm_from_fit = -static_intercept_deg / static_gain_deg_per_us;
    else
        neutral_pwm_from_fit = NaN;
    end

    centerWindow = abs(servo_us - SERVO_CENTER_US) <= CENTER_FIT_WINDOW_US;
    validCenterFit = centerWindow & isfinite(servo_us) & isfinite(theta_deg);

    if nnz(validCenterFit) >= 3
        p_center = polyfit(servo_us(validCenterFit), theta_deg(validCenterFit), 1);
        local_gain_deg_per_us = p_center(1);
        local_intercept_deg = p_center(2);
    else
        p_center = [NaN, NaN];
        local_gain_deg_per_us = NaN;
        local_intercept_deg = NaN;
    end

    %% Estimate Hysteresis Between Up and Down Sweeps

    [hysteresisTable, meanAbsHysteresisDeg, maxAbsHysteresisDeg] = ...
        estimateHysteresisByCommand(servo_us, theta_deg, direction);

    %% Print File Summary

    fprintf("\nSamples: %d\n", height(T));
    fprintf("Servo command range: %.1f to %.1f us\n", min(servo_us), max(servo_us));
    fprintf("Measured angle range: %.3f to %.3f deg\n", min(theta_deg), max(theta_deg));

    fprintf("\nStatic command-to-angle fit:\n");
    fprintf("  theta_deg = %.6f * servo_us + %.6f\n", static_gain_deg_per_us, static_intercept_deg);
    fprintf("  Static gain: %.6f deg/us\n", static_gain_deg_per_us);
    fprintf("  Static gain: %.8f rad/us\n", static_gain_deg_per_us * pi / 180.0);
    fprintf("  Neutral PWM from fit: %.2f us\n", neutral_pwm_from_fit);

    fprintf("\nLocal center fit around %.1f ± %.1f us:\n", SERVO_CENTER_US, CENTER_FIT_WINDOW_US);
    fprintf("  Local gain: %.6f deg/us\n", local_gain_deg_per_us);
    fprintf("  Local gain: %.8f rad/us\n", local_gain_deg_per_us * pi / 180.0);

    fprintf("\nHysteresis / repeatability estimate:\n");
    fprintf("  Mean abs up-down difference: %.4f deg\n", meanAbsHysteresisDeg);
    fprintf("  Max abs up-down difference: %.4f deg\n", maxAbsHysteresisDeg);

    %% Store File Summary

    newRow = table( ...
        dataFiles(fileIdx), ...
        height(T), ...
        min(servo_us), ...
        max(servo_us), ...
        min(theta_deg), ...
        max(theta_deg), ...
        static_gain_deg_per_us, ...
        static_gain_deg_per_us * pi / 180.0, ...
        neutral_pwm_from_fit, ...
        local_gain_deg_per_us, ...
        local_gain_deg_per_us * pi / 180.0, ...
        meanAbsHysteresisDeg, ...
        maxAbsHysteresisDeg, ...
        'VariableNames', { ...
            'file', ...
            'num_samples', ...
            'servo_us_min', ...
            'servo_us_max', ...
            'theta_deg_min', ...
            'theta_deg_max', ...
            'static_gain_deg_per_us', ...
            'static_gain_rad_per_us', ...
            'neutral_pwm_from_fit_us', ...
            'local_gain_deg_per_us', ...
            'local_gain_rad_per_us', ...
            'mean_abs_hysteresis_deg', ...
            'max_abs_hysteresis_deg' ...
        });

    summaryRows = [summaryRows; newRow];

    %% Plot Individual File Diagnostics

    figure;
    plot(t, servo_us, "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("Servo Command (\mus)");
    title("Servo Static Sweep: Command vs Time");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    figure;
    plot(t, theta_deg, "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("Measured Servo Angle (deg)");
    title("Servo Static Sweep: Measured Angle vs Time");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    figure;
    plot(servo_us, theta_deg, "o", "LineWidth", 1.2);
    hold on;
    plot(servo_us, theta_fit_deg, "-", "LineWidth", 1.5);
    xline(SERVO_CENTER_US, "--", "Configured center");
    yline(0, "--", "Zero angle");
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Measured Servo Angle (deg)");
    title("Static Command-to-Angle Map");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Measured", "Linear fit", "Location", "best");

    figure;
    hold on;
    idxUp = direction == "up";
    idxDown = direction == "down";
    plot(servo_us(idxUp), theta_deg(idxUp), "o", "LineWidth", 1.1);
    plot(servo_us(idxDown), theta_deg(idxDown), "o", "LineWidth", 1.1);
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Measured Servo Angle (deg)");
    title("Static Sweep Direction Comparison");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Up sweep", "Down sweep", "Location", "best");

    if ~isempty(hysteresisTable)
        figure;
        plot(hysteresisTable.servo_us, hysteresisTable.theta_up_deg - hysteresisTable.theta_down_deg, "o-", "LineWidth", 1.2);
        yline(0, "--");
        grid on;
        xlabel("Servo Command (\mus)");
        ylabel("Up - Down Angle Difference (deg)");
        title("Static Sweep Hysteresis Estimate");
        subtitle(dataFiles(fileIdx), "Interpreter", "none");
    end

    %% Save Individual Figures

    if SAVE_FIGURES
        if ~exist(plotDir, "dir")
            mkdir(plotDir);
        end

        fileStem = erase(dataFiles(fileIdx), ".csv");
        fileStem = sanitizeFileStem(fileStem);

        figBase = 5 * (fileIdx - 1);

        savedPaths = [
            saveFigurePair(figure(figBase + 1), plotDir, fileStem + "_command_vs_time")
            saveFigurePair(figure(figBase + 2), plotDir, fileStem + "_angle_vs_time")
            saveFigurePair(figure(figBase + 3), plotDir, fileStem + "_static_command_to_angle_map")
            saveFigurePair(figure(figBase + 4), plotDir, fileStem + "_direction_comparison")
        ];

        if ~isempty(hysteresisTable)
            savedPaths = [
                savedPaths
                saveFigurePair(figure(figBase + 5), plotDir, fileStem + "_hysteresis_estimate")
            ];
        end

        fprintf("\nSaved figures for %s:\n", dataFiles(fileIdx));
        for p = 1:numel(savedPaths)
            fprintf("  %s\n", savedPaths(p));
        end
    end
end

%% Print Combined Summary

fprintf("\n============================================================\n");
fprintf("Combined static servo sweep summary\n");
fprintf("============================================================\n");

disp(summaryRows);

validCombined = isfinite(allServoUs) & isfinite(allThetaDeg);
p_combined = polyfit(allServoUs(validCombined), allThetaDeg(validCombined), 1);

combined_gain_deg_per_us = p_combined(1);
combined_intercept_deg = p_combined(2);
combined_gain_rad_per_us = combined_gain_deg_per_us * pi / 180.0;

if abs(combined_gain_deg_per_us) > eps
    combined_neutral_pwm_us = -combined_intercept_deg / combined_gain_deg_per_us;
else
    combined_neutral_pwm_us = NaN;
end

fprintf("\nCombined static command-to-angle fit:\n");
fprintf("  theta_deg = %.6f * servo_us + %.6f\n", combined_gain_deg_per_us, combined_intercept_deg);
fprintf("  Static gain: %.6f deg/us\n", combined_gain_deg_per_us);
fprintf("  Static gain: %.8f rad/us\n", combined_gain_rad_per_us);
fprintf("  Neutral PWM from fit: %.2f us\n", combined_neutral_pwm_us);

fprintf("\nStatic map interpretation:\n");
fprintf("  The static sweep provides the measured servo_us -> theta_deg map.\n");
fprintf("  This map uses the Section 2 encoder conversion of %.1f counts/rev.\n", COUNTS_PER_REV);
fprintf("  Use this map to define the command range and static gain for dynamic tests.\n");

%% Save Summary Table

if SAVE_SUMMARY_TABLE
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    summaryPath = fullfile(plotDir, "servo_static_sweep_summary_table.csv");
    writetable(summaryRows, summaryPath);

    fprintf("\nSaved summary table to:\n  %s\n", summaryPath);
end

%% Local Function: Infer Sweep Direction

function direction = inferSweepDirection(servo_us)

    direction = strings(size(servo_us));

    dServo = [0; diff(servo_us)];

    direction(dServo >= 0) = "up";
    direction(dServo < 0) = "down";

    % First sample inherits the first nonzero direction if possible.
    firstNonzero = find(dServo ~= 0, 1, "first");

    if ~isempty(firstNonzero)
        direction(1) = direction(firstNonzero);
    else
        direction(:) = "unknown";
    end
end

%% Local Function: Estimate Hysteresis

function [hysteresisTable, meanAbsHysteresisDeg, maxAbsHysteresisDeg] = ...
    estimateHysteresisByCommand(servo_us, theta_deg, direction)

    uniqueCommands = unique(servo_us);

    rows = [];

    for k = 1:numel(uniqueCommands)
        cmd = uniqueCommands(k);

        idxUp = servo_us == cmd & direction == "up";
        idxDown = servo_us == cmd & direction == "down";

        if any(idxUp) && any(idxDown)
            thetaUp = mean(theta_deg(idxUp), "omitnan");
            thetaDown = mean(theta_deg(idxDown), "omitnan");

            rows = [rows; cmd, thetaUp, thetaDown, thetaUp - thetaDown];
        end
    end

    if isempty(rows)
        hysteresisTable = table();
        meanAbsHysteresisDeg = NaN;
        maxAbsHysteresisDeg = NaN;
        return;
    end

    hysteresisTable = array2table(rows, ...
        'VariableNames', {'servo_us', 'theta_up_deg', 'theta_down_deg', 'theta_up_minus_down_deg'});

    diffs = hysteresisTable.theta_up_minus_down_deg;

    meanAbsHysteresisDeg = mean(abs(diffs), "omitnan");
    maxAbsHysteresisDeg = max(abs(diffs), [], "omitnan");
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