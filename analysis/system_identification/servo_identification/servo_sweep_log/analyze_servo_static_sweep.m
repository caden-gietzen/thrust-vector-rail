%% analyze_servo_static_sweep.m
% Analyze static servo PWM sweep data from Pico CSV logs.
%
% Goal:
%   Identify the steady-state static map:
%
%       servo_us -> theta_deg
%
% using the spec-derived encoder count-to-angle conversion:
%
%       theta_deg = count_delta * 0.15   (spec-derived, encoderAngleScale())
%
% Expected minimum CSV columns:
%   t_s, servo_us, count
%
% Optional CSV columns supported:
%   sweep_idx, direction, count_zero, count_delta, theta_deg, theta_rad

clear; clc; close all;

%% User Options

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"
ANALYZE_MODE = "single";        % "latest", "single", or "all"

% Used only when ANALYZE_MODE = "single"
dataFile = "2026_06_29_servo_static_pwm_to_angle_sweep_01.csv";

% Spec-derived count-to-angle scale (1:1 GT2 16T belt, 2400 counts/rev, exact);
% single source of truth in encoderAngleScale().
encScale = encoderAngleScale();
COUNTS_PER_REV = encScale.counts_per_rev;

SERVO_CENTER_US = 1431;          % data-derived neutral (was a round 1450)

% Within-branch linearity diagnostic: slide a local linear fit along each
% direction's branch (kept separate so the center-out backlash step is never
% mixed into one fit). Reports the gain ripple within each branch.
LOCAL_GAIN_WINDOW_US = 200;      % half-width of each sliding local-gain fit
LOCAL_GAIN_STEP_US   = 25;       % spacing between sliding-fit centers

SAVE_FIGURES = true;
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

    theta_deg = count_delta * encScale.deg_per_count;
    theta_rad = count_delta * encScale.rad_per_count;

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

    % Normalize discovery-sweep labels (up_from_center / down_from_center) to
    % the up/down convention the plots and hysteresis helper expect.
    direction(contains(direction, "up"))   = "up";
    direction(contains(direction, "down")) = "down";

    if ismember("segment", string(T.Properties.VariableNames))
        segment = string(T.segment);
        segment(ismissing(segment) | strlength(strtrim(segment)) == 0) = "sweep";
    else
        segment = strings(height(T), 1);
        segment(:) = "sweep";
    end
    
    isSweepSegment = segment == "sweep";

    % Stalled samples are at/past saturation (encoder flatlined) and must not
    % enter any gain fit, or they bias the slope toward zero.
    if ismember("is_stalled", string(T.Properties.VariableNames))
        isStalled = T.is_stalled == 1;
    else
        isStalled = false(height(T), 1);
    end

    isUsable = isSweepSegment & ~isStalled;

    allServoUs = [allServoUs; servo_us(isUsable)];
    allThetaDeg = [allThetaDeg; theta_deg(isUsable)];

    % Direction masks (up / down legs of the center-out walk).
    idxUp   = isUsable & direction == "up";
    idxDown = isUsable & direction == "down";

    %% Fit Static Command-to-Angle Map (full-range chord)

    validFit = isUsable & isfinite(servo_us) & isfinite(theta_deg);

    p_all = polyfit(servo_us(validFit), theta_deg(validFit), 1);

    static_gain_deg_per_us = p_all(1);
    static_intercept_deg = p_all(2);

    if abs(static_gain_deg_per_us) > eps
        neutral_pwm_from_fit = -static_intercept_deg / static_gain_deg_per_us;
    else
        neutral_pwm_from_fit = NaN;
    end

    suUsable = servo_us(isUsable);
    responsiveLoUs = min(suUsable);
    responsiveHiUs = max(suUsable);

    %% Per-Direction (Two-Branch) Fit and Backlash
    %
    % The center-out walk preloads gear lash in each leg's direction, so the up
    % and down legs ARE the two backlash branches. Fitting them separately gives
    % the lash-free single-branch gain; their vertical gap at center is the
    % lost-motion (backlash). A single fit through center -- or any window
    % straddling it -- mixes the branches and flattens the slope (the former
    % "center-region gain" artifact), so that fit is no longer reported.

    pUp   = polyfit(servo_us(idxUp),   theta_deg(idxUp),   1);
    pDown = polyfit(servo_us(idxDown), theta_deg(idxDown), 1);

    up_gain_deg_per_us   = pUp(1);
    down_gain_deg_per_us = pDown(1);
    branch_mean_gain_deg_per_us = (up_gain_deg_per_us + down_gain_deg_per_us) / 2;
    branch_mean_intercept_deg   = (pUp(2) + pDown(2)) / 2;

    % Neutral consistent with the lash-free branch-mean line (zero-angle command).
    if abs(branch_mean_gain_deg_per_us) > eps
        branch_mean_neutral_us = -branch_mean_intercept_deg / branch_mean_gain_deg_per_us;
    else
        branch_mean_neutral_us = NaN;
    end

    theta_up_at_center   = polyval(pUp,   SERVO_CENTER_US);
    theta_down_at_center = polyval(pDown, SERVO_CENTER_US);
    backlash_deg = theta_up_at_center - theta_down_at_center;   % lost motion

    branch_slope_agreement_pct = ...
        100 * abs(up_gain_deg_per_us - down_gain_deg_per_us) / ...
        abs(branch_mean_gain_deg_per_us);

    %% Within-Branch Linearity (per-direction sliding local gain)

    [upCenters, upLocalGain] = slidingLocalGain( ...
        servo_us(idxUp), theta_deg(idxUp), LOCAL_GAIN_WINDOW_US, LOCAL_GAIN_STEP_US);
    [downCenters, downLocalGain] = slidingLocalGain( ...
        servo_us(idxDown), theta_deg(idxDown), LOCAL_GAIN_WINDOW_US, LOCAL_GAIN_STEP_US);

    up_ripple_frac = (max(upLocalGain) - min(upLocalGain)) / abs(median(upLocalGain, "omitnan"));
    down_ripple_frac = (max(downLocalGain) - min(downLocalGain)) / abs(median(downLocalGain, "omitnan"));

    %% Print File Summary

    fprintf("\nSamples: %d\n", height(T));
    fprintf("Servo command range: %.1f to %.1f us\n", min(servo_us), max(servo_us));
    fprintf("Measured angle range: %.3f to %.3f deg\n", min(theta_deg), max(theta_deg));
    fprintf("Responsive (non-stalled) command range: %.0f to %.0f us\n", ...
        responsiveLoUs, responsiveHiUs);

    fprintf("\nStatic command-to-angle fit (full-range chord):\n");
    fprintf("  theta_deg = %.6f * servo_us + %.6f\n", static_gain_deg_per_us, static_intercept_deg);
    fprintf("  Static gain: %.6f deg/us (%.8f rad/us)\n", ...
        static_gain_deg_per_us, static_gain_deg_per_us * pi / 180.0);
    fprintf("  Neutral PWM from fit: %.2f us\n", neutral_pwm_from_fit);

    fprintf("\nPer-direction (two-branch) fit:\n");
    fprintf("  Up   gain: %.6f deg/us  (n=%d, %.0f..%.0f us)\n", ...
        up_gain_deg_per_us, nnz(idxUp), min(servo_us(idxUp)), max(servo_us(idxUp)));
    fprintf("  Down gain: %.6f deg/us  (n=%d, %.0f..%.0f us)\n", ...
        down_gain_deg_per_us, nnz(idxDown), min(servo_us(idxDown)), max(servo_us(idxDown)));
    fprintf("  Slope agreement: %.1f%%\n", branch_slope_agreement_pct);
    fprintf("  Branch-mean gain (lash-free): %.6f deg/us (%.8f rad/us)\n", ...
        branch_mean_gain_deg_per_us, branch_mean_gain_deg_per_us * pi / 180.0);
    fprintf("  Branch-mean neutral (lash-free zero-angle command): %.2f us\n", branch_mean_neutral_us);
    fprintf("  Backlash (lost motion, up-down gap at center): %.3f deg\n", backlash_deg);

    fprintf("\nWithin-branch linearity (sliding ±%.0f us local gain):\n", LOCAL_GAIN_WINDOW_US);
    fprintf("  Up   gain ripple: %.1f%%   Down gain ripple: %.1f%%\n", ...
        100 * up_ripple_frac, 100 * down_ripple_frac);

    %% Store File Summary

    newRow = table( ...
        dataFiles(fileIdx), ...
        height(T), ...
        responsiveLoUs, ...
        responsiveHiUs, ...
        min(theta_deg), ...
        max(theta_deg), ...
        static_gain_deg_per_us, ...
        static_gain_deg_per_us * pi / 180.0, ...
        neutral_pwm_from_fit, ...
        up_gain_deg_per_us, ...
        down_gain_deg_per_us, ...
        branch_mean_gain_deg_per_us, ...
        branch_mean_neutral_us, ...
        backlash_deg, ...
        'VariableNames', { ...
            'file', ...
            'num_samples', ...
            'responsive_us_min', ...
            'responsive_us_max', ...
            'theta_deg_min', ...
            'theta_deg_max', ...
            'static_chord_gain_deg_per_us', ...
            'static_chord_gain_rad_per_us', ...
            'chord_neutral_pwm_us', ...
            'up_gain_deg_per_us', ...
            'down_gain_deg_per_us', ...
            'branch_mean_gain_deg_per_us', ...
            'branch_mean_neutral_us', ...
            'backlash_deg' ...
        });

    summaryRows = [summaryRows; newRow];

    %% Plot Individual File Diagnostics

    hCmd = figure;
    plot(t, servo_us, "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("Servo Command (\mus)");
    title("Servo Static Sweep: Command vs Time");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    hAng = figure;
    plot(t, theta_deg, "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("Measured Servo Angle (deg)");
    title("Servo Static Sweep: Measured Angle vs Time");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    hMap = figure;
    plot(servo_us(isUsable), theta_deg(isUsable), "o", "LineWidth", 1.2);
    hold on;
    upSpan = linspace(min(servo_us(idxUp)), max(servo_us(idxUp)), 50)';
    downSpan = linspace(min(servo_us(idxDown)), max(servo_us(idxDown)), 50)';
    plot(upSpan, polyval(pUp, upSpan), "-", "LineWidth", 1.5);
    plot(downSpan, polyval(pDown, downSpan), "-", "LineWidth", 1.5);
    if any(isStalled)
        plot(servo_us(isStalled), theta_deg(isStalled), "x", "LineWidth", 1.5, "MarkerSize", 9);
    end
    xline(SERVO_CENTER_US, "--", "Center");
    yline(0, "--");
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Measured Servo Angle (deg)");
    title(sprintf("Static Command-to-Angle Map (backlash %.2f deg at center)", backlash_deg));
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Measured", "Up branch fit", "Down branch fit", "Stalled (saturated)", "Location", "best");

    hDir = figure;
    hold on;
    plot(servo_us(idxUp), theta_deg(idxUp), "o", "LineWidth", 1.1);
    plot(servo_us(idxDown), theta_deg(idxDown), "o", "LineWidth", 1.1);
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Measured Servo Angle (deg)");
    title("Static Sweep Direction Comparison");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Up sweep", "Down sweep", "Location", "best");

    % Within-branch linearity: sliding local gain per direction (computed
    % separately so the center-out backlash step is never mixed into a fit).
    hLin = figure;
    plot(upCenters, upLocalGain, "o-", "LineWidth", 1.3);
    hold on;
    plot(downCenters, downLocalGain, "o-", "LineWidth", 1.3);
    yline(up_gain_deg_per_us, "--");
    yline(down_gain_deg_per_us, "--");
    xline(SERVO_CENTER_US, "--", "Center");
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Local Gain (deg/\mus)");
    title(sprintf("Within-Branch Linearity (ripple: up %.1f%%, down %.1f%%)", ...
        100 * up_ripple_frac, 100 * down_ripple_frac));
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Up branch", "Down branch", "Up branch fit", "Down branch fit", "Location", "best");

    %% Save Individual Figures

    if SAVE_FIGURES
        if ~exist(plotDir, "dir")
            mkdir(plotDir);
        end

        fileStem = erase(dataFiles(fileIdx), ".csv");
        fileStem = sanitizeFileStem(fileStem);

        savedPaths = [
            saveFigurePair(hCmd, plotDir, fileStem + "_command_vs_time")
            saveFigurePair(hAng, plotDir, fileStem + "_angle_vs_time")
            saveFigurePair(hMap, plotDir, fileStem + "_static_command_to_angle_map")
            saveFigurePair(hDir, plotDir, fileStem + "_direction_comparison")
            saveFigurePair(hLin, plotDir, fileStem + "_linearity_local_gain")
        ];

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
fprintf("  Count-to-angle is spec-derived: %.0f counts/rev (1:1 GT2 16T), %.4f deg/count exact.\n", COUNTS_PER_REV, encScale.deg_per_count);
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

%% Local Function: Sliding Local Gain
%
% Slides a window of half-width winUs along a single monotonic branch and fits
% a local line in each window, returning the local gain (slope) vs window
% center. Operates on one direction at a time so the center-out backlash step
% between branches is never mixed into a fit.

function [centers, localGain] = slidingLocalGain(servo_us, theta_deg, winUs, stepUs)

    [su, ord] = sort(servo_us);
    th = theta_deg(ord);

    centers = (min(su) + winUs : stepUs : max(su) - winUs)';
    localGain = nan(size(centers));

    for c = 1:numel(centers)
        win = abs(su - centers(c)) <= winUs;
        if nnz(win) >= 3
            pp = polyfit(su(win), th(win), 1);
            localGain(c) = pp(1);
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