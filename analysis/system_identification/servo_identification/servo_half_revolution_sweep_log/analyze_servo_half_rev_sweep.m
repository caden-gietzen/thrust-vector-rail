%% analyze_servo_half_rev_sweep.m
% Analyze servo half-revolution sweep data from Pico CSV logs.
%
% Expected CSV columns:
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

%% User Options

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"
ANALYZE_MODE = "all";           % "latest", "single", or "all"

% Used only when ANALYZE_MODE = "single"
dataFile = "servo_450_to_2450_half_rev_sweep.csv";

EXPECTED_HALF_REV_COUNTS = 1200;
EXPECTED_COUNTS_PER_REV = encoderAngleScale().counts_per_rev;   % spec-derived (1:1 GT2 16T)

SERVO_0_DEG_US = 450;
SERVO_180_DEG_US = 2450;

SAVE_FIGURES = true;
SAVE_SUMMARY_TABLE = false;

%% Locate Project Paths

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

fprintf("\nServo half-revolution sweep analysis\n");
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

allEndpointHalfRevCounts = [];
allEndpointCountsPerRev = [];
allReturnErrorsCounts = [];

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
            error("Missing required CSV column in %s: %s", dataFiles(fileIdx), requiredVars(k));
        end
    end

    t = T.t_s;
    servo_us = T.servo_us;
    count_delta = T.count_delta_from_start;
    expected_deg = T.expected_deg_from_command;
    estimated_cpr = T.estimated_counts_per_rev;
    direction = string(T.direction);

    %% Extract Endpoint Rows

    idx_180 = direction == "up_endpoint_settled";
    idx_0_return = direction == "down_endpoint_settled";

    if ~any(idx_180)
        warning("No up_endpoint_settled rows found in %s. Using max servo command rows instead.", dataFiles(fileIdx));
        idx_180 = servo_us == max(servo_us);
    end

    if ~any(idx_0_return)
        warning("No down_endpoint_settled rows found in %s. Using min servo command rows instead.", dataFiles(fileIdx));
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
    meanReturnErrorDeg = meanReturnErrorCounts / EXPECTED_COUNTS_PER_REV * 360;

    allEndpointHalfRevCounts = [allEndpointHalfRevCounts; absDelta180(:)];
    allEndpointCountsPerRev = [allEndpointCountsPerRev; cpr180(:)];
    allReturnErrorsCounts = [allReturnErrorsCounts; abs(delta0Return(:))];

    %% Fit Full Sweep

    validFit = isfinite(expected_deg) & isfinite(count_delta);

    p_deg_to_count = polyfit(expected_deg(validFit), count_delta(validFit), 1);

    counts_per_deg_fit = p_deg_to_count(1);
    counts_per_rev_fit = abs(counts_per_deg_fit) * 360;

    fitErrorCounts = counts_per_rev_fit - EXPECTED_COUNTS_PER_REV;
    fitErrorPct = 100 * fitErrorCounts / EXPECTED_COUNTS_PER_REV;

    %% Print File Summary

    fprintf("\nSamples: %d\n", height(T));
    fprintf("Servo command range: %.1f to %.1f us\n", min(servo_us), max(servo_us));
    fprintf("Encoder count delta range: %.1f to %.1f counts\n", min(count_delta), max(count_delta));

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
    fprintf("  Equivalent angle error: %.3f deg\n", meanReturnErrorDeg);

    fprintf("\nFull-sweep linear fit:\n");
    fprintf("  count_delta = %.6f * expected_deg + %.6f\n", ...
        p_deg_to_count(1), p_deg_to_count(2));
    fprintf("  Fit-implied counts/rev: %.2f\n", counts_per_rev_fit);
    fprintf("  Fit-implied counts/rev error: %.2f counts (%.2f%%)\n", ...
        fitErrorCounts, fitErrorPct);

    %% Store File Summary

    passStatus = "FAIL";

    if abs(halfRevErrorPct) <= 5
        passStatus = "PASS";
    elseif abs(halfRevErrorPct) <= 12.5
        passStatus = "MARGINAL";
    end

    newRow = table( ...
        dataFiles(fileIdx), ...
        height(T), ...
        min(servo_us), ...
        max(servo_us), ...
        meanHalfRevCounts, ...
        stdHalfRevCounts, ...
        halfRevErrorCounts, ...
        halfRevErrorPct, ...
        meanCountsPerRev, ...
        stdCountsPerRev, ...
        cprErrorCounts, ...
        cprErrorPct, ...
        meanReturnErrorCounts, ...
        meanReturnErrorDeg, ...
        counts_per_rev_fit, ...
        fitErrorPct, ...
        passStatus, ...
        'VariableNames', { ...
            'file', ...
            'num_samples', ...
            'servo_us_min', ...
            'servo_us_max', ...
            'mean_half_rev_counts', ...
            'std_half_rev_counts', ...
            'half_rev_error_counts', ...
            'half_rev_error_pct', ...
            'mean_counts_per_rev', ...
            'std_counts_per_rev', ...
            'counts_per_rev_error', ...
            'counts_per_rev_error_pct', ...
            'mean_return_error_counts', ...
            'mean_return_error_deg', ...
            'fit_counts_per_rev', ...
            'fit_counts_per_rev_error_pct', ...
            'status' ...
        });

    summaryRows = [summaryRows; newRow];

    %% Plot Individual File Diagnostics

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
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    figure;
    plot(servo_us, abs(count_delta), "o-", "LineWidth", 1.2);
    hold on;
    yline(EXPECTED_HALF_REV_COUNTS, "--", "Expected 1200 counts");
    ylim([0, 1.25 * EXPECTED_HALF_REV_COUNTS]);
    grid on;
    xlabel("Servo Command (\mus)");
    ylabel("Absolute Encoder Count Delta from Start");
    title("Command vs Absolute Encoder Count Delta");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    validCpr = isfinite(estimated_cpr) & estimated_cpr > 0;

    figure;
    plot(expected_deg(validCpr), estimated_cpr(validCpr), "o", "LineWidth", 1.1);
    hold on;
    yline(EXPECTED_COUNTS_PER_REV, "--", sprintf("Expected %.0f counts/rev", EXPECTED_COUNTS_PER_REV));
    grid on;
    xlabel("Expected Commanded Angle (deg)");
    ylabel("Estimated Counts per Revolution");
    title("Estimated Encoder Counts per Revolution");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");

    %% Save Individual Figures

    if SAVE_FIGURES
        if ~exist(plotDir, "dir")
            mkdir(plotDir);
        end

        fileStem = erase(dataFiles(fileIdx), ".csv");
        fileStem = matlab.lang.makeValidName(fileStem);

        saveas(figure(3 * fileIdx - 2), fullfile(plotDir, fileStem + "_command_and_count.png"));
        saveas(figure(3 * fileIdx - 1), fullfile(plotDir, fileStem + "_command_vs_abs_count.png"));
        saveas(figure(3 * fileIdx), fullfile(plotDir, fileStem + "_counts_per_rev.png"));
    end
end

%% Print Combined Summary

fprintf("\n============================================================\n");
fprintf("Combined half-revolution calibration summary\n");
fprintf("============================================================\n");

disp(summaryRows);

combinedMeanHalfRevCounts = mean(allEndpointHalfRevCounts, "omitnan");
combinedStdHalfRevCounts = std(allEndpointHalfRevCounts, "omitnan");

combinedMeanCountsPerRev = mean(allEndpointCountsPerRev, "omitnan");
combinedStdCountsPerRev = std(allEndpointCountsPerRev, "omitnan");

combinedMeanReturnErrorCounts = mean(allReturnErrorsCounts, "omitnan");
combinedMeanReturnErrorDeg = combinedMeanReturnErrorCounts / EXPECTED_COUNTS_PER_REV * 360;

combinedHalfRevErrorCounts = combinedMeanHalfRevCounts - EXPECTED_HALF_REV_COUNTS;
combinedHalfRevErrorPct = 100 * combinedHalfRevErrorCounts / EXPECTED_HALF_REV_COUNTS;

combinedCprErrorCounts = combinedMeanCountsPerRev - EXPECTED_COUNTS_PER_REV;
combinedCprErrorPct = 100 * combinedCprErrorCounts / EXPECTED_COUNTS_PER_REV;

fprintf("\nCombined endpoint-based result:\n");
fprintf("  Expected half-rev count change: %.1f counts\n", EXPECTED_HALF_REV_COUNTS);
fprintf("  Combined mean half-rev count change: %.2f counts\n", combinedMeanHalfRevCounts);
fprintf("  Combined half-rev count std: %.2f counts\n", combinedStdHalfRevCounts);
fprintf("  Combined half-rev error: %.2f counts (%.2f%%)\n", ...
    combinedHalfRevErrorCounts, combinedHalfRevErrorPct);

fprintf("\nCombined counts-per-revolution estimate:\n");
fprintf("  Expected counts/rev: %.1f\n", EXPECTED_COUNTS_PER_REV);
fprintf("  Combined mean counts/rev: %.2f\n", combinedMeanCountsPerRev);
fprintf("  Combined counts/rev std: %.2f\n", combinedStdCountsPerRev);
fprintf("  Combined counts/rev error: %.2f counts (%.2f%%)\n", ...
    combinedCprErrorCounts, combinedCprErrorPct);

fprintf("\nCombined return-to-zero check:\n");
fprintf("  Combined mean absolute return error: %.2f counts\n", combinedMeanReturnErrorCounts);
fprintf("  Equivalent angle error: %.3f deg\n", combinedMeanReturnErrorDeg);

fprintf("\nPractical interpretation:\n");

if abs(combinedHalfRevErrorPct) <= 5
    fprintf("  PASS: Combined endpoint result is within 5%% of 1200 counts.\n");
    fprintf("  Sanity check OK: consistent with the spec-derived %.0f counts/rev.\n", EXPECTED_COUNTS_PER_REV);
elseif abs(combinedHalfRevErrorPct) <= 12.5
    fprintf("  MARGINAL: Combined endpoint result is in the rough usable range, but not tight.\n");
    fprintf("  Check belt tension, endpoint assumptions, and servo settling.\n");
else
    fprintf("  FAIL / INVESTIGATE: Combined endpoint result is far from 1200 counts.\n");
    fprintf("  Check encoder counts/rev, quadrature mode, belt ratio, slip, or servo range.\n");
end

fprintf("\nAngle conversion if accepted:\n");
fprintf("  theta_deg = count_delta / %.1f * 360\n", EXPECTED_COUNTS_PER_REV);
fprintf("  theta_rad = count_delta / %.1f * 2*pi\n", EXPECTED_COUNTS_PER_REV);

%% Save Summary Table

if SAVE_SUMMARY_TABLE
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    summaryPath = fullfile(plotDir, "servo_half_rev_summary_table.csv");
    writetable(summaryRows, summaryPath);

    fprintf("\nSaved summary table to:\n  %s\n", summaryPath);
end