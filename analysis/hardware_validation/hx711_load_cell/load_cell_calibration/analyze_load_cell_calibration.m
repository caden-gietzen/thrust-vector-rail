%% analyze_load_cell_calibration.m
% Analyze HX711 load cell calibration CSV.
%
% Expected CSV columns:
% t_ms,t_s,phase,known_mass_g,raw_count,avg_count,tared_count,counts_per_g_estimate

clear; clc; close all;

% ----------------------------
% User options
% ----------------------------

USE_LATEST_CSV = true;
SAVE_FIGURES = true;

% Used only when USE_LATEST_CSV = false
dataFile = "2026_05_10_servo_sweep_800us_2125us_00.csv";

% ----------------------------
% Locate project paths
% ----------------------------

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

% Set true if load cell output is negative with positive applied mass.
flipSign = true;

%% ----------------------------
% Load data
% ----------------------------

T = readtable(dataPath);

disp("Loaded file:");
disp(dataPath);

% Keep only calibration rows for nonzero masses.
cal = T(strcmp(T.phase, "calibration") & T.known_mass_g > 0, :);

if isempty(cal)
    error("No calibration rows found. Check phase and known_mass_g columns.");
end

if flipSign
    cal.tared_count = -cal.tared_count;
    T.tared_count = -T.tared_count;
end

masses = unique(cal.known_mass_g);

%% ----------------------------
% Group statistics by mass
% ----------------------------

meanCounts = zeros(size(masses));
stdCounts  = zeros(size(masses));
nSamples   = zeros(size(masses));

for i = 1:numel(masses)
    idx = cal.known_mass_g == masses(i);

    meanCounts(i) = mean(cal.tared_count(idx));
    stdCounts(i)  = std(cal.tared_count(idx));
    nSamples(i)   = sum(idx);
end

summaryTable = table(masses, meanCounts, stdCounts, nSamples, ...
    'VariableNames', {'mass_g', 'mean_tared_counts', 'std_tared_counts', 'n_samples'});

disp("Calibration summary:");
disp(summaryTable);

%% ----------------------------
% Linear calibration fit
% ----------------------------
% Model:
%   counts = slope * mass_g + intercept

p = polyfit(masses, meanCounts, 1);

slope_counts_per_g = p(1);
intercept_counts = p(2);

fitCounts = polyval(p, masses);
residuals = meanCounts - fitCounts;

grams_per_count = 1 / slope_counts_per_g;

rmse_counts = sqrt(mean(residuals.^2));
rmse_g = rmse_counts * grams_per_count;

fprintf("\nCalibration fit:\n");
fprintf("  counts = %.6f * mass_g + %.6f\n", slope_counts_per_g, intercept_counts);
fprintf("  counts_per_g = %.6f\n", slope_counts_per_g);
fprintf("  grams_per_count = %.9f\n", grams_per_count);
fprintf("  RMSE = %.3f counts\n", rmse_counts);
fprintf("  RMSE = %.6f g\n", rmse_g);

%% ----------------------------
% Validation on earlier CSV files
% ----------------------------

allCsvFiles = dir(fullfile(dataDir, "*.csv"));

if numel(allCsvFiles) <= 1
    fprintf("\nNo earlier CSV files found for validation.\n");
else
    latestFileName = string(dataFile);

    validationRows = [];

    fprintf("\nValidation sets using latest-fit calibration:\n");

    for k = 1:numel(allCsvFiles)
        validationFile = string(allCsvFiles(k).name);

        % Skip the learning/training file.
        if validationFile == latestFileName
            continue;
        end

        validationPath = fullfile(allCsvFiles(k).folder, allCsvFiles(k).name);

        V = readtable(validationPath);

        valCal = V(strcmp(V.phase, "calibration") & V.known_mass_g > 0, :);

        if isempty(valCal)
            fprintf("  Skipping %s: no calibration rows found.\n", validationFile);
            continue;
        end

        if flipSign
            valCal.tared_count = -valCal.tared_count;
        end

        valMasses = unique(valCal.known_mass_g);

        for i = 1:numel(valMasses)
            mass_g = valMasses(i);
            idx = valCal.known_mass_g == mass_g;

            mean_count = mean(valCal.tared_count(idx));
            std_count = std(valCal.tared_count(idx));
            n = sum(idx);

            % Fit was:
            %   counts = slope_counts_per_g * mass_g + intercept_counts
            %
            % So estimated mass is:
            %   mass_g_est = (counts - intercept_counts) / slope_counts_per_g

            predicted_mass_g = (mean_count - intercept_counts) / slope_counts_per_g;
            error_g = predicted_mass_g - mass_g;
            abs_error_g = abs(error_g);

            validationRows = [validationRows; {
                validationFile, ...
                mass_g, ...
                mean_count, ...
                std_count, ...
                n, ...
                predicted_mass_g, ...
                error_g, ...
                abs_error_g ...
            }];
        end
    end

    if isempty(validationRows)
        fprintf("  No usable validation calibration rows found.\n");
    else
        validationTable = cell2table(validationRows, ...
            'VariableNames', { ...
                'file', ...
                'known_mass_g', ...
                'mean_tared_counts', ...
                'std_tared_counts', ...
                'n_samples', ...
                'predicted_mass_g', ...
                'error_g', ...
                'abs_error_g' ...
            });

        disp(validationTable);

        validation_rmse_g = sqrt(mean(validationTable.error_g.^2));
        validation_mae_g = mean(validationTable.abs_error_g);
        validation_max_abs_error_g = max(validationTable.abs_error_g);

        fprintf("\nValidation performance across earlier CSVs:\n");
        fprintf("  RMSE = %.6f g\n", validation_rmse_g);
        fprintf("  MAE = %.6f g\n", validation_mae_g);
        fprintf("  Max absolute error = %.6f g\n", validation_max_abs_error_g);
    end
end

%% ----------------------------
% Plot 1: tared counts over time
% ----------------------------

figure;
hold on; grid on;

plot(T.t_s, T.tared_count, '.', 'MarkerSize', 10);

xlabel("Time (s)");
ylabel("Tared count");
title("HX711 Load Cell Calibration Samples");

%% ----------------------------
% Plot 2: calibration fit
% ----------------------------

figure;
hold on; grid on;

errorbar(masses, meanCounts, stdCounts, 'o', 'LineWidth', 1.5);

massFit = linspace(min(masses), max(masses), 200);
countFit = polyval(p, massFit);

plot(massFit, countFit, '-', 'LineWidth', 2);

xlabel("Known mass (g)");
ylabel("Mean tared count");
title("Load Cell Calibration Fit");
legend("Measured mean \pm 1 std", "Linear fit", "Location", "best");

%% ----------------------------
% Plot 3: residuals
% ----------------------------

figure;
hold on; grid on;

plot(masses, residuals, 'o-', 'LineWidth', 1.5);

yline(0, '--');

xlabel("Known mass (g)");
ylabel("Residual (counts)");
title("Calibration Fit Residuals");

%% ----------------------------
% Plot 4: validation error by mass
% ----------------------------

if exist("validationTable", "var") && ~isempty(validationTable)
    figure;
    hold on; grid on;

    scatter(validationTable.known_mass_g, validationTable.error_g, 60, 'filled');
    yline(0, '--');

    xlabel("Known mass (g)");
    ylabel("Validation error (g)");
    title("Validation Error Using Latest Calibration Fit");
end

%% ----------------------------
% Optional: print usable MicroPython constants
% ----------------------------

fprintf("\nUse in MicroPython:\n");
fprintf("  SCALE_G_PER_COUNT = %.9f\n", grams_per_count);
fprintf("  OFFSET_COUNTS = measured tare offset from script\n");

saveAllFiguresIfEnabled(SAVE_FIGURES);