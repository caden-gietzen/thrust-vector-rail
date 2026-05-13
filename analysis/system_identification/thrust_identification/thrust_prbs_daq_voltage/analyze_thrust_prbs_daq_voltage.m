%% analyze_thrust_prbs_daq_voltage.m
% Analyze thrust PRBS DAQ files.
%
% Expected older CSV columns:
%   t_ms,t_s,run_name,segment,pwm_us,raw_count,tared_count,force_N,phase
%
% Expected newer CSV columns with MAVLink battery telemetry:
%   t_ms,t_s,set_index,prbs_seed,run_order_seed,run_name,segment,pwm_us,
%   raw_count,tared_count,force_N,
%   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
%   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
%   phase
%
% Identification goal:
%   Compare thrust dynamics models with and without battery voltage.
%
%   PWM-only:
%       y[k+1] = a*y[k] + b*u[k] + c
%
%   PWM quadratic:
%       y[k+1] = a*y[k] + b1*u[k] + b2*u[k]^2 + c
%
%   PWM + voltage:
%       y[k+1] = a*y[k] + b*u[k] + d*V[k] + c
%
%   PWM + voltage + interaction:
%       y[k+1] = a*y[k] + b*u[k] + d*V[k] + e*u[k]*V[k] + c
%
%   PWM quadratic + voltage + interaction:
%       y[k+1] = a*y[k] + b1*u[k] + b2*u[k]^2 + d*V[k] + e*u[k]*V[k] + c
%
% where:
%   y[k] = thrust at sample k, in newtons
%   u[k] = PWM command at sample k, in microseconds
%   V[k] = battery voltage at sample k, in volts
%   a,b,c,d,e = fitted coefficients

clear; clc; close all;

%% ----------------------------
% User options
% ----------------------------

SAVE_FIGURES = false;

% Train on first N CSV files in sorted folder order.
TRAIN_ON_FIRST_N_FILES = true;
NUM_TRAINING_FILES = 4;

% Used only if TRAIN_ON_FIRST_N_FILES = false
trainingFile = "2026_05_11_thrust_prbs_daq_00.csv";

runNames = [
    "global_1100_1950"
    "local_1400_1650"
    "local_1700_1850"
];

%% ----------------------------
% Validation options
% ----------------------------

RUN_VALIDATION = true;

% If true, models that require voltage are only validated on files/runs that
% also contain voltage.
SKIP_VOLTAGE_MODEL_VALIDATION_IF_NO_VOLTAGE = true;

%% ----------------------------
% Plot toggles
% ----------------------------

PLOT_CROSS_RUN_FORCE_OVERLAY      = true;
PLOT_SEQUENTIAL_THRUST_DRIFT      = true;

PLOT_TRAINING_FIT                 = true;
PLOT_VALIDATION_FIT               = true;

PLOT_PRBS_INPUT                   = false;

PLOT_TRAINING_VOLTAGE_TIME        = true;
PLOT_VALIDATION_VOLTAGE_TIME      = true;

PLOT_CURRENT_TIME                 = false;
PLOT_VOLTAGE_VS_PWM               = false;
PLOT_VOLTAGE_VS_CURRENT           = true;
PLOT_FORCE_VS_VOLTAGE             = false;
PLOT_RESIDUAL_VS_VOLTAGE          = false;
PLOT_RESIDUAL_TIME                = false;

PLOT_MODEL_COMPARISON_BAR         = true;
PLOT_SAMPLE_TIME_DIAGNOSTIC       = false;

PLOT_BODE_FIT = true;

%% ----------------------------
% Identification options
% ----------------------------

USE_VOLTAGE_IF_AVAILABLE = true;

% Candidate model toggles
INCLUDE_PWM_ONLY_MODEL                 = true;
INCLUDE_PWM_QUADRATIC_MODEL            = false;
INCLUDE_PWM_VOLTAGE_MODEL              = true;
INCLUDE_PWM_VOLTAGE_INTERACTION_MODEL  = true;
INCLUDE_PWM_QUADRATIC_VOLTAGE_MODEL    = false;

% If true, compare all enabled structures.
% If false, fit only the richest available enabled model.
COMPARE_MODEL_STRUCTURES = true;

% Voltage samples older than this are considered stale and removed.
% Use Inf to disable filtering.
MAX_BATTERY_AGE_MS = 1000;

% Strongly recommended because u, u^2, V, and u*V have very different scales.
NORMALIZE_REGRESSORS = true;

% Optional moving-median filter for voltage diagnostics/modeling.
% Set to 1 to disable.
VOLTAGE_FILTER_WINDOW_SAMPLES = 1;

% Minimum samples required after filtering.
MIN_SAMPLES_PER_RUN = 10;

% Model selection:
%   "lowest_rmse"          = choose lowest training RMSE
%   "simplicity_tolerance" = choose simplest model within tolerance of best RMSE
MODEL_SELECTION_MODE = "simplicity_tolerance";

% If simplicity mode is used, choose the simplest model whose RMSE is within
% this relative tolerance of the best model.
% Example: 0.01 means within 1 percent of best RMSE.
SIMPLE_MODEL_RELATIVE_TOLERANCE = 0.01;

%% ----------------------------
% Locate mirrored folders
% ----------------------------

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath);
plotDir = getMirroredPlotDir(scriptPath);

addpath(genpath(fullfile(findRepoRoot(scriptPath), "analysis", "util")));

csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No CSV files found in:\n%s", dataDir);
end

% Sort by filename so "first five" is deterministic.
[~, sortIdx] = sort(string({csvFiles.name}));
csvFiles = csvFiles(sortIdx);

if TRAIN_ON_FIRST_N_FILES
    nTrain = min(NUM_TRAINING_FILES, numel(csvFiles));

    trainingCsvFiles = csvFiles(1:nTrain);
    validationCsvFiles = csvFiles(nTrain+1:end);

    fprintf("\nTraining / identification files:\n");
    for k = 1:numel(trainingCsvFiles)
        fprintf("  %s\n", fullfile(trainingCsvFiles(k).folder, trainingCsvFiles(k).name));
    end

    if RUN_VALIDATION
        fprintf("\nValidation files:\n");
        if isempty(validationCsvFiles)
            fprintf("  None. Not enough CSV files beyond first %d training files.\n", nTrain);
        else
            for k = 1:numel(validationCsvFiles)
                fprintf("  %s\n", fullfile(validationCsvFiles(k).folder, validationCsvFiles(k).name));
            end
        end
    end

else
    trainingPath = fullfile(dataDir, trainingFile);

    trainingCsvFiles = dir(trainingPath);

    if isempty(trainingCsvFiles)
        error("Training file not found:\n%s", trainingPath);
    end

    validationCsvFiles = csvFiles(string({csvFiles.name}) ~= string(trainingFile));

    fprintf("\nTraining / identification file:\n  %s\n", trainingPath);
end

trainingLabel = makeFileListLabel(trainingCsvFiles);

%% ----------------------------
% Cross-run overlay for drift / repeatability check
% ----------------------------

if PLOT_CROSS_RUN_FORCE_OVERLAY
    for r = 1:numel(runNames)
        runName = runNames(r);

        figure;
        hold on; grid on;

        for k = 1:numel(csvFiles)
            thisFile = string(csvFiles(k).name);
            thisPath = fullfile(csvFiles(k).folder, csvFiles(k).name);

            T = readtable(thisPath);
            D = getRunData(T, runName, MAX_BATTERY_AGE_MS);

            if height(D) < MIN_SAMPLES_PER_RUN
                continue;
            end

            t_s = D.t_ms / 1000;
            y_N = D.force_N;

            plot(t_s, y_N, "LineWidth", 1.1, ...
                "DisplayName", erase(thisFile, ".csv"));
        end

        xlabel("Time since PRBS start (s)");
        ylabel("Force (N)");
        title("Cross-Run Force Overlay - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");
    end
end

%% ----------------------------
% Sequential max-thrust drift analysis
% ----------------------------

driftRows = [];

for r = 1:numel(runNames)
    runName = runNames(r);

    for k = 1:numel(csvFiles)
        thisFile = string(csvFiles(k).name);
        thisPath = fullfile(csvFiles(k).folder, csvFiles(k).name);

        T = readtable(thisPath);
        D = getRunData(T, runName, MAX_BATTERY_AGE_MS);

        if height(D) < MIN_SAMPLES_PER_RUN
            continue;
        end

        y_N = D.force_N;
        u_pwm = D.pwm_us;

        maxThrust_N = max(y_N);
        meanTop10Thrust_N = mean(maxk(y_N, min(10, numel(y_N))));
        meanThrust_N = mean(y_N);

        maxPwm_us = max(u_pwm);

        if hasVoltage(D)
            meanVoltage_V = mean(D.battery_voltage_V, "omitnan");
            minVoltage_V = min(D.battery_voltage_V, [], "omitnan");
        else
            meanVoltage_V = NaN;
            minVoltage_V = NaN;
        end

        driftRows = [driftRows; {
            thisFile, ...
            k, ...
            runName, ...
            maxPwm_us, ...
            maxThrust_N, ...
            meanTop10Thrust_N, ...
            meanThrust_N, ...
            meanVoltage_V, ...
            minVoltage_V ...
        }];
    end
end

if ~isempty(driftRows)
    driftTable = cell2table(driftRows, ...
        'VariableNames', { ...
            'file', ...
            'run_index', ...
            'run_name', ...
            'max_pwm_us', ...
            'max_thrust_N', ...
            'mean_top10_thrust_N', ...
            'mean_thrust_N', ...
            'mean_voltage_V', ...
            'min_voltage_V' ...
        });

    disp("Sequential thrust drift summary:");
    disp(driftTable);
else
    driftTable = table();
    fprintf("\nNo drift rows generated.\n");
end

%% ----------------------------
% Compute drop relative to first run for each run_name
% ----------------------------

if ~isempty(driftTable)
    driftTable.delta_max_from_first_N = zeros(height(driftTable), 1);
    driftTable.percent_max_from_first = zeros(height(driftTable), 1);

    for r = 1:numel(runNames)
        runName = runNames(r);
        idx = driftTable.run_name == runName;

        subIdx = find(idx);

        if isempty(subIdx)
            continue;
        end

        firstMax = driftTable.max_thrust_N(subIdx(1));

        driftTable.delta_max_from_first_N(subIdx) = ...
            driftTable.max_thrust_N(subIdx) - firstMax;

        driftTable.percent_max_from_first(subIdx) = ...
            100 * driftTable.max_thrust_N(subIdx) / firstMax;
    end

    disp("Sequential max-thrust drop relative to earliest run:");
    disp(driftTable(:, { ...
        'file', ...
        'run_name', ...
        'max_thrust_N', ...
        'delta_max_from_first_N', ...
        'percent_max_from_first', ...
        'mean_voltage_V', ...
        'min_voltage_V' ...
    }));
end

%% ----------------------------
% Plot max thrust drift by run order
% ----------------------------

if PLOT_SEQUENTIAL_THRUST_DRIFT && ~isempty(driftTable)
    for r = 1:numel(runNames)
        runName = runNames(r);
        idx = driftTable.run_name == runName;

        if ~any(idx)
            continue;
        end

        sub = driftTable(idx, :);

        figure;
        hold on; grid on;

        yyaxis left;
        plot(sub.run_index, sub.max_thrust_N, 'o-', 'LineWidth', 1.5, ...
            'DisplayName', 'Max thrust');
        plot(sub.run_index, sub.mean_top10_thrust_N, 's--', 'LineWidth', 1.5, ...
            'DisplayName', 'Mean top 10 thrust samples');
        ylabel("Thrust (N)");

        yyaxis right;
        plot(sub.run_index, sub.mean_voltage_V, 'd:', 'LineWidth', 1.5, ...
            'DisplayName', 'Mean voltage');
        ylabel("Battery voltage (V)");

        xlabel("Sequential CSV run index");
        title("Sequential Thrust/Voltage Drift - " + runName, "Interpreter", "none");
        legend("Location", "best");
    end
end

%% ----------------------------
% Load combined training data
% ----------------------------

Ttrain = readCombinedCsvTables(trainingCsvFiles);

models = struct();
modelSummaryRows = [];

%% ----------------------------
% Identify model for each PRBS run
% ----------------------------

for r = 1:numel(runNames)
    runName = runNames(r);

    D = getRunData(Ttrain, runName, MAX_BATTERY_AGE_MS);

    if height(D) < MIN_SAMPLES_PER_RUN
        warning("Skipping %s: not enough PRBS samples.", runName);
        continue;
    end

    D = preprocessRunData(D, VOLTAGE_FILTER_WINDOW_SAMPLES);

    candidateModels = fitCandidateModels( ...
        D, ...
        USE_VOLTAGE_IF_AVAILABLE, ...
        COMPARE_MODEL_STRUCTURES, ...
        NORMALIZE_REGRESSORS, ...
        INCLUDE_PWM_ONLY_MODEL, ...
        INCLUDE_PWM_QUADRATIC_MODEL, ...
        INCLUDE_PWM_VOLTAGE_MODEL, ...
        INCLUDE_PWM_VOLTAGE_INTERACTION_MODEL, ...
        INCLUDE_PWM_QUADRATIC_VOLTAGE_MODEL);

    bestModel = selectBestModel( ...
        candidateModels, ...
        MODEL_SELECTION_MODE, ...
        SIMPLE_MODEL_RELATIVE_TOLERANCE);

    modelKey = matlab.lang.makeValidName(runName);
    models.(modelKey) = bestModel;

    fprintf("\nRun: %s\n", runName);
    fprintf("  Best model type: %s\n", bestModel.model_type);
    fprintf("  Equation: %s\n", bestModel.equation_text);
    fprintf("  Selection mode: %s\n", MODEL_SELECTION_MODE);
    fprintf("  Training RMSE = %.6f N\n", bestModel.rmse_N);
    fprintf("  Training MAE  = %.6f N\n", bestModel.mae_N);
    fprintf("  Max abs error = %.6f N\n", bestModel.max_abs_error_N);
    fprintf("  Approx tau    = %.6f s\n", bestModel.tau_s);
    fprintf("  dt median     = %.6f s\n", bestModel.dt_s);

    disp("  Coefficients:");
    disp(bestModel.coeff_table);

    for m = 1:numel(candidateModels)
        cm = candidateModels{m};

        modelSummaryRows = [modelSummaryRows; {
            trainingLabel, ...
            runName, ...
            cm.model_type, ...
            cm.model_complexity, ...
            cm.rmse_N, ...
            cm.mae_N, ...
            cm.max_abs_error_N, ...
            cm.tau_s, ...
            cm.n_samples, ...
            cm.has_voltage_available, ...
            cm.requires_voltage ...
        }];
    end

    %% Bode plot of fitted PWM-to-thrust dynamics
    if PLOT_BODE_FIT
        plotBodeFromFittedModel(bestModel, runName);
    end

    %% Plot training measured vs predicted
    if PLOT_TRAINING_FIT
        figure;
        hold on; grid on;

        plot(bestModel.t_s, bestModel.y_N, "LineWidth", 1.2, "DisplayName", "Measured");
        plot(bestModel.t_s, bestModel.yhat_N, "--", "LineWidth", 1.5, ...
            "DisplayName", "Best one-step prediction: " + bestModel.model_type);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("Thrust (N)");
        title("Training Fit - " + runName, "Interpreter", "none");
        legend("Location", "best");
    end

    %% Optional PRBS input plot
    if PLOT_PRBS_INPUT
        figure;
        hold on; grid on;

        plot(bestModel.t_s, bestModel.u_pwm, "LineWidth", 1.2);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("PWM command (\mus)");
        title("PRBS Input - " + runName, "Interpreter", "none");
    end

    %% Training voltage over time
    if PLOT_TRAINING_VOLTAGE_TIME && bestModel.has_voltage_available
        figure;
        hold on; grid on;

        plot(bestModel.t_s, bestModel.V_batt, "LineWidth", 1.2);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("Battery voltage (V)");
        title("Training Battery Voltage - " + runName + " - " + trainingLabel, ...
            "Interpreter", "none");
    end

    %% Optional current over time
    if PLOT_CURRENT_TIME && hasCurrent(D)
        figure;
        hold on; grid on;

        t_s = makeContinuousPlotTime(D.t_ms / 1000, getSequenceGroup(D));
        plot(t_s, D.battery_current_A, "LineWidth", 1.2);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("Battery current (A)");
        title("Battery Current During PRBS - " + runName, "Interpreter", "none");
    end

    %% Voltage vs PWM
    if PLOT_VOLTAGE_VS_PWM && bestModel.has_voltage_available
        figure;
        hold on; grid on;

        scatter(bestModel.u_pwm, bestModel.V_batt, 18, "filled");

        xlabel("PWM command (\mus)");
        ylabel("Battery voltage (V)");
        title("Voltage vs PWM - " + runName, "Interpreter", "none");
    end

    %% Voltage vs current
    if PLOT_VOLTAGE_VS_CURRENT && hasCurrent(D) && bestModel.has_voltage_available
        figure;
        hold on; grid on;

        scatter(D.battery_current_A, D.battery_voltage_V, 18, "filled");

        xlabel("Battery current (A)");
        ylabel("Battery voltage (V)");
        title("Voltage vs Current - " + runName, "Interpreter", "none");
    end

    %% Force vs voltage
    if PLOT_FORCE_VS_VOLTAGE && bestModel.has_voltage_available
        figure;
        hold on; grid on;

        scatter(bestModel.V_batt, bestModel.y_N, 18, "filled");

        xlabel("Battery voltage (V)");
        ylabel("Thrust (N)");
        title("Force vs Voltage - " + runName, "Interpreter", "none");
    end

    %% Residual vs voltage
    if PLOT_RESIDUAL_VS_VOLTAGE && bestModel.has_voltage_available
        figure;
        hold on; grid on;

        scatter(bestModel.V_batt, bestModel.err_N, 18, "filled");
        yline(0, "k--", "LineWidth", 1.1);

        xlabel("Battery voltage (V)");
        ylabel("Residual error (N)");
        title("Residual vs Voltage - " + runName + " - " + bestModel.model_type, ...
            "Interpreter", "none");
    end

    %% Residual over time
    if PLOT_RESIDUAL_TIME
        figure;
        hold on; grid on;

        plot(bestModel.t_s, bestModel.err_N, "LineWidth", 1.2);
        yline(0, "k--", "LineWidth", 1.1);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("Residual error (N)");
        title("Residual Over Time - " + runName + " - " + bestModel.model_type, ...
            "Interpreter", "none");
    end

    %% Model comparison bar
    if PLOT_MODEL_COMPARISON_BAR
        figure;
        hold on; grid on;

        names = strings(numel(candidateModels), 1);
        rmses = zeros(numel(candidateModels), 1);

        for m = 1:numel(candidateModels)
            cm = candidateModels{m};
            names(m) = cm.model_type;
            rmses(m) = cm.rmse_N;
        end

        bar(categorical(names), rmses);

        ylabel("Training RMSE (N)");
        title("Model Structure Comparison - " + runName, "Interpreter", "none");
    end

    %% Sample-time diagnostic
    if PLOT_SAMPLE_TIME_DIAGNOSTIC
        figure;
        hold on; grid on;

        dt_s = diff(bestModel.t_s);
        plot(bestModel.t_s(2:end), dt_s, "LineWidth", 1.1);

        xlabel("Time since PRBS start, grouped with gaps (s)");
        ylabel("Sample interval \Deltat (s)");
        title("Sample-Time Diagnostic - " + runName, "Interpreter", "none");
    end
end

if ~isempty(modelSummaryRows)
    modelSummaryTable = cell2table(modelSummaryRows, ...
        'VariableNames', { ...
            'file', ...
            'run_name', ...
            'model_type', ...
            'model_complexity', ...
            'rmse_N', ...
            'mae_N', ...
            'max_abs_error_N', ...
            'tau_s', ...
            'n_samples', ...
            'has_voltage_available', ...
            'requires_voltage' ...
        });

    disp("Training model comparison summary:");
    disp(modelSummaryTable);
else
    fprintf("\nNo training models generated.\n");
end

%% ----------------------------
% Validation on remaining CSV files
% ----------------------------

validationRows = [];

if RUN_VALIDATION

    if isempty(validationCsvFiles)
        fprintf("\nValidation enabled, but no validation files remain after training split.\n");
    end

    for k = 1:numel(validationCsvFiles)
        validationFile = string(validationCsvFiles(k).name);
        validationPath = fullfile(validationCsvFiles(k).folder, validationCsvFiles(k).name);
        Vtable = readtable(validationPath);

        fprintf("\nValidation file:\n  %s\n", validationPath);

        for r = 1:numel(runNames)
            runName = runNames(r);
            modelKey = matlab.lang.makeValidName(runName);

            if ~isfield(models, modelKey)
                continue;
            end

            Dval = getRunData(Vtable, runName, MAX_BATTERY_AGE_MS);

            if height(Dval) < MIN_SAMPLES_PER_RUN
                warning("Skipping validation %s in %s: not enough samples.", runName, validationFile);
                continue;
            end

            Dval = preprocessRunData(Dval, VOLTAGE_FILTER_WINDOW_SAMPLES);

            model = models.(modelKey);

            if model.requires_voltage && ~hasVoltage(Dval) && SKIP_VOLTAGE_MODEL_VALIDATION_IF_NO_VOLTAGE
                fprintf("Skipping voltage-model validation on non-voltage file: %s, run: %s\n", ...
                    validationFile, runName);
                continue;
            end

            val = simulateFirstOrderModel(Dval, model);

            validationRows = [validationRows; {
                validationFile, ...
                runName, ...
                model.model_type, ...
                val.rmse_N, ...
                val.mae_N, ...
                val.max_abs_error_N, ...
                height(Dval), ...
                val.has_voltage_available, ...
                model.requires_voltage ...
            }];

            %% Validation fit
            if PLOT_VALIDATION_FIT
                figure;
                hold on; grid on;

                plot(val.t_s, val.y_N, "LineWidth", 1.2, "DisplayName", "Measured");
                plot(val.t_s, val.yhat_N, "--", "LineWidth", 1.5, ...
                    "DisplayName", "Predicted: " + model.model_type);

                xlabel("Time since PRBS start, grouped with gaps (s)");
                ylabel("Thrust (N)");
                title("Validation Fit - " + runName + " - " + erase(validationFile, ".csv"), ...
                    "Interpreter", "none");
                legend("Location", "best");
            end

            %% Validation voltage over time
            if PLOT_VALIDATION_VOLTAGE_TIME && val.has_voltage_available
                figure;
                hold on; grid on;

                plot(val.t_s, val.V_batt, "LineWidth", 1.2);

                xlabel("Time since PRBS start, grouped with gaps (s)");
                ylabel("Battery voltage (V)");
                title("Validation Battery Voltage - " + runName + " - " + erase(validationFile, ".csv"), ...
                    "Interpreter", "none");
            end
        end
    end

else
    fprintf("\nValidation disabled: RUN_VALIDATION = false\n");
end

if ~isempty(validationRows)
    validationTable = cell2table(validationRows, ...
        'VariableNames', { ...
            'file', ...
            'run_name', ...
            'model_type', ...
            'rmse_N', ...
            'mae_N', ...
            'max_abs_error_N', ...
            'n_samples', ...
            'has_voltage_available', ...
            'model_requires_voltage' ...
        });

    disp("Validation summary:");
    disp(validationTable);
else
    fprintf("\nNo validation results generated.\n");
end

%% ----------------------------
% Save figures
% ----------------------------

cd(plotDir);
saveAllFiguresIfEnabled(SAVE_FIGURES);

%% ============================================================
% Local helper functions
% ============================================================

function D = getRunData(T, runName, maxBatteryAgeMs)
    idx = strcmp(string(T.run_name), runName) & ...
          strcmpi(string(T.phase), "prbs");

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.t_ms = D.t_ms - D.t_ms(1);

    % Convert battery columns to numeric if needed.
    if ismember("battery_voltage_V", string(D.Properties.VariableNames))
        D.battery_voltage_V = forceNumeric(D.battery_voltage_V);
    end

    if ismember("battery_current_A", string(D.Properties.VariableNames))
        D.battery_current_A = forceNumeric(D.battery_current_A);
    end

    if ismember("battery_age_ms", string(D.Properties.VariableNames))
        D.battery_age_ms = forceNumeric(D.battery_age_ms);

        if isfinite(maxBatteryAgeMs)
            freshIdx = ismissing(D.battery_age_ms) | D.battery_age_ms <= maxBatteryAgeMs;
            D = D(freshIdx, :);
        end
    end

    D.pwm_us = forceNumeric(D.pwm_us);
    D.force_N = forceNumeric(D.force_N);

    requiredIdx = isfinite(D.pwm_us) & isfinite(D.force_N);
    D = D(requiredIdx, :);
end

function D = preprocessRunData(D, voltageFilterWindowSamples)
    if hasVoltage(D)
        if voltageFilterWindowSamples > 1
            D.battery_voltage_V = movmedian(D.battery_voltage_V, voltageFilterWindowSamples, ...
                "omitnan");
        end
    end
end

function candidateModels = fitCandidateModels( ...
    D, ...
    useVoltageIfAvailable, ...
    compareModelStructures, ...
    normalizeRegressors, ...
    includePwmOnly, ...
    includePwmQuadratic, ...
    includePwmVoltage, ...
    includePwmVoltageInteraction, ...
    includePwmQuadraticVoltage)

    voltageAvailable = useVoltageIfAvailable && hasVoltage(D);

    modelTypes = strings(0, 1);

    if includePwmOnly
        modelTypes(end+1, 1) = "pwm_only";
    end

    if includePwmQuadratic
        modelTypes(end+1, 1) = "pwm_quadratic";
    end

    if voltageAvailable
        if includePwmVoltage
            modelTypes(end+1, 1) = "pwm_voltage";
        end

        if includePwmVoltageInteraction
            modelTypes(end+1, 1) = "pwm_voltage_interaction";
        end

        if includePwmQuadraticVoltage
            modelTypes(end+1, 1) = "pwm_quadratic_voltage_interaction";
        end
    end

    if isempty(modelTypes)
        error("No candidate model types enabled.");
    end

    if ~compareModelStructures
        modelTypes = modelTypes(end);
    end

    candidateModels = cell(numel(modelTypes), 1);

    for i = 1:numel(modelTypes)
        candidateModels{i} = fitFirstOrderDiscreteModel(D, modelTypes(i), normalizeRegressors);
    end
end

function model = fitFirstOrderDiscreteModel(D, modelType, normalizeRegressors)
    t_s_raw = D.t_ms / 1000;
    u_raw = D.pwm_us;
    y_raw = D.force_N;

    group_raw = getSequenceGroup(D);

    has_voltage_available = hasVoltage(D);
    requires_voltage = modelRequiresVoltage(modelType);

    if has_voltage_available
        V_raw = D.battery_voltage_V;
    else
        V_raw = NaN(size(y_raw));
    end

    if requires_voltage
        validSeq = isfinite(t_s_raw) & isfinite(u_raw) & isfinite(y_raw) & isfinite(V_raw);
    else
        validSeq = isfinite(t_s_raw) & isfinite(u_raw) & isfinite(y_raw);
    end

    t_s = t_s_raw(validSeq);
    u = u_raw(validSeq);
    y = y_raw(validSeq);
    V = V_raw(validSeq);
    group = group_raw(validSeq);

    if numel(y) < 3
        error("Not enough valid samples after filtering for model type %s.", modelType);
    end

    uniqueGroups = unique(group, "stable");

    XrawAll = [];
    ykp1All = [];
    dtAll = [];

    for g = 1:numel(uniqueGroups)
        idx = find(group == uniqueGroups(g));

        if numel(idx) < 2
            continue;
        end

        yg = y(idx);
        ug = u(idx);
        Vg = V(idx);
        tg = t_s(idx);

        yk = yg(1:end-1);
        uk = ug(1:end-1);
        ykp1 = yg(2:end);

        if requires_voltage
            Vk = Vg(1:end-1);
        else
            Vk = NaN(size(uk));
        end

        [Xraw, regressorNames, equationText, complexity] = buildRegressorMatrix( ...
            yk, uk, Vk, modelType);

        validRows = all(isfinite(Xraw), 2) & isfinite(ykp1);

        XrawAll = [XrawAll; Xraw(validRows, :)];
        ykp1All = [ykp1All; ykp1(validRows)];

        if numel(tg) >= 2
            dtAll = [dtAll; diff(tg)];
        end
    end

    if isempty(XrawAll)
        error("No valid one-step training pairs for model type %s.", modelType);
    end

    if size(XrawAll, 1) < size(XrawAll, 2) + 1
        warning("Few samples relative to parameters for model type %s.", modelType);
    end

    if normalizeRegressors
        xMean = mean(XrawAll, 1);
        xStd = std(XrawAll, 0, 1);
        xStd(xStd == 0) = 1;

        X = (XrawAll - xMean) ./ xStd;
    else
        xMean = zeros(1, size(XrawAll, 2));
        xStd = ones(1, size(XrawAll, 2));

        X = XrawAll;
    end

    Phi = [X, ones(size(X, 1), 1)];

    theta = Phi \ ykp1All;

    yhat = simulateGroupedPrediction(y, u, V, group, modelType, theta, xMean, xStd);

    err = y - yhat;

    if isempty(dtAll)
        dt_s = NaN;
    else
        dt_s = median(dtAll, "omitnan");
    end

    yCoeffPhysical = theta(1) / xStd(1);

    if yCoeffPhysical > 0 && yCoeffPhysical < 1 && isfinite(dt_s)
        tau_s = -dt_s / log(yCoeffPhysical);
    else
        tau_s = NaN;
    end

    coeffNames = [regressorNames; "constant"];
    coeffTable = table(coeffNames, theta, ...
        'VariableNames', {'term', 'coefficient_normalized_basis'});

    model.model_type = string(modelType);
    model.model_complexity = complexity;
    model.equation_text = equationText;

    model.theta = theta;
    model.regressor_names = regressorNames;
    model.coeff_table = coeffTable;
    model.xMean = xMean;
    model.xStd = xStd;
    model.normalize_regressors = normalizeRegressors;

    model.t_s = makeContinuousPlotTime(t_s, group);
    model.u_pwm = u;
    model.y_N = y;
    model.yhat_N = yhat;
    model.err_N = err;
    model.sequence_group = group;

    model.has_voltage_available = has_voltage_available;
    model.requires_voltage = requires_voltage;
    model.V_batt = V;

    model.rmse_N = sqrt(mean(err.^2, "omitnan"));
    model.mae_N = mean(abs(err), "omitnan");
    model.max_abs_error_N = max(abs(err), [], "omitnan");

    model.dt_s = dt_s;
    model.tau_s = tau_s;
    model.n_samples = numel(y);
end

function [Xraw, regressorNames, equationText, complexity] = buildRegressorMatrix(yk, uk, Vk, modelType)
    switch string(modelType)
        case "pwm_only"
            Xraw = [yk, uk];

            regressorNames = [
                "y_k"
                "pwm_us"
            ];

            equationText = "y[k+1] = a*y[k] + b*u[k] + c";
            complexity = 2;

        case "pwm_quadratic"
            Xraw = [yk, uk, uk.^2];

            regressorNames = [
                "y_k"
                "pwm_us"
                "pwm_us_squared"
            ];

            equationText = "y[k+1] = a*y[k] + b1*u[k] + b2*u[k]^2 + c";
            complexity = 3;

        case "pwm_voltage"
            Xraw = [yk, uk, Vk];

            regressorNames = [
                "y_k"
                "pwm_us"
                "battery_voltage_V"
            ];

            equationText = "y[k+1] = a*y[k] + b*u[k] + d*V[k] + c";
            complexity = 3;

        case "pwm_voltage_interaction"
            Xraw = [yk, uk, Vk, uk .* Vk];

            regressorNames = [
                "y_k"
                "pwm_us"
                "battery_voltage_V"
                "pwm_times_voltage"
            ];

            equationText = "y[k+1] = a*y[k] + b*u[k] + d*V[k] + e*u[k]*V[k] + c";
            complexity = 4;

        case "pwm_quadratic_voltage_interaction"
            Xraw = [yk, uk, uk.^2, Vk, uk .* Vk];

            regressorNames = [
                "y_k"
                "pwm_us"
                "pwm_us_squared"
                "battery_voltage_V"
                "pwm_times_voltage"
            ];

            equationText = "y[k+1] = a*y[k] + b1*u[k] + b2*u[k]^2 + d*V[k] + e*u[k]*V[k] + c";
            complexity = 5;

        otherwise
            error("Unknown modelType: %s", modelType);
    end
end

function xRaw = buildRegressorRow(y, u, V, modelType)
    switch string(modelType)
        case "pwm_only"
            xRaw = [y, u];

        case "pwm_quadratic"
            xRaw = [y, u, u^2];

        case "pwm_voltage"
            xRaw = [y, u, V];

        case "pwm_voltage_interaction"
            xRaw = [y, u, V, u * V];

        case "pwm_quadratic_voltage_interaction"
            xRaw = [y, u, u^2, V, u * V];

        otherwise
            error("Unknown modelType: %s", modelType);
    end
end

function tf = modelRequiresVoltage(modelType)
    tf = any(string(modelType) == [
        "pwm_voltage"
        "pwm_voltage_interaction"
        "pwm_quadratic_voltage_interaction"
    ]);
end

function bestModel = selectBestModel(candidateModels, selectionMode, relativeTolerance)
    rmses = zeros(numel(candidateModels), 1);
    complexities = zeros(numel(candidateModels), 1);

    for i = 1:numel(candidateModels)
        rmses(i) = candidateModels{i}.rmse_N;
        complexities(i) = candidateModels{i}.model_complexity;
    end

    [bestRmse, bestIdx] = min(rmses);

    switch string(selectionMode)
        case "lowest_rmse"
            bestModel = candidateModels{bestIdx};

        case "simplicity_tolerance"
            eligible = rmses <= bestRmse * (1 + relativeTolerance);

            eligibleIdx = find(eligible);
            eligibleComplexities = complexities(eligibleIdx);

            minComplexity = min(eligibleComplexities);
            simplestEligibleIdx = eligibleIdx(eligibleComplexities == minComplexity);

            if numel(simplestEligibleIdx) > 1
                [~, localBest] = min(rmses(simplestEligibleIdx));
                chosenIdx = simplestEligibleIdx(localBest);
            else
                chosenIdx = simplestEligibleIdx;
            end

            bestModel = candidateModels{chosenIdx};

        otherwise
            error("Unknown MODEL_SELECTION_MODE: %s", selectionMode);
    end
end

function val = simulateFirstOrderModel(D, model)
    t_s_raw = D.t_ms / 1000;
    u_raw = D.pwm_us;
    y_raw = D.force_N;

    group_raw = getSequenceGroup(D);

    has_voltage_available = hasVoltage(D);

    if has_voltage_available
        V_raw = D.battery_voltage_V;
    else
        V_raw = NaN(size(y_raw));
    end

    if model.requires_voltage
        validSeq = isfinite(t_s_raw) & isfinite(u_raw) & isfinite(y_raw) & isfinite(V_raw);
    else
        validSeq = isfinite(t_s_raw) & isfinite(u_raw) & isfinite(y_raw);
    end

    t_s = t_s_raw(validSeq);
    u = u_raw(validSeq);
    y = y_raw(validSeq);
    V = V_raw(validSeq);
    group = group_raw(validSeq);

    if numel(y) < 3
        error("Not enough valid validation samples for model type %s.", model.model_type);
    end

    yhat = simulateGroupedPrediction( ...
        y, u, V, group, model.model_type, model.theta, model.xMean, model.xStd);

    err = y - yhat;

    val.t_s = makeContinuousPlotTime(t_s, group);
    val.u_pwm = u;
    val.y_N = y;
    val.yhat_N = yhat;
    val.err_N = err;

    val.has_voltage_available = has_voltage_available;
    val.V_batt = V;

    val.rmse_N = sqrt(mean(err.^2, "omitnan"));
    val.mae_N = mean(abs(err), "omitnan");
    val.max_abs_error_N = max(abs(err), [], "omitnan");
end

function T = readCombinedCsvTables(csvFileStruct)
    tables = cell(numel(csvFileStruct), 1);
    allVarNames = strings(0, 1);

    % First pass: read tables and collect union of variable names.
    for k = 1:numel(csvFileStruct)
        thisPath = fullfile(csvFileStruct(k).folder, csvFileStruct(k).name);
        Tk = readtable(thisPath);

        Tk.source_file = repmat(string(csvFileStruct(k).name), height(Tk), 1);
        Tk.source_file_index = repmat(k, height(Tk), 1);

        tables{k} = Tk;

        allVarNames = union(allVarNames, string(Tk.Properties.VariableNames), "stable");
    end

    % Second pass: add missing variables to each table.
    for k = 1:numel(tables)
        Tk = tables{k};
        currentVars = string(Tk.Properties.VariableNames);

        for v = 1:numel(allVarNames)
            varName = allVarNames(v);

            if ~ismember(varName, currentVars)
                % Add missing columns as NaN by default.
                % String-like metadata columns get blank strings.
                if any(varName == ["source_file", "run_name", "phase", "segment"])
                    Tk.(varName) = repmat("", height(Tk), 1);
                else
                    Tk.(varName) = NaN(height(Tk), 1);
                end
            end
        end

        % Reorder columns consistently.
        Tk = Tk(:, cellstr(allVarNames));

        tables{k} = Tk;
    end

    T = vertcat(tables{:});
end

function label = makeFileListLabel(csvFileStruct)
    names = strings(numel(csvFileStruct), 1);

    for k = 1:numel(csvFileStruct)
        names(k) = string(csvFileStruct(k).name);
    end

    if numel(names) == 1
        label = names(1);
    else
        label = "combined_" + string(numel(names)) + "_files";
    end
end

function group = getSequenceGroup(D)
    if ismember("source_file_index", string(D.Properties.VariableNames))
        group = D.source_file_index;
    elseif ismember("set_index", string(D.Properties.VariableNames))
        group = D.set_index;
    else
        group = ones(height(D), 1);
    end

    group = double(group);
end

function yhat = simulateGroupedPrediction(y, u, V, group, modelType, theta, xMean, xStd)
    yhat = NaN(size(y));

    uniqueGroups = unique(group, "stable");

    for g = 1:numel(uniqueGroups)
        idx = find(group == uniqueGroups(g));

        if isempty(idx)
            continue;
        end

        yhat(idx(1)) = y(idx(1));

        for j = 1:numel(idx)-1
            k = idx(j);
            kNext = idx(j+1);

            xkRaw = buildRegressorRow(yhat(k), u(k), V(k), modelType);
            xk = (xkRaw - xMean) ./ xStd;

            yhat(kNext) = [xk, 1] * theta;
        end
    end
end

function tPlot = makeContinuousPlotTime(t_s, group)
    tPlot = NaN(size(t_s));

    uniqueGroups = unique(group, "stable");

    offset = 0;
    gap_s = 5;

    for g = 1:numel(uniqueGroups)
        idx = find(group == uniqueGroups(g));

        if isempty(idx)
            continue;
        end

        tg = t_s(idx);
        tg = tg - tg(1);

        tPlot(idx) = tg + offset;

        offset = offset + max(tg) + gap_s;
    end
end

function tf = hasVoltage(D)
    tf = ismember("battery_voltage_V", string(D.Properties.VariableNames)) && ...
         any(isfinite(forceNumeric(D.battery_voltage_V)));
end

function tf = hasCurrent(D)
    tf = ismember("battery_current_A", string(D.Properties.VariableNames)) && ...
         any(isfinite(forceNumeric(D.battery_current_A)));
end

function x = forceNumeric(x)
    if isnumeric(x)
        return;
    end

    if iscell(x)
        x = string(x);
    end

    if isstring(x) || ischar(x) || iscategorical(x)
        x = str2double(string(x));
        return;
    end

    try
        x = double(x);
    catch
        x = NaN(size(x));
    end
end

function plotBodeFromFittedModel(model, runName)
    % Plot the local small-signal Bode response from PWM command to thrust.
    %
    % The fitted model is:
    %   y[k+1] = f(y[k], u[k], V[k])
    %
    % This function extracts the local linearized form:
    %   delta_y[k+1] = a * delta_y[k] + b * delta_u[k]
    %
    % giving:
    %   G(z) = delta_y(z) / delta_u(z) = b / (z - a)
    %
    % Units:
    %   output: N
    %   input: microseconds PWM
    %   magnitude: N / microsecond

    if ~isfinite(model.dt_s) || model.dt_s <= 0
        warning("Cannot plot Bode for %s: invalid sample time.", runName);
        return;
    end

    [a, b] = getLocalLinearizedAB(model);

    if ~isfinite(a) || ~isfinite(b)
        warning("Cannot plot Bode for %s: invalid linearized coefficients.", runName);
        return;
    end

    Ts = model.dt_s;

    % Discrete transfer function:
    % y[k+1] = a y[k] + b u[k]
    %
    % Z-transform:
    % zY(z) = aY(z) + bU(z)
    % Y(z)/U(z) = b / (z - a)
    %
    % MATLAB tf form uses powers of z^-1:
    % G(z) = b z^-1 / (1 - a z^-1)
    Gz = tf([0 b], [1 -a], Ts);

    figure;
    bode(Gz);
    grid on;
    title("Bode Plot of Fitted PWM-to-Thrust Model - " + runName, ...
        "Interpreter", "none");

    fprintf("\nBode model for %s:\n", runName);
    fprintf("  Small-signal discrete model:\n");
    fprintf("    delta_y[k+1] = %.6f delta_y[k] + %.9f delta_u[k]\n", a, b);
    fprintf("  DC gain = %.9f N/us\n", dcgain(Gz));

    if abs(a) < 1
        tau_s = -Ts / log(abs(a));
        fprintf("  Equivalent first-order tau = %.6f s\n", tau_s);
        fprintf("  Approx bandwidth = %.6f rad/s = %.6f Hz\n", ...
            1 / tau_s, 1 / (2*pi*tau_s));
    else
        fprintf("  Warning: |a| >= 1, fitted discrete pole is unstable or non-decaying.\n");
    end
end

function [aPhysical, bPhysical] = getLocalLinearizedAB(model)
    % Extract local linearized coefficients from the normalized fitted model.
    %
    % model.theta is fitted in normalized-regressor coordinates:
    %   y[k+1] = theta_1*x1_norm + theta_2*x2_norm + ... + constant
    %
    % where:
    %   x_norm = (x_raw - xMean) ./ xStd
    %
    % Therefore the physical coefficient for raw regressor i is:
    %   theta_i / xStd_i

    coeffPhysical = model.theta(1:end-1) ./ model.xStd(:);

    names = string(model.regressor_names);

    yIdx = find(names == "y_k", 1);
    uIdx = find(names == "pwm_us", 1);
    uvIdx = find(names == "pwm_times_voltage", 1);
    u2Idx = find(names == "pwm_us_squared", 1);

    if isempty(yIdx) || isempty(uIdx)
        aPhysical = NaN;
        bPhysical = NaN;
        return;
    end

    aPhysical = coeffPhysical(yIdx);

    % Base PWM slope
    bPhysical = coeffPhysical(uIdx);

    % If model has u*V, linearized d/du includes coeff * V0
    if ~isempty(uvIdx) && isfield(model, "V_batt") && any(isfinite(model.V_batt))
        V0 = mean(model.V_batt, "omitnan");
        bPhysical = bPhysical + coeffPhysical(uvIdx) * V0;
    end

    % If model has u^2, linearized d/du includes 2*coeff*u0
    if ~isempty(u2Idx) && isfield(model, "u_pwm") && any(isfinite(model.u_pwm))
        u0 = mean(model.u_pwm, "omitnan");
        bPhysical = bPhysical + 2 * coeffPhysical(u2Idx) * u0;
    end
end