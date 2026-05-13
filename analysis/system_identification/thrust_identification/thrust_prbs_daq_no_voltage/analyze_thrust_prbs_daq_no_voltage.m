%% analyze_thrust_prbs_daq_no_voltage.m
% Analyze thrust PRBS DAQ files.
%
% Expected CSV columns:
% t_ms,run_name,segment,pwm_us,raw_avg,force_N,phase

clear; clc; close all;

%% ----------------------------
% User options
% ----------------------------

USE_LATEST_CSV = true;
SAVE_FIGURES = true;

% Used only if USE_LATEST_CSV = false
trainingFile = "2026_05_11_thrust_prbs_daq_00.csv";

runNames = [
    "global_1100_1950"
    "local_1400_1650"
    "local_1700_1850"
];

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

if USE_LATEST_CSV
    trainingFile = selectLatestIndexedCsv(csvFiles);
end

trainingPath = fullfile(dataDir, trainingFile);

fprintf("\nTraining / identification file:\n  %s\n", trainingPath);

%% ----------------------------
% Cross-run overlay for drift / repeatability check
% ----------------------------

for r = 1:numel(runNames)
    runName = runNames(r);

    figure;
    hold on; grid on;

    for k = 1:numel(csvFiles)
        thisFile = string(csvFiles(k).name);
        thisPath = fullfile(csvFiles(k).folder, csvFiles(k).name);

        T = readtable(thisPath);

        D = getRunData(T, runName);

        if height(D) < 5
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

        D = getRunData(T, runName);

        if height(D) < 5
            continue;
        end

        y_N = D.force_N;
        u_pwm = D.pwm_us;

        maxThrust_N = max(y_N);
        meanTop10Thrust_N = mean(maxk(y_N, min(10, numel(y_N))));
        meanThrust_N = mean(y_N);

        maxPwm_us = max(u_pwm);

        driftRows = [driftRows; {
            thisFile, ...
            k, ...
            runName, ...
            maxPwm_us, ...
            maxThrust_N, ...
            meanTop10Thrust_N, ...
            meanThrust_N ...
        }];
    end
end

driftTable = cell2table(driftRows, ...
    'VariableNames', { ...
        'file', ...
        'run_index', ...
        'run_name', ...
        'max_pwm_us', ...
        'max_thrust_N', ...
        'mean_top10_thrust_N', ...
        'mean_thrust_N' ...
    });

disp("Sequential thrust drift summary:");
disp(driftTable);

%% ----------------------------
% Compute drop relative to first run for each run_name
% ----------------------------

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
    'percent_max_from_first' ...
}));

%% ----------------------------
% Plot max thrust drift by run order
% ----------------------------

for r = 1:numel(runNames)
    runName = runNames(r);
    idx = driftTable.run_name == runName;

    if ~any(idx)
        continue;
    end

    figure;
    hold on; grid on;

    sub = driftTable(idx, :);

    plot(sub.run_index, sub.max_thrust_N, 'o-', 'LineWidth', 1.5, ...
        'DisplayName', 'Max thrust');

    plot(sub.run_index, sub.mean_top10_thrust_N, 's--', 'LineWidth', 1.5, ...
        'DisplayName', 'Mean top 10 thrust samples');

    xlabel("Sequential CSV run index");
    ylabel("Thrust (N)");
    title("Sequential Thrust Drift - " + runName, "Interpreter", "none");
    legend("Location", "best");
end

%% ----------------------------
% Load training data
% ----------------------------

Ttrain = readtable(trainingPath);

models = struct();

%% ----------------------------
% Identify model for each PRBS run
% ----------------------------

for r = 1:numel(runNames)
    runName = runNames(r);

    D = getRunData(Ttrain, runName);

    if height(D) < 5
        warning("Skipping %s: not enough PRBS samples.", runName);
        continue;
    end

    model = fitFirstOrderDiscreteModel(D);

    models.(matlab.lang.makeValidName(runName)) = model;

    fprintf("\nRun: %s\n", runName);
    fprintf("  Model: y[k+1] = a*y[k] + b*u[k] + c\n");
    fprintf("  a = %.6f\n", model.a);
    fprintf("  b = %.9f\n", model.b);
    fprintf("  c = %.6f\n", model.c);
    fprintf("  Training RMSE = %.6f N\n", model.rmse_N);
    fprintf("  Approx tau = %.6f s\n", model.tau_s);

    %% Plot training measured vs predicted
    figure;
    hold on; grid on;

    plot(model.t_s, model.y_N, "LineWidth", 1.2, "DisplayName", "Measured");
    plot(model.t_s, model.yhat_N, "--", "LineWidth", 1.5, "DisplayName", "One-step prediction");

    xlabel("Time (s)");
    ylabel("Thrust (N)");
    title("Training Fit - " + runName, "Interpreter", "none");
    legend("Location", "best");

    %% Plot input
    figure;
    hold on; grid on;

    plot(model.t_s, model.u_pwm, "LineWidth", 1.2);

    xlabel("Time (s)");
    ylabel("PWM command (\mus)");
    title("PRBS Input - " + runName, "Interpreter", "none");
end

%% ----------------------------
% Validation on earlier CSV files
% ----------------------------

validationRows = [];

for k = 1:numel(csvFiles)
    validationFile = string(csvFiles(k).name);

    if validationFile == string(trainingFile)
        continue;
    end

    validationPath = fullfile(csvFiles(k).folder, csvFiles(k).name);
    V = readtable(validationPath);

    fprintf("\nValidation file:\n  %s\n", validationPath);

    for r = 1:numel(runNames)
        runName = runNames(r);
        modelKey = matlab.lang.makeValidName(runName);

        if ~isfield(models, modelKey)
            continue;
        end

        Dval = getRunData(V, runName);

        if height(Dval) < 5
            warning("Skipping validation %s in %s: not enough samples.", runName, validationFile);
            continue;
        end

        model = models.(modelKey);
        val = simulateFirstOrderModel(Dval, model);

        validationRows = [validationRows; {
            validationFile, ...
            runName, ...
            val.rmse_N, ...
            val.mae_N, ...
            val.max_abs_error_N, ...
            height(Dval) ...
        }];

        figure;
        hold on; grid on;

        plot(val.t_s, val.y_N, "LineWidth", 1.2, "DisplayName", "Measured");
        plot(val.t_s, val.yhat_N, "--", "LineWidth", 1.5, "DisplayName", "Predicted");

        xlabel("Time (s)");
        ylabel("Thrust (N)");
        title("Validation Fit - " + runName + " - " + erase(validationFile, ".csv"), ...
            "Interpreter", "none");
        legend("Location", "best");
    end
end

if ~isempty(validationRows)
    validationTable = cell2table(validationRows, ...
        'VariableNames', { ...
            'file', ...
            'run_name', ...
            'rmse_N', ...
            'mae_N', ...
            'max_abs_error_N', ...
            'n_samples' ...
        });

    disp("Validation summary:");
    disp(validationTable);
else
    fprintf("\nNo validation files found besides training file.\n");
end

%% ----------------------------
% Save figures
% ----------------------------

cd(plotDir);
saveAllFiguresIfEnabled(SAVE_FIGURES);

%% ============================================================
% Local helper functions
% ============================================================

function D = getRunData(T, runName)
    idx = strcmp(string(T.run_name), runName) & ...
          strcmpi(string(T.phase), "prbs");

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.t_ms = D.t_ms - D.t_ms(1);
end

function model = fitFirstOrderDiscreteModel(D)
    t_s = D.t_ms / 1000;
    u = D.pwm_us;
    y = D.force_N;

    yk = y(1:end-1);
    uk = u(1:end-1);
    ykp1 = y(2:end);

    Phi = [yk, uk, ones(size(yk))];

    theta = Phi \ ykp1;

    a = theta(1);
    b = theta(2);
    c = theta(3);

    yhat = zeros(size(y));
    yhat(1) = y(1);

    for k = 1:numel(y)-1
        yhat(k+1) = a*y(k) + b*u(k) + c;
    end

    err = y - yhat;

    dt_s = median(diff(t_s));

    if a > 0 && a < 1
        tau_s = -dt_s / log(a);
    else
        tau_s = NaN;
    end

    model.a = a;
    model.b = b;
    model.c = c;
    model.t_s = t_s;
    model.u_pwm = u;
    model.y_N = y;
    model.yhat_N = yhat;
    model.err_N = err;
    model.rmse_N = sqrt(mean(err.^2));
    model.mae_N = mean(abs(err));
    model.max_abs_error_N = max(abs(err));
    model.dt_s = dt_s;
    model.tau_s = tau_s;
end

function val = simulateFirstOrderModel(D, model)
    t_s = D.t_ms / 1000;
    u = D.pwm_us;
    y = D.force_N;

    yhat = zeros(size(y));
    yhat(1) = y(1);

    for k = 1:numel(y)-1
        yhat(k+1) = model.a*yhat(k) + model.b*u(k) + model.c;
    end

    err = y - yhat;

    val.t_s = t_s;
    val.u_pwm = u;
    val.y_N = y;
    val.yhat_N = yhat;
    val.err_N = err;
    val.rmse_N = sqrt(mean(err.^2));
    val.mae_N = mean(abs(err));
    val.max_abs_error_N = max(abs(err));
end