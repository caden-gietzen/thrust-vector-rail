%% analyze_thrust_prps_frequency_fit.m
% Analyze thrust PRPS DAQ files using frequency-domain fitting.
%
% PRPS = pseudo-random periodic signal.
%
% Expected newer CSV columns from thrust_prps_daq_voltage.py:
%   t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,pwm_us,
%   raw_count,tared_count,force_N,
%   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
%   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
%   phase,period_index,period_sample_index
%
% Identification goal:
%   Estimate empirical frequency response:
%
%       G_emp(jw) = Delta Force(jw) / Delta PWM(jw)
%
%   Then fit continuous-time transfer functions:
%
%       1st order:
%           G(s) = K / (tau*s + 1)
%
%       1st order + delay:
%           G(s) = K*exp(-L*s) / (tau*s + 1)
%
%       2nd order lag:
%           G(s) = K / ((tau1*s + 1)*(tau2*s + 1))
%
%       2nd order lag + delay:
%           G(s) = K*exp(-L*s) / ((tau1*s + 1)*(tau2*s + 1))
%
% Frequency fitting is done using weighted complex error.
%
% Recommended weighting:
%   Relative complex error:
%
%       error_i = (G_model_i - G_emp_i) / max(abs(G_emp_i), gain_floor)
%
%   optionally multiplied by coherence weighting.
%
% Why:
%   - Fitting raw complex error makes large low-frequency magnitudes dominate.
%   - Fitting pure inverse magnitude can overweight tiny noisy high-frequency points.
%   - The gain floor prevents garbage high-frequency points from dominating.
%
% Units:
%   Input: PWM command in microseconds
%   Output: thrust in newtons
%   Transfer function magnitude: N/us

clear; clc; close all

%% Dataset split options
% ============================================================

% Data split modes:
%   "folders"       = load training/validation files from explicit folders
%   "first_n_files" = old behavior, train on first N sorted files from candidate
%   "single_file"   = old behavior, train on one named file from candidate
DATA_SPLIT_MODE = "folders";

% Used only if DATA_SPLIT_MODE = "folders".
TRAIN_FOLDER_NAME = fullfile("accepted", "train");
VALIDATION_FOLDER_NAME = fullfile("accepted", "validation");

% Used only if DATA_SPLIT_MODE = "first_n_files".
NUM_TRAINING_FILES = 4;

% Used only if DATA_SPLIT_MODE = "single_file".
trainingFile = "thrust_prps_daq_voltage_set01_seed2001.csv";

%% User options

SAVE_FIGURES = true;

% Leave empty to auto-detect run names from training data.
runNames = strings(0, 1);

USE_NOMINAL_COMMAND_DT_FOR_FRF = true;
NOMINAL_COMMAND_DT_S = 0.020;

%% PRPS / frequency extraction options

% If true, use only phase == "prps".
USE_ONLY_PRPS_PHASE = true;

% Minimum complete periods required per file/run.
MIN_PERIODS_PER_RUN = 1;

% Minimum samples required in one period.
MIN_SAMPLES_PER_PERIOD = 20;

% If true, infer excited frequencies from the input PWM spectrum.
% This is robust because the Pico CSV stores period_sample_index but not
% the frequency list directly.
INFER_EXCITED_FREQUENCIES_FROM_INPUT = true;

% Used only if INFER_EXCITED_FREQUENCIES_FROM_INPUT = false.
MANUAL_EXCITED_FREQS_HZ = [
    0.05
    0.075
    0.10
    0.15
    0.20
    0.30
    0.40
    0.60
    0.80
    1.00
    1.25
    1.50
    2.00
];

% If inferring frequencies, keep DFT bins whose input magnitude is above:
%
%   threshold = max_input_bin_magnitude * INPUT_BIN_RELATIVE_THRESHOLD
%
% Increase if too many tiny bins are detected.
% Decrease if valid injected frequencies are being missed.
INPUT_BIN_RELATIVE_THRESHOLD = 0.05;

% Restrict frequency range used for fitting and plotting.
FREQ_MIN_HZ = 0.03;
FREQ_MAX_HZ = 3.00;

% Reject frequency points with coherence below this value for fitting.
% Coherence is estimated across repeated periods/files.
MIN_COHERENCE_FOR_FIT = 0.50;

% If true, still plot rejected low-coherence points as faded/marked points.
PLOT_REJECTED_FREQ_POINTS = true;

% Remove per-period DC offsets before Fourier coefficient extraction.
REMOVE_PERIOD_MEAN = true;

% Optional linear detrend per period before Fourier extraction.
% Usually false for short periodic records; mean removal is enough.
DETREND_EACH_PERIOD = false;

%% Frequency-domain fitting options

% Candidate model toggles.
FIT_FIRST_ORDER                 = true;
FIT_FIRST_ORDER_DELAY           = true;
FIT_SECOND_ORDER_LAG            = true;
FIT_SECOND_ORDER_LAG_DELAY      = true;

% Model selection:
%   "lowest_validation_error" = prefer validation if available
%   "lowest_training_error"   = choose best training fit
%   "simplicity_tolerance"    = simplest model within tolerance of best training fit
MODEL_SELECTION_MODE = "simplicity_tolerance";

% If simplicity mode is used, choose simplest model whose weighted training
% error is within this relative tolerance of the best model.
SIMPLE_MODEL_RELATIVE_TOLERANCE = 0.05;

% Weighting mode:
%   "relative_complex"      = divide complex error by magnitude floor
%   "relative_with_coh"     = relative error weighted by sqrt(coherence)
%   "absolute_complex"      = raw complex error, not recommended initially
WEIGHTING_MODE = "relative_with_coh";

% Gain floor strategy:
%   "fraction_of_median" = floor = fraction * median(abs(G_emp))
%   "manual"             = floor = MANUAL_GAIN_FLOOR_N_PER_US
GAIN_FLOOR_MODE = "fraction_of_median";
GAIN_FLOOR_FRACTION_OF_MEDIAN = 0.20;
MANUAL_GAIN_FLOOR_N_PER_US = 1e-4;

% Optimization settings.
MAX_FMINSEARCH_ITER = 5000;
MAX_FMINSEARCH_EVAL = 10000;

% Initial parameter guesses.
% These are intentionally conservative.
INITIAL_K_N_PER_US = 0.003;
INITIAL_TAU_S = 0.40;
INITIAL_DELAY_S = 0.1;
INITIAL_TAU2_S = 0.08;

% Bound-like soft limits through parameter transform/clamping.
MIN_TAU_S = 0.02;
MAX_TAU_S = 10.0;
MIN_DELAY_S = 0.0;
MAX_DELAY_S = 2.0;
MIN_GAIN_ABS = 1e-6;
MAX_GAIN_ABS = 0.02;

%% Time-domain translation options
% ============================================================

% Translate fitted frequency-domain model into time-domain prediction using
% lsim on the PRPS validation/training input.
PLOT_TIME_DOMAIN_TRANSLATION_TRAINING   = false;
PLOT_TIME_DOMAIN_TRANSLATION_VALIDATION = false;

% Time-domain fit uses measured initial output offset:
% y_model_total = mean(y_measured) + lsim(G, u - mean(u))
CENTER_TIME_DOMAIN_SIGNALS = true;

USE_RAW_TIMESTAMP_GRID_FOR_LSIM = true;

%% Plot toggles
% ============================================================

PLOT_TRAINING_EMPIRICAL_BODE_WITH_FITS      = false;
PLOT_VALIDATION_BODE_VS_TRAINING_MODEL      = false;
PLOT_TRAINING_VS_VALIDATION_EMPIRICAL_BODE  = false;
PLOT_COHERENCE_BY_RUN                       = false;
PLOT_MODEL_COMPARISON_BAR                   = false;
PLOT_NORMALIZED_BODE                        = true;
PLOT_COMPREHENSIVE_MODEL_COMPARISON_BAR     = true;

PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_TRAINING    = false;
PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_VALIDATION  = false;
PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS            = false;

PLOT_RAW_PRPS_TIME_SNIPPET                  = false;
PLOT_INPUT_SPECTRUM_DIAGNOSTIC              = false;
PLOT_PERIOD_OVERLAY_DIAGNOSTIC              = false;
PLOT_SAMPLE_TIME_DIAGNOSTIC                 = false;
PLOT_VOLTAGE_TIME                           = false;

PLOT_PHASE_RESIDUAL_BY_FREQUENCY            = false;

PLOT_VALIDATION_BEST_FIT_TIME_WINDOW        = true;
VALIDATION_BEST_FIT_TIME_WINDOW_S          = [60, 70];

% Usually keep false at first to avoid figure explosion.
% false = plot first validation group/file only for each run.
% true  = plot every validation group/file for each run.
PLOT_ALL_VALIDATION_TIME_WINDOW_GROUPS      = false;

%% Locate mirrored folders and select train/validation files
% ============================================================

scriptPath = mfilename("fullpath");

% Base raw-data folder for this dataset.
% Keep candidate as the anchor because getMirroredRawDataDir already knows
% how to find the mirrored thrust_prps_daq_voltage/candidate folder.
candidateDataDir = getMirroredRawDataDir(scriptPath, "candidate");

% Parent dataset directory:
%   .../thrust_prps_daq_voltage/
datasetRootDir = fileparts(candidateDataDir);

plotDir = getMirroredPlotDir(scriptPath);

addpath(genpath(fullfile(findRepoRoot(scriptPath), "analysis", "util")));

switch string(DATA_SPLIT_MODE)

    case "folders"
        trainDir = fullfile(datasetRootDir, TRAIN_FOLDER_NAME);
        validationDir = fullfile(datasetRootDir, VALIDATION_FOLDER_NAME);

        trainingCsvFiles = getSortedCsvFilesFromFolder(trainDir, true);
        validationCsvFiles = getSortedCsvFilesFromFolder(validationDir, false);

        fprintf("\nDataset split mode: folders\n");
        fprintf("Training folder:\n  %s\n", trainDir);
        fprintf("Validation folder:\n  %s\n", validationDir);

        fprintf("\nTraining / identification files:\n");
        printCsvFileList(trainingCsvFiles);

        fprintf("\nValidation files:\n");
        printCsvFileList(validationCsvFiles);

    case "first_n_files"
        dataDir = candidateDataDir;

        csvFiles = getSortedCsvFilesFromFolder(dataDir, true);

        nTrain = min(NUM_TRAINING_FILES, numel(csvFiles));

        trainingCsvFiles = csvFiles(1:nTrain);
        validationCsvFiles = csvFiles(nTrain+1:end);

        fprintf("\nDataset split mode: first_n_files\n");
        fprintf("Candidate folder:\n  %s\n", dataDir);

        fprintf("\nTraining / identification files:\n");
        printCsvFileList(trainingCsvFiles);

        fprintf("\nValidation files:\n");
        printCsvFileList(validationCsvFiles);

    case "single_file"
        dataDir = candidateDataDir;

        csvFiles = getSortedCsvFilesFromFolder(dataDir, true);

        trainingPath = fullfile(dataDir, trainingFile);
        trainingCsvFiles = dir(trainingPath);

        if isempty(trainingCsvFiles)
            error("Training file not found:\n%s", trainingPath);
        end

        validationCsvFiles = csvFiles(string({csvFiles.name}) ~= string(trainingFile));

        fprintf("\nDataset split mode: single_file\n");
        fprintf("Candidate folder:\n  %s\n", dataDir);

        fprintf("\nTraining / identification file:\n");
        printCsvFileList(trainingCsvFiles);

        fprintf("\nValidation files:\n");
        printCsvFileList(validationCsvFiles);

    otherwise
        error("Unknown DATA_SPLIT_MODE: %s", DATA_SPLIT_MODE);
end

trainingLabel = makeFileListLabel(trainingCsvFiles);

%% Load combined tables
% ============================================================

Ttrain = readCombinedCsvTables(trainingCsvFiles);

if isempty(validationCsvFiles)
    Tval = table();
else
    Tval = readCombinedCsvTables(validationCsvFiles);
end

if isempty(runNames)
    runNames = unique(string(Ttrain.run_name), "stable");
    runNames = runNames(strlength(strtrim(runNames)) > 0);
    fprintf("\nAuto-detected run names from training data:\n");
    for k = 1:numel(runNames)
        fprintf("  %s\n", runNames(k));
    end
end

%% Main frequency-domain identification loop
% ============================================================

allSummaryRows = [];
allBestRows = [];

bestModels = struct();
trainingFrfs = struct();
validationFrfs = struct();
candidateModelSets = struct();

for r = 1:numel(runNames)
    runName = runNames(r);
    modelKey = matlab.lang.makeValidName(runName);

    fprintf("\n============================================================\n");
    fprintf("Run: %s\n", runName);
    fprintf("============================================================\n");

    Dtrain = getRunData(Ttrain, runName, USE_ONLY_PRPS_PHASE);

    printSamplingRateDiagnostic(Dtrain, runName, "training");

    if height(Dtrain) < MIN_SAMPLES_PER_PERIOD
        warning("Skipping %s: not enough training samples.", runName);
        continue;
    end

    trainFrf = estimatePrpsFrfFromTable( ...
        Dtrain, ...
        runName, ...
        "training", ...
        INFER_EXCITED_FREQUENCIES_FROM_INPUT, ...
        MANUAL_EXCITED_FREQS_HZ, ...
        INPUT_BIN_RELATIVE_THRESHOLD, ...
        FREQ_MIN_HZ, ...
        FREQ_MAX_HZ, ...
        REMOVE_PERIOD_MEAN, ...
        DETREND_EACH_PERIOD, ...
        MIN_PERIODS_PER_RUN, ...
        MIN_SAMPLES_PER_PERIOD, ...
        USE_NOMINAL_COMMAND_DT_FOR_FRF, ...
        NOMINAL_COMMAND_DT_S);

    if isempty(trainFrf.f_Hz)
        warning("Skipping %s: no training FRF points.", runName);
        continue;
    end

    trainingFrfs.(modelKey) = trainFrf;

    if ~isempty(Tval)
        DvalAll = getRunData(Tval, runName, USE_ONLY_PRPS_PHASE);

        if height(DvalAll) >= MIN_SAMPLES_PER_PERIOD
            valFrf = estimatePrpsFrfFromTable( ...
                DvalAll, ...
                runName, ...
                "validation", ...
                false, ...
                trainFrf.f_Hz, ...
                INPUT_BIN_RELATIVE_THRESHOLD, ...
                FREQ_MIN_HZ, ...
                FREQ_MAX_HZ, ...
                REMOVE_PERIOD_MEAN, ...
                DETREND_EACH_PERIOD, ...
                MIN_PERIODS_PER_RUN, ...
                MIN_SAMPLES_PER_PERIOD, ...
                USE_NOMINAL_COMMAND_DT_FOR_FRF, ...
                NOMINAL_COMMAND_DT_S);

            validationFrfs.(modelKey) = valFrf;
        else
            valFrf = emptyFrf();
        end
    else
        valFrf = emptyFrf();
    end

    if PLOT_RAW_PRPS_TIME_SNIPPET
        plotRawTimeSnippet(Dtrain, runName, "training");
    end

    if PLOT_INPUT_SPECTRUM_DIAGNOSTIC
        plotInputSpectrumDiagnostic(trainFrf, runName, "training");
    end

    if PLOT_COHERENCE_BY_RUN
        plotCoherence(trainFrf, valFrf, runName);
    end

    fitMask = trainFrf.coherence >= MIN_COHERENCE_FOR_FIT & ...
              isfinite(trainFrf.G_emp) & ...
              isfinite(trainFrf.f_Hz) & ...
              trainFrf.f_Hz >= FREQ_MIN_HZ & ...
              trainFrf.f_Hz <= FREQ_MAX_HZ;

    if nnz(fitMask) < 2
        warning("Skipping %s: fewer than 2 usable frequency points after coherence filtering.", runName);
        continue;
    end

    candidateModels = fitFrequencyModels( ...
        trainFrf.f_Hz(fitMask), ...
        trainFrf.G_emp(fitMask), ...
        trainFrf.coherence(fitMask), ...
        FIT_FIRST_ORDER, ...
        FIT_FIRST_ORDER_DELAY, ...
        FIT_SECOND_ORDER_LAG, ...
        FIT_SECOND_ORDER_LAG_DELAY, ...
        WEIGHTING_MODE, ...
        GAIN_FLOOR_MODE, ...
        GAIN_FLOOR_FRACTION_OF_MEDIAN, ...
        MANUAL_GAIN_FLOOR_N_PER_US, ...
        INITIAL_K_N_PER_US, ...
        INITIAL_TAU_S, ...
        INITIAL_DELAY_S, ...
        INITIAL_TAU2_S, ...
        MIN_TAU_S, ...
        MAX_TAU_S, ...
        MIN_DELAY_S, ...
        MAX_DELAY_S, ...
        MIN_GAIN_ABS, ...
        MAX_GAIN_ABS, ...
        MAX_FMINSEARCH_ITER, ...
        MAX_FMINSEARCH_EVAL);

    if isempty(candidateModels)
        warning("No models fit for %s.", runName);
        continue;
    end

    candidateModels = scoreModelsOnFrf(candidateModels, trainFrf, "training");

    if ~isempty(valFrf.f_Hz)
        candidateModels = scoreModelsOnFrf(candidateModels, valFrf, "validation");
    end

    bestModel = selectBestFrequencyModel( ...
        candidateModels, ...
        MODEL_SELECTION_MODE, ...
        SIMPLE_MODEL_RELATIVE_TOLERANCE);

    bestModels.(modelKey) = bestModel;

    candidateModelSets.(modelKey) = candidateModels;

    allBestRows = [allBestRows; {
        trainingLabel, ...
        runName, ...
        bestModel.model_type, ...
        bestModel.complexity, ...
        bestModel.K, ...
        bestModel.tau1_s, ...
        bestModel.tau2_s, ...
        bestModel.delay_s, ...
        bestModel.train_weighted_error, ...
        getfieldwithdefault(bestModel, "validation_weighted_error", NaN), ...
        bestModel.train_mag_rmse_dB, ...
        bestModel.train_phase_rmse_deg, ...
        getfieldwithdefault(bestModel, "validation_mag_rmse_dB", NaN), ...
        getfieldwithdefault(bestModel, "validation_phase_rmse_deg", NaN), ...
        bandwidthHzFromTau(bestModel.tau1_s)
    }];

    fprintf("\nBest model for %s:\n", runName);
    printFrequencyModel(bestModel);

    for m = 1:numel(candidateModels)
        cm = candidateModels(m);

        isBestModel = string(cm.model_type) == string(bestModel.model_type);

        allSummaryRows = [allSummaryRows; {
            trainingLabel, ...
            runName, ...
            cm.model_type, ...
            isBestModel, ...
            cm.complexity, ...
            cm.K, ...
            cm.tau1_s, ...
            cm.tau2_s, ...
            cm.delay_s, ...
            cm.train_weighted_error, ...
            getfieldwithdefault(cm, "validation_weighted_error", NaN), ...
            cm.train_mag_rmse_dB, ...
            cm.train_phase_rmse_deg, ...
            getfieldwithdefault(cm, "validation_mag_rmse_dB", NaN), ...
            getfieldwithdefault(cm, "validation_phase_rmse_deg", NaN), ...
            bandwidthHzFromTau(cm.tau1_s)
        }];
    end

    if PLOT_NORMALIZED_BODE
        plotNormalizedBodeBestFitAllData(trainFrf, valFrf, bestModel, runName, ...
            MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS);
    end

    if PLOT_TRAINING_EMPIRICAL_BODE_WITH_FITS
        plotEmpiricalBodeWithFits(trainFrf, candidateModels, bestModel, runName, "training", ...
            MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS);
    end

    if PLOT_VALIDATION_BODE_VS_TRAINING_MODEL && ~isempty(valFrf.f_Hz)
        plotValidationBodeVsTrainingModel(valFrf, bestModel, runName, MIN_COHERENCE_FOR_FIT);
    end

    if PLOT_TRAINING_VS_VALIDATION_EMPIRICAL_BODE && ~isempty(valFrf.f_Hz)
        plotTrainingVsValidationEmpiricalBode(trainFrf, valFrf, runName, MIN_COHERENCE_FOR_FIT);
    end

    if PLOT_MODEL_COMPARISON_BAR
        plotModelComparison(candidateModels, runName);
    end

    if PLOT_TIME_DOMAIN_TRANSLATION_TRAINING
        plotTimeDomainTranslationForDataset( ...
            Dtrain, ...
            bestModel, ...
            candidateModels, ...
            runName, ...
            "training", ...
            CENTER_TIME_DOMAIN_SIGNALS, ...
            PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_TRAINING, ...
            PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS, ...
            USE_RAW_TIMESTAMP_GRID_FOR_LSIM);
    end

    
    if PLOT_TIME_DOMAIN_TRANSLATION_VALIDATION && ~isempty(Tval)
        DvalTime = getRunData(Tval, runName, USE_ONLY_PRPS_PHASE);

        if height(DvalTime) >= MIN_SAMPLES_PER_PERIOD
            plotTimeDomainTranslationForDataset( ...
                DvalTime, ...
                bestModel, ...
                candidateModels, ...
                runName, ...
                "validation", ...
                CENTER_TIME_DOMAIN_SIGNALS, ...
                PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_VALIDATION, ...
                PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS, ...
                USE_RAW_TIMESTAMP_GRID_FOR_LSIM);
        end
    end

        if PLOT_VALIDATION_BEST_FIT_TIME_WINDOW && ~isempty(Tval)
            DvalWindow = getRunData(Tval, runName, USE_ONLY_PRPS_PHASE);
    
            if height(DvalWindow) >= MIN_SAMPLES_PER_PERIOD
                plotValidationBestFitTimeWindow( ...
                    DvalWindow, ...
                    bestModel, ...
                    runName, ...
                    VALIDATION_BEST_FIT_TIME_WINDOW_S, ...
                    CENTER_TIME_DOMAIN_SIGNALS, ...
                    USE_RAW_TIMESTAMP_GRID_FOR_LSIM, ...
                    PLOT_ALL_VALIDATION_TIME_WINDOW_GROUPS);
            end
        end

    if PLOT_SAMPLE_TIME_DIAGNOSTIC
        plotSampleTimeDiagnostic(Dtrain, runName, "training");
    end

    if PLOT_VOLTAGE_TIME && hasVoltage(Dtrain)
        plotVoltageTime(Dtrain, runName, "training");
    end

    if PLOT_PHASE_RESIDUAL_BY_FREQUENCY
        plotPhaseResidualByFrequency(trainFrf, bestModel, runName, "training", MIN_COHERENCE_FOR_FIT);
    
        if ~isempty(valFrf.f_Hz)
            plotPhaseResidualByFrequency(valFrf, bestModel, runName, "validation", MIN_COHERENCE_FOR_FIT);
        end
end
end

%% Summary tables and cross-region parameter comparisons
% ============================================================

if ~isempty(allSummaryRows)
    summaryTable = cell2table(allSummaryRows, ...
        'VariableNames', { ...
            'file', ...
            'run_name', ...
            'model_type', ...
            'is_best_model', ...
            'complexity', ...
            'K_N_per_us', ...
            'tau1_s', ...
            'tau2_s', ...
            'delay_s', ...
            'train_weighted_error', ...
            'validation_weighted_error', ...
            'train_mag_rmse_dB', ...
            'train_phase_rmse_deg', ...
            'validation_mag_rmse_dB', ...
            'validation_phase_rmse_deg', ...
            'dominant_bandwidth_Hz' ...
        });

    disp("Frequency-domain model summary, all candidate models:");
    disp(summaryTable);

    bestModelTable = cell2table(allBestRows, ...
        'VariableNames', { ...
            'file', ...
            'run_name', ...
            'best_model_type', ...
            'complexity', ...
            'K_N_per_us', ...
            'tau1_s', ...
            'tau2_s', ...
            'delay_s', ...
            'train_weighted_error', ...
            'validation_weighted_error', ...
            'train_mag_rmse_dB', ...
            'train_phase_rmse_deg', ...
            'validation_mag_rmse_dB', ...
            'validation_phase_rmse_deg', ...
            'dominant_bandwidth_Hz' ...
        });

    disp("Best model selected for each global/local region:");
    disp(bestModelTable);

    fprintf("\n============================================================\n");
    fprintf("Cross-region parameter comparison by model type\n");
    fprintf("Use this to check whether only K changes, or whether tau/delay also move.\n");
    fprintf("============================================================\n");

    compareParametersAcrossRegions(summaryTable, runNames);

    fprintf("\n============================================================\n");
    fprintf("Best-model parameter comparison across regions\n");
    fprintf("This compares only the selected best model for each run.\n");
    fprintf("Be careful: model structures may differ, so K/tau/delay are not always apples-to-apples.\n");
    fprintf("============================================================\n");

    compareBestModelsAcrossRegions(bestModelTable);

    if PLOT_COMPREHENSIVE_MODEL_COMPARISON_BAR
        plotComprehensiveModelComparisonBar(bestModelTable, SIMPLE_MODEL_RELATIVE_TOLERANCE);
    end

else
    fprintf("\nNo frequency-domain models generated.\n");
end


%% Save figures
% ============================================================

saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);

%% Local helper functions
% ============================================================

function csvFiles = getSortedCsvFilesFromFolder(folderPath, requireNonempty)
    if ~isfolder(folderPath)
        if requireNonempty
            error("CSV folder does not exist:\n%s", folderPath);
        else
            warning("CSV folder does not exist:\n%s", folderPath);
            csvFiles = [];
            return;
        end
    end

    csvFiles = dir(fullfile(folderPath, "*.csv"));

    if isempty(csvFiles)
        if requireNonempty
            error("No CSV files found in:\n%s", folderPath);
        else
            fprintf("  None. No CSV files found in:\n  %s\n", folderPath);
            return;
        end
    end

    [~, sortIdx] = sort(string({csvFiles.name}));
    csvFiles = csvFiles(sortIdx);
end

function printCsvFileList(csvFiles)
    if isempty(csvFiles)
        fprintf("  None.\n");
        return;
    end

    for k = 1:numel(csvFiles)
        fprintf("  %s\n", fullfile(csvFiles(k).folder, csvFiles(k).name));
    end
end

function D = getRunData(T, runName, useOnlyPrpsPhase)
    if isempty(T)
        D = table();
        return;
    end

    if ~ismember("run_name", string(T.Properties.VariableNames))
        error("CSV does not contain run_name column.");
    end

    idx = strcmp(string(T.run_name), runName);

    if useOnlyPrpsPhase && ismember("phase", string(T.Properties.VariableNames))
        idx = idx & strcmpi(string(T.phase), "prps");
    end

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.t_ms = forceNumeric(D.t_ms);
    D.t_s = forceNumeric(D.t_s);
    D.pwm_us = forceNumeric(D.pwm_us);
    D.force_N = forceNumeric(D.force_N);

    if ismember("period_index", string(D.Properties.VariableNames))
        D.period_index = forceNumeric(D.period_index);
    else
        D.period_index = zeros(height(D), 1);
    end

    if ismember("period_sample_index", string(D.Properties.VariableNames))
        D.period_sample_index = forceNumeric(D.period_sample_index);
    else
        D.period_sample_index = forceNumeric(D.segment);
    end

    if ismember("battery_voltage_V", string(D.Properties.VariableNames))
        D.battery_voltage_V = forceNumeric(D.battery_voltage_V);
    end

    if ismember("battery_current_A", string(D.Properties.VariableNames))
        D.battery_current_A = forceNumeric(D.battery_current_A);
    end

    valid = isfinite(D.t_ms) & ...
            isfinite(D.pwm_us) & ...
            isfinite(D.force_N) & ...
            isfinite(D.period_index) & ...
            isfinite(D.period_sample_index);

    D = D(valid, :);

    if isempty(D)
        return;
    end

    D.t_ms = D.t_ms - D.t_ms(1);
end

function frf = estimatePrpsFrfFromTable( ...
    D, ...
    runName, ...
    datasetLabel, ...
    inferFreqsFromInput, ...
    freqListHz, ...
    inputBinRelativeThreshold, ...
    freqMinHz, ...
    freqMaxHz, ...
    removePeriodMean, ...
    detrendEachPeriod, ...
    minPeriodsPerRun, ...
    minSamplesPerPeriod, ...
    useNominalCommandDtForFrf, ...
    nominalCommandDtS)

    frf = emptyFrf();

    if height(D) < minSamplesPerPeriod
        return;
    end

    if ismember("source_file_index", string(D.Properties.VariableNames))
        fileGroups = double(D.source_file_index);
    else
        fileGroups = ones(height(D), 1);
    end

    uniqueFiles = unique(fileGroups, "stable");

    allU = [];
    allY = [];
    allF = [];
    allBins = [];
    allFileIndex = [];
    allPeriodIndex = [];

    inferredFreqsHz = [];

    for fg = 1:numel(uniqueFiles)
        fileIdx = uniqueFiles(fg);
        Df = D(fileGroups == fileIdx, :);

        periodIds = unique(Df.period_index, "stable");
        periodIds = periodIds(periodIds >= 0);

        if numel(periodIds) < minPeriodsPerRun
            continue;
        end

        for p = 1:numel(periodIds)
            periodId = periodIds(p);

            Dp = Df(Df.period_index == periodId, :);

            [uPeriod, yPeriod, tPeriod, sampleIndex] = makeUniformPeriodVectors(Dp);

            if numel(uPeriod) < minSamplesPerPeriod
                continue;
            end

            if removePeriodMean
                uPeriod = uPeriod - mean(uPeriod, "omitnan");
                yPeriod = yPeriod - mean(yPeriod, "omitnan");
            end

            if detrendEachPeriod
                uPeriod = detrend(uPeriod);
                yPeriod = detrend(yPeriod);
            end

            if useNominalCommandDtForFrf
                dt_s = nominalCommandDtS;
            else
                dt_s = median(diff(tPeriod), "omitnan");
            end

            if ~isfinite(dt_s) || dt_s <= 0
                continue;
            end

            Fs = 1 / dt_s;
            N = numel(uPeriod);
            period_s = N * dt_s;

            Ufft = fft(uPeriod) / N;
            Yfft = fft(yPeriod) / N;

            positiveBins = (1:floor(N/2)).';
            fHz = positiveBins / period_s;

            freqMask = fHz >= freqMinHz & fHz <= freqMaxHz;

            if inferFreqsFromInput && isempty(inferredFreqsHz)
                Uabs = abs(Ufft(positiveBins));
                UabsMasked = Uabs;
                UabsMasked(~freqMask) = 0;

                maxU = max(UabsMasked, [], "omitnan");

                if maxU <= 0 || ~isfinite(maxU)
                    continue;
                end

                keep = UabsMasked >= maxU * inputBinRelativeThreshold & freqMask;

                inferredFreqsHz = fHz(keep);
                inferredFreqsHz = unique(round(inferredFreqsHz, 10), "stable");
            end

            if inferFreqsFromInput
                targetFreqsHz = inferredFreqsHz;
            else
                targetFreqsHz = freqListHz(:);
            end

            if isempty(targetFreqsHz)
                continue;
            end

            for q = 1:numel(targetFreqsHz)
                [~, localIdx] = min(abs(fHz - targetFreqsHz(q)));
                bin = positiveBins(localIdx);
                fActual = fHz(localIdx);

                if fActual < freqMinHz || fActual > freqMaxHz
                    continue;
                end

                allU(end+1, 1) = Ufft(bin);
                allY(end+1, 1) = Yfft(bin);
                allF(end+1, 1) = fActual;
                allBins(end+1, 1) = bin;
                allFileIndex(end+1, 1) = fileIdx;
                allPeriodIndex(end+1, 1) = periodId;
            end
        end
    end

    if isempty(allF)
        return;
    end

    uniqueFreqs = unique(round(allF, 10), "stable");

    G_emp = NaN(numel(uniqueFreqs), 1);
    coherence = NaN(numel(uniqueFreqs), 1);
    Suu = NaN(numel(uniqueFreqs), 1);
    Syy = NaN(numel(uniqueFreqs), 1);
    Syu = NaN(numel(uniqueFreqs), 1);
    nAvg = zeros(numel(uniqueFreqs), 1);

    for i = 1:numel(uniqueFreqs)
        idx = abs(allF - uniqueFreqs(i)) < 1e-9;

        U = allU(idx);
        Y = allY(idx);

        valid = isfinite(U) & isfinite(Y) & abs(U) > 0;

        U = U(valid);
        Y = Y(valid);

        if isempty(U)
            continue;
        end

        Syu_i = mean(Y .* conj(U), "omitnan");
        Suu_i = mean(abs(U).^2, "omitnan");
        Syy_i = mean(abs(Y).^2, "omitnan");

        G_emp(i) = Syu_i / Suu_i;

        coherence(i) = abs(Syu_i)^2 / max(Suu_i * Syy_i, eps);

        Suu(i) = Suu_i;
        Syy(i) = Syy_i;
        Syu(i) = Syu_i;
        nAvg(i) = numel(U);
    end

    validFrf = isfinite(G_emp) & isfinite(coherence) & isfinite(uniqueFreqs(:));

    frf.f_Hz = uniqueFreqs(validFrf);
    frf.w_rad_s = 2*pi*frf.f_Hz;
    frf.G_emp = G_emp(validFrf);
    frf.coherence = coherence(validFrf);
    frf.Suu = Suu(validFrf);
    frf.Syy = Syy(validFrf);
    frf.Syu = Syu(validFrf);
    frf.n_avg = nAvg(validFrf);
    frf.run_name = string(runName);
    frf.dataset_label = string(datasetLabel);
    frf.raw_U = allU;
    frf.raw_Y = allY;
    frf.raw_f_Hz = allF;
    frf.raw_bins = allBins;
    frf.raw_file_index = allFileIndex;
    frf.raw_period_index = allPeriodIndex;

    [frf.f_Hz, sortIdx] = sort(frf.f_Hz);
    frf.w_rad_s = frf.w_rad_s(sortIdx);
    frf.G_emp = frf.G_emp(sortIdx);
    frf.coherence = frf.coherence(sortIdx);
    frf.Suu = frf.Suu(sortIdx);
    frf.Syy = frf.Syy(sortIdx);
    frf.Syu = frf.Syu(sortIdx);
    frf.n_avg = frf.n_avg(sortIdx);

    fprintf("\n%s FRF for %s:\n", datasetLabel, runName);
    fprintf("  Frequency points: %d\n", numel(frf.f_Hz));
    fprintf("  Frequency range: %.4f to %.4f Hz\n", min(frf.f_Hz), max(frf.f_Hz));
    fprintf("  Median coherence: %.3f\n", median(frf.coherence, "omitnan"));
end

function [uPeriod, yPeriod, tPeriod, sampleIndex] = makeUniformPeriodVectors(Dp)
    sampleIndexRaw = forceNumeric(Dp.period_sample_index);
    uRaw = forceNumeric(Dp.pwm_us);
    yRaw = forceNumeric(Dp.force_N);
    tRaw = forceNumeric(Dp.t_ms) / 1000;

    valid = isfinite(sampleIndexRaw) & isfinite(uRaw) & isfinite(yRaw) & isfinite(tRaw);
    sampleIndexRaw = sampleIndexRaw(valid);
    uRaw = uRaw(valid);
    yRaw = yRaw(valid);
    tRaw = tRaw(valid);

    if isempty(sampleIndexRaw)
        uPeriod = [];
        yPeriod = [];
        tPeriod = [];
        sampleIndex = [];
        return;
    end

    sampleIndex = unique(sampleIndexRaw, "stable");
    sampleIndex = sort(sampleIndex);

    uPeriod = NaN(numel(sampleIndex), 1);
    yPeriod = NaN(numel(sampleIndex), 1);
    tPeriod = NaN(numel(sampleIndex), 1);

    for i = 1:numel(sampleIndex)
        idx = sampleIndexRaw == sampleIndex(i);

        uPeriod(i) = mean(uRaw(idx), "omitnan");
        yPeriod(i) = mean(yRaw(idx), "omitnan");
        tPeriod(i) = mean(tRaw(idx), "omitnan");
    end

    valid = isfinite(uPeriod) & isfinite(yPeriod) & isfinite(tPeriod);
    uPeriod = uPeriod(valid);
    yPeriod = yPeriod(valid);
    tPeriod = tPeriod(valid);
    sampleIndex = sampleIndex(valid);

    [sampleIndex, sortIdx] = sort(sampleIndex);
    uPeriod = uPeriod(sortIdx);
    yPeriod = yPeriod(sortIdx);
    tPeriod = tPeriod(sortIdx);
end

function candidateModels = fitFrequencyModels( ...
    f_Hz, ...
    G_emp, ...
    coherence, ...
    fitFirstOrder, ...
    fitFirstOrderDelay, ...
    fitSecondOrderLag, ...
    fitSecondOrderLagDelay, ...
    weightingMode, ...
    gainFloorMode, ...
    gainFloorFractionOfMedian, ...
    manualGainFloor, ...
    initialK, ...
    initialTau, ...
    initialDelay, ...
    initialTau2, ...
    minTau, ...
    maxTau, ...
    minDelay, ...
    maxDelay, ...
    minGainAbs, ...
    maxGainAbs, ...
    maxIter, ...
    maxEval)

    modelTypes = strings(0, 1);

    if fitFirstOrder
        modelTypes(end+1, 1) = "first_order";
    end

    if fitFirstOrderDelay
        modelTypes(end+1, 1) = "first_order_delay";
    end

    if fitSecondOrderLag
        modelTypes(end+1, 1) = "second_order_lag";
    end

    if fitSecondOrderLagDelay
        modelTypes(end+1, 1) = "second_order_lag_delay";
    end

    candidateModels = struct([]);

    for i = 1:numel(modelTypes)
        modelType = modelTypes(i);

        model = fitOneFrequencyModel( ...
            modelType, ...
            f_Hz, ...
            G_emp, ...
            coherence, ...
            weightingMode, ...
            gainFloorMode, ...
            gainFloorFractionOfMedian, ...
            manualGainFloor, ...
            initialK, ...
            initialTau, ...
            initialDelay, ...
            initialTau2, ...
            minTau, ...
            maxTau, ...
            minDelay, ...
            maxDelay, ...
            minGainAbs, ...
            maxGainAbs, ...
            maxIter, ...
            maxEval);

        if isempty(candidateModels)
            candidateModels = model;
        else
            candidateModels(end+1) = model;
        end
    end
end

function model = fitOneFrequencyModel( ...
    modelType, ...
    f_Hz, ...
    G_emp, ...
    coherence, ...
    weightingMode, ...
    gainFloorMode, ...
    gainFloorFractionOfMedian, ...
    manualGainFloor, ...
    initialK, ...
    initialTau, ...
    initialDelay, ...
    initialTau2, ...
    minTau, ...
    maxTau, ...
    minDelay, ...
    maxDelay, ...
    minGainAbs, ...
    maxGainAbs, ...
    maxIter, ...
    maxEval)

    w = 2*pi*f_Hz(:);
    G_emp = G_emp(:);
    coherence = coherence(:);

    if strcmpi(gainFloorMode, "fraction_of_median")
        gainFloor = gainFloorFractionOfMedian * median(abs(G_emp), "omitnan");
    elseif strcmpi(gainFloorMode, "manual")
        gainFloor = manualGainFloor;
    else
        error("Unknown GAIN_FLOOR_MODE: %s", gainFloorMode);
    end

    if ~isfinite(gainFloor) || gainFloor <= 0
        gainFloor = manualGainFloor;
    end

    initialK = clampScalar(initialK, minGainAbs, maxGainAbs);
    initialTau = clampScalar(initialTau, minTau, maxTau);
    initialTau2 = clampScalar(initialTau2, minTau, maxTau);
    initialDelay = clampScalar(initialDelay, minDelay, maxDelay);

    p0 = packParams(modelType, initialK, initialTau, initialTau2, initialDelay);

    costFun = @(p) frequencyFitCost( ...
        p, modelType, w, G_emp, coherence, weightingMode, gainFloor, ...
        minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    opts = optimset( ...
        "Display", "off", ...
        "MaxIter", maxIter, ...
        "MaxFunEvals", maxEval, ...
        "TolX", 1e-10, ...
        "TolFun", 1e-12);

    [pBest, bestCost] = fminsearch(costFun, p0, opts);

    params = unpackParams(modelType, pBest, minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    G_fit = evalFrequencyModel(modelType, params, w);

    err = weightedComplexError(G_fit, G_emp, coherence, weightingMode, gainFloor);
    weightedError = sqrt(mean(abs(err).^2, "omitnan"));

    model.model_type = string(modelType);
    model.params = params;
    model.K = params.K;
    model.tau1_s = params.tau1_s;
    model.tau2_s = params.tau2_s;
    model.delay_s = params.delay_s;
    model.complexity = getModelComplexity(modelType);
    model.best_cost = bestCost;
    model.fit_weighted_error = weightedError;
    model.gain_floor = gainFloor;
    model.weighting_mode = string(weightingMode);

    model.train_weighted_error = weightedError;

    [magRmseDb, phaseRmseDeg] = bodeErrorMetrics(G_fit, G_emp);
    model.train_mag_rmse_dB = magRmseDb;
    model.train_phase_rmse_deg = phaseRmseDeg;
end

function J = frequencyFitCost( ...
    p, modelType, w, G_emp, coherence, weightingMode, gainFloor, ...
    minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs)

    params = unpackParams(modelType, p, minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    G_fit = evalFrequencyModel(modelType, params, w);

    err = weightedComplexError(G_fit, G_emp, coherence, weightingMode, gainFloor);

    J = mean(abs(err).^2, "omitnan");

    if ~isfinite(J)
        J = 1e30;
    end
end

function err = weightedComplexError(G_fit, G_emp, coherence, weightingMode, gainFloor)
    switch string(weightingMode)
        case "relative_complex"
            denom = max(abs(G_emp), gainFloor);
            err = (G_fit - G_emp) ./ denom;

        case "relative_with_coh"
            denom = max(abs(G_emp), gainFloor);
            cohWeight = sqrt(max(coherence, 0));
            err = cohWeight .* (G_fit - G_emp) ./ denom;

        case "absolute_complex"
            err = G_fit - G_emp;

        otherwise
            error("Unknown WEIGHTING_MODE: %s", weightingMode);
    end
end

function params = unpackParams(modelType, p, minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs)
    switch string(modelType)
        case "first_order"
            K = expClamp(p(1), minGainAbs, maxGainAbs);
            tau1 = expClamp(p(2), minTau, maxTau);

            params.K = K;
            params.tau1_s = tau1;
            params.tau2_s = NaN;
            params.delay_s = 0;

        case "first_order_delay"
            K = expClamp(p(1), minGainAbs, maxGainAbs);
            tau1 = expClamp(p(2), minTau, maxTau);
            delay = expClamp(p(3), max(minDelay, 1e-6), maxDelay);

            params.K = K;
            params.tau1_s = tau1;
            params.tau2_s = NaN;
            params.delay_s = delay;

        case "second_order_lag"
            K = expClamp(p(1), minGainAbs, maxGainAbs);
            tau1 = expClamp(p(2), minTau, maxTau);
            tau2 = expClamp(p(3), minTau, maxTau);

            params.K = K;
            params.tau1_s = max(tau1, tau2);
            params.tau2_s = min(tau1, tau2);
            params.delay_s = 0;

        case "second_order_lag_delay"
            K = expClamp(p(1), minGainAbs, maxGainAbs);
            tau1 = expClamp(p(2), minTau, maxTau);
            tau2 = expClamp(p(3), minTau, maxTau);
            delay = expClamp(p(4), max(minDelay, 1e-6), maxDelay);

            params.K = K;
            params.tau1_s = max(tau1, tau2);
            params.tau2_s = min(tau1, tau2);
            params.delay_s = delay;

        otherwise
            error("Unknown modelType: %s", modelType);
    end
end

function p = packParams(modelType, K, tau1, tau2, delay)
    K = max(K, 1e-12);

    switch string(modelType)
        case "first_order"
            p = [log(K), log(tau1)];

        case "first_order_delay"
            p = [log(K), log(tau1), log(max(delay, 1e-6))];

        case "second_order_lag"
            p = [log(K), log(tau1), log(tau2)];

        case "second_order_lag_delay"
            p = [log(K), log(tau1), log(tau2), log(max(delay, 1e-6))];

        otherwise
            error("Unknown modelType: %s", modelType);
    end
end

function x = signedLogParam(K)
    if K >= 0
        x = log(max(abs(K), 1e-12));
    else
        x = -log(max(abs(K), 1e-12));
    end
end

function K = signedExpClamp(p, minAbs, maxAbs)
    if p >= 0
        K = exp(p);
    else
        K = -exp(-p);
    end

    if abs(K) < minAbs
        K = signNonzero(K) * minAbs;
    end

    if abs(K) > maxAbs
        K = signNonzero(K) * maxAbs;
    end
end

function x = expClamp(p, lo, hi)
    x = exp(p);
    x = clampScalar(x, lo, hi);
end

function s = signNonzero(x)
    if x >= 0
        s = 1;
    else
        s = -1;
    end
end

function y = clampScalar(x, lo, hi)
    y = min(max(x, lo), hi);
end

function G = evalFrequencyModel(modelType, params, w)
    s = 1j*w;

    switch string(modelType)
        case "first_order"
            G = params.K ./ (params.tau1_s*s + 1);

        case "first_order_delay"
            G = params.K .* exp(-s*params.delay_s) ./ (params.tau1_s*s + 1);

        case "second_order_lag"
            G = params.K ./ ((params.tau1_s*s + 1) .* (params.tau2_s*s + 1));

        case "second_order_lag_delay"
            G = params.K .* exp(-s*params.delay_s) ./ ...
                ((params.tau1_s*s + 1) .* (params.tau2_s*s + 1));

        otherwise
            error("Unknown modelType: %s", modelType);
    end
end

function sys = makeTransferFunction(model)
    s = tf("s");

    switch string(model.model_type)
        case "first_order"
            sys = model.K / (model.tau1_s*s + 1);

        case "first_order_delay"
            sys = model.K / (model.tau1_s*s + 1);
            sys.InputDelay = model.delay_s;

        case "second_order_lag"
            sys = model.K / ((model.tau1_s*s + 1)*(model.tau2_s*s + 1));

        case "second_order_lag_delay"
            sys = model.K / ((model.tau1_s*s + 1)*(model.tau2_s*s + 1));
            sys.InputDelay = model.delay_s;

        otherwise
            error("Unknown model type: %s", model.model_type);
    end
end

function complexity = getModelComplexity(modelType)
    switch string(modelType)
        case "first_order"
            complexity = 2;
        case "first_order_delay"
            complexity = 3;
        case "second_order_lag"
            complexity = 3;
        case "second_order_lag_delay"
            complexity = 4;
        otherwise
            complexity = 999;
    end
end

function models = scoreModelsOnFrf(models, frf, label)
    for i = 1:numel(models)
        model = models(i);
        G_fit = evalFrequencyModel(model.model_type, model.params, frf.w_rad_s);

        err = weightedComplexError( ...
            G_fit, ...
            frf.G_emp, ...
            frf.coherence, ...
            model.weighting_mode, ...
            model.gain_floor);

        weightedError = sqrt(mean(abs(err).^2, "omitnan"));

        [magRmseDb, phaseRmseDeg] = bodeErrorMetrics(G_fit, frf.G_emp);

        switch string(label)
            case "training"
                models(i).train_weighted_error = weightedError;
                models(i).train_mag_rmse_dB = magRmseDb;
                models(i).train_phase_rmse_deg = phaseRmseDeg;

            case "validation"
                models(i).validation_weighted_error = weightedError;
                models(i).validation_mag_rmse_dB = magRmseDb;
                models(i).validation_phase_rmse_deg = phaseRmseDeg;
        end
    end
end

function [magRmseDb, phaseRmseDeg] = bodeErrorMetrics(G_fit, G_emp)
    magFitDb = 20*log10(abs(G_fit));
    magEmpDb = 20*log10(abs(G_emp));

    phaseFit = unwrap(angle(G_fit));
    phaseEmp = unwrap(angle(G_emp));

    phaseErrDeg = (phaseFit - phaseEmp) * 180/pi;
    magErrDb = magFitDb - magEmpDb;

    magRmseDb = sqrt(mean(magErrDb.^2, "omitnan"));
    phaseRmseDeg = sqrt(mean(phaseErrDeg.^2, "omitnan"));
end

function bestModel = selectBestFrequencyModel(models, selectionMode, relativeTolerance)
    trainErrors = [models.train_weighted_error].';
    complexities = [models.complexity].';

    hasValidation = arrayfun(@(m) isfield(m, "validation_weighted_error") && ...
        isfinite(m.validation_weighted_error), models);

    switch string(selectionMode)
        case "lowest_validation_error"
            if any(hasValidation)
                valErrors = NaN(numel(models), 1);
                for i = 1:numel(models)
                    if isfield(models(i), "validation_weighted_error")
                        valErrors(i) = models(i).validation_weighted_error;
                    end
                end
                [~, idx] = min(valErrors);
                bestModel = models(idx);
            else
                [~, idx] = min(trainErrors);
                bestModel = models(idx);
            end

        case "lowest_training_error"
            [~, idx] = min(trainErrors);
            bestModel = models(idx);

        case "simplicity_tolerance"
            bestTrain = min(trainErrors);
            eligible = trainErrors <= bestTrain * (1 + relativeTolerance);

            eligibleIdx = find(eligible);
            eligibleComplexities = complexities(eligibleIdx);
            minComplexity = min(eligibleComplexities);

            simplestIdx = eligibleIdx(eligibleComplexities == minComplexity);

            if numel(simplestIdx) > 1
                [~, local] = min(trainErrors(simplestIdx));
                idx = simplestIdx(local);
            else
                idx = simplestIdx;
            end

            bestModel = models(idx);

        otherwise
            error("Unknown MODEL_SELECTION_MODE: %s", selectionMode);
    end
end

function printFrequencyModel(model)
    fprintf("  Model type: %s\n", model.model_type);
    fprintf("  K = %.9g N/us\n", model.K);
    fprintf("  tau1 = %.6f s\n", model.tau1_s);

    if isfinite(model.tau2_s)
        fprintf("  tau2 = %.6f s\n", model.tau2_s);
    end

    fprintf("  delay = %.6f s\n", model.delay_s);
    fprintf("  training weighted error = %.6f\n", model.train_weighted_error);
    fprintf("  training magnitude RMSE = %.3f dB\n", model.train_mag_rmse_dB);
    fprintf("  training phase RMSE = %.3f deg\n", model.train_phase_rmse_deg);

    if isfield(model, "validation_weighted_error")
        fprintf("  validation weighted error = %.6f\n", model.validation_weighted_error);
        fprintf("  validation magnitude RMSE = %.3f dB\n", model.validation_mag_rmse_dB);
        fprintf("  validation phase RMSE = %.3f deg\n", model.validation_phase_rmse_deg);
    end

    if model.tau1_s > 0
        fprintf("  dominant bandwidth approx = %.6f rad/s = %.6f Hz\n", ...
            1/model.tau1_s, 1/(2*pi*model.tau1_s));
    end
end

function [tUniform, uUniform, yUniform, dtUsed] = makeLsimUniformGridFromRawTimestamps( ...
    tRaw, ...
    uRaw, ...
    yRaw, ...
    useRawTimestampGridForLsim)

    tUniform = [];
    uUniform = [];
    yUniform = [];
    dtUsed = NaN;

    valid = isfinite(tRaw) & isfinite(uRaw) & isfinite(yRaw);
    tRaw = tRaw(valid);
    uRaw = uRaw(valid);
    yRaw = yRaw(valid);

    if numel(tRaw) < 5
        return;
    end

    % Sort by time.
    [tRaw, sortIdx] = sort(tRaw(:));
    uRaw = uRaw(sortIdx);
    yRaw = yRaw(sortIdx);

    % Remove duplicate timestamps by averaging samples at same timestamp.
    [tUnique, ~, groupIdx] = unique(tRaw, "stable");

    uUnique = accumarray(groupIdx, uRaw, [], @(x) mean(x, "omitnan"));
    yUnique = accumarray(groupIdx, yRaw, [], @(x) mean(x, "omitnan"));

    tRaw = tUnique;
    uRaw = uUnique;
    yRaw = yUnique;

    % Re-zero time after duplicate removal.
    tRaw = tRaw - tRaw(1);

    dtRaw = diff(tRaw);
    dtRaw = dtRaw(isfinite(dtRaw) & dtRaw > 0);

    if numel(dtRaw) < 3
        return;
    end

    dtMedian = median(dtRaw, "omitnan");

    if ~isfinite(dtMedian) || dtMedian <= 0
        return;
    end

    dtUsed = dtMedian;

    if useRawTimestampGridForLsim
        % Use a uniform grid with the same number of samples as the raw record.
        % This avoids accidentally adding or dropping samples due to colon
        % endpoint behavior.
        %
        % It still must be uniform for lsim, but it stays aligned with the
        % raw record start/end better than blindly using 0:dt:tEnd.
        n = numel(tRaw);
        tUniform = linspace(0, tRaw(end), n).';

        % Recompute effective dt used by this grid.
        if numel(tUniform) >= 2
            dtUsed = median(diff(tUniform), "omitnan");
        end
    else
        % Old style: use median dt and fill the interval.
        tUniform = (0:dtMedian:tRaw(end)).';
    end

    if numel(tUniform) < 5
        tUniform = [];
        return;
    end

    % For PWM command, use previous/zero-order hold interpolation.
    % This is more physically correct than linear interpolation because the
    % PWM command is held between updates.
    uUniform = interp1(tRaw, uRaw, tUniform, "previous", "extrap");

    % For measured thrust, linear interpolation is acceptable for plotting.
    yUniform = interp1(tRaw, yRaw, tUniform, "linear", "extrap");
end

function printSamplingRateDiagnostic(D, runName, datasetLabel)
    t = forceNumeric(D.t_ms) / 1000;
    t = t(isfinite(t));

    if numel(t) < 3
        warning("Not enough samples for sampling-rate diagnostic.");
        return;
    end

    t = t - t(1);
    dt = diff(t);
    dt = dt(isfinite(dt) & dt > 0);

    Ts_median = median(dt, "omitnan");
    Ts_mean = mean(dt, "omitnan");
    Ts_std = std(dt, "omitnan");

    fs_median = 1 / Ts_median;
    fs_mean = 1 / Ts_mean;

    fprintf("\nSampling diagnostic: %s / %s\n", datasetLabel, runName);
    fprintf("  Median sample period Ts = %.6f s\n", Ts_median);
    fprintf("  Mean sample period Ts   = %.6f s\n", Ts_mean);
    fprintf("  Std of sample period    = %.6f s\n", Ts_std);
    fprintf("  Median sample rate fs   = %.3f Hz\n", fs_median);
    fprintf("  Mean sample rate fs     = %.3f Hz\n", fs_mean);
    fprintf("  Nyquist from median fs  = %.3f Hz\n", fs_median / 2);
end

function bwHz = bandwidthHzFromTau(tau_s)
    if isfinite(tau_s) && tau_s > 0
        bwHz = 1 / (2*pi*tau_s);
    else
        bwHz = NaN;
    end
end

function compareParametersAcrossRegions(summaryTable, runNames)
    modelTypes = unique(string(summaryTable.model_type), "stable");

    for mt = 1:numel(modelTypes)
        modelType = modelTypes(mt);

        rows = summaryTable(string(summaryTable.model_type) == modelType, :);

        if isempty(rows)
            continue;
        end

        fprintf("\n------------------------------------------------------------\n");
        fprintf("Model type: %s\n", modelType);
        fprintf("------------------------------------------------------------\n");

        rows = sortRowsByRunOrder(rows, runNames);

        comparisonTable = rows(:, { ...
            'run_name', ...
            'K_N_per_us', ...
            'tau1_s', ...
            'tau2_s', ...
            'delay_s', ...
            'dominant_bandwidth_Hz', ...
            'train_weighted_error', ...
            'validation_weighted_error', ...
            'train_mag_rmse_dB', ...
            'train_phase_rmse_deg', ...
            'validation_mag_rmse_dB', ...
            'validation_phase_rmse_deg', ...
            'is_best_model' ...
        });

        disp(comparisonTable);

        printParameterSpreadDiagnostics(rows, modelType);
    end
end

function compareBestModelsAcrossRegions(bestModelTable)
    rows = bestModelTable;

    comparisonTable = rows(:, { ...
        'run_name', ...
        'best_model_type', ...
        'K_N_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'delay_s', ...
        'dominant_bandwidth_Hz', ...
        'train_weighted_error', ...
        'validation_weighted_error', ...
        'train_mag_rmse_dB', ...
        'train_phase_rmse_deg', ...
        'validation_mag_rmse_dB', ...
        'validation_phase_rmse_deg' ...
    });

    disp(comparisonTable);

    fprintf("\nBest-model spread diagnostics:\n");
    printParameterSpreadDiagnosticsForBestModels(rows);
end

function rowsOut = sortRowsByRunOrder(rowsIn, runNames)
    runOrder = NaN(height(rowsIn), 1);

    for i = 1:height(rowsIn)
        idx = find(string(runNames) == string(rowsIn.run_name(i)), 1, "first");

        if isempty(idx)
            runOrder(i) = 9999 + i;
        else
            runOrder(i) = idx;
        end
    end

    rowsIn.run_order_for_sorting = runOrder;
    rowsIn = sortrows(rowsIn, "run_order_for_sorting");
    rowsIn.run_order_for_sorting = [];

    rowsOut = rowsIn;
end

function printParameterSpreadDiagnostics(rows, modelType)
    K = rows.K_N_per_us;
    tau1 = rows.tau1_s;
    tau2 = rows.tau2_s;
    delay = rows.delay_s;

    fprintf("Parameter spread for %s:\n", modelType);
    printOneSpread("K_N_per_us", K, "N/us");
    printOneSpread("tau1_s", tau1, "s");
    printOneSpread("tau2_s", tau2, "s");
    printOneSpread("delay_s", delay, "s");

    globalIdx = find(contains(string(rows.run_name), "global"), 1, "first");

    if ~isempty(globalIdx)
        fprintf("Ratios relative to global model:\n");

        for i = 1:height(rows)
            fprintf("  %s:\n", string(rows.run_name(i)));
            printRatioToReference("K", rows.K_N_per_us(i), rows.K_N_per_us(globalIdx));
            printRatioToReference("tau1", rows.tau1_s(i), rows.tau1_s(globalIdx));
            printRatioToReference("tau2", rows.tau2_s(i), rows.tau2_s(globalIdx));
            printRatioToReference("delay", rows.delay_s(i), rows.delay_s(globalIdx));
        end
    end
end

function printParameterSpreadDiagnosticsForBestModels(rows)
    fprintf("These spreads mix model structures if different best models were selected.\n");
    fprintf("Use the per-model-type tables above for cleaner apples-to-apples comparison.\n");

    printOneSpread("K_N_per_us", rows.K_N_per_us, "N/us");
    printOneSpread("tau1_s", rows.tau1_s, "s");
    printOneSpread("tau2_s", rows.tau2_s, "s");
    printOneSpread("delay_s", rows.delay_s, "s");
end

function printOneSpread(name, x, units)
    x = x(isfinite(x));

    if isempty(x)
        fprintf("  %s: no finite values\n", name);
        return;
    end

    xMin = min(x);
    xMax = max(x);
    xMean = mean(x, "omitnan");

    if abs(xMean) > eps
        pctSpread = 100 * (xMax - xMin) / abs(xMean);
    else
        pctSpread = NaN;
    end

    fprintf("  %s: min = %.6g %s, max = %.6g %s, mean = %.6g %s, spread = %.2f %% of mean\n", ...
        name, xMin, units, xMax, units, xMean, units, pctSpread);
end

function printRatioToReference(name, value, reference)
    if isfinite(value) && isfinite(reference) && abs(reference) > eps
        fprintf("    %s ratio = %.4f\n", name, value / reference);
    else
        fprintf("    %s ratio = NaN\n", name);
    end
end

%% Plotting functions
% ============================================================

function plotEmpiricalBodeWithFits(frf, models, bestModel, runName, datasetLabel, minCoherence, plotRejected)
    f = frf.f_Hz(:);
    G = frf.G_emp(:);
    coh = frf.coherence(:);

    good = coh >= minCoherence;

    figure;
    hold on; grid on;

    if plotRejected && any(~good)
        semilogx(f(~good), 20*log10(abs(G(~good))), "x", ...
            "DisplayName", "Rejected empirical points");
    end

    semilogx(f(good), 20*log10(abs(G(good))), "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Empirical PRPS FRF");

    fFine = logspace(log10(min(f)), log10(max(f)), 500).';
    wFine = 2*pi*fFine;

    for i = 1:numel(models)
        model = models(i);
        Gfit = evalFrequencyModel(model.model_type, model.params, wFine);

        if string(model.model_type) == string(bestModel.model_type)
            lw = 2.2;
            name = "BEST fit: " + model.model_type;
        else
            lw = 1.0;
            name = "Candidate: " + model.model_type;
        end

        semilogx(fFine, 20*log10(abs(Gfit)), "LineWidth", lw, ...
            "DisplayName", name);
    end

    xlabel("Frequency (Hz)");
    ylabel("Magnitude (dB, N/us)");
    title("Training Empirical Bode Magnitude with Frequency-Domain Fits - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");

    figure;
    hold on; grid on;

    if plotRejected && any(~good)
        semilogx(f(~good), unwrap(angle(G(~good))) * 180/pi, "x", ...
            "DisplayName", "Rejected empirical points");
    end

    semilogx(f(good), unwrap(angle(G(good))) * 180/pi, "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Empirical PRPS FRF");

    for i = 1:numel(models)
        model = models(i);
        Gfit = evalFrequencyModel(model.model_type, model.params, wFine);

        if string(model.model_type) == string(bestModel.model_type)
            lw = 2.2;
            name = "BEST fit: " + model.model_type;
        else
            lw = 1.0;
            name = "Candidate: " + model.model_type;
        end

        semilogx(fFine, unwrap(angle(Gfit)) * 180/pi, "LineWidth", lw, ...
            "DisplayName", name);
    end

    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    title("Training Empirical Bode Phase with Frequency-Domain Fits - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

function plotValidationBodeVsTrainingModel(valFrf, bestModel, runName, minCoherence)
    f = valFrf.f_Hz(:);
    G = valFrf.G_emp(:);
    coh = valFrf.coherence(:);

    good = coh >= minCoherence;

    fFine = logspace(log10(min(f)), log10(max(f)), 500).';
    wFine = 2*pi*fFine;
    Gfit = evalFrequencyModel(bestModel.model_type, bestModel.params, wFine);

    figure;
    hold on; grid on;

    semilogx(f(good), 20*log10(abs(G(good))), "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical FRF");

    if any(~good)
        semilogx(f(~good), 20*log10(abs(G(~good))), "x", ...
            "DisplayName", "Validation low-coherence points");
    end

    semilogx(fFine, 20*log10(abs(Gfit)), "-", ...
        "LineWidth", 2.0, ...
        "DisplayName", "Training fitted model");

    xlabel("Frequency (Hz)");
    ylabel("Magnitude (dB, N/us)");
    title("Validation Bode vs Training-Fitted Model - Magnitude - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");

    figure;
    hold on; grid on;

    semilogx(f(good), unwrap(angle(G(good))) * 180/pi, "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical FRF");

    if any(~good)
        semilogx(f(~good), unwrap(angle(G(~good))) * 180/pi, "x", ...
            "DisplayName", "Validation low-coherence points");
    end

    semilogx(fFine, unwrap(angle(Gfit)) * 180/pi, "-", ...
        "LineWidth", 2.0, ...
        "DisplayName", "Training fitted model");

    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    title("Validation Bode vs Training-Fitted Model - Phase - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

function plotTrainingVsValidationEmpiricalBode(trainFrf, valFrf, runName, minCoherence)
    trainGood = trainFrf.coherence >= minCoherence;
    valGood = valFrf.coherence >= minCoherence;

    figure;
    hold on; grid on;

    semilogx(trainFrf.f_Hz(trainGood), 20*log10(abs(trainFrf.G_emp(trainGood))), "o-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Training empirical");

    semilogx(valFrf.f_Hz(valGood), 20*log10(abs(valFrf.G_emp(valGood))), "s--", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical");

    xlabel("Frequency (Hz)");
    ylabel("Magnitude (dB, N/us)");
    title("Training vs Validation Empirical Bode - Magnitude - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");

    figure;
    hold on; grid on;

    semilogx(trainFrf.f_Hz(trainGood), unwrap(angle(trainFrf.G_emp(trainGood))) * 180/pi, "o-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Training empirical");

    semilogx(valFrf.f_Hz(valGood), unwrap(angle(valFrf.G_emp(valGood))) * 180/pi, "s--", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical");

    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    title("Training vs Validation Empirical Bode - Phase - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

function plotCoherence(trainFrf, valFrf, runName)
    figure;
    hold on; grid on;

    if ~isempty(trainFrf.f_Hz)
        semilogx(trainFrf.f_Hz, trainFrf.coherence, "o-", ...
            "LineWidth", 1.2, ...
            "DisplayName", "Training coherence");
    end

    if ~isempty(valFrf.f_Hz)
        semilogx(valFrf.f_Hz, valFrf.coherence, "s--", ...
            "LineWidth", 1.2, ...
            "DisplayName", "Validation coherence");
    end

    yline(0.5, "--", "0.5");
    yline(0.8, "--", "0.8");

    xlabel("Frequency (Hz)");
    ylabel("Coherence");
    title("PRPS Input/Output Coherence - " + runName, "Interpreter", "none");
    ylim([0 1.05]);
    legend("Location", "best");
end

function plotModelComparison(models, runName)
    names = strings(numel(models), 1);
    trainErr = zeros(numel(models), 1);
    valErr = NaN(numel(models), 1);

    for i = 1:numel(models)
        names(i) = models(i).model_type;
        trainErr(i) = models(i).train_weighted_error;

        if isfield(models(i), "validation_weighted_error")
            valErr(i) = models(i).validation_weighted_error;
        end
    end

    figure;
    hold on; grid on;

    X = categorical(names);
    X = reordercats(X, cellstr(names));

    if any(isfinite(valErr))
        bar(X, [trainErr, valErr]);
        legend("Training", "Validation", "Location", "best");
    else
        bar(X, trainErr);
    end

    ylabel("Weighted complex FRF error");
    title("Frequency-Domain Model Comparison - " + runName, "Interpreter", "none");
end

function plotTimeDomainTranslationForDataset( ...
    D, ...
    bestModel, ...
    candidateModels, ...
    runName, ...
    datasetLabel, ...
    centerSignals, ...
    plotBestOnly, ...
    plotAllModels, ...
    useRawTimestampGridForLsim)

    if isempty(D) || height(D) < 5
        return;
    end

    groups = getTimeGroups(D);
    uniqueGroups = unique(groups, "stable");

    for g = 1:numel(uniqueGroups)
        idx = find(groups == uniqueGroups(g));
        Dg = D(idx, :);

        tRaw = forceNumeric(Dg.t_ms) / 1000;
        uRaw = forceNumeric(Dg.pwm_us);
        yRaw = forceNumeric(Dg.force_N);

        valid = isfinite(tRaw) & isfinite(uRaw) & isfinite(yRaw);
        tRaw = tRaw(valid);
        uRaw = uRaw(valid);
        yRaw = yRaw(valid);

        if numel(tRaw) < 5
            continue;
        end

        tRaw = tRaw - tRaw(1);

        % Remove duplicate timestamps.
        [tRaw, uniqueIdx] = unique(tRaw, "stable");
        uRaw = uRaw(uniqueIdx);
        yRaw = yRaw(uniqueIdx);

        [tUniform, uUniform, yUniform, dtUsed] = makeLsimUniformGridFromRawTimestamps( ...
            tRaw, ...
            uRaw, ...
            yRaw, ...
            useRawTimestampGridForLsim);

        if numel(tUniform) < 5
            continue;
        end

        if centerSignals
            uInput = uUniform - mean(uUniform, "omitnan");
            yOffset = mean(yUniform, "omitnan");
        else
            uInput = uUniform;
            yOffset = 0;
        end

        figure;
        hold on; grid on;

        % Plot measured thrust once.
        plot(tUniform, yUniform, "LineWidth", 1.2, ...
            "DisplayName", "Measured thrust");

        if plotBestOnly && ~plotAllModels
            sys = makeTransferFunction(bestModel);

            try
                yModel = lsim(sys, uInput, tUniform) + yOffset;

                plot(tUniform, yModel, "--", "LineWidth", 1.8, ...
                    "DisplayName", "Best frequency-fit model: " + bestModel.model_type);

            catch ME
                warning("lsim failed for best model %s / %s: %s", ...
                    datasetLabel, runName, ME.message);
            end
        end

        if plotAllModels
            for m = 1:numel(candidateModels)
                sys = makeTransferFunction(candidateModels(m));

                try
                    yModel = lsim(sys, uInput, tUniform) + yOffset;

                    if string(candidateModels(m).model_type) == string(bestModel.model_type)
                        lineWidth = 1.8;
                        modelLabel = "BEST: " + candidateModels(m).model_type;
                    else
                        lineWidth = 1.0;
                        modelLabel = "Candidate: " + candidateModels(m).model_type;
                    end

                    plot(tUniform, yModel, "--", "LineWidth", lineWidth, ...
                        "DisplayName", modelLabel);

                catch ME
                    warning("lsim failed for candidate model %s / %s / %s: %s", ...
                        candidateModels(m).model_type, datasetLabel, runName, ME.message);
                end
            end
        end

        xlabel("Time (s)");
        ylabel("Thrust (N)");
        title("Time-Domain Translation of Frequency Fit - " + datasetLabel + ...
            " - " + runName + " - group " + string(uniqueGroups(g)), ...
            "Interpreter", "none");

        legend("Location", "best", "Interpreter", "none");

        % Usually one group plot is enough to avoid figure explosion.
        break;
    end
end


function groups = getTimeGroups(D)
    if ismember("source_file_index", string(D.Properties.VariableNames))
        groups = double(D.source_file_index);
    elseif ismember("set_index", string(D.Properties.VariableNames))
        groups = double(D.set_index);
    else
        groups = ones(height(D), 1);
    end
end

function plotRawTimeSnippet(D, runName, datasetLabel)
    t = forceNumeric(D.t_ms) / 1000;
    u = forceNumeric(D.pwm_us);
    y = forceNumeric(D.force_N);

    figure;
    hold on; grid on;

    yyaxis left;
    plot(t, y, "LineWidth", 1.1);
    ylabel("Thrust (N)");

    yyaxis right;
    plot(t, u, "LineWidth", 1.1);
    ylabel("PWM (\mus)");

    xlabel("Time (s)");
    title("Raw PRPS Time Snippet - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");
end

function plotInputSpectrumDiagnostic(frf, runName, datasetLabel)
    figure;
    hold on; grid on;

    scatter(frf.raw_f_Hz, abs(frf.raw_U), 20, "filled");

    xlabel("Frequency (Hz)");
    ylabel("|U|");
    title("Input Spectrum Diagnostic - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");
end

function plotSampleTimeDiagnostic(D, runName, datasetLabel)
    t = forceNumeric(D.t_ms) / 1000;

    figure;
    hold on; grid on;

    plot(t(2:end), diff(t), "LineWidth", 1.1);

    xlabel("Time (s)");
    ylabel("Sample interval \Deltat (s)");
    title("Sample-Time Diagnostic - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");
end

function plotVoltageTime(D, runName, datasetLabel)
    if ~hasVoltage(D)
        return;
    end

    t = forceNumeric(D.t_ms) / 1000;
    V = forceNumeric(D.battery_voltage_V);

    figure;
    hold on; grid on;

    plot(t, V, "LineWidth", 1.1);

    xlabel("Time (s)");
    ylabel("Battery voltage (V)");
    title("Battery Voltage During PRPS - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");
end

function plotPhaseResidualByFrequency(frf, model, runName, datasetLabel, minCoherence)
    if isempty(frf.f_Hz)
        return;
    end

    f = frf.f_Hz(:);
    w = frf.w_rad_s(:);
    G_emp = frf.G_emp(:);
    coh = frf.coherence(:);

    valid = isfinite(f) & ...
            isfinite(w) & ...
            isfinite(G_emp) & ...
            isfinite(coh) & ...
            coh >= minCoherence;

    if nnz(valid) < 2
        warning("Skipping phase residual plot for %s %s: fewer than 2 valid points.", ...
            datasetLabel, runName);
        return;
    end

    fValid = f(valid);
    wValid = w(valid);
    GempValid = G_emp(valid);
    cohValid = coh(valid);

    GfitValid = evalFrequencyModel(model.model_type, model.params, wValid);

    phaseEmp_rad = unwrap(angle(GempValid));
    phaseFit_rad = unwrap(angle(GfitValid));

    phaseResidual_deg = (phaseFit_rad - phaseEmp_rad) * 180/pi;

    % Estimate equivalent missing/additional delay from phase residual.
    %
    % If:
    %   phaseResidual_rad = phase_model - phase_empirical
    %
    % and the model is too early, then phase_model is usually less negative
    % than phase_empirical, giving positive residual.
    %
    % Extra delay needed approximately satisfies:
    %   phase_empirical ≈ phase_model - w*dL
    %
    % Therefore:
    %   dL ≈ phaseResidual_rad / w
    %
    % A positive dL means "add this much delay to the model."
    phaseResidual_rad = phaseFit_rad - phaseEmp_rad;
    equivalentExtraDelay_s = phaseResidual_rad ./ wValid;

    meanExtraDelay_s = mean(equivalentExtraDelay_s, "omitnan");
    medianExtraDelay_s = median(equivalentExtraDelay_s, "omitnan");

    figure;
    hold on; grid on;

    semilogx(fValid, phaseResidual_deg, "o-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Phase residual");

    yline(0, "--", "Zero residual");

    xlabel("Frequency (Hz)");
    ylabel("Phase residual: model - empirical (deg)");
    title("Phase Residual vs Frequency - " + datasetLabel + ...
        " - " + runName + " - " + model.model_type, ...
        "Interpreter", "none");

    subtitle("Positive residual often means model is too early / needs more delay. " + ...
        "Median extra delay ≈ " + sprintf("%.4f", medianExtraDelay_s) + " s", ...
        "Interpreter", "none");

    legend("Location", "best", "Interpreter", "none");

    figure;
    hold on; grid on;

    semilogx(fValid, equivalentExtraDelay_s * 1000, "o-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Equivalent extra delay");

    yline(0, "--", "Zero extra delay");
    yline(meanExtraDelay_s * 1000, ":", ...
        "Mean = " + sprintf("%.1f", meanExtraDelay_s * 1000) + " ms");
    yline(medianExtraDelay_s * 1000, ":", ...
        "Median = " + sprintf("%.1f", medianExtraDelay_s * 1000) + " ms");

    xlabel("Frequency (Hz)");
    ylabel("Equivalent extra delay from phase residual (ms)");
    title("Equivalent Delay Implied by Phase Residual - " + datasetLabel + ...
        " - " + runName + " - " + model.model_type, ...
        "Interpreter", "none");

    legend("Location", "best", "Interpreter", "none");

    fprintf("\nPhase residual diagnostic for %s / %s / %s:\n", ...
        datasetLabel, runName, model.model_type);
    fprintf("  Mean equivalent extra delay   = %.6f s = %.2f ms\n", ...
        meanExtraDelay_s, meanExtraDelay_s * 1000);
    fprintf("  Median equivalent extra delay = %.6f s = %.2f ms\n", ...
        medianExtraDelay_s, medianExtraDelay_s * 1000);
end

function plotNormalizedBodeBestFitAllData(trainFrf, valFrf, bestModel, runName, minCoherence, plotRejected)
    if isempty(trainFrf.f_Hz)
        return;
    end

    K_norm = abs(bestModel.K);

    if ~isfinite(K_norm) || K_norm <= 0
        warning("Skipping normalized Bode for %s: invalid K.", runName);
        return;
    end

    fTrain  = trainFrf.f_Hz(:);
    GTrain  = trainFrf.G_emp(:);
    cohTrain = trainFrf.coherence(:);
    goodTrain = isfinite(fTrain) & isfinite(GTrain) & isfinite(cohTrain) & cohTrain >= minCoherence;

    hasVal = ~isempty(valFrf.f_Hz);
    if hasVal
        fVal   = valFrf.f_Hz(:);
        GVal   = valFrf.G_emp(:);
        cohVal = valFrf.coherence(:);
        goodVal = isfinite(fVal) & isfinite(GVal) & isfinite(cohVal) & cohVal >= minCoherence;
    end

    fGoodAll = fTrain(goodTrain);
    if hasVal
        fGoodAll = [fGoodAll; fVal(goodVal)];
    end

    if isempty(fGoodAll)
        return;
    end

    fFine = logspace(log10(min(fGoodAll)), log10(max(fGoodAll)), 500).';
    wFine = 2*pi*fFine;
    Gfit  = evalFrequencyModel(bestModel.model_type, bestModel.params, wFine);

    cTrain = [0    0.9  1  ];
    cVal   = [1    0.65 0  ];
    cFit   = [0.45 1    0.45];
    cFaded = [0.5  0.5  0.5];

    figure;

    subplot(2,1,1);
    hold on; grid on;

    if plotRejected && any(~goodTrain)
        semilogx(fTrain(~goodTrain), 20*log10(abs(GTrain(~goodTrain)) / K_norm), "x", ...
            "Color", cFaded, "HandleVisibility", "off");
    end

    semilogx(fTrain(goodTrain), 20*log10(abs(GTrain(goodTrain)) / K_norm), "o", ...
        "Color", cTrain, "LineWidth", 1.2, "DisplayName", "Training FRF");

    if hasVal
        if plotRejected && any(~goodVal)
            semilogx(fVal(~goodVal), 20*log10(abs(GVal(~goodVal)) / K_norm), "x", ...
                "Color", cFaded, "HandleVisibility", "off");
        end
        semilogx(fVal(goodVal), 20*log10(abs(GVal(goodVal)) / K_norm), "s", ...
            "Color", cVal, "LineWidth", 1.2, "DisplayName", "Validation FRF");
    end

    semilogx(fFine, 20*log10(abs(Gfit) / K_norm), "-", ...
        "Color", cFit, "LineWidth", 2.2, "DisplayName", "Best fit: " + bestModel.model_type);

    yline(0,  "--", "0 dB");
    yline(-3, "--", "-3 dB");

    ylabel("Normalized magnitude (dB re: |K|)");
    title("Normalized Bode - " + runName, "Interpreter", "none");
    subtitle(bestModel.model_type + "  |K| = " + sprintf("%.6g", K_norm) + " N/us", ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");

    subplot(2,1,2);
    hold on; grid on;

    if plotRejected && any(~goodTrain)
        semilogx(fTrain(~goodTrain), unwrap(angle(GTrain(~goodTrain))) * 180/pi, "x", ...
            "Color", cFaded, "HandleVisibility", "off");
    end

    semilogx(fTrain(goodTrain), unwrap(angle(GTrain(goodTrain))) * 180/pi, "o", ...
        "Color", cTrain, "LineWidth", 1.2, "DisplayName", "Training FRF");

    if hasVal
        if plotRejected && any(~goodVal)
            semilogx(fVal(~goodVal), unwrap(angle(GVal(~goodVal))) * 180/pi, "x", ...
                "Color", cFaded, "HandleVisibility", "off");
        end
        semilogx(fVal(goodVal), unwrap(angle(GVal(goodVal))) * 180/pi, "s", ...
            "Color", cVal, "LineWidth", 1.2, "DisplayName", "Validation FRF");
    end

    semilogx(fFine, unwrap(angle(Gfit)) * 180/pi, "-", ...
        "Color", cFit, "LineWidth", 2.2, "DisplayName", "Best fit: " + bestModel.model_type);

    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    legend("Location", "best", "Interpreter", "none");
end

function plotNormalizedBodeMagnitude(frf, models, bestModel, runName, datasetLabel, minCoherence, plotRejected)
    if isempty(frf.f_Hz)
        return;
    end

    f = frf.f_Hz(:);
    G = frf.G_emp(:);
    coh = frf.coherence(:);

    good = isfinite(f) & isfinite(G) & isfinite(coh) & coh >= minCoherence;

    if nnz(good) < 2
        warning("Skipping normalized Bode magnitude for %s %s: fewer than 2 valid points.", ...
            datasetLabel, runName);
        return;
    end

    % Use best model steady-state gain as normalization.
    K_norm = abs(bestModel.K);

    if ~isfinite(K_norm) || K_norm <= 0
        warning("Skipping normalized Bode magnitude for %s %s: invalid K_norm.", ...
            datasetLabel, runName);
        return;
    end

    figure;
    hold on; grid on;

    if plotRejected && any(~good)
        semilogx(f(~good), 20*log10(abs(G(~good)) / K_norm), "x", ...
            "DisplayName", "Rejected empirical points");
    end

    semilogx(f(good), 20*log10(abs(G(good)) / K_norm), "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Empirical PRPS FRF / |K|");

    fFine = logspace(log10(min(f(good))), log10(max(f(good))), 500).';
    wFine = 2*pi*fFine;

    for i = 1:numel(models)
        model = models(i);
        Gfit = evalFrequencyModel(model.model_type, model.params, wFine);

        if string(model.model_type) == string(bestModel.model_type)
            lw = 2.2;
            name = "BEST fit / |K|: " + model.model_type;
        else
            lw = 1.0;
            name = "Candidate / |K|: " + model.model_type;
        end

        semilogx(fFine, 20*log10(abs(Gfit) / K_norm), ...
            "LineWidth", lw, ...
            "DisplayName", name);
    end

    yline(0, "--", "0 dB");
    yline(-3, "--", "-3 dB");

    xlabel("Frequency (Hz)");
    ylabel("Normalized magnitude (dB re: fitted DC gain)");
    title("Normalized Bode Magnitude - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");

    subtitle("Magnitude normalized by |K| = " + sprintf("%.6g", K_norm) + " N/us", ...
        "Interpreter", "none");

    legend("Location", "best", "Interpreter", "none");
end

function plotValidationBestFitTimeWindow( ...
    D, ...
    bestModel, ...
    runName, ...
    timeWindow_s, ...
    centerSignals, ...
    useRawTimestampGridForLsim, ...
    plotAllGroups)

    if isempty(D) || height(D) < 5
        return;
    end

    if numel(timeWindow_s) ~= 2 || timeWindow_s(2) <= timeWindow_s(1)
        error("timeWindow_s must be [t_start, t_end] with t_end > t_start.");
    end

    tStart_s = timeWindow_s(1);
    tEnd_s   = timeWindow_s(2);

    groups = getTimeGroups(D);
    uniqueGroups = unique(groups, "stable");

    for g = 1:numel(uniqueGroups)
        idx = find(groups == uniqueGroups(g));
        Dg = D(idx, :);

        tRaw = forceNumeric(Dg.t_ms) / 1000;
        uRaw = forceNumeric(Dg.pwm_us);
        yRaw = forceNumeric(Dg.force_N);

        valid = isfinite(tRaw) & isfinite(uRaw) & isfinite(yRaw);
        tRaw = tRaw(valid);
        uRaw = uRaw(valid);
        yRaw = yRaw(valid);

        if numel(tRaw) < 5
            continue;
        end

        tRaw = tRaw - tRaw(1);

        % Remove duplicate timestamps.
        [tRaw, uniqueIdx] = unique(tRaw, "stable");
        uRaw = uRaw(uniqueIdx);
        yRaw = yRaw(uniqueIdx);

        [tUniform, uUniform, yUniform, dtUsed] = makeLsimUniformGridFromRawTimestamps( ...
            tRaw, ...
            uRaw, ...
            yRaw, ...
            useRawTimestampGridForLsim);

        if numel(tUniform) < 5
            continue;
        end

        % Important:
        % Simulate the full validation record first, then crop.
        % This avoids giving the model a fake initial-condition reset at 60 s.
        if centerSignals
            uInput = uUniform - mean(uUniform, "omitnan");
            yOffset = mean(yUniform, "omitnan");
        else
            uInput = uUniform;
            yOffset = 0;
        end

        sys = makeTransferFunction(bestModel);

        try
            yModel = lsim(sys, uInput, tUniform) + yOffset;
        catch ME
            warning("lsim failed for validation time-window plot %s / group %s: %s", ...
                runName, string(uniqueGroups(g)), ME.message);
            continue;
        end

        windowMask = tUniform >= tStart_s & tUniform <= tEnd_s;

        if nnz(windowMask) < 5
            warning("Skipping %s / group %s: fewer than 5 samples in %.2f to %.2f s window.", ...
                runName, string(uniqueGroups(g)), tStart_s, tEnd_s);
            continue;
        end

        tPlot = tUniform(windowMask);
        uPlot = uUniform(windowMask);
        yPlot = yUniform(windowMask);
        yModelPlot = yModel(windowMask);

        err = yPlot - yModelPlot;
        rmse_N = sqrt(mean(err.^2, "omitnan"));
        relRmse_pct = 100 * rmse_N / max(range(yPlot), eps);

        figure;
        hold on; grid on;

        %yyaxis left;
        plot(tPlot, yPlot, "LineWidth", 1.3, ...
            "DisplayName", "Measured thrust");

        plot(tPlot, yModelPlot, "g--", "LineWidth", 1.8, ...
            "DisplayName", "Best fit model: " + bestModel.model_type);

        ylabel("Thrust (N)");

        %yyaxis right;
        %plot(tPlot, uPlot, ":", "LineWidth", 1.0, ...
        %    "DisplayName", "PWM command");
        %ylabel("PWM command (\mus)");

        xlabel("Time (s)");

        title( ...
            "Validation Best-Fit Time Window - " + runName + ...
            " - group " + string(uniqueGroups(g)) + ...
            " - " + string(tStart_s) + " to " + string(tEnd_s) + " s" + ...
            " - RMSE = " + string(sprintf("%.4f", rmse_N)) + " N" + ...
            " - rel RMSE = " + string(sprintf("%.2f", relRmse_pct)) + "%", ...
            "Interpreter", "none");

        legend("Location", "best", "Interpreter", "none");

        if ~plotAllGroups
            break;
        end
    end
end

function plotComprehensiveModelComparisonBar(bestModelTable, tolerancePct)
    if isempty(bestModelTable)
        return;
    end

    runLabels  = string(bestModelTable.run_name);
    modelTypes = string(bestModelTable.best_model_type);
    trainMag   = bestModelTable.train_mag_rmse_dB;
    trainPhase = bestModelTable.train_phase_rmse_deg;
    valMag     = bestModelTable.validation_mag_rmse_dB;
    valPhase   = bestModelTable.validation_phase_rmse_deg;

    hasVal = any(isfinite(valMag));

    modelShort = replace(modelTypes, ...
        ["second_order_lag_delay", "second_order_lag", "first_order_delay", "first_order"], ...
        ["2nd+L",                  "2nd",              "1st+L",             "1st"]);

    shortRegion = replace(runLabels, ["global_", "local_"], ["G:", "L:"]);
    shortRegion = replace(shortRegion, "_", "-");
    xLabels     = shortRegion + " (" + modelShort + ")";

    cTrain = [0    0.9  1   ];
    cVal   = [1    0.65 0   ];

    nRuns = height(bestModelTable);
    x = (1:nRuns).';

    figure;
    sgtitle(sprintf("Best-Model RMSE by Region  (simplicity tolerance %.0f%%)", ...
        tolerancePct * 100), "Interpreter", "none");

    subplot(2,1,1);
    hold on; grid on;

    if hasVal
        b = bar(x, [trainMag, valMag], "grouped");
        b(1).FaceColor = cTrain;
        b(2).FaceColor = cVal;
        legend(["Training", "Validation"], "Location", "best");
    else
        b = bar(x, trainMag);
        b.FaceColor = cTrain;
    end

    set(gca, "XTick", x, "XTickLabel", cellstr(xLabels), "XTickLabelRotation", 20);
    ylabel("Magnitude RMSE (dB)");

    subplot(2,1,2);
    hold on; grid on;

    if hasVal
        b = bar(x, [trainPhase, valPhase], "grouped");
        b(1).FaceColor = cTrain;
        b(2).FaceColor = cVal;
        legend(["Training", "Validation"], "Location", "best");
    else
        b = bar(x, trainPhase);
        b.FaceColor = cTrain;
    end

    set(gca, "XTick", x, "XTickLabel", cellstr(xLabels), "XTickLabelRotation", 20);
    ylabel("Phase RMSE (deg)");
    xlabel("Region  (selected model)");
end

%% General table / utility helpers
% ============================================================

function T = readCombinedCsvTables(csvFileStruct)
    tables = cell(numel(csvFileStruct), 1);
    allVarNames = strings(0, 1);

    for k = 1:numel(csvFileStruct)
        thisPath = fullfile(csvFileStruct(k).folder, csvFileStruct(k).name);
        Tk = readtable(thisPath);

        Tk.source_file = repmat(string(csvFileStruct(k).name), height(Tk), 1);
        Tk.source_file_index = repmat(k, height(Tk), 1);

        tables{k} = Tk;
        allVarNames = union(allVarNames, string(Tk.Properties.VariableNames), "stable");
    end

    for k = 1:numel(tables)
        Tk = tables{k};
        currentVars = string(Tk.Properties.VariableNames);

        for v = 1:numel(allVarNames)
            varName = allVarNames(v);

            if ~ismember(varName, currentVars)
                if any(varName == ["source_file", "run_name", "phase", "segment"])
                    Tk.(varName) = repmat("", height(Tk), 1);
                else
                    Tk.(varName) = NaN(height(Tk), 1);
                end
            end
        end

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

function tf = hasVoltage(D)
    tf = ismember("battery_voltage_V", string(D.Properties.VariableNames)) && ...
         any(isfinite(forceNumeric(D.battery_voltage_V)));
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

function value = getfieldwithdefault(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function frf = emptyFrf()
    frf.f_Hz = [];
    frf.w_rad_s = [];
    frf.G_emp = [];
    frf.coherence = [];
    frf.Suu = [];
    frf.Syy = [];
    frf.Syu = [];
    frf.n_avg = [];
    frf.run_name = "";
    frf.dataset_label = "";
    frf.raw_U = [];
    frf.raw_Y = [];
    frf.raw_f_Hz = [];
    frf.raw_bins = [];
    frf.raw_file_index = [];
    frf.raw_period_index = [];
end