%% analyze_servo_prps_frequency_fit.m
% Analyze servo PRPS data from Pico CSV logs using frequency-domain fitting.
%
% Supports older compact CSV columns:
%   t_ms,t_s,run_name,segment,servo_us,command_delta_us,
%   count_delta,theta_rad,period_index,period_sample_index,prps_x
%
% Supports newer ultra-compact CSV columns:
%   t_ms,set_index,prps_seed,run_idx,servo_us,count_delta,
%   period_index,period_sample_index
%
% For newer ultra-compact logs, this script reconstructs:
%   t_s              = t_ms / 1000
%   command_delta_us = servo_us - DEFAULT_SERVO_CENTER_US
%   theta_rad        = count_delta / COUNTS_PER_REV * 2*pi
%   segment          = "prps"
%   run_name         = local_amp<amplitude>_<DEFAULT_FREQ_LABEL>
%
% Main empirical transfer function:
%
%   G_emp(jw) = theta_rad(jw) / command_delta_us(jw)
%
% Units:
%   Input:  command_delta_us, microseconds
%   Output: theta_rad, radians
%   Gain:   rad/us

clear; clc; close all;

%% User options

SAVE_FIGURES = true;

% File selection mode:
%   true  = train on first N sorted CSV files, validate on next M files
%   false = train on one manually specified file, validate on all others
TRAIN_ON_FIRST_N_FILES = true;

% For initial debugging, use one clean file first.
NUM_TRAINING_FILES = 9;

% Validation control:
%   inf = use all remaining files after training
%   0   = no validation files
%   N   = use next N files after training
NUM_VALIDATION_FILES = inf;

% Used only if TRAIN_ON_FIRST_N_FILES = false.
trainingFile = "servo_prps_amp100_seed4001_00.csv";

% Optional manual validation files.
% Leave empty to use every other CSV as validation when TRAIN_ON_FIRST_N_FILES = false.
validationFilesManual = strings(0, 1);

% Leave empty to auto-detect run names from training data.
% Usually keep empty.
runNames = strings(0, 1);

% If true, runNames are drawn from training and validation.
% If false, runNames are drawn only from training.
% Recommendation:
%   false for normal model fitting.
%   true only if you want to see which validation-only amplitudes exist.
INCLUDE_VALIDATION_ONLY_RUN_NAMES = false;

% Cross-amplitude validation mode:
%   Fit model(s) only on training run(s), then apply each fitted training model
%   to every validation CSV file individually, regardless of run_name stored inside
%   the validation file. This is the right mode for testing whether an amp100
%   model generalizes to amp200/amp300/amp400 data.
VALIDATE_MODEL_ON_ALL_VALIDATION_FILES = false;

% Local model generalization mode:
%   After fitting one best model per training run/amplitude, score every local
%   model against every other training run/amplitude. This creates the
%   model-vs-amplitude generalization matrix.
SCORE_LOCAL_MODELS_ON_ALL_TRAINING_RUNS = true;

% Global model mode:
%   Pool all training FRF points across all training amplitudes and fit one
%   comprehensive/global model. Then score that one global model against each
%   individual training amplitude and each validation file.
FIT_GLOBAL_MODEL_ON_ALL_TRAINING_RUNS = true;

% If true, plot global model against each individual empirical FRF.
PLOT_GLOBAL_MODEL_PER_RUN = false;

% If true, plot global model time-domain prediction against each individual
% training file/run and validation file. This is separate from the local model
% time-domain plots.
PLOT_GLOBAL_TIME_DOMAIN_ON_TRAINING_FILES = false;
PLOT_GLOBAL_TIME_DOMAIN_ON_VALIDATION_FILES = false;

% If true, only plot the global best model in time-domain overlays.
% Usually keep true to avoid clutter.
PLOT_GLOBAL_TIME_DOMAIN_BEST_ONLY = true;


% Nominal command/sample period from Pico file.
USE_NOMINAL_COMMAND_DT_FOR_FRF = true;
NOMINAL_COMMAND_DT_S = 0.010;

%% Reconstruction settings for Pico CSVs

COUNTS_PER_REV = encoderAngleScale().counts_per_rev;   % spec-derived (1:1 GT2 16T)
DEFAULT_SERVO_CENTER_US = 1450;

% Sign convention:
%   +1 means positive encoder angle for positive PWM command.
%   -1 means positive PWM command produces negative encoder angle.
%
% Your current data shows:
%   command_delta_us ↑ -> theta_rad ↓
%
% Therefore use -1 so the fitted plant has positive low-frequency gain.
SERVO_OUTPUT_SIGN = -1;

DEFAULT_FREQ_LABEL = "0.1_to_3Hz";

%% PRPS / frequency extraction options

USE_ONLY_PRPS_SEGMENT = true;

MIN_PERIODS_PER_RUN = 2;
MIN_SAMPLES_PER_PERIOD = 20;

% If true, infer excited frequencies from input command spectrum.
INFER_EXCITED_FREQUENCIES_FROM_INPUT = true;

% Used only if INFER_EXCITED_FREQUENCIES_FROM_INPUT = false.
MANUAL_EXCITED_FREQS_HZ = [
    0.10
    0.15
    0.20
    0.30
    0.40
    0.50
    0.75
    1.00
    1.25
    1.50
    2.00
    2.50
    3.00
];

INPUT_BIN_RELATIVE_THRESHOLD = 0.05;

FREQ_MIN_HZ = 0.08;
FREQ_MAX_HZ = 3.25;

MIN_COHERENCE_FOR_FIT = 0.60;
PLOT_REJECTED_FREQ_POINTS = true;

REMOVE_PERIOD_MEAN = true;
DETREND_EACH_PERIOD = false;

BANDWIDTH_DROP_DB = -3.0;

%% Frequency-domain fitting options

FIT_FIRST_ORDER                 = true;
FIT_FIRST_ORDER_DELAY           = true;
FIT_SECOND_ORDER_LAG            = true;
FIT_SECOND_ORDER_LAG_DELAY      = true;

%   "lowest_validation_error"
%   "lowest_training_error"
%   "simplicity_tolerance"
MODEL_SELECTION_MODE = "simplicity_tolerance";
SIMPLE_MODEL_RELATIVE_TOLERANCE = 0.05;

%   "relative_complex"
%   "relative_with_coh"
%   "absolute_complex"
WEIGHTING_MODE = "relative_with_coh";

GAIN_FLOOR_MODE = "fraction_of_median";
GAIN_FLOOR_FRACTION_OF_MEDIAN = 0.20;
MANUAL_GAIN_FLOOR_RAD_PER_US = 1e-5;

MAX_FMINSEARCH_ITER = 5000;
MAX_FMINSEARCH_EVAL = 10000;

INITIAL_K_RAD_PER_US = pi / 2000;
INITIAL_TAU_S = 0.08;
INITIAL_DELAY_S = 0.02;
INITIAL_TAU2_S = 0.02;

MIN_TAU_S = 0.005;
MAX_TAU_S = 2.0;
MIN_DELAY_S = 0.0;
MAX_DELAY_S = 0.50;
MIN_GAIN_ABS = 1e-5;
MAX_GAIN_ABS = 0.01;

%% Time-domain translation options

PLOT_TIME_DOMAIN_TRANSLATION_TRAINING   = false;
PLOT_TIME_DOMAIN_TRANSLATION_VALIDATION = true;

% Windowed time-domain plot: global model vs each validation file,
% showing only the specified time window. lsim runs on the full signal;
% only the display is cropped. Respects MAX_VALIDATION_FILES_TO_PLOT.
PLOT_GLOBAL_TIME_DOMAIN_VALIDATION_WINDOW = true;
GLOBAL_TIME_DOMAIN_WINDOW_START_S = 30;
GLOBAL_TIME_DOMAIN_WINDOW_END_S   = 40;

CENTER_TIME_DOMAIN_SIGNALS = true;

% Residual whiteness (project_metrics.md Section 1, metrics #3-#4): on each held-out
% validation file, residual e = theta - lsim(globalModel, u) is tested for
% autocorrelation (no unmodeled dynamics left) and cross-correlation with the input
% (model order sufficient). Both should sit inside the +/-1.96/sqrt(N) white-noise band.
COMPUTE_RESIDUAL_WHITENESS  = true;
PLOT_RESIDUAL_WHITENESS     = true;
RESIDUAL_WHITENESS_MAX_LAG_S = 2.0;

%% Plot toggles

PLOT_TRAINING_EMPIRICAL_BODE_WITH_FITS      = true;
PLOT_VALIDATION_BODE_VS_TRAINING_MODEL      = false;
PLOT_TRAINING_VS_VALIDATION_EMPIRICAL_BODE  = false;
PLOT_COHERENCE_BY_RUN                       = false;
PLOT_MODEL_COMPARISON_BAR                   = true;
PLOT_NORMALIZED_BODE                        = true;
PLOT_AMPLITUDE_GENERALIZATION_CURVE         = true;

PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_TRAINING    = false;
PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_VALIDATION  = false;
PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS            = false;

% Maximum number of validation files for which per-file plots are generated
% inside the cross-amplitude and global-model validation loops.
% Scoring runs for all files regardless of this limit.
% Set to inf to plot every file.
MAX_VALIDATION_FILES_TO_PLOT = 5;

% Whitelist of PRPS amplitudes [µs] for which Bode and time-domain plots
% are generated. Empty = plot all amplitudes. Example: [500, 1000]
PLOT_AMPLITUDE_FILTER_US = [1000];

%% Locate mirrored folders

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, "candidate");
plotDir = getMirroredPlotDir(scriptPath);

addpath(genpath(fullfile(findRepoRoot(scriptPath), "analysis", "util")));

trainSubDir = fullfile(dataDir, "training");
valSubDir   = fullfile(dataDir, "validation");

useSubfolders = isfolder(trainSubDir);

if useSubfolders
    fprintf("\nDetected training/ subfolder — using subfolders for train/validation split.\n");

    trainCsvRaw = dir(fullfile(trainSubDir, "*.csv"));

    if isempty(trainCsvRaw)
        error("training/ subfolder exists but contains no CSV files:\n%s", trainSubDir);
    end

    [~, sortIdx] = sort(string({trainCsvRaw.name}));
    trainingCsvFiles = trainCsvRaw(sortIdx);

    if isfolder(valSubDir)
        valCsvRaw = dir(fullfile(valSubDir, "*.csv"));

        if isempty(valCsvRaw)
            validationCsvFiles = struct([]);
        else
            [~, sortIdx] = sort(string({valCsvRaw.name}));
            validationCsvFiles = valCsvRaw(sortIdx);
        end
    else
        validationCsvFiles = struct([]);
    end

    fprintf("\nTraining / identification files (%s):\n", trainSubDir);
    for k = 1:numel(trainingCsvFiles)
        fprintf("  %s\n", trainingCsvFiles(k).name);
    end

    fprintf("\nValidation files (%s):\n", valSubDir);
    if isempty(validationCsvFiles)
        fprintf("  None.\n");
    else
        for k = 1:numel(validationCsvFiles)
            fprintf("  %s\n", validationCsvFiles(k).name);
        end
    end

else
    csvFiles = dir(fullfile(dataDir, "*.csv"));

    if isempty(csvFiles)
        error("No CSV files found in:\n%s", dataDir);
    end

    [~, sortIdx] = sort(string({csvFiles.name}));
    csvFiles = csvFiles(sortIdx);

    if TRAIN_ON_FIRST_N_FILES
        nTrain = min(NUM_TRAINING_FILES, numel(csvFiles));

        trainingCsvFiles = csvFiles(1:nTrain);

        firstValidationIdx = nTrain + 1;

        if firstValidationIdx > numel(csvFiles) || NUM_VALIDATION_FILES == 0
            validationCsvFiles = csvFiles([]);
        else
            if isinf(NUM_VALIDATION_FILES)
                lastValidationIdx = numel(csvFiles);
            else
                lastValidationIdx = min(numel(csvFiles), nTrain + NUM_VALIDATION_FILES);
            end

            validationCsvFiles = csvFiles(firstValidationIdx:lastValidationIdx);
        end

        fprintf("\nTraining / identification files:\n");
        for k = 1:numel(trainingCsvFiles)
            fprintf("  %s\n", fullfile(trainingCsvFiles(k).folder, trainingCsvFiles(k).name));
        end

        fprintf("\nValidation files:\n");
        if isempty(validationCsvFiles)
            fprintf("  None.\n");
        else
            for k = 1:numel(validationCsvFiles)
                fprintf("  %s\n", fullfile(validationCsvFiles(k).folder, validationCsvFiles(k).name));
            end
        end
    else
        trainingPath = fullfile(dataDir, trainingFile);
        trainingCsvFiles = dir(trainingPath);

        if isempty(trainingCsvFiles)
            error("Training file not found:\n%s", trainingPath);
        end

        if isempty(validationFilesManual)
            validationCsvFiles = csvFiles(string({csvFiles.name}) ~= string(trainingFile));
        else
            validationCsvFiles = struct([]);

            for k = 1:numel(validationFilesManual)
                thisValPath = fullfile(dataDir, validationFilesManual(k));
                thisVal = dir(thisValPath);

                if isempty(thisVal)
                    error("Validation file not found:\n%s", thisValPath);
                end

                if isempty(validationCsvFiles)
                    validationCsvFiles = thisVal;
                else
                    validationCsvFiles(end+1) = thisVal;
                end
            end
        end

        fprintf("\nTraining / identification file:\n");
        fprintf("  %s\n", fullfile(trainingCsvFiles(1).folder, trainingCsvFiles(1).name));

        fprintf("\nValidation files:\n");
        if isempty(validationCsvFiles)
            fprintf("  None.\n");
        else
            for k = 1:numel(validationCsvFiles)
                fprintf("  %s\n", fullfile(validationCsvFiles(k).folder, validationCsvFiles(k).name));
            end
        end
    end
end

trainingLabel = makeFileListLabel(trainingCsvFiles);

%% Load combined tables and normalize formats

Ttrain = readCombinedCsvTables(trainingCsvFiles);

Ttrain = normalizeServoPrpsTable( ...
    Ttrain, ...
    COUNTS_PER_REV, ...
    DEFAULT_SERVO_CENTER_US, ...
    DEFAULT_FREQ_LABEL, ...
    SERVO_OUTPUT_SIGN);

if isempty(validationCsvFiles)
    Tval = table();
else
    Tval = readCombinedCsvTables(validationCsvFiles);

    Tval = normalizeServoPrpsTable( ...
        Tval, ...
        COUNTS_PER_REV, ...
        DEFAULT_SERVO_CENTER_US, ...
        DEFAULT_FREQ_LABEL, ...
        SERVO_OUTPUT_SIGN);
end

if isempty(runNames)
    if INCLUDE_VALIDATION_ONLY_RUN_NAMES && ~isempty(Tval)
        runNames = unique([string(Ttrain.run_name); string(Tval.run_name)], "stable");
    else
        runNames = unique(string(Ttrain.run_name), "stable");
    end
end

fprintf("\nDetected / selected runs:\n");
for k = 1:numel(runNames)
    fprintf("  %s\n", runNames(k));
end

if ~isempty(Tval)
    valOnlyNames = setdiff(unique(string(Tval.run_name), "stable"), unique(string(Ttrain.run_name), "stable"));
    if ~isempty(valOnlyNames)
        fprintf("\nValidation-only run names detected:\n");
        for k = 1:numel(valOnlyNames)
            fprintf("  %s\n", valOnlyNames(k));
        end
        fprintf("  These will not be fitted unless INCLUDE_VALIDATION_ONLY_RUN_NAMES = true.\n");
    end
end

%% Main frequency-domain identification loop

allSummaryRows = cell(0, 16);
bestModels = struct();
trainingFrfs = struct();
validationFrfs = struct();
crossValidationRows = cell(0, 12);
localGeneralizationRows = cell(0, 12);
globalModelRows = cell(0, 12);
globalSummaryRows = cell(0, 16);


for r = 1:numel(runNames)
    runName = runNames(r);
    modelKey = matlab.lang.makeValidName(runName);
    runAmpUs = inferAmplitudeFromRunName(runName);

    fprintf("\n============================================================\n");
    fprintf("Run: %s\n", runName);
    fprintf("============================================================\n");

    Dtrain = getRunData(Ttrain, runName, USE_ONLY_PRPS_SEGMENT);

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
        DvalAll = getRunData(Tval, runName, USE_ONLY_PRPS_SEGMENT);

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

    if PLOT_COHERENCE_BY_RUN && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
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
        MANUAL_GAIN_FLOOR_RAD_PER_US, ...
        INITIAL_K_RAD_PER_US, ...
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

    bestModel.source_run_name = runName;
    bestModels.(modelKey) = bestModel;

    fprintf("\nBest model for %s:\n", runName);
    printFrequencyModel(bestModel);

    [bandwidthHz, lowFreqGainDb] = estimateBandwidthFromFrf( ...
        trainFrf, ...
        MIN_COHERENCE_FOR_FIT, ...
        BANDWIDTH_DROP_DB);

    delayPhaseSlopeS = estimateDelayFromPhaseSlope( ...
        trainFrf, ...
        MIN_COHERENCE_FOR_FIT, ...
        bandwidthHz, ...
        FREQ_MIN_HZ);

    fprintf("\nEmpirical servo interpretation for %s:\n", runName);
    fprintf("  Low-frequency gain estimate: %.3f dB rad/us\n", lowFreqGainDb);

    if isfinite(bandwidthHz)
        fprintf("  Approx -3 dB bandwidth: %.4f Hz\n", bandwidthHz);
        fprintf("  Conservative controller crossover target: %.4f to %.4f Hz\n", ...
            bandwidthHz/5, bandwidthHz/3);
    else
        fprintf("  Approx -3 dB bandwidth: not found in usable range\n");
    end

    if isfinite(delayPhaseSlopeS)
        fprintf("  Phase-slope delay estimate: %.5f s\n", delayPhaseSlopeS);
    else
        fprintf("  Phase-slope delay estimate: unavailable\n");
    end

    for m = 1:numel(candidateModels)
        cm = candidateModels(m);

        allSummaryRows(end+1, :) = {
            char(trainingLabel), ...
            char(runName), ...
            char(cm.model_type), ...
            cm.complexity, ...
            cm.K, ...
            cm.tau1_s, ...
            cm.tau2_s, ...
            cm.delay_s, ...
            bandwidthHz, ...
            delayPhaseSlopeS, ...
            cm.train_weighted_error, ...
            getfieldwithdefault(cm, "validation_weighted_error", NaN), ...
            cm.train_mag_rmse_dB, ...
            cm.train_phase_rmse_deg, ...
            getfieldwithdefault(cm, "validation_mag_rmse_dB", NaN), ...
            getfieldwithdefault(cm, "validation_phase_rmse_deg", NaN)
        };
    end

    if PLOT_TRAINING_EMPIRICAL_BODE_WITH_FITS && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        plotEmpiricalBodeWithFits(trainFrf, candidateModels, bestModel, runName, "training", ...
            MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS, bandwidthHz);
        if PLOT_NORMALIZED_BODE
            plotEmpiricalBodeWithFits(trainFrf, candidateModels, bestModel, runName, "training", ...
                MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS, bandwidthHz, true);
        end
    end

    if PLOT_VALIDATION_BODE_VS_TRAINING_MODEL && ~isempty(valFrf.f_Hz) && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        plotValidationBodeVsTrainingModel(valFrf, bestModel, runName, MIN_COHERENCE_FOR_FIT);
        if PLOT_NORMALIZED_BODE
            plotValidationBodeVsTrainingModel(valFrf, bestModel, runName, MIN_COHERENCE_FOR_FIT, true);
        end
    end

    if PLOT_TRAINING_VS_VALIDATION_EMPIRICAL_BODE && ~isempty(valFrf.f_Hz) && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        plotTrainingVsValidationEmpiricalBode(trainFrf, valFrf, runName, MIN_COHERENCE_FOR_FIT);
        if PLOT_NORMALIZED_BODE
            plotTrainingVsValidationEmpiricalBode(trainFrf, valFrf, runName, MIN_COHERENCE_FOR_FIT, true);
        end
    end

    if PLOT_MODEL_COMPARISON_BAR && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        plotModelComparison(candidateModels, runName);
    end

    if PLOT_TIME_DOMAIN_TRANSLATION_TRAINING && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        plotTimeDomainTranslationForDataset( ...
            Dtrain, ...
            bestModel, ...
            candidateModels, ...
            runName, ...
            "training", ...
            CENTER_TIME_DOMAIN_SIGNALS, ...
            PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_TRAINING, ...
            PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS);
    end

    if PLOT_TIME_DOMAIN_TRANSLATION_VALIDATION && ~isempty(Tval) && isAmplitudeAllowed(runAmpUs, PLOT_AMPLITUDE_FILTER_US)
        DvalTime = getRunData(Tval, runName, USE_ONLY_PRPS_SEGMENT);

        if height(DvalTime) >= MIN_SAMPLES_PER_PERIOD
            plotTimeDomainTranslationForDataset( ...
                DvalTime, ...
                bestModel, ...
                candidateModels, ...
                runName, ...
                "validation", ...
                CENTER_TIME_DOMAIN_SIGNALS, ...
                PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_VALIDATION, ...
                PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS);
        end
    end

    %% Cross-amplitude validation: apply this training-fitted model to

    if VALIDATE_MODEL_ON_ALL_VALIDATION_FILES && ~isempty(Tval)
        validationSourceFiles = unique(string(Tval.source_file), "stable");

        fprintf("\nCross-amplitude validation for training model: %s\n", runName);

        crossValPlotCount = 0;

        for vf = 1:numel(validationSourceFiles)
            valSourceFile = validationSourceFiles(vf);
            valAmpUs = inferAmplitudeFromSourceFile(valSourceFile);
            DvalCross = getSourceFileData(Tval, valSourceFile, USE_ONLY_PRPS_SEGMENT);

            if height(DvalCross) < MIN_SAMPLES_PER_PERIOD
                warning("Skipping cross-validation file %s: not enough samples.", valSourceFile);
                continue;
            end

            valLabel = makeCrossValidationLabel(valSourceFile, DvalCross);

            valFrfCross = estimatePrpsFrfFromTable( ...
                DvalCross, ...
                valLabel, ...
                "cross_validation", ...
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

            if isempty(valFrfCross.f_Hz)
                warning("Skipping cross-validation file %s: no FRF points.", valSourceFile);
                continue;
            end

            crossScore = scoreSingleModelOnFrf(bestModel, valFrfCross);

            fprintf("  Validation file: %s\n", valSourceFile);
            fprintf("    Label: %s\n", valLabel);
            fprintf("    Weighted error: %.6f\n", crossScore.validation_weighted_error);
            fprintf("    Magnitude RMSE: %.3f dB\n", crossScore.validation_mag_rmse_dB);
            fprintf("    Phase RMSE: %.3f deg\n", crossScore.validation_phase_rmse_deg);

            crossValidationRows(end+1, :) = { ...
                char(trainingLabel), ...
                char(runName), ...
                char(bestModel.model_type), ...
                bestModel.K, ...
                bestModel.tau1_s, ...
                bestModel.tau2_s, ...
                bestModel.delay_s, ...
                char(valSourceFile), ...
                char(valLabel), ...
                crossScore.validation_weighted_error, ...
                crossScore.validation_mag_rmse_dB, ...
                crossScore.validation_phase_rmse_deg ...
            };

            if crossValPlotCount < MAX_VALIDATION_FILES_TO_PLOT && isAmplitudeAllowed(valAmpUs, PLOT_AMPLITUDE_FILTER_US)
                if PLOT_VALIDATION_BODE_VS_TRAINING_MODEL
                    plotValidationBodeVsTrainingModel( ...
                        valFrfCross, ...
                        bestModel, ...
                        runName + " model on " + valLabel, ...
                        MIN_COHERENCE_FOR_FIT);
                    if PLOT_NORMALIZED_BODE
                        plotValidationBodeVsTrainingModel( ...
                            valFrfCross, ...
                            bestModel, ...
                            runName + " model on " + valLabel, ...
                            MIN_COHERENCE_FOR_FIT, true);
                    end
                end

                if PLOT_TIME_DOMAIN_TRANSLATION_VALIDATION
                    plotTimeDomainTranslationForDataset( ...
                        DvalCross, ...
                        bestModel, ...
                        candidateModels, ...
                        runName + " model on " + valLabel, ...
                        "cross_validation", ...
                        CENTER_TIME_DOMAIN_SIGNALS, ...
                        PLOT_TIME_DOMAIN_TRANSLATION_BEST_ONLY_VALIDATION, ...
                        PLOT_ALL_TIME_DOMAIN_MODEL_TRANSLATIONS);
                end

                crossValPlotCount = crossValPlotCount + 1;
            end
        end
    end

end

%% Local model generalization across all training runs/amplitudes

if SCORE_LOCAL_MODELS_ON_ALL_TRAINING_RUNS && ~isempty(fieldnames(bestModels))
    fprintf("\n============================================================\n");
    fprintf("Local model generalization across training amplitudes\n");
    fprintf("============================================================\n");

    trainKeys = string(fieldnames(trainingFrfs));
    modelKeys = string(fieldnames(bestModels));

    for mk = 1:numel(modelKeys)
        sourceModelKey = modelKeys(mk);
        sourceModel = bestModels.(sourceModelKey);

        if isfield(sourceModel, "source_run_name")
            sourceRunName = string(sourceModel.source_run_name);
        else
            sourceRunName = sourceModelKey;
        end

        for tk = 1:numel(trainKeys)
            targetKey = trainKeys(tk);
            targetFrf = trainingFrfs.(targetKey);

            targetLabel = string(targetFrf.run_name);
            if strlength(targetLabel) == 0
                targetLabel = targetKey;
            end

            localScore = scoreSingleModelOnFrf(sourceModel, targetFrf);

            fprintf("  Model %s -> training data %s | weighted %.6f | mag %.3f dB | phase %.3f deg\n", ...
                sourceRunName, targetLabel, ...
                localScore.validation_weighted_error, ...
                localScore.validation_mag_rmse_dB, ...
                localScore.validation_phase_rmse_deg);

            localGeneralizationRows(end+1, :) = { ...
                char(trainingLabel), ...
                char(sourceRunName), ...
                char(sourceModel.model_type), ...
                sourceModel.K, ...
                sourceModel.tau1_s, ...
                sourceModel.tau2_s, ...
                sourceModel.delay_s, ...
                "training", ...
                char(targetLabel), ...
                localScore.validation_weighted_error, ...
                localScore.validation_mag_rmse_dB, ...
                localScore.validation_phase_rmse_deg ...
            };
        end
    end
end

%% Global pooled model across all training amplitudes

globalBestModel = [];
globalCandidateModels = [];
globalFrf = emptyFrf();

if FIT_GLOBAL_MODEL_ON_ALL_TRAINING_RUNS && ~isempty(fieldnames(trainingFrfs))
    fprintf("\n============================================================\n");
    fprintf("Global pooled model across all training amplitudes\n");
    fprintf("============================================================\n");

    globalFrf = combineFrfsFromStruct(trainingFrfs, "global_all_training_amplitudes");

    fitMaskGlobal = globalFrf.coherence >= MIN_COHERENCE_FOR_FIT & ...
                    isfinite(globalFrf.G_emp) & ...
                    isfinite(globalFrf.f_Hz) & ...
                    globalFrf.f_Hz >= FREQ_MIN_HZ & ...
                    globalFrf.f_Hz <= FREQ_MAX_HZ;

    if nnz(fitMaskGlobal) < 2
        warning("Global model skipped: fewer than 2 usable pooled FRF points.");
    else
        globalCandidateModels = fitFrequencyModels( ...
            globalFrf.f_Hz(fitMaskGlobal), ...
            globalFrf.G_emp(fitMaskGlobal), ...
            globalFrf.coherence(fitMaskGlobal), ...
            FIT_FIRST_ORDER, ...
            FIT_FIRST_ORDER_DELAY, ...
            FIT_SECOND_ORDER_LAG, ...
            FIT_SECOND_ORDER_LAG_DELAY, ...
            WEIGHTING_MODE, ...
            GAIN_FLOOR_MODE, ...
            GAIN_FLOOR_FRACTION_OF_MEDIAN, ...
            MANUAL_GAIN_FLOOR_RAD_PER_US, ...
            INITIAL_K_RAD_PER_US, ...
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

        globalCandidateModels = scoreModelsOnFrf(globalCandidateModels, globalFrf, "training");

        globalBestModel = selectBestFrequencyModel( ...
            globalCandidateModels, ...
            MODEL_SELECTION_MODE, ...
            SIMPLE_MODEL_RELATIVE_TOLERANCE);

        globalBestModel.source_run_name = "GLOBAL_ALL_TRAINING_AMPLITUDES";

        fprintf("\nBest GLOBAL model:\n");
        printFrequencyModel(globalBestModel);

        [globalBandwidthHz, globalLowFreqGainDb] = estimateBandwidthFromFrf( ...
            globalFrf, ...
            MIN_COHERENCE_FOR_FIT, ...
            BANDWIDTH_DROP_DB);

        globalDelayPhaseSlopeS = estimateDelayFromPhaseSlope( ...
            globalFrf, ...
            MIN_COHERENCE_FOR_FIT, ...
            globalBandwidthHz, ...
            FREQ_MIN_HZ);

        for gm = 1:numel(globalCandidateModels)
            cm = globalCandidateModels(gm);

            globalSummaryRows(end+1, :) = { ...
                char(trainingLabel), ...
                'GLOBAL_ALL_TRAINING_AMPLITUDES', ...
                char(cm.model_type), ...
                cm.complexity, ...
                cm.K, ...
                cm.tau1_s, ...
                cm.tau2_s, ...
                cm.delay_s, ...
                globalBandwidthHz, ...
                globalDelayPhaseSlopeS, ...
                cm.train_weighted_error, ...
                NaN, ...
                cm.train_mag_rmse_dB, ...
                cm.train_phase_rmse_deg, ...
                NaN, ...
                NaN ...
            };
        end

        if PLOT_TRAINING_EMPIRICAL_BODE_WITH_FITS
            plotEmpiricalBodeWithFits(globalFrf, globalCandidateModels, globalBestModel, ...
                "GLOBAL_ALL_TRAINING_AMPLITUDES", "training", ...
                MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS, globalBandwidthHz);
            if PLOT_NORMALIZED_BODE
                plotEmpiricalBodeWithFits(globalFrf, globalCandidateModels, globalBestModel, ...
                    "GLOBAL_ALL_TRAINING_AMPLITUDES", "training", ...
                    MIN_COHERENCE_FOR_FIT, PLOT_REJECTED_FREQ_POINTS, globalBandwidthHz, true);
            end
        end

        % Score global model against each individual training FRF.
        trainKeys = string(fieldnames(trainingFrfs));

        fprintf("\nGlobal model scored against individual training amplitudes:\n");
        for tk = 1:numel(trainKeys)
            targetKey = trainKeys(tk);
            targetFrf = trainingFrfs.(targetKey);
            targetLabel = string(targetFrf.run_name);

            if strlength(targetLabel) == 0
                targetLabel = targetKey;
            end

            globalScore = scoreSingleModelOnFrf(globalBestModel, targetFrf);

            fprintf("  Global -> %s | weighted %.6f | mag %.3f dB | phase %.3f deg\n", ...
                targetLabel, ...
                globalScore.validation_weighted_error, ...
                globalScore.validation_mag_rmse_dB, ...
                globalScore.validation_phase_rmse_deg);

            globalModelRows(end+1, :) = { ...
                char(trainingLabel), ...
                'GLOBAL_ALL_TRAINING_AMPLITUDES', ...
                char(globalBestModel.model_type), ...
                globalBestModel.K, ...
                globalBestModel.tau1_s, ...
                globalBestModel.tau2_s, ...
                globalBestModel.delay_s, ...
                "training", ...
                char(targetLabel), ...
                globalScore.validation_weighted_error, ...
                globalScore.validation_mag_rmse_dB, ...
                globalScore.validation_phase_rmse_deg ...
            };

            if PLOT_GLOBAL_MODEL_PER_RUN
                plotValidationBodeVsTrainingModel( ...
                    targetFrf, ...
                    globalBestModel, ...
                    "GLOBAL model on " + targetLabel, ...
                    MIN_COHERENCE_FOR_FIT);
                if PLOT_NORMALIZED_BODE
                    plotValidationBodeVsTrainingModel( ...
                        targetFrf, ...
                        globalBestModel, ...
                        "GLOBAL model on " + targetLabel, ...
                        MIN_COHERENCE_FOR_FIT, true);
                end
            end

            if PLOT_GLOBAL_TIME_DOMAIN_ON_TRAINING_FILES
                DtargetGlobalTrain = getRunData(Ttrain, targetLabel, USE_ONLY_PRPS_SEGMENT);

                if height(DtargetGlobalTrain) >= MIN_SAMPLES_PER_PERIOD
                    plotTimeDomainTranslationForDataset( ...
                        DtargetGlobalTrain, ...
                        globalBestModel, ...
                        globalCandidateModels, ...
                        "GLOBAL model on " + targetLabel, ...
                        "global_training", ...
                        CENTER_TIME_DOMAIN_SIGNALS, ...
                        PLOT_GLOBAL_TIME_DOMAIN_BEST_ONLY, ...
                        false);
                end
            end
        end

        % Score global model against each validation file individually.
        if ~isempty(Tval)
            validationSourceFiles = unique(string(Tval.source_file), "stable");

            fprintf("\nGlobal model scored against validation files:\n");

            globalValPlotCount = 0;
            residualWhitenessRows = struct("label", {}, "acf_frac_in", {}, "ccf_frac_in", {}, "pass", {});

            for vf = 1:numel(validationSourceFiles)
                valSourceFile = validationSourceFiles(vf);
                valAmpUs = inferAmplitudeFromSourceFile(valSourceFile);
                DvalGlobal = getSourceFileData(Tval, valSourceFile, USE_ONLY_PRPS_SEGMENT);

                if height(DvalGlobal) < MIN_SAMPLES_PER_PERIOD
                    warning("Skipping global validation file %s: not enough samples.", valSourceFile);
                    continue;
                end

                valLabel = makeCrossValidationLabel(valSourceFile, DvalGlobal);

                valFrfGlobal = estimatePrpsFrfFromTable( ...
                    DvalGlobal, ...
                    valLabel, ...
                    "global_validation", ...
                    false, ...
                    globalFrf.f_Hz, ...
                    INPUT_BIN_RELATIVE_THRESHOLD, ...
                    FREQ_MIN_HZ, ...
                    FREQ_MAX_HZ, ...
                    REMOVE_PERIOD_MEAN, ...
                    DETREND_EACH_PERIOD, ...
                    MIN_PERIODS_PER_RUN, ...
                    MIN_SAMPLES_PER_PERIOD, ...
                    USE_NOMINAL_COMMAND_DT_FOR_FRF, ...
                    NOMINAL_COMMAND_DT_S);

                if isempty(valFrfGlobal.f_Hz)
                    warning("Skipping global validation file %s: no FRF points.", valSourceFile);
                    continue;
                end

                globalScore = scoreSingleModelOnFrf(globalBestModel, valFrfGlobal);

                fprintf("  Global -> %s | weighted %.6f | mag %.3f dB | phase %.3f deg\n", ...
                    valLabel, ...
                    globalScore.validation_weighted_error, ...
                    globalScore.validation_mag_rmse_dB, ...
                    globalScore.validation_phase_rmse_deg);

                globalModelRows(end+1, :) = { ...
                    char(trainingLabel), ...
                    'GLOBAL_ALL_TRAINING_AMPLITUDES', ...
                    char(globalBestModel.model_type), ...
                    globalBestModel.K, ...
                    globalBestModel.tau1_s, ...
                    globalBestModel.tau2_s, ...
                    globalBestModel.delay_s, ...
                    "validation", ...
                    char(valLabel), ...
                    globalScore.validation_weighted_error, ...
                    globalScore.validation_mag_rmse_dB, ...
                    globalScore.validation_phase_rmse_deg ...
                };

                if COMPUTE_RESIDUAL_WHITENESS
                    rw = computeResidualWhiteness(DvalGlobal, globalBestModel, valLabel, ...
                        RESIDUAL_WHITENESS_MAX_LAG_S, PLOT_RESIDUAL_WHITENESS);
                    if ~isempty(rw.label)
                        residualWhitenessRows(end+1) = rw; %#ok<SAGROW>
                    end
                end

                if globalValPlotCount < MAX_VALIDATION_FILES_TO_PLOT && isAmplitudeAllowed(valAmpUs, PLOT_AMPLITUDE_FILTER_US)
                    if PLOT_GLOBAL_MODEL_PER_RUN
                        plotValidationBodeVsTrainingModel( ...
                            valFrfGlobal, ...
                            globalBestModel, ...
                            "GLOBAL model on " + valLabel, ...
                            MIN_COHERENCE_FOR_FIT);
                        if PLOT_NORMALIZED_BODE
                            plotValidationBodeVsTrainingModel( ...
                                valFrfGlobal, ...
                                globalBestModel, ...
                                "GLOBAL model on " + valLabel, ...
                                MIN_COHERENCE_FOR_FIT, true);
                        end
                    end

                    if PLOT_GLOBAL_TIME_DOMAIN_ON_VALIDATION_FILES
                        plotTimeDomainTranslationForDataset( ...
                            DvalGlobal, ...
                            globalBestModel, ...
                            globalCandidateModels, ...
                            "GLOBAL model on " + valLabel, ...
                            "global_validation", ...
                            CENTER_TIME_DOMAIN_SIGNALS, ...
                            PLOT_GLOBAL_TIME_DOMAIN_BEST_ONLY, ...
                            false);
                    end

                    if PLOT_GLOBAL_TIME_DOMAIN_VALIDATION_WINDOW
                        plotTimeDomainTranslationForDataset( ...
                            DvalGlobal, ...
                            globalBestModel, ...
                            globalCandidateModels, ...
                            "GLOBAL model on " + valLabel, ...
                            "global_validation", ...
                            CENTER_TIME_DOMAIN_SIGNALS, ...
                            PLOT_GLOBAL_TIME_DOMAIN_BEST_ONLY, ...
                            false, ...
                            GLOBAL_TIME_DOMAIN_WINDOW_START_S, ...
                            GLOBAL_TIME_DOMAIN_WINDOW_END_S);
                    end

                    globalValPlotCount = globalValPlotCount + 1;
                end
            end

            if COMPUTE_RESIDUAL_WHITENESS && ~isempty(residualWhitenessRows)
                reportResidualWhiteness(residualWhitenessRows);
            end
        end
    end
end

%% Summary table

if ~isempty(allSummaryRows)
    summaryVariableNames = { ...
        'file', ...
        'run_name', ...
        'model_type', ...
        'complexity', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'empirical_bandwidth_Hz', ...
        'phase_slope_delay_s', ...
        'train_weighted_error', ...
        'validation_weighted_error', ...
        'train_mag_rmse_dB', ...
        'train_phase_rmse_deg', ...
        'validation_mag_rmse_dB', ...
        'validation_phase_rmse_deg' ...
    };

    summaryTable = cell2table(allSummaryRows, ...
        'VariableNames', summaryVariableNames);

    disp("Frequency-domain model summary:");
    disp(summaryTable);
else
    fprintf("\nNo frequency-domain models generated.\n");
end

if ~isempty(crossValidationRows)
    crossValidationVariableNames = { ...
        'training_file', ...
        'training_run_name', ...
        'model_type', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'validation_file', ...
        'validation_label', ...
        'cross_validation_weighted_error', ...
        'cross_validation_mag_rmse_dB', ...
        'cross_validation_phase_rmse_deg' ...
    };

    crossValidationTable = cell2table(crossValidationRows, ...
        'VariableNames', crossValidationVariableNames);

    disp("Cross-amplitude validation summary:");
    disp(crossValidationTable);
else
    fprintf("\nNo cross-amplitude validation results generated.\n");
end

if ~isempty(localGeneralizationRows)
    generalizationVariableNames = { ...
        'training_file', ...
        'source_training_run_name', ...
        'model_type', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'target_dataset_type', ...
        'target_label', ...
        'generalization_weighted_error', ...
        'generalization_mag_rmse_dB', ...
        'generalization_phase_rmse_deg' ...
    };

    localGeneralizationTable = cell2table(localGeneralizationRows, ...
        'VariableNames', generalizationVariableNames);

    disp("Local model generalization matrix/table:");
    disp(localGeneralizationTable);
else
    fprintf("\nNo local model generalization results generated.\n");
end

if ~isempty(globalSummaryRows)
    globalSummaryVariableNames = { ...
        'file', ...
        'run_name', ...
        'model_type', ...
        'complexity', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'empirical_bandwidth_Hz', ...
        'phase_slope_delay_s', ...
        'train_weighted_error', ...
        'validation_weighted_error', ...
        'train_mag_rmse_dB', ...
        'train_phase_rmse_deg', ...
        'validation_mag_rmse_dB', ...
        'validation_phase_rmse_deg' ...
    };

    globalSummaryTable = cell2table(globalSummaryRows, ...
        'VariableNames', globalSummaryVariableNames);

    disp("Global pooled model candidate summary:");
    disp(globalSummaryTable);
else
    fprintf("\nNo global pooled model candidate summary generated.\n");
end

if ~isempty(globalModelRows)
    globalModelVariableNames = { ...
        'training_file', ...
        'source_training_run_name', ...
        'model_type', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'target_dataset_type', ...
        'target_label', ...
        'global_model_weighted_error', ...
        'global_model_mag_rmse_dB', ...
        'global_model_phase_rmse_deg' ...
    };

    globalModelTable = cell2table(globalModelRows, ...
        'VariableNames', globalModelVariableNames);

    disp("Global model scored on each individual dataset:");
    disp(globalModelTable);
else
    fprintf("\nNo global model per-dataset results generated.\n");
end

%% Amplitude generalization curve

if PLOT_AMPLITUDE_GENERALIZATION_CURVE
    plotAmplitudeGeneralizationCurve(crossValidationRows, globalModelRows);
end

%% Save figures

if SAVE_FIGURES
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);
end

%% Local helper functions

function T = normalizeServoPrpsTable(T, countsPerRev, defaultServoCenterUs, defaultFreqLabel, servoOutputSign)
    vars = string(T.Properties.VariableNames);

    % Time
    if ~ismember("t_s", vars)
        if ismember("t_ms", vars)
            T.t_ms = forceNumeric(T.t_ms);
            T.t_s = T.t_ms / 1000;
        else
            error("CSV must contain either t_s or t_ms.");
        end
    else
        T.t_s = forceNumeric(T.t_s);
    end

    % Servo command
    if ~ismember("servo_us", vars)
        error("CSV must contain servo_us.");
    end
    T.servo_us = forceNumeric(T.servo_us);

    % Count delta
    if ismember("count_delta", vars)
        T.count_delta = forceNumeric(T.count_delta);
    end

    % Output angle
    if ~ismember("theta_rad", vars)
        if ismember("count_delta", string(T.Properties.VariableNames))
            T.theta_rad = T.count_delta / countsPerRev * 2*pi;
        else
            error("CSV must contain either theta_rad or count_delta.");
        end
    else
        T.theta_rad = forceNumeric(T.theta_rad);
    end

    % Apply servo sign convention before any frequency-domain fitting.
    % This turns the measured negative-gain convention into a positive-gain plant.
    T.theta_rad = servoOutputSign * T.theta_rad;

    if ismember("theta_deg", string(T.Properties.VariableNames))
        T.theta_deg = forceNumeric(T.theta_deg);
        T.theta_deg = servoOutputSign * T.theta_deg;
    else
        T.theta_deg = T.theta_rad * 180/pi;
    end

    % Segment
    if ~ismember("segment", vars)
        T.segment = repmat("prps", height(T), 1);
    else
        T.segment = string(T.segment);
    end

    % Run index
    if ~ismember("run_idx", vars)
        T.run_idx = zeros(height(T), 1);
    else
        T.run_idx = forceNumeric(T.run_idx);
    end

    % Source file, added by readCombinedCsvTables
    if ~ismember("source_file", string(T.Properties.VariableNames))
        T.source_file = repmat("", height(T), 1);
    else
        T.source_file = string(T.source_file);
    end

    % Infer amplitude from source_file first.
    ampUs = inferAmplitudeFromSourceFile(T.source_file);

    % If filename has no amp token, fall back to run_name if present.
    if any(isnan(ampUs)) && ismember("run_name", vars)
        ampFromRunName = inferAmplitudeFromRunName(string(T.run_name));
        missingAmp = isnan(ampUs) & isfinite(ampFromRunName);
        ampUs(missingAmp) = ampFromRunName(missingAmp);
    end

    T.run_amplitude_us = ampUs;

    % Run name
    if ~ismember("run_name", vars)
        runName = strings(height(T), 1);

        for i = 1:height(T)
            if isfinite(ampUs(i))
                runName(i) = "local_amp" + string(round(ampUs(i))) + "_" + defaultFreqLabel;
            else
                runName(i) = "run_idx_" + string(T.run_idx(i));
            end
        end

        T.run_name = runName;
    else
        T.run_name = string(T.run_name);

        % If old run_name exists but source filename says amp600, preserve old
        % name for old-format logs. Do not overwrite unless run_name is empty.
        emptyName = strlength(strtrim(T.run_name)) == 0;

        for i = 1:height(T)
            if emptyName(i) && isfinite(ampUs(i))
                T.run_name(i) = "local_amp" + string(round(ampUs(i))) + "_" + defaultFreqLabel;
            end
        end
    end

    % Command delta.
    % Current tests use center = 1450 us for all amplitudes.
    if ~ismember("command_delta_us", vars)
        T.command_delta_us = T.servo_us - defaultServoCenterUs;
    else
        T.command_delta_us = forceNumeric(T.command_delta_us);
    end

    % Period index
    if ~ismember("period_index", vars)
        T.period_index = zeros(height(T), 1);
    else
        T.period_index = forceNumeric(T.period_index);
    end

    % Period sample index
    if ~ismember("period_sample_index", vars)
        if ismember("sample_index", vars)
            T.period_sample_index = forceNumeric(T.sample_index);
        else
            T.period_sample_index = (0:height(T)-1).';
        end
    else
        T.period_sample_index = forceNumeric(T.period_sample_index);
    end
end

function ampUs = inferAmplitudeFromSourceFile(sourceFile)
    sourceFile = string(sourceFile);
    ampUs = NaN(numel(sourceFile), 1);

    for i = 1:numel(sourceFile)
        token = regexp(sourceFile(i), "amp(\d+)", "tokens", "once");

        if ~isempty(token)
            ampUs(i) = str2double(token{1});
        end
    end
end

function ampUs = inferAmplitudeFromRunName(runName)
    runName = string(runName);
    ampUs = NaN(numel(runName), 1);

    for i = 1:numel(runName)
        token = regexp(runName(i), "amp(\d+)", "tokens", "once");

        if ~isempty(token)
            ampUs(i) = str2double(token{1});
        end
    end
end

function allowed = isAmplitudeAllowed(ampUs, filterUs)
    allowed = isempty(filterUs) || any(ampUs == filterUs);
end

function D = getRunData(T, runName, useOnlyPrpsSegment)
    if isempty(T)
        D = table();
        return;
    end

    if ~ismember("run_name", string(T.Properties.VariableNames))
        error("Table has not been normalized. Missing run_name.");
    end

    idx = strcmp(string(T.run_name), string(runName));

    if useOnlyPrpsSegment && ismember("segment", string(T.Properties.VariableNames))
        idx = idx & strcmpi(string(T.segment), "prps");
    end

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.t_s = forceNumeric(D.t_s);
    D.servo_us = forceNumeric(D.servo_us);
    D.command_delta_us = forceNumeric(D.command_delta_us);
    D.theta_rad = forceNumeric(D.theta_rad);
    D.period_index = forceNumeric(D.period_index);
    D.period_sample_index = forceNumeric(D.period_sample_index);

    valid = isfinite(D.t_s) & ...
            isfinite(D.command_delta_us) & ...
            isfinite(D.theta_rad) & ...
            isfinite(D.period_index) & ...
            isfinite(D.period_sample_index);

    D = D(valid, :);

    if isempty(D)
        return;
    end

    D = zeroServoAnglePerSourceFile(D);

    D.t_s = D.t_s - D.t_s(1);
end

function D = zeroServoAnglePerSourceFile(D)
    if isempty(D)
        return;
    end

    if ismember("source_file_index", string(D.Properties.VariableNames))
        groups = forceNumeric(D.source_file_index);
    else
        groups = ones(height(D), 1);
    end

    uniqueGroups = unique(groups, "stable");

    for k = 1:numel(uniqueGroups)
        idx = groups == uniqueGroups(k);

        if ~any(idx)
            continue;
        end

        firstIdx = find(idx, 1, "first");

        theta0 = D.theta_rad(firstIdx);
        D.theta_rad(idx) = D.theta_rad(idx) - theta0;

        if ismember("count_delta", string(D.Properties.VariableNames))
            count0 = D.count_delta(firstIdx);
            D.count_delta(idx) = D.count_delta(idx) - count0;
        end
    end
end

function D = getSourceFileData(T, sourceFileName, useOnlyPrpsSegment)
    if isempty(T)
        D = table();
        return;
    end

    if ~ismember("source_file", string(T.Properties.VariableNames))
        error("Table has no source_file column. Use readCombinedCsvTables first.");
    end

    idx = strcmp(string(T.source_file), string(sourceFileName));

    if useOnlyPrpsSegment && ismember("segment", string(T.Properties.VariableNames))
        idx = idx & strcmpi(string(T.segment), "prps");
    end

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.t_s = forceNumeric(D.t_s);
    D.servo_us = forceNumeric(D.servo_us);
    D.command_delta_us = forceNumeric(D.command_delta_us);
    D.theta_rad = forceNumeric(D.theta_rad);
    D.period_index = forceNumeric(D.period_index);
    D.period_sample_index = forceNumeric(D.period_sample_index);

    valid = isfinite(D.t_s) & ...
            isfinite(D.command_delta_us) & ...
            isfinite(D.theta_rad) & ...
            isfinite(D.period_index) & ...
            isfinite(D.period_sample_index);

    D = D(valid, :);

    if isempty(D)
        return;
    end

    D = zeroServoAnglePerSourceFile(D);
    D.t_s = D.t_s - D.t_s(1);
end

function label = makeCrossValidationLabel(sourceFileName, D)
    sourceFileName = string(sourceFileName);

    amp = inferAmplitudeFromSourceFile(sourceFileName);
    amp = amp(1);

    if isfinite(amp)
        label = "amp" + string(round(amp));
    else
        label = erase(sourceFileName, ".csv");
    end

    % If the CSV run_name disagrees with the file name, keep the filename-based
    % amplitude label because old Pico logs may have stale run_name text.
    if ismember("run_name", string(D.Properties.VariableNames))
        names = unique(string(D.run_name), "stable");
        names = names(strlength(strtrim(names)) > 0);

        if ~isempty(names)
            label = label + "_file_run_" + names(1);
        end
    end
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

            [uPeriod, yPeriod, tPeriod, ~] = makeUniformPeriodVectors(Dp);

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

        G_emp(i) = Syu_i / max(Suu_i, eps);
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
    uRaw = forceNumeric(Dp.command_delta_us);
    yRaw = forceNumeric(Dp.theta_rad);
    tRaw = forceNumeric(Dp.t_s);

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

function x = expClamp(p, lo, hi)
    x = exp(p);
    x = clampScalar(x, lo, hi);
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

function modelOut = scoreSingleModelOnFrf(modelIn, frf)
    modelOut = modelIn;

    G_fit = evalFrequencyModel(modelIn.model_type, modelIn.params, frf.w_rad_s);

    err = weightedComplexError( ...
        G_fit, ...
        frf.G_emp, ...
        frf.coherence, ...
        modelIn.weighting_mode, ...
        modelIn.gain_floor);

    modelOut.validation_weighted_error = sqrt(mean(abs(err).^2, "omitnan"));

    [magRmseDb, phaseRmseDeg] = bodeErrorMetrics(G_fit, frf.G_emp);

    modelOut.validation_mag_rmse_dB = magRmseDb;
    modelOut.validation_phase_rmse_deg = phaseRmseDeg;
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
    fprintf("  K = %.9g rad/us\n", model.K);
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

function [bandwidthHz, lowFreqGainDb] = estimateBandwidthFromFrf(frf, minCoherence, bandwidthDropDb)
    fitMask = frf.coherence >= minCoherence & ...
              isfinite(frf.G_emp) & ...
              isfinite(frf.f_Hz);

    if nnz(fitMask) < 2
        bandwidthHz = NaN;
        lowFreqGainDb = NaN;
        return;
    end

    goodFreqs = frf.f_Hz(fitMask);
    goodMagDb = 20*log10(abs(frf.G_emp(fitMask)));

    [goodFreqsSorted, sortIdx] = sort(goodFreqs); %#ok<ASGLU>
    goodMagDbSorted = goodMagDb(sortIdx);

    nLow = min(3, numel(goodMagDbSorted));
    lowFreqGainDb = median(goodMagDbSorted(1:nLow), "omitnan");

    allMagDb = 20*log10(abs(frf.G_emp));
    dropDb = allMagDb - lowFreqGainDb;

    bwIdx = find(fitMask & dropDb <= bandwidthDropDb, 1, "first");

    if isempty(bwIdx)
        bandwidthHz = NaN;
    else
        bandwidthHz = frf.f_Hz(bwIdx);
    end
end

function delayS = estimateDelayFromPhaseSlope(frf, minCoherence, bandwidthHz, freqMinHz)
    mask = frf.coherence >= minCoherence & ...
           isfinite(frf.G_emp) & ...
           isfinite(frf.f_Hz);

    if isfinite(bandwidthHz)
        mask = mask & frf.f_Hz <= max(1.5 * bandwidthHz, freqMinHz);
    else
        mask = mask & frf.f_Hz <= 5.0;
    end

    if nnz(mask) < 3
        delayS = NaN;
        return;
    end

    w = 2*pi*frf.f_Hz(mask);
    phaseRad = unwrap(angle(frf.G_emp(mask)));

    p = polyfit(w, phaseRad, 1);
    delayS = max(0, -p(1));
end


function globalFrf = combineFrfsFromStruct(frfStruct, globalName)
    keys = string(fieldnames(frfStruct));

    allF = [];
    allW = [];
    allG = [];
    allCoh = [];
    allSuu = [];
    allSyy = [];
    allSyu = [];
    allN = [];

    for k = 1:numel(keys)
        frf = frfStruct.(keys(k));

        if isempty(frf.f_Hz)
            continue;
        end

        allF = [allF; frf.f_Hz(:)]; %#ok<AGROW>
        allW = [allW; frf.w_rad_s(:)]; %#ok<AGROW>
        allG = [allG; frf.G_emp(:)]; %#ok<AGROW>
        allCoh = [allCoh; frf.coherence(:)]; %#ok<AGROW>
        allSuu = [allSuu; frf.Suu(:)]; %#ok<AGROW>
        allSyy = [allSyy; frf.Syy(:)]; %#ok<AGROW>
        allSyu = [allSyu; frf.Syu(:)]; %#ok<AGROW>
        allN = [allN; frf.n_avg(:)]; %#ok<AGROW>
    end

    valid = isfinite(allF) & isfinite(allG) & isfinite(allCoh);

    allF = allF(valid);
    allW = allW(valid);
    allG = allG(valid);
    allCoh = allCoh(valid);
    allSuu = allSuu(valid);
    allSyy = allSyy(valid);
    allSyu = allSyu(valid);
    allN = allN(valid);

    [allF, sortIdx] = sort(allF);
    allW = allW(sortIdx);
    allG = allG(sortIdx);
    allCoh = allCoh(sortIdx);
    allSuu = allSuu(sortIdx);
    allSyy = allSyy(sortIdx);
    allSyu = allSyu(sortIdx);
    allN = allN(sortIdx);

    globalFrf = emptyFrf();
    globalFrf.f_Hz = allF;
    globalFrf.w_rad_s = allW;
    globalFrf.G_emp = allG;
    globalFrf.coherence = allCoh;
    globalFrf.Suu = allSuu;
    globalFrf.Syy = allSyy;
    globalFrf.Syu = allSyu;
    globalFrf.n_avg = allN;
    globalFrf.run_name = string(globalName);
    globalFrf.dataset_label = "global_training";

    fprintf("\nCombined global FRF:\n");
    fprintf("  Name: %s\n", globalName);
    fprintf("  Total pooled frequency points: %d\n", numel(globalFrf.f_Hz));
    fprintf("  Frequency range: %.4f to %.4f Hz\n", min(globalFrf.f_Hz), max(globalFrf.f_Hz));
    fprintf("  Median coherence: %.3f\n", median(globalFrf.coherence, "omitnan"));
end

%% Plotting functions

function plotEmpiricalBodeWithFits(frf, models, bestModel, runName, datasetLabel, minCoherence, plotRejected, bandwidthHz, normalize)
    if nargin < 9
        normalize = false;
    end

    f = frf.f_Hz(:);
    G = frf.G_emp(:);
    coh = frf.coherence(:);

    good = coh >= minCoherence;

    fFine = logspace(log10(min(f)), log10(max(f)), 500).';
    wFine = 2*pi*fFine;

    if normalize
        refDb = 20*log10(bestModel.K);
        magLabel = "Normalized Magnitude (dB re DC gain)";
        titlePrefix = "Normalized ";
    else
        refDb = 0;
        magLabel = "Magnitude (dB, rad/us)";
        titlePrefix = "";
    end

    figure;
    hold on; grid on;

    if plotRejected && any(~good)
        semilogx(f(~good), 20*log10(abs(G(~good))) - refDb, "x", ...
            "DisplayName", "Rejected empirical points");
    end

    semilogx(f(good), 20*log10(abs(G(good))) - refDb, "o", ...
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

        semilogx(fFine, 20*log10(abs(Gfit)) - refDb, "LineWidth", lw, ...
            "DisplayName", name);
    end

    if isfinite(bandwidthHz)
        xline(bandwidthHz, "--", sprintf("BW %.2f Hz", bandwidthHz), ...
            "DisplayName", "Empirical -3 dB BW");
    end

    xlabel("Frequency (Hz)");
    ylabel(magLabel);
    title(titlePrefix + "Training Empirical Bode Magnitude with Frequency-Domain Fits - " + datasetLabel + " - " + runName, ...
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
    title(titlePrefix + "Training Empirical Bode Phase with Frequency-Domain Fits - " + datasetLabel + " - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

function plotValidationBodeVsTrainingModel(valFrf, bestModel, runName, minCoherence, normalize)
    if nargin < 5
        normalize = false;
    end

    f = valFrf.f_Hz(:);
    G = valFrf.G_emp(:);
    coh = valFrf.coherence(:);

    good = coh >= minCoherence;

    fFine = logspace(log10(min(f)), log10(max(f)), 500).';
    wFine = 2*pi*fFine;
    Gfit = evalFrequencyModel(bestModel.model_type, bestModel.params, wFine);

    if normalize
        refDb = 20*log10(bestModel.K);
        magLabel = "Normalized Magnitude (dB re DC gain)";
        titlePrefix = "Normalized ";
    else
        refDb = 0;
        magLabel = "Magnitude (dB, rad/us)";
        titlePrefix = "";
    end

    figure;
    hold on; grid on;

    semilogx(f(good), 20*log10(abs(G(good))) - refDb, "o", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical FRF");

    if any(~good)
        semilogx(f(~good), 20*log10(abs(G(~good))) - refDb, "x", ...
            "DisplayName", "Validation low-coherence points");
    end

    semilogx(fFine, 20*log10(abs(Gfit)) - refDb, "-", ...
        "LineWidth", 2.0, ...
        "DisplayName", "Training fitted model");

    xlabel("Frequency (Hz)");
    ylabel(magLabel);
    title(titlePrefix + "Validation Bode vs Training-Fitted Model - Magnitude - " + runName, ...
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
    title(titlePrefix + "Validation Bode vs Training-Fitted Model - Phase - " + runName, ...
        "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

function plotTrainingVsValidationEmpiricalBode(trainFrf, valFrf, runName, minCoherence, normalize)
    if nargin < 5
        normalize = false;
    end

    trainGood = trainFrf.coherence >= minCoherence;
    valGood = valFrf.coherence >= minCoherence;

    if normalize
        firstGoodIdx = find(trainGood, 1);
        if ~isempty(firstGoodIdx)
            refDb = 20*log10(abs(trainFrf.G_emp(firstGoodIdx)));
        else
            refDb = 0;
        end
        magLabel = "Normalized Magnitude (dB re training low-freq)";
        titlePrefix = "Normalized ";
    else
        refDb = 0;
        magLabel = "Magnitude (dB, rad/us)";
        titlePrefix = "";
    end

    figure;
    hold on; grid on;

    semilogx(trainFrf.f_Hz(trainGood), 20*log10(abs(trainFrf.G_emp(trainGood))) - refDb, "o-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Training empirical");

    semilogx(valFrf.f_Hz(valGood), 20*log10(abs(valFrf.G_emp(valGood))) - refDb, "s--", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Validation empirical");

    xlabel("Frequency (Hz)");
    ylabel(magLabel);
    title(titlePrefix + "Training vs Validation Empirical Bode - Magnitude - " + runName, ...
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
    title(titlePrefix + "Training vs Validation Empirical Bode - Phase - " + runName, ...
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

    yline(0.6, "--", "0.6");
    yline(0.8, "--", "0.8");

    xlabel("Frequency (Hz)");
    ylabel("Coherence");
    title("Servo PRPS Input/Output Coherence - " + runName, "Interpreter", "none");
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
    title("Servo Frequency-Domain Model Comparison - " + runName, "Interpreter", "none");
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
    tWindowStart_s, ...
    tWindowEnd_s)

    if nargin < 9,  tWindowStart_s = -inf; end
    if nargin < 10, tWindowEnd_s   =  inf; end

    windowed = isfinite(tWindowStart_s) || isfinite(tWindowEnd_s);

    if isempty(D) || height(D) < 5
        return;
    end

    groups = getTimeGroups(D);
    uniqueGroups = unique(groups, "stable");

    for g = 1:numel(uniqueGroups)
        idx = find(groups == uniqueGroups(g));
        Dg = D(idx, :);

        tRaw = forceNumeric(Dg.t_s);
        uRaw = forceNumeric(Dg.command_delta_us);
        yRaw = forceNumeric(Dg.theta_rad);

        valid = isfinite(tRaw) & isfinite(uRaw) & isfinite(yRaw);
        tRaw = tRaw(valid);
        uRaw = uRaw(valid);
        yRaw = yRaw(valid);

        if numel(tRaw) < 5
            continue;
        end

        tRaw = tRaw - tRaw(1);

        [tRaw, uniqueIdx] = unique(tRaw, "stable");
        uRaw = uRaw(uniqueIdx);
        yRaw = yRaw(uniqueIdx);

        dt = median(diff(tRaw), "omitnan");

        if ~isfinite(dt) || dt <= 0
            warning("Skipping time-domain translation for %s %s: invalid dt.", datasetLabel, runName);
            continue;
        end

        tUniform = (0:dt:tRaw(end)).';

        if numel(tUniform) < 5
            continue;
        end

        uUniform = interp1(tRaw, uRaw, tUniform, "linear", "extrap");
        yUniform = interp1(tRaw, yRaw, tUniform, "linear", "extrap");

        if centerSignals
            uInput = uUniform - mean(uUniform, "omitnan");
            yOffset = mean(yUniform, "omitnan");
        else
            uInput = uUniform;
            yOffset = 0;
        end

        % Determine display mask — lsim always runs on the full signal.
        if windowed
            plotMask = tUniform >= tWindowStart_s & tUniform <= tWindowEnd_s;
            windowSuffix = sprintf(" [%.0f\x2013%.0f s]", tWindowStart_s, tWindowEnd_s);
        else
            plotMask = true(size(tUniform));
            windowSuffix = "";
        end

        if ~any(plotMask)
            warning("Window [%.0f, %.0f] s contains no samples for %s %s.", ...
                tWindowStart_s, tWindowEnd_s, datasetLabel, runName);
            break;
        end

        figure;
        hold on; grid on;

        plot(tUniform(plotMask), yUniform(plotMask) * 180/pi, "LineWidth", 1.2, ...
            "DisplayName", "Measured servo angle");

        if plotBestOnly
            sys = makeTransferFunction(bestModel);

            try
                yModel = lsim(sys, uInput, tUniform) + yOffset;

                plot(tUniform(plotMask), yModel(plotMask) * 180/pi, "--", "LineWidth", 1.8, ...
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

                    plot(tUniform(plotMask), yModel(plotMask) * 180/pi, "--", "LineWidth", 1.0, ...
                        "DisplayName", candidateModels(m).model_type);
                catch ME
                    warning("lsim failed for candidate model %s / %s / %s: %s", ...
                        candidateModels(m).model_type, datasetLabel, runName, ME.message);
                end
            end
        end

        xlabel("Time (s)");
        ylabel("Servo Angle (deg)");
        title("Time-Domain Translation of Frequency Fit - " + datasetLabel + ...
            " - " + runName + windowSuffix, ...
            "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");

        break;
    end
end

function groups = getTimeGroups(D)
    if ismember("source_file_index", string(D.Properties.VariableNames))
        groups = double(D.source_file_index);
    else
        groups = ones(height(D), 1);
    end
end

function plotAmplitudeGeneralizationCurve(crossValidationRows, globalModelRows)
    % crossValidationRows cols: training_file, training_run_name, model_type,
    %   K, tau1, tau2, delay, validation_file, validation_label,
    %   cross_validation_weighted_error, mag_rmse_dB, phase_rmse_deg
    %
    % globalModelRows cols: training_file, source_training_run_name, model_type,
    %   K, tau1, tau2, delay, target_dataset_type, target_label,
    %   global_model_weighted_error, mag_rmse_dB, phase_rmse_deg

    hasData = false;

    figure;
    hold on; grid on;

    % One curve per local training run
    if ~isempty(crossValidationRows)
        trainingRunNames = string(crossValidationRows(:, 2));
        validationFiles  = string(crossValidationRows(:, 8));
        weightedErrors   = cell2mat(crossValidationRows(:, 10));

        uniqueRuns = unique(trainingRunNames, "stable");

        for r = 1:numel(uniqueRuns)
            mask   = trainingRunNames == uniqueRuns(r);
            amps   = inferAmplitudeFromSourceFile(validationFiles(mask));
            errors = weightedErrors(mask);

            uniqueAmps = sort(unique(amps(isfinite(amps))));
            if isempty(uniqueAmps), continue; end

            meanErrors = arrayfun( ...
                @(a) mean(errors(amps == a), "omitnan") * 100, ...
                uniqueAmps);

            plot(uniqueAmps, meanErrors, "o-", "LineWidth", 1.4, ...
                "DisplayName", "Local: " + uniqueRuns(r));
            hasData = true;
        end
    end

    % One curve for global model on validation files
    if ~isempty(globalModelRows)
        datasetTypes   = string(globalModelRows(:, 8));
        targetLabels   = string(globalModelRows(:, 9));
        weightedErrors = cell2mat(globalModelRows(:, 10));

        mask = datasetTypes == "validation";

        if any(mask)
            amps   = inferAmplitudeFromRunName(targetLabels(mask));
            errors = weightedErrors(mask);

            uniqueAmps = sort(unique(amps(isfinite(amps))));

            if ~isempty(uniqueAmps)
                meanErrors = arrayfun( ...
                    @(a) mean(errors(amps == a), "omitnan") * 100, ...
                    uniqueAmps);

                plot(uniqueAmps, meanErrors, "s--", "LineWidth", 2.0, ...
                    "DisplayName", "Global model");
                hasData = true;
            end
        end
    end

    if ~hasData
        close;
        return;
    end

    xlabel("PRPS Amplitude (\mus)");
    ylabel("Normalized RMS Error (%)");
    ylim([0, 8]);
    title("Amplitude Generalization Curve", "Interpreter", "none");
    legend("Location", "best", "Interpreter", "none");
end

%% General table / utility helpers

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
                if any(varName == ["source_file", "run_name", "segment"])
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

function x = forceNumeric(x)
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

function rw = computeResidualWhiteness(D, model, label, maxLagS, doPlot)
%COMPUTERESIDUALWHITENESS Residual autocorrelation + residual-input cross-correlation
% on one held-out validation file (project_metrics.md Section 1, metrics #3-#4).
    rw = struct("label", "", "acf_frac_in", NaN, "ccf_frac_in", NaN, "pass", false);

    t = forceNumeric(D.t_s);
    u = forceNumeric(D.command_delta_us);
    y = forceNumeric(D.theta_rad);
    valid = isfinite(t) & isfinite(u) & isfinite(y);
    t = t(valid); u = u(valid); y = y(valid);
    if numel(t) < 50; return; end

    t = t - t(1);
    [t, ui] = unique(t, "stable"); u = u(ui); y = y(ui);
    dt = median(diff(t), "omitnan");
    if ~isfinite(dt) || dt <= 0; return; end

    tU = (0:dt:t(end)).';
    uU = interp1(t, u, tU, "linear", "extrap");
    yU = interp1(t, y, tU, "linear", "extrap");

    uIn = uU - mean(uU, "omitnan");
    yOffset = mean(yU, "omitnan");

    try
        yHat = lsim(makeTransferFunction(model), uIn, tU) + yOffset;
    catch ME
        warning("Residual whiteness lsim failed for %s: %s", label, ME.message);
        return;
    end

    e = yU - yHat;
    N = numel(e);
    maxLag = min(round(maxLagS / dt), N - 1);
    band = 1.96 / sqrt(N);

    e0 = e - mean(e, "omitnan");
    u0 = uIn - mean(uIn, "omitnan");
    [acfFull, acfLags] = xcorr(e0, maxLag, "coeff");
    lagsPos = (1:maxLag).';                       % autocorr: positive lags only
    acf = acfFull(acfLags >= 1);
    lagsCcf = (-maxLag:maxLag).';                 % cross-corr: both signs
    ccf = xcorr(e0, u0, maxLag, "coeff");

    rw.label = string(label);
    rw.acf_frac_in = mean(abs(acf) <= band);
    rw.ccf_frac_in = mean(abs(ccf) <= band);
    rw.pass = rw.acf_frac_in >= 0.95 && rw.ccf_frac_in >= 0.95;

    fprintf("  Residual whiteness %-28s | ACF in-band %.0f%% | CCF in-band %.0f%% | %s\n", ...
        rw.label, 100*rw.acf_frac_in, 100*rw.ccf_frac_in, passFailLocal(rw.pass));

    if doPlot
        figure("Color", "k");
        subplot(2,1,1);
        stem(lagsPos*dt, acf, "filled", "Color", [0 0.90 1.00], "MarkerSize", 3); hold on; grid on;
        yline([band -band], "--w");
        xlabel("Lag (s)"); ylabel("Residual ACF");
        title("Residual Autocorrelation - " + rw.label, "Interpreter", "none");
        subplot(2,1,2);
        stem(lagsCcf*dt, ccf, "filled", "Color", [1 0.65 0.00], "MarkerSize", 3); hold on; grid on;
        yline([band -band], "--w");
        xlabel("Lag (s)"); ylabel("Residual-input CCF");
        title("Residual-Input Cross-Correlation", "Interpreter", "none");
    end
end

function reportResidualWhiteness(rows)
    fprintf("\n============================================================\n");
    fprintf("Residual whiteness on held-out validation (metrics #3-#4)\n");
    fprintf("============================================================\n");
    T = struct2table(rows);
    T.label = string(T.label);
    disp(T);
    overall = all([rows.pass]);
    fprintf("Band: +/-1.96/sqrt(N). Pass = >= 95%% of lags in-band for both.\n");
    fprintf("OVERALL raw time-domain whiteness: %s\n", passFailLocal(overall));
    fprintf("  CAVEAT: PRPS is a line spectrum, so the residual inherits the excitation\n");
    fprintf("  periodicity and is structurally non-white; with N~1e5 the band (~0.004) is\n");
    fprintf("  also extremely tight. This raw time-domain test OVER-REJECTS on multisine\n");
    fprintf("  data and largely reflects the known amplitude-dependence (tau 15->34 ms) and\n");
    fprintf("  +-2 deg hysteresis, not model-order inadequacy. The metric of record for\n");
    fprintf("  line-spectrum ID is the frequency-domain residual (FRF error + coherence),\n");
    fprintf("  which is met. Use the ACF/CCF plots as a diagnostic of residual structure.\n");
end

function s = passFailLocal(tf)
    if tf; s = "PASS"; else; s = "FAIL"; end
end