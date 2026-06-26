%% bootstrap_servo_fopd_parameters.m
% Bootstrap the servo PRPS data to estimate a first-order-plus-delay
% parameter cloud for downstream uncertainty sweeps.

clearvars -except N_BOOT RESAMPLE_MODE DATASET_STATUS STRATIFY_BY_RUN RNG_SEED SAVE_FIGURES;
clc; close all;

%% User options

if ~exist("N_BOOT", "var"), N_BOOT = 500; end
if ~exist("RESAMPLE_MODE", "var"), RESAMPLE_MODE = "period"; end % "period" or "episode"
if ~exist("DATASET_STATUS", "var"), DATASET_STATUS = "candidate"; end
if ~exist("STRATIFY_BY_RUN", "var"), STRATIFY_BY_RUN = true; end
if ~exist("RNG_SEED", "var"), RNG_SEED = 1; end
if ~exist("SAVE_FIGURES", "var"), SAVE_FIGURES = true; end

RESAMPLE_MODE = string(RESAMPLE_MODE);
DATASET_STATUS = string(DATASET_STATUS);

%% Servo and PRPS settings

COUNTS_PER_REV = encoderAngleScale().counts_per_rev;   % spec-derived (1:1 GT2 16T)
DEFAULT_SERVO_CENTER_US = 1450;
SERVO_OUTPUT_SIGN = -1;
DEFAULT_FREQ_LABEL = "0.1_to_3Hz";

FRF_OPTS = struct();
FRF_OPTS.inferFreqsFromInput = true;
FRF_OPTS.freqListHz = [];
FRF_OPTS.inputBinRelativeThreshold = 0.05;
FRF_OPTS.freqMinHz = 0.08;
FRF_OPTS.freqMaxHz = 3.25;
FRF_OPTS.removePeriodMean = true;
FRF_OPTS.detrendEachPeriod = false;
FRF_OPTS.minPeriodsPerRun = 2;
FRF_OPTS.minSamplesPerPeriod = 20;
FRF_OPTS.useNominalCommandDtForFrf = true;
FRF_OPTS.nominalCommandDtS = 0.010;

FIT_OPTS = struct();
FIT_OPTS.weightingMode = "relative_with_coh";
FIT_OPTS.gainFloorMode = "fraction_of_median";
FIT_OPTS.gainFloorFractionOfMedian = 0.20;
FIT_OPTS.manualGainFloor = 1e-5;
FIT_OPTS.initialK = pi / 2000;
FIT_OPTS.initialTau = 0.08;
FIT_OPTS.initialDelay = 0.02;
FIT_OPTS.initialTau2 = 0.02;
FIT_OPTS.minTau = 0.005;
FIT_OPTS.maxTau = 2.0;
FIT_OPTS.minDelay = 0.0;
FIT_OPTS.maxDelay = 0.50;
FIT_OPTS.minGainAbs = 1e-5;
FIT_OPTS.maxGainAbs = 0.01;
FIT_OPTS.maxIter = 5000;
FIT_OPTS.maxEval = 10000;

MIN_COHERENCE_FOR_FIT = 0.60;

%% Locate data and output folders

scriptPath = mfilename("fullpath");
repoRoot = findRepoRoot(scriptPath);
addpath(fullfile(repoRoot, "analysis", "utils"));

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = fullfile(getMirroredPlotDir(scriptPath), "bootstrap_fopd");
reportDir = strrep(plotDir, fullfile(repoRoot, "plots"), fullfile(repoRoot, "reports"));

trainSubDir = fullfile(dataDir, "training");
valSubDir = fullfile(dataDir, "validation");

if ~isfolder(trainSubDir)
    error("Training folder not found:\n%s", trainSubDir);
end

trainingCsvFiles = listSortedCsvFiles(trainSubDir);
validationCsvFiles = listSortedCsvFiles(valSubDir);

if isempty(trainingCsvFiles)
    error("No training CSV files found in:\n%s", trainSubDir);
end

if ~exist(reportDir, "dir")
    mkdir(reportDir);
end

if ~exist(plotDir, "dir")
    mkdir(plotDir);
end

fprintf("\nServo FOPD bootstrap\n");
fprintf("  Dataset: %s\n", dataDir);
fprintf("  Training files: %d\n", numel(trainingCsvFiles));
fprintf("  Validation files: %d\n", numel(validationCsvFiles));
fprintf("  Resampling mode: %s\n", RESAMPLE_MODE);
fprintf("  Bootstrap draws: %d\n", N_BOOT);

%% Load and normalize data

Ttrain = ServoPrpsId.readCombinedCsvTables(trainingCsvFiles);
Ttrain = ServoPrpsId.normalizeServoPrpsTable( ...
    Ttrain, ...
    COUNTS_PER_REV, ...
    DEFAULT_SERVO_CENTER_US, ...
    DEFAULT_FREQ_LABEL, ...
    SERVO_OUTPUT_SIGN);

if isempty(validationCsvFiles)
    Tval = table();
else
    Tval = ServoPrpsId.readCombinedCsvTables(validationCsvFiles);
    Tval = ServoPrpsId.normalizeServoPrpsTable( ...
        Tval, ...
        COUNTS_PER_REV, ...
        DEFAULT_SERVO_CENTER_US, ...
        DEFAULT_FREQ_LABEL, ...
        SERVO_OUTPUT_SIGN);
end

units = buildBootstrapUnits(Ttrain, RESAMPLE_MODE, FRF_OPTS.minSamplesPerPeriod);

if isempty(units)
    error("No bootstrap units were created. Check period_index, segment, and sample counts.");
end

unitMeta = unitsToTable(units);
runNames = unique(string(unitMeta.run_name), "stable");

fprintf("  Bootstrap units: %d\n", numel(units));
fprintf("  Runs/amplitudes:\n");
for k = 1:numel(runNames)
    fprintf("    %s: %d units\n", runNames(k), nnz(string(unitMeta.run_name) == runNames(k)));
end

if isempty(Tval)
    validationFrf = ServoPrpsId.emptyFrf();
else
    Dval = ServoPrpsId.cleanServoPrpsTable(Tval);
    validationFrf = ServoPrpsId.estimatePrpsFrfFromTable( ...
        Dval, ...
        "global_validation", ...
        "validation", ...
        FRF_OPTS);
end

%% Bootstrap loop

rng(RNG_SEED, "twister");

sampleRows = cell(N_BOOT, 15);
sampledUnitIds = strings(N_BOOT, 1);
sampledUnitIndices = cell(N_BOOT, 1);
bootstrapFrfs = cell(N_BOOT, 1);

for b = 1:N_BOOT
    sampledIdx = drawBootstrapUnitIndices(unitMeta, runNames, STRATIFY_BY_RUN);
    sampledUnitIndices{b} = sampledIdx(:);
    sampledUnitIds(b) = strjoin(string(unitMeta.unit_id(sampledIdx)), ";");

    syntheticTrain = assembleSyntheticTrainingTable(units, sampledIdx, RESAMPLE_MODE);

    try
        trainFrf = ServoPrpsId.estimatePrpsFrfFromTable( ...
            syntheticTrain, ...
            "global_bootstrap_" + string(b), ...
            "bootstrap_training", ...
            FRF_OPTS);

        bootstrapFrfs{b} = trainFrf;

        fitMask = trainFrf.coherence >= MIN_COHERENCE_FOR_FIT & ...
                  isfinite(trainFrf.G_emp) & ...
                  isfinite(trainFrf.f_Hz) & ...
                  trainFrf.f_Hz >= FRF_OPTS.freqMinHz & ...
                  trainFrf.f_Hz <= FRF_OPTS.freqMaxHz;

        if nnz(fitMask) < 2
            error("Fewer than 2 usable FRF points after coherence filtering.");
        end

        model = ServoPrpsId.fitOneFrequencyModel( ...
            "first_order_delay", ...
            trainFrf.f_Hz(fitMask), ...
            trainFrf.G_emp(fitMask), ...
            trainFrf.coherence(fitMask), ...
            FIT_OPTS);

        model = ServoPrpsId.scoreSingleModelOnFrf(model, validationFrf);

        sampleRows(b, :) = {
            b, ...
            char(RESAMPLE_MODE), ...
            "ok", ...
            "", ...
            model.K, ...
            model.tau1_s, ...
            model.delay_s, ...
            model.train_weighted_error, ...
            model.train_mag_rmse_dB, ...
            model.train_phase_rmse_deg, ...
            model.validation_weighted_error, ...
            model.validation_mag_rmse_dB, ...
            model.validation_phase_rmse_deg, ...
            nnz(fitMask), ...
            sampledUnitIds(b) ...
        };
    catch ME
        sampleRows(b, :) = {
            b, ...
            char(RESAMPLE_MODE), ...
            "failed", ...
            string(ME.message), ...
            NaN, NaN, NaN, ...
            NaN, NaN, NaN, ...
            NaN, NaN, NaN, ...
            0, ...
            sampledUnitIds(b) ...
        };
        bootstrapFrfs{b} = ServoPrpsId.emptyFrf();
        warning("Bootstrap draw %d failed: %s", b, ME.message);
    end

    if mod(b, max(1, floor(N_BOOT/10))) == 0 || b == N_BOOT
        fprintf("  Completed %d / %d bootstrap draws\n", b, N_BOOT);
    end
end

samples = cell2table(sampleRows, 'VariableNames', { ...
    'bootstrap_index', ...
    'resample_mode', ...
    'fit_status', ...
    'fit_message', ...
    'K_rad_per_us', ...
    'tau_s', ...
    'delay_s', ...
    'train_weighted_error', ...
    'train_mag_rmse_dB', ...
    'train_phase_rmse_deg', ...
    'validation_weighted_error', ...
    'validation_mag_rmse_dB', ...
    'validation_phase_rmse_deg', ...
    'n_fit_points', ...
    'sampled_unit_ids' ...
});

validMask = string(samples.fit_status) == "ok" & ...
            isfinite(samples.K_rad_per_us) & samples.K_rad_per_us > 0 & ...
            isfinite(samples.tau_s) & samples.tau_s > 0 & ...
            isfinite(samples.delay_s) & samples.delay_s >= 0;

validSamples = samples(validMask, :);
summary = summarizeBootstrapSamples(validSamples);

%% Save outputs

samplesCsvPath = fullfile(plotDir, "bootstrap_parameter_samples.csv");
summaryCsvPath = fullfile(plotDir, "bootstrap_parameter_summary.csv");
matPath = fullfile(plotDir, "bootstrap_parameter_samples.mat");
reportPath = fullfile(reportDir, "bootstrap_servo_fopd_parameters.report.md");

writetable(samples, samplesCsvPath);
writetable(summary, summaryCsvPath);
save(matPath, ...
    "samples", ...
    "validSamples", ...
    "summary", ...
    "unitMeta", ...
    "sampledUnitIndices", ...
    "bootstrapFrfs", ...
    "validationFrf", ...
    "FRF_OPTS", ...
    "FIT_OPTS", ...
    "RESAMPLE_MODE", ...
    "DATASET_STATUS", ...
    "STRATIFY_BY_RUN", ...
    "RNG_SEED", ...
    "trainingCsvFiles", ...
    "validationCsvFiles");

plotParameterHistograms(validSamples, fullfile(plotDir, "bootstrap_parameter_histograms.png"));
plotParameterPairs(validSamples, fullfile(plotDir, "bootstrap_parameter_pairs.png"));
plotBodeEnvelope(validSamples, validationFrf, fullfile(plotDir, "bootstrap_bode_envelope.png"));

writeBootstrapReport( ...
    reportPath, ...
    samples, ...
    summary, ...
    unitMeta, ...
    trainingCsvFiles, ...
    validationCsvFiles, ...
    dataDir, ...
    RESAMPLE_MODE, ...
    STRATIFY_BY_RUN, ...
    RNG_SEED);

fprintf("\nBootstrap outputs written to:\n  %s\n", plotDir);
fprintf("Report written to:\n  %s\n", reportPath);

%% Local functions

function csvFiles = listSortedCsvFiles(folderPath)
    if ~isfolder(folderPath)
        csvFiles = struct([]);
        return;
    end

    csvFiles = dir(fullfile(folderPath, "*.csv"));

    if isempty(csvFiles)
        return;
    end

    [~, sortIdx] = sort(string({csvFiles.name}));
    csvFiles = csvFiles(sortIdx);
end

function units = buildBootstrapUnits(T, resampleMode, minSamplesPerPeriod)
    resampleMode = string(resampleMode);

    switch resampleMode
        case "episode"
            units = buildEpisodeUnits(T, minSamplesPerPeriod);
        case "period"
            units = buildPeriodUnits(T, minSamplesPerPeriod);
        otherwise
            error("Unknown RESAMPLE_MODE: %s. Use ""period"" or ""episode"".", resampleMode);
    end
end

function units = buildEpisodeUnits(T, minSamplesPerPeriod)
    units = struct("unit_id", {}, "run_name", {}, "source_file", {}, ...
        "source_file_index", {}, "period_index", {}, "data", {});

    fileIds = unique(ServoPrpsId.forceNumeric(T.source_file_index), "stable");

    for f = 1:numel(fileIds)
        fileIdx = fileIds(f);
        Dfile = T(ServoPrpsId.forceNumeric(T.source_file_index) == fileIdx, :);
        runNames = unique(string(Dfile.run_name), "stable");

        for r = 1:numel(runNames)
            Drun = ServoPrpsId.getRunData(Dfile, runNames(r), true);

            if height(Drun) < minSamplesPerPeriod
                continue;
            end

            periodIds = unique(ServoPrpsId.forceNumeric(Drun.period_index), "stable");
            periodIds = periodIds(periodIds >= 0);

            if numel(periodIds) < 2
                continue;
            end

            unit.unit_id = sprintf("episode_file%03d_%s", fileIdx, matlab.lang.makeValidName(runNames(r)));
            unit.run_name = runNames(r);
            unit.source_file = string(Drun.source_file(1));
            unit.source_file_index = fileIdx;
            unit.period_index = NaN;
            unit.data = Drun;
            units(end+1) = unit; %#ok<AGROW>
        end
    end
end

function units = buildPeriodUnits(T, minSamplesPerPeriod)
    units = struct("unit_id", {}, "run_name", {}, "source_file", {}, ...
        "source_file_index", {}, "period_index", {}, "data", {});

    fileIds = unique(ServoPrpsId.forceNumeric(T.source_file_index), "stable");

    for f = 1:numel(fileIds)
        fileIdx = fileIds(f);
        Dfile = T(ServoPrpsId.forceNumeric(T.source_file_index) == fileIdx, :);
        runNames = unique(string(Dfile.run_name), "stable");

        for r = 1:numel(runNames)
            Drun = ServoPrpsId.getRunData(Dfile, runNames(r), true);

            if isempty(Drun)
                continue;
            end

            periodIds = unique(ServoPrpsId.forceNumeric(Drun.period_index), "stable");
            periodIds = periodIds(periodIds >= 0);

            for p = 1:numel(periodIds)
                periodId = periodIds(p);
                Dperiod = Drun(ServoPrpsId.forceNumeric(Drun.period_index) == periodId, :);

                if height(Dperiod) < minSamplesPerPeriod
                    continue;
                end

                unit.unit_id = sprintf("period_file%03d_p%04d_%s", ...
                    fileIdx, periodId, matlab.lang.makeValidName(runNames(r)));
                unit.run_name = runNames(r);
                unit.source_file = string(Dperiod.source_file(1));
                unit.source_file_index = fileIdx;
                unit.period_index = periodId;
                unit.data = Dperiod;
                units(end+1) = unit; %#ok<AGROW>
            end
        end
    end
end

function unitMeta = unitsToTable(units)
    n = numel(units);
    unit_id = strings(n, 1);
    run_name = strings(n, 1);
    source_file = strings(n, 1);
    source_file_index = NaN(n, 1);
    period_index = NaN(n, 1);
    n_rows = NaN(n, 1);

    for i = 1:n
        unit_id(i) = string(units(i).unit_id);
        run_name(i) = string(units(i).run_name);
        source_file(i) = string(units(i).source_file);
        source_file_index(i) = units(i).source_file_index;
        period_index(i) = units(i).period_index;
        n_rows(i) = height(units(i).data);
    end

    unitMeta = table(unit_id, run_name, source_file, source_file_index, period_index, n_rows);
end

function sampledIdx = drawBootstrapUnitIndices(unitMeta, runNames, stratifyByRun)
    if stratifyByRun
        sampledIdx = [];

        for r = 1:numel(runNames)
            idx = find(string(unitMeta.run_name) == string(runNames(r)));
            drawLocal = idx(randi(numel(idx), numel(idx), 1));
            sampledIdx = [sampledIdx; drawLocal(:)]; %#ok<AGROW>
        end
    else
        n = height(unitMeta);
        sampledIdx = randi(n, n, 1);
    end
end

function syntheticTrain = assembleSyntheticTrainingTable(units, sampledIdx, resampleMode)
    tables = cell(numel(sampledIdx), 1);
    resampleMode = string(resampleMode);

    for i = 1:numel(sampledIdx)
        D = units(sampledIdx(i)).data;

        switch resampleMode
            case "period"
                D.source_file_index = ones(height(D), 1);
                D.period_index = repmat(i - 1, height(D), 1);
            case "episode"
                D.source_file_index = repmat(i, height(D), 1);
            otherwise
                error("Unknown resample mode: %s", resampleMode);
        end

        tables{i} = D;
    end

    syntheticTrain = vertcat(tables{:});
    syntheticTrain.t_s = (0:height(syntheticTrain)-1).' * 0.010;
end

function summary = summarizeBootstrapSamples(validSamples)
    parameterNames = ["K_rad_per_us"; "tau_s"; "delay_s"];
    n = numel(parameterNames);

    parameter = strings(n, 1);
    n_valid = zeros(n, 1);
    mean_value = NaN(n, 1);
    std_value = NaN(n, 1);
    p2p5 = NaN(n, 1);
    p50 = NaN(n, 1);
    p97p5 = NaN(n, 1);
    min_value = NaN(n, 1);
    max_value = NaN(n, 1);

    for i = 1:n
        parameter(i) = parameterNames(i);
        x = validSamples.(parameterNames(i));
        x = x(isfinite(x));
        n_valid(i) = numel(x);

        if isempty(x)
            continue;
        end

        mean_value(i) = mean(x, "omitnan");
        std_value(i) = std(x, "omitnan");
        p2p5(i) = percentileLocal(x, 2.5);
        p50(i) = percentileLocal(x, 50);
        p97p5(i) = percentileLocal(x, 97.5);
        min_value(i) = min(x);
        max_value(i) = max(x);
    end

    summary = table(parameter, n_valid, mean_value, std_value, ...
        p2p5, p50, p97p5, min_value, max_value);
end

function q = percentileLocal(x, pct)
    x = sort(x(:));
    x = x(isfinite(x));

    if isempty(x)
        q = NaN;
        return;
    end

    if numel(x) == 1
        q = x(1);
        return;
    end

    pos = 1 + (pct / 100) * (numel(x) - 1);
    lo = floor(pos);
    hi = ceil(pos);

    if lo == hi
        q = x(lo);
    else
        q = x(lo) + (pos - lo) * (x(hi) - x(lo));
    end
end

function plotParameterHistograms(validSamples, pngPath)
    fig = figure("Name", "bootstrap_parameter_histograms", "Color", "w");
    tiledlayout(1, 3, "TileSpacing", "compact");
    vars = ["K_rad_per_us", "tau_s", "delay_s"];
    labels = ["K (rad/us)", "tau (s)", "delay (s)"];

    for i = 1:numel(vars)
        nexttile;
        histogram(validSamples.(vars(i)), 24, "FaceColor", [0 0.7 0.9], "EdgeColor", "w");
        xlabel(labels(i));
        ylabel("Count");
        title(labels(i));
        grid on;
    end

    exportgraphics(fig, pngPath, "Resolution", 300);
end

function plotParameterPairs(validSamples, pngPath)
    fig = figure("Name", "bootstrap_parameter_pairs", "Color", "w");
    tiledlayout(1, 3, "TileSpacing", "compact");
    c = [1 0.45 0];

    nexttile;
    scatter(validSamples.K_rad_per_us, validSamples.tau_s, 18, c, "filled", "MarkerFaceAlpha", 0.45);
    xlabel("K (rad/us)");
    ylabel("tau (s)");
    grid on;

    nexttile;
    scatter(validSamples.K_rad_per_us, validSamples.delay_s, 18, c, "filled", "MarkerFaceAlpha", 0.45);
    xlabel("K (rad/us)");
    ylabel("delay (s)");
    grid on;

    nexttile;
    scatter(validSamples.tau_s, validSamples.delay_s, 18, c, "filled", "MarkerFaceAlpha", 0.45);
    xlabel("tau (s)");
    ylabel("delay (s)");
    grid on;

    exportgraphics(fig, pngPath, "Resolution", 300);
end

function plotBodeEnvelope(validSamples, validationFrf, pngPath)
    fig = figure("Name", "bootstrap_bode_envelope", "Color", "w");
    tiledlayout(2, 1, "TileSpacing", "compact");

    fHz = logspace(log10(0.08), log10(3.25), 180).';
    w = 2*pi*fHz;
    n = height(validSamples);
    G = NaN(numel(w), n);

    for i = 1:n
        params.K = validSamples.K_rad_per_us(i);
        params.tau1_s = validSamples.tau_s(i);
        params.tau2_s = NaN;
        params.delay_s = validSamples.delay_s(i);
        G(:, i) = ServoPrpsId.evalFrequencyModel("first_order_delay", params, w);
    end

    magDb = 20*log10(abs(G));
    phaseDeg = unwrap(angle(G), [], 1) * 180/pi;

    magLo = rowPercentile(magDb, 2.5);
    magMed = rowPercentile(magDb, 50);
    magHi = rowPercentile(magDb, 97.5);
    phaseLo = rowPercentile(phaseDeg, 2.5);
    phaseMed = rowPercentile(phaseDeg, 50);
    phaseHi = rowPercentile(phaseDeg, 97.5);

    nexttile;
    fill([fHz; flipud(fHz)], [magLo; flipud(magHi)], [0 0.75 0.9], ...
        "FaceAlpha", 0.25, "EdgeColor", "none");
    hold on;
    semilogx(fHz, magMed, "Color", [0 0.45 0.95], "LineWidth", 1.8);

    if ~isempty(validationFrf.f_Hz)
        scatter(validationFrf.f_Hz, 20*log10(abs(validationFrf.G_emp)), ...
            28, [1 0.65 0], "filled");
    end

    ylabel("Magnitude (dB)");
    title("Bootstrap FOPD Bode Envelope");
    grid on;

    nexttile;
    fill([fHz; flipud(fHz)], [phaseLo; flipud(phaseHi)], [0 0.75 0.9], ...
        "FaceAlpha", 0.25, "EdgeColor", "none");
    hold on;
    semilogx(fHz, phaseMed, "Color", [0 0.45 0.95], "LineWidth", 1.8);

    if ~isempty(validationFrf.f_Hz)
        scatter(validationFrf.f_Hz, unwrap(angle(validationFrf.G_emp))*180/pi, ...
            28, [1 0.65 0], "filled");
    end

    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    grid on;

    exportgraphics(fig, pngPath, "Resolution", 300);
end

function q = rowPercentile(X, pct)
    q = NaN(size(X, 1), 1);

    for i = 1:size(X, 1)
        q(i) = percentileLocal(X(i, :), pct);
    end
end

function writeBootstrapReport(path, samples, summary, unitMeta, trainingCsvFiles, validationCsvFiles, dataDir, resampleMode, stratifyByRun, rngSeed)
    fid = fopen(path, "w");

    if fid < 0
        error("Could not open report for writing:\n%s", path);
    end

    cleanup = onCleanup(@() fclose(fid));

    nOk = nnz(string(samples.fit_status) == "ok");
    nFailed = height(samples) - nOk;

    fprintf(fid, "# Servo FOPD Bootstrap Report\n\n");
    fprintf(fid, "**Generated:** %s  \n", datestr(now, "yyyy-mm-dd HH:MM:SS"));
    fprintf(fid, "**Dataset:** `%s`  \n", strrep(char(dataDir), "\", "/"));
    fprintf(fid, "**Resampling mode:** `%s`  \n", resampleMode);
    fprintf(fid, "**Stratify by run:** `%d`  \n", stratifyByRun);
    fprintf(fid, "**RNG seed:** `%d`  \n", rngSeed);
    fprintf(fid, "**Bootstrap draws:** %d  \n", height(samples));
    fprintf(fid, "**Successful fits:** %d  \n", nOk);
    fprintf(fid, "**Failed fits:** %d\n\n", nFailed);

    fprintf(fid, "Sampling with replacement means each bootstrap draw can pick the same complete episode or PRPS period more than once while omitting others.\n\n");

    fprintf(fid, "## Parameter Summary\n\n");
    fprintf(fid, "| Parameter | n | mean | std | p2.5 | median | p97.5 | min | max |\n");
    fprintf(fid, "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n");

    for i = 1:height(summary)
        fprintf(fid, "| `%s` | %d | %.9g | %.9g | %.9g | %.9g | %.9g | %.9g | %.9g |\n", ...
            summary.parameter(i), ...
            summary.n_valid(i), ...
            summary.mean_value(i), ...
            summary.std_value(i), ...
            summary.p2p5(i), ...
            summary.p50(i), ...
            summary.p97p5(i), ...
            summary.min_value(i), ...
            summary.max_value(i));
    end

    fprintf(fid, "\n## Bootstrap Units\n\n");
    fprintf(fid, "| run_name | units |\n");
    fprintf(fid, "|---|---:|\n");
    runNames = unique(string(unitMeta.run_name), "stable");

    for r = 1:numel(runNames)
        fprintf(fid, "| `%s` | %d |\n", runNames(r), nnz(string(unitMeta.run_name) == runNames(r)));
    end

    fprintf(fid, "\n## Files\n\n");
    fprintf(fid, "**Training files:**\n\n");
    for k = 1:numel(trainingCsvFiles)
        fprintf(fid, "- `%s`\n", trainingCsvFiles(k).name);
    end

    fprintf(fid, "\n**Validation files:**\n\n");
    if isempty(validationCsvFiles)
        fprintf(fid, "- None\n");
    else
        for k = 1:numel(validationCsvFiles)
            fprintf(fid, "- `%s`\n", validationCsvFiles(k).name);
        end
    end

    if nFailed > 0
        failed = samples(string(samples.fit_status) ~= "ok", :);
        fprintf(fid, "\n## Failed Fits\n\n");
        fprintf(fid, "| draw | message |\n");
        fprintf(fid, "|---:|---|\n");

        for i = 1:height(failed)
            fprintf(fid, "| %d | `%s` |\n", failed.bootstrap_index(i), failed.fit_message(i));
        end
    end

    clear cleanup;
end
