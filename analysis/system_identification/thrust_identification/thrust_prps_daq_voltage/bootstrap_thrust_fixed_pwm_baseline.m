%% bootstrap_thrust_fixed_pwm_baseline.m
% Estimate fixed-thrust uncertainty from PRPS baseline segments.
%
% This analysis is intentionally scoped to the crude stabilizer plan:
% thrust is held fixed at a feedforward PWM, so the useful uncertainty is the
% observed thrust at that fixed command under the available run/voltage
% conditions, not the full dynamic thrust model.

clearvars -except TARGET_PWM_US RUN_NAME_FILTER N_BOOT BASELINE_TAIL_SAMPLES INCLUDE_VALIDATION_FILES RNG_SEED SAVE_FIGURES;
clc; close all;

%% User options

if ~exist("TARGET_PWM_US", "var"), TARGET_PWM_US = 1825; end
if ~exist("RUN_NAME_FILTER", "var"), RUN_NAME_FILTER = "local_1700_1950"; end
if ~exist("N_BOOT", "var"), N_BOOT = 500; end
if ~exist("BASELINE_TAIL_SAMPLES", "var"), BASELINE_TAIL_SAMPLES = 40; end
if ~exist("INCLUDE_VALIDATION_FILES", "var"), INCLUDE_VALIDATION_FILES = true; end
if ~exist("RNG_SEED", "var"), RNG_SEED = 1; end
if ~exist("SAVE_FIGURES", "var"), SAVE_FIGURES = true; end

TARGET_PWM_US = double(TARGET_PWM_US);
RUN_NAME_FILTER = string(RUN_NAME_FILTER);

%% Locate folders

scriptPath = mfilename("fullpath");
repoRoot = findRepoRoot(scriptPath);
addpath(fullfile(repoRoot, "analysis", "utils"));

dataRoot = fullfile( ...
    repoRoot, ...
    "data", "raw", "system_identification", "thrust_identification", ...
    "thrust_prps_daq_voltage", "accepted");

trainDir = fullfile(dataRoot, "train");
validationDir = fullfile(dataRoot, "validation");

plotDir = fullfile( ...
    getMirroredPlotDir(scriptPath), ...
    "bootstrap_fixed_pwm_" + string(round(TARGET_PWM_US)));
reportDir = strrep(plotDir, fullfile(repoRoot, "plots"), fullfile(repoRoot, "reports"));

if ~isfolder(trainDir)
    error("Training folder not found:\n%s", trainDir);
end

if ~exist(plotDir, "dir")
    mkdir(plotDir);
end
if ~exist(reportDir, "dir")
    mkdir(reportDir);
end

trainFiles = listSortedCsvFiles(trainDir);

if INCLUDE_VALIDATION_FILES
    validationFiles = listSortedCsvFiles(validationDir);
else
    validationFiles = struct([]);
end

allFiles = [trainFiles; validationFiles];

if isempty(allFiles)
    error("No accepted thrust PRPS CSV files found.");
end

fprintf("\nThrust fixed-PWM baseline bootstrap\n");
fprintf("  Target PWM: %.0f us\n", TARGET_PWM_US);
fprintf("  Run filter: %s\n", RUN_NAME_FILTER);
fprintf("  Tail samples per baseline: %d\n", BASELINE_TAIL_SAMPLES);
fprintf("  Bootstrap draws: %d\n", N_BOOT);
fprintf("  Include validation files: %d\n", INCLUDE_VALIDATION_FILES);

%% Build baseline units

baselineUnits = buildBaselineUnits( ...
    allFiles, ...
    TARGET_PWM_US, ...
    RUN_NAME_FILTER, ...
    BASELINE_TAIL_SAMPLES);

if isempty(baselineUnits)
    error("No baseline units found for run_name=%s at PWM %.0f us.", RUN_NAME_FILTER, TARGET_PWM_US);
end

fprintf("  Baseline units found: %d\n", height(baselineUnits));

disp("Baseline units:");
disp(baselineUnits(:, { ...
    'unit_id', ...
    'split', ...
    'prps_seed', ...
    'phase', ...
    'force_mean_N', ...
    'force_std_N', ...
    'voltage_mean_V', ...
    'voltage_min_V', ...
    'voltage_max_V' ...
}));

%% Bootstrap fixed-thrust mean

rng(RNG_SEED, "twister");

sampleRows = cell(N_BOOT, 8);

for b = 1:N_BOOT
    sampledIdx = randi(height(baselineUnits), height(baselineUnits), 1);
    sampled = baselineUnits(sampledIdx, :);

    sampleRows(b, :) = { ...
        b, ...
        mean(sampled.force_mean_N, "omitnan"), ...
        std(sampled.force_mean_N, "omitnan"), ...
        min(sampled.force_mean_N), ...
        max(sampled.force_mean_N), ...
        mean(sampled.voltage_mean_V, "omitnan"), ...
        std(sampled.voltage_mean_V, "omitnan"), ...
        strjoin(string(sampled.unit_id), ";") ...
    };
end

bootstrapSamples = cell2table(sampleRows, 'VariableNames', { ...
    'bootstrap_index', ...
    'mean_thrust_N', ...
    'std_thrust_N', ...
    'min_sampled_thrust_N', ...
    'max_sampled_thrust_N', ...
    'mean_voltage_V', ...
    'std_voltage_V', ...
    'sampled_unit_ids' ...
});

summary = makeSummaryTable(baselineUnits, bootstrapSamples);

%% Save outputs

unitsCsvPath = fullfile(plotDir, "fixed_pwm_baseline_units.csv");
samplesCsvPath = fullfile(plotDir, "fixed_pwm_bootstrap_samples.csv");
summaryCsvPath = fullfile(plotDir, "fixed_pwm_bootstrap_summary.csv");
matPath = fullfile(plotDir, "fixed_pwm_bootstrap_samples.mat");
reportPath = fullfile(reportDir, "bootstrap_thrust_fixed_pwm_baseline.report.md");

writetable(baselineUnits, unitsCsvPath);
writetable(bootstrapSamples, samplesCsvPath);
writetable(summary, summaryCsvPath);

save(matPath, ...
    "baselineUnits", ...
    "bootstrapSamples", ...
    "summary", ...
    "TARGET_PWM_US", ...
    "RUN_NAME_FILTER", ...
    "N_BOOT", ...
    "BASELINE_TAIL_SAMPLES", ...
    "INCLUDE_VALIDATION_FILES", ...
    "RNG_SEED");

if SAVE_FIGURES
    plotThrustHistogram(baselineUnits, bootstrapSamples, fullfile(plotDir, "fixed_pwm_thrust_histogram.png"));
    plotThrustVsVoltage(baselineUnits, bootstrapSamples, fullfile(plotDir, "fixed_pwm_thrust_vs_voltage.png"));
end

writeReport( ...
    reportPath, ...
    baselineUnits, ...
    bootstrapSamples, ...
    summary, ...
    TARGET_PWM_US, ...
    RUN_NAME_FILTER, ...
    N_BOOT, ...
    BASELINE_TAIL_SAMPLES, ...
    INCLUDE_VALIDATION_FILES, ...
    RNG_SEED, ...
    dataRoot);

fprintf("\nFixed-PWM thrust bootstrap outputs written to:\n  %s\n", plotDir);
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

function units = buildBaselineUnits(csvFiles, targetPwmUs, runNameFilter, tailSamples)
    unitRows = cell(0, 17);
    phases = ["baseline_pre", "baseline_post"];

    for f = 1:numel(csvFiles)
        filePath = fullfile(csvFiles(f).folder, csvFiles(f).name);
        T = readtable(filePath);
        T = normalizeThrustTable(T);

        if ~any(string(T.run_name) == runNameFilter)
            continue;
        end

        for p = 1:numel(phases)
            phaseName = phases(p);
            idx = string(T.run_name) == runNameFilter & ...
                  string(T.phase) == phaseName & ...
                  abs(T.pwm_us - targetPwmUs) < 0.5;

            D = T(idx, :);

            if isempty(D)
                continue;
            end

            nTail = min(tailSamples, height(D));
            Dtail = D(end-nTail+1:end, :);

            splitName = string(csvFiles(f).folder);
            if contains(splitName, filesep + "validation")
                splitName = "validation";
            elseif contains(splitName, filesep + "train")
                splitName = "train";
            else
                splitName = "unknown";
            end

            unitId = sprintf("%s_seed%d_%s", splitName, round(mode(Dtail.prps_seed)), phaseName);

            unitRows(end+1, :) = { ...
                string(unitId), ...
                splitName, ...
                string(csvFiles(f).name), ...
                string(Dtail.run_name(1)), ...
                round(mode(Dtail.prps_seed)), ...
                string(phaseName), ...
                targetPwmUs, ...
                height(D), ...
                nTail, ...
                mean(Dtail.force_N, "omitnan"), ...
                std(Dtail.force_N, "omitnan"), ...
                min(Dtail.force_N), ...
                max(Dtail.force_N), ...
                mean(Dtail.battery_voltage_V, "omitnan"), ...
                min(Dtail.battery_voltage_V), ...
                max(Dtail.battery_voltage_V), ...
                meanIfPresent(Dtail, "battery_current_A") ...
            }; %#ok<AGROW>
        end
    end

    units = cell2table(unitRows, 'VariableNames', { ...
        'unit_id', ...
        'split', ...
        'source_file', ...
        'run_name', ...
        'prps_seed', ...
        'phase', ...
        'pwm_us', ...
        'segment_sample_count', ...
        'tail_sample_count', ...
        'force_mean_N', ...
        'force_std_N', ...
        'force_min_N', ...
        'force_max_N', ...
        'voltage_mean_V', ...
        'voltage_min_V', ...
        'voltage_max_V', ...
        'current_mean_A' ...
    });
end

function T = normalizeThrustTable(T)
    required = ["run_name", "phase", "pwm_us", "force_N", "battery_voltage_V"];

    for i = 1:numel(required)
        if ~ismember(required(i), string(T.Properties.VariableNames))
            error("Missing required column '%s'.", required(i));
        end
    end

    T.run_name = string(T.run_name);
    T.phase = string(T.phase);
    T.pwm_us = forceNumeric(T.pwm_us);
    T.force_N = forceNumeric(T.force_N);
    T.battery_voltage_V = forceNumeric(T.battery_voltage_V);

    if ismember("battery_current_A", string(T.Properties.VariableNames))
        T.battery_current_A = forceNumeric(T.battery_current_A);
    end

    if ismember("prps_seed", string(T.Properties.VariableNames))
        T.prps_seed = forceNumeric(T.prps_seed);
    else
        T.prps_seed = NaN(height(T), 1);
    end
end

function summary = makeSummaryTable(units, samples)
    metric = [
        "observed_unit_thrust_N"
        "bootstrap_mean_thrust_N"
        "observed_unit_voltage_V"
        "bootstrap_mean_voltage_V"
    ];

    values = {
        units.force_mean_N
        samples.mean_thrust_N
        units.voltage_mean_V
        samples.mean_voltage_V
    };

    n = numel(metric);
    n_valid = zeros(n, 1);
    mean_value = NaN(n, 1);
    std_value = NaN(n, 1);
    p2p5 = NaN(n, 1);
    p50 = NaN(n, 1);
    p97p5 = NaN(n, 1);
    min_value = NaN(n, 1);
    max_value = NaN(n, 1);

    for i = 1:n
        x = values{i};
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

    summary = table(metric, n_valid, mean_value, std_value, p2p5, p50, p97p5, min_value, max_value);
end

function plotThrustHistogram(units, samples, pngPath)
    fig = figure("Name", "fixed_pwm_thrust_histogram", "Color", "w");
    histogram(samples.mean_thrust_N, 24, "FaceColor", [0 0.7 0.9], "EdgeColor", "w");
    hold on;
    xline(mean(units.force_mean_N, "omitnan"), "Color", [1 0.45 0], "LineWidth", 1.8, ...
        "Label", "Observed mean");
    xline(min(units.force_mean_N), "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.4, ...
        "Label", "Observed min/max");
    xline(max(units.force_mean_N), "--", "Color", [0.45 0.45 0.45], "LineWidth", 1.4);
    xlabel("Bootstrap mean thrust at fixed PWM (N)");
    ylabel("Count");
    title("Fixed-PWM Thrust Bootstrap");
    grid on;
    exportgraphics(fig, pngPath, "Resolution", 300);
end

function plotThrustVsVoltage(units, samples, pngPath)
    fig = figure("Name", "fixed_pwm_thrust_vs_voltage", "Color", "w");
    scatter(units.voltage_mean_V, units.force_mean_N, 80, [1 0.55 0], "filled");
    hold on;
    scatter(samples.mean_voltage_V, samples.mean_thrust_N, 16, [0 0.7 0.9], "filled", ...
        "MarkerFaceAlpha", 0.25);
    xlabel("Battery voltage (V)");
    ylabel("Thrust at fixed PWM (N)");
    title("Fixed-PWM Thrust vs Voltage");
    legend("Baseline units", "Bootstrap draw means", "Location", "best");
    grid on;
    exportgraphics(fig, pngPath, "Resolution", 300);
end

function writeReport(path, units, samples, summary, targetPwmUs, runNameFilter, nBoot, tailSamples, includeValidation, rngSeed, dataRoot)
    fid = fopen(path, "w");

    if fid < 0
        error("Could not open report for writing:\n%s", path);
    end

    cleanup = onCleanup(@() fclose(fid));

    observedLow = min(units.force_mean_N);
    observedHigh = max(units.force_mean_N);
    bootLow = percentileLocal(samples.mean_thrust_N, 2.5);
    bootMid = percentileLocal(samples.mean_thrust_N, 50);
    bootHigh = percentileLocal(samples.mean_thrust_N, 97.5);
    observedVoltageLow = min(units.voltage_mean_V);
    observedVoltageHigh = max(units.voltage_mean_V);

    fprintf(fid, "# Fixed-PWM Thrust Baseline Bootstrap Report\n\n");
    fprintf(fid, "**Generated:** %s  \n", datestr(now, "yyyy-mm-dd HH:MM:SS"));
    fprintf(fid, "**Dataset:** `%s`  \n", strrep(char(dataRoot), "\", "/"));
    fprintf(fid, "**Target PWM:** %.0f us  \n", targetPwmUs);
    fprintf(fid, "**Run filter:** `%s`  \n", runNameFilter);
    fprintf(fid, "**Baseline tail samples:** %d  \n", tailSamples);
    fprintf(fid, "**Include validation files:** `%d`  \n", includeValidation);
    fprintf(fid, "**RNG seed:** `%d`  \n", rngSeed);
    fprintf(fid, "**Bootstrap draws:** %d  \n", nBoot);
    fprintf(fid, "**Baseline units:** %d\n\n", height(units));

    fprintf(fid, "This analysis estimates fixed-thrust uncertainty for the crude stabilizer, which holds thrust constant at $u_T^\\ast = %.0f~\\mu\\text{s}$. It is not a dynamic FOPD bootstrap.\n\n", targetPwmUs);

    fprintf(fid, "## Baseline Units\n\n");
    fprintf(fid, "| Split | Seed | Phase | Force mean (N) | Force std (N) | Voltage mean (V) | Voltage range (V) | File |\n");
    fprintf(fid, "|---|---:|---|---:|---:|---:|---:|---|\n");

    for i = 1:height(units)
        fprintf(fid, "| `%s` | %d | `%s` | %.4f | %.4f | %.3f | %.3f-%.3f | `%s` |\n", ...
            units.split(i), ...
            units.prps_seed(i), ...
            units.phase(i), ...
            units.force_mean_N(i), ...
            units.force_std_N(i), ...
            units.voltage_mean_V(i), ...
            units.voltage_min_V(i), ...
            units.voltage_max_V(i), ...
            units.source_file(i));
    end

    fprintf(fid, "\n## Summary\n\n");
    fprintf(fid, "- Observed thrust envelope: %.4f to %.4f N\n", observedLow, observedHigh);
    fprintf(fid, "- Observed voltage envelope: %.3f to %.3f V\n", observedVoltageLow, observedVoltageHigh);
    fprintf(fid, "- Bootstrap mean thrust 95%% interval: %.4f to %.4f N\n", bootLow, bootHigh);
    fprintf(fid, "- Bootstrap median mean thrust: %.4f N\n\n", bootMid);

    fprintf(fid, "For rough PID design, use the raw observed envelope as the conservative fixed-thrust range because only %d baseline units are available. The bootstrap summarizes how the observed units reweight the mean; it does not discover unobserved battery states.\n\n", height(units));

    fprintf(fid, "Recommended conservative range for the fixed thrust command:\n\n");
    fprintf(fid, "$$\n");
    fprintf(fid, "T^\\ast(%.0f~\\mu\\text{s}) \\in [%.3f,\\ %.3f]~\\text{N}\n", targetPwmUs, observedLow, observedHigh);
    fprintf(fid, "$$\n\n");

    fprintf(fid, "## Bootstrap Summary Table\n\n");
    fprintf(fid, "| Metric | n | mean | std | p2.5 | median | p97.5 | min | max |\n");
    fprintf(fid, "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n");

    for i = 1:height(summary)
        fprintf(fid, "| `%s` | %d | %.6g | %.6g | %.6g | %.6g | %.6g | %.6g | %.6g |\n", ...
            summary.metric(i), ...
            summary.n_valid(i), ...
            summary.mean_value(i), ...
            summary.std_value(i), ...
            summary.p2p5(i), ...
            summary.p50(i), ...
            summary.p97p5(i), ...
            summary.min_value(i), ...
            summary.max_value(i));
    end

    clear cleanup;
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

function y = meanIfPresent(T, varName)
    if ismember(varName, string(T.Properties.VariableNames))
        y = mean(T.(varName), "omitnan");
    else
        y = NaN;
    end
end
