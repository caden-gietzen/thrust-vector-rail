%% analyze_load_cell_characterization.m
% HX711 load-cell uncertainty characterization.
%
% Goal:
%   From a single dead-weight characterization run (warm-up -> tare -> repeated
%   up/down weight ladder), produce the instrument uncertainty budget on force:
%
%       scale (g/count) + its uncertainty, nonlinearity, hysteresis,
%       repeatability, and averaged-noise floor
%
%   combined (root-sum-square) into u(F) in grams, %FS, and Newtons. This is the
%   number that feeds Monte Carlo perturbation ranges on the constant thrust T
%   (and optionally the EKF prior / process noise on the unmeasured T state).
%
% Expected CSV columns (from load_cell_characterization.py):
%   t_ms,t_s,phase,repeat,hold_index,direction,known_mass_g,sample_idx,
%   raw_count,tared_count

clear; clc; close all;

%% User Options

DATASET_STATUS = "candidate";   % candidate | accepted | rejected | diagnostics
ANALYZE_MODE   = "all";         % all (compose every CSV) | latest | single
dataFile       = "";            % used only when ANALYZE_MODE = "single"

WINDOW_SECONDS = 5;             % dwell-window length for averaged-noise floor
G_TO_N         = 9.80665 / 1000.0;

% Bright palette for MATLAB dark mode (avoid dark primaries).
C_DATA = [0 0.9 1];     % cyan
C_FIT  = [1 0.65 0];    % orange
C_ALT  = [0.45 1 0.45]; % lime
C_LT   = [0.85 0.85 0.85];

SAVE_FIGURES = true;

%% Locate Project Paths

scriptPath = mfilename("fullpath");
dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

fprintf("\nHX711 load-cell uncertainty characterization\n");
fprintf("Dataset status folder: %s\n", DATASET_STATUS);
fprintf("Data directory:\n  %s\n", dataDir);

csvFiles = dir(fullfile(dataDir, "*.csv"));
if isempty(csvFiles)
    error("No CSV files found in %s:\n%s", DATASET_STATUS, dataDir);
end

switch ANALYZE_MODE
    case "all"
        dataFiles = string({csvFiles.name});
    case "latest"
        dataFiles = string(selectLatestIndexedCsv(csvFiles));
    case "single"
        if strlength(dataFile) == 0
            error("ANALYZE_MODE='single' requires dataFile to be set.");
        end
        dataFiles = string(dataFile);
    otherwise
        error("Unknown ANALYZE_MODE: %s. Use 'all', 'latest', or 'single'.", ANALYZE_MODE);
end

fprintf("Composing %d dataset(s) -- each run = one pass:\n", numel(dataFiles));
for k = 1:numel(dataFiles)
    fprintf("  %s\n", dataFiles(k));
end

if numel(dataFiles) == 1
    datasetLabel = dataFiles(1);
else
    datasetLabel = sprintf("%d datasets composed", numel(dataFiles));
end

%% Load Each File, Recompute Per-File Offset, Build Per-Hold Stats
%
% Each run logs ONE up/down pass, so each FILE is a separate pass. The tare
% offset is recomputed per file from that file's 0 g holds; per-hold means
% across files then give the run-to-run repeatability term.

H_mass = []; H_dir = strings(0,1); H_rep = [];
H_meanTared = []; H_stdRaw = []; H_n = []; H_winStd = [];
fileOffsets = nan(numel(dataFiles), 1);

for fileIdx = 1:numel(dataFiles)
    dataPath = fullfile(dataDir, dataFiles(fileIdx));

    opts = detectImportOptions(dataPath, "FileType", "text", ...
        "Delimiter", ",", "VariableNamingRule", "preserve");
    T = readtable(dataPath, opts);
    names = matlab.lang.makeValidName(erase(strtrim(string(T.Properties.VariableNames)), char(65279)));
    T.Properties.VariableNames = cellstr(names);

    required = ["t_s","phase","hold_index","direction","known_mass_g","raw_count"];
    for k = 1:numel(required)
        if ~ismember(required(k), string(T.Properties.VariableNames))
            error("Missing required CSV column in %s: %s", dataFiles(fileIdx), required(k));
        end
    end

    phase     = string(T.phase);
    direction = string(T.direction);
    holdIndex = double(T.hold_index);
    massG     = double(T.known_mass_g);
    raw       = double(T.raw_count);
    t_s       = double(T.t_s);

    isLadder = phase == "ladder";
    if ~any(isLadder)
        warning("No 'ladder' rows in %s; skipping.", dataFiles(fileIdx));
        continue;
    end

    zeroMask = isLadder & massG == 0;
    if any(zeroMask)
        offset = mean(raw(zeroMask));
    else
        offset = raw(find(isLadder, 1));
        warning("No 0 g holds in %s; using first ladder sample as offset.", dataFiles(fileIdx));
    end
    fileOffsets(fileIdx) = offset;
    tared = raw - offset;

    holds = unique(holdIndex(isLadder));
    for i = 1:numel(holds)
        m = isLadder & holdIndex == holds(i);
        tv = tared(m);
        ts = t_s(m);

        H_mass(end+1,1)      = massG(find(m,1));     %#ok<SAGROW>
        H_dir(end+1,1)       = direction(find(m,1)); %#ok<SAGROW>
        H_rep(end+1,1)       = fileIdx;              %#ok<SAGROW>
        H_meanTared(end+1,1) = mean(tv);             %#ok<SAGROW>
        H_stdRaw(end+1,1)    = std(tv);              %#ok<SAGROW>
        H_n(end+1,1)         = numel(tv);            %#ok<SAGROW>

        % Dwell-window means: chunk the hold's samples, take std of the means.
        ws = NaN;
        if numel(ts) >= 4
            approxFs = (numel(ts)-1) / max(ts(end)-ts(1), eps);
            winN = max(round(WINDOW_SECONDS * approxFs), 2);
            nWin = floor(numel(tv) / winN);
            if nWin >= 2
                wm = zeros(nWin,1);
                for w = 1:nWin
                    wm(w) = mean(tv((w-1)*winN + (1:winN)));
                end
                ws = std(wm);
            end
        end
        H_winStd(end+1,1) = ws; %#ok<SAGROW>
    end
end

nH = numel(H_mass);
if nH == 0
    error("No ladder holds found across the selected files.");
end
fprintf("\nComposed %d holds from %d file(s).\n", nH, numel(dataFiles));
fprintf("Per-file tare offsets (counts): %s\n", mat2str(round(fileOffsets')));

%% Calibration Regression (mass_g = slope * tared_count + intercept)

% Base-MATLAB ordinary least squares (no Statistics Toolbox dependency).
xfit = H_meanTared; yfit = H_mass; nfit = numel(xfit);
pfit = polyfit(xfit, yfit, 1);
slope_g_per_count = pfit(1);
intercept_g       = pfit(2);
residFit = yfit - polyval(pfit, xfit);
SSE = sum(residFit.^2);
SST = sum((yfit - mean(yfit)).^2);
Sxx = sum((xfit - mean(xfit)).^2);
sigma2 = SSE / max(nfit - 2, 1);
slope_SE = sqrt(sigma2 / Sxx);
R2 = 1 - SSE / SST;
relScale = abs(slope_SE / slope_g_per_count);   % fractional scale uncertainty

scale_N_per_count = slope_g_per_count * G_TO_N;
maxMass = max(H_mass);

%% Nonlinearity (level-mean deviation from the line)

uMass = unique(H_mass);
levelMeanTared = arrayfun(@(mm) mean(H_meanTared(H_mass==mm)), uMass);
levelPredMass  = slope_g_per_count * levelMeanTared + intercept_g;
nonlin_resid_g = levelPredMass - uMass;
nonlin_rms_g   = sqrt(mean(nonlin_resid_g.^2));
nonlin_pct_fs  = 100 * nonlin_rms_g / maxMass;

%% Hysteresis (up vs down at shared masses) in grams

sharedMass = intersect(H_mass(H_dir=="up"), H_mass(H_dir=="down"));
hyst_g = zeros(numel(sharedMass),1);
for i = 1:numel(sharedMass)
    mm = sharedMass(i);
    up_c   = mean(H_meanTared(H_mass==mm & H_dir=="up"));
    down_c = mean(H_meanTared(H_mass==mm & H_dir=="down"));
    hyst_g(i) = abs(slope_g_per_count * (down_c - up_c));
end
hyst_max_g = max([hyst_g; 0]);

%% Repeatability (run-to-run spread of per-hold means) in grams

repeat_std_g = [];
for mm = uMass'
    for dd = ["up","down"]
        sel = H_mass==mm & H_dir==dd;
        if nnz(sel) >= 2
            repeat_std_g(end+1) = slope_g_per_count * std(H_meanTared(sel)); %#ok<SAGROW>
        end
    end
end
repeat_med_g = median([repeat_std_g, 0]);

%% Noise Floor (per-sample and dwell-averaged) in grams

persample_g   = slope_g_per_count * median(H_stdRaw);
avgNoise_g    = slope_g_per_count * median(H_stdRaw ./ sqrt(H_n));   % white-averaged
winDrift_g    = slope_g_per_count * median(H_winStd, "omitnan");      % within-hold drift
% Effective averaged-mean floor: the larger of white-averaged noise and drift.
noiseFloor_g  = max(avgNoise_g, max(winDrift_g, 0));

%% Combined Uncertainty Budget (RSS), in grams, at representative loads

loadEval = [200, maxMass];
u_load_g = zeros(size(loadEval));
for i = 1:numel(loadEval)
    u_scale_g = relScale * loadEval(i);
    u_load_g(i) = sqrt(noiseFloor_g^2 + repeat_med_g^2 + nonlin_rms_g^2 + ...
                       (hyst_max_g/2)^2 + u_scale_g^2);
end

%% Print Budget

fprintf("\n============================================================\n");
fprintf("Calibration\n");
fprintf("============================================================\n");
fprintf("  scale: %.6g g/count  (%.6g N/count)\n", slope_g_per_count, scale_N_per_count);
fprintf("  scale uncertainty (1-sigma): %.3g %% (SE %.4g g/count)\n", 100*relScale, slope_SE);
fprintf("  intercept: %.4g g    R^2: %.5f\n", intercept_g, R2);

fprintf("\n============================================================\n");
fprintf("Error components (1-sigma, grams)\n");
fprintf("============================================================\n");
fprintf("  per-sample noise:        %.3g g\n", persample_g);
fprintf("  averaged-noise floor:    %.3g g  (white %.3g, drift %.3g)\n", noiseFloor_g, avgNoise_g, winDrift_g);
fprintf("  repeatability (run-run): %.3g g\n", repeat_med_g);
fprintf("  nonlinearity (RMS):      %.3g g  (%.2f %% FS)\n", nonlin_rms_g, nonlin_pct_fs);
fprintf("  hysteresis (max gap):    %.3g g  (half-gap %.3g g)\n", hyst_max_g, hyst_max_g/2);
fprintf("  scale @ %g g:            %.3g g\n", maxMass, relScale*maxMass);

fprintf("\n============================================================\n");
fprintf("Combined standard uncertainty u(F)  [feeds Monte Carlo on T]\n");
fprintf("============================================================\n");
for i = 1:numel(loadEval)
    fprintf("  at %4g g:  u = %.3g g  = %.4g N  = %.2f %% of load  (95%%: %.4g N)\n", ...
        loadEval(i), u_load_g(i), u_load_g(i)*G_TO_N, 100*u_load_g(i)/loadEval(i), 2*u_load_g(i)*G_TO_N);
end

%% Plots

% 1. Ladder overview by pass (inferred mass per hold; each file = one pass)
figure("Name","overview");
hold on;
cyc = {C_DATA, C_FIT, C_ALT, C_LT};
passes = unique(H_rep);
legEntries = strings(0,1);
for k = 1:numel(passes)
    sel = H_rep == passes(k);
    col = cyc{mod(k-1, numel(cyc)) + 1};
    plot(1:nnz(sel), slope_g_per_count*H_meanTared(sel), "o-", ...
        "Color", col, "MarkerFaceColor", col, "LineWidth", 1.3);
    legEntries(end+1) = "pass " + string(passes(k)); %#ok<SAGROW>
end
grid on; xlabel("Hold # within pass"); ylabel("Inferred mass (g)");
title("Ladder overview by pass");
subtitle(datasetLabel, "Interpreter", "none");
legend(legEntries, "Location", "best", "TextColor", C_LT);

% 2. Calibration fit + residuals
figure("Name","calibration");
plot(H_meanTared, H_mass, "o", "Color", C_DATA, "MarkerFaceColor", C_DATA, "MarkerSize", 6);
hold on;
xline_span = linspace(min(H_meanTared), max(H_meanTared), 50);
plot(xline_span, slope_g_per_count*xline_span + intercept_g, "-", "Color", C_FIT, "LineWidth", 1.6);
grid on; xlabel("Tared count"); ylabel("Known mass (g)");
title(sprintf("Calibration fit: %.5g g/count, R^2=%.5f", slope_g_per_count, R2));
subtitle(datasetLabel, "Interpreter", "none");
legend("Per-hold means", "Linear fit", "Location", "best", "TextColor", C_LT);

% 3. Per-level noise / repeatability vs load
figure("Name","noise_vs_load");
perLevelStd_g = arrayfun(@(mm) slope_g_per_count*std(H_meanTared(H_mass==mm)), uMass);
plot(uMass, perLevelStd_g, "o-", "Color", C_DATA, "LineWidth", 1.4, "MarkerFaceColor", C_DATA);
hold on;
yline(noiseFloor_g, "--", "avg-noise floor", "Color", C_ALT);
grid on; xlabel("Load (g)"); ylabel("Per-hold mean spread (g)");
title("Repeatability / noise vs load");
subtitle(datasetLabel, "Interpreter", "none");

% 4. Hysteresis (up vs down per-level mean)
figure("Name","hysteresis");
hold on;
upMask = H_dir=="up"; dnMask = H_dir=="down";
plot(H_mass(upMask), slope_g_per_count*H_meanTared(upMask), "^", "Color", C_DATA, "MarkerFaceColor", C_DATA, "MarkerSize", 7);
plot(H_mass(dnMask), slope_g_per_count*H_meanTared(dnMask), "v", "Color", C_FIT, "MarkerFaceColor", C_FIT, "MarkerSize", 7);
plot(uMass, uMass, "--", "Color", C_LT);
grid on; xlabel("Known mass (g)"); ylabel("Inferred mass (g)");
title(sprintf("Hysteresis (max up-down gap %.3g g)", hyst_max_g));
subtitle(datasetLabel, "Interpreter", "none");
legend("Up leg", "Down leg", "Ideal", "Location", "best", "TextColor", C_LT);

%% Save Figures

if SAVE_FIGURES
    if ~exist(plotDir, "dir"); mkdir(plotDir); end
    saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);
end

%% Write Report

reportPath = fullfile(plotDir, "analyze_load_cell_characterization.report.md");
if ~exist(plotDir, "dir"); mkdir(plotDir); end
fid = fopen(reportPath, "w");
fprintf(fid, "# Load-cell characterization report\n\n");
fprintf(fid, "Datasets (%d, each = one pass), status: %s\n\n", numel(dataFiles), DATASET_STATUS);
for i = 1:numel(dataFiles)
    fprintf(fid, "- `%s`\n", dataFiles(i));
end
fprintf(fid, "\n");
fprintf(fid, "## Calibration\n\n");
fprintf(fid, "- scale: %.6g g/count (%.6g N/count)\n", slope_g_per_count, scale_N_per_count);
fprintf(fid, "- scale uncertainty (1-sigma): %.3g %%\n", 100*relScale);
fprintf(fid, "- intercept: %.4g g, R^2: %.5f\n\n", intercept_g, R2);
fprintf(fid, "## Error components (1-sigma, grams)\n\n");
fprintf(fid, "| component | value (g) | note |\n|---|---|---|\n");
fprintf(fid, "| per-sample noise | %.3g | single read |\n", persample_g);
fprintf(fid, "| averaged-noise floor | %.3g | after dwell averaging |\n", noiseFloor_g);
fprintf(fid, "| repeatability | %.3g | run-to-run |\n", repeat_med_g);
fprintf(fid, "| nonlinearity (RMS) | %.3g | %.2f %% FS |\n", nonlin_rms_g, nonlin_pct_fs);
fprintf(fid, "| hysteresis (max) | %.3g | half-gap %.3g |\n", hyst_max_g, hyst_max_g/2);
fprintf(fid, "| scale @ %g g | %.3g | load-dependent |\n\n", maxMass, relScale*maxMass);
fprintf(fid, "## Combined u(F) -- Monte Carlo perturbation on T\n\n");
fprintf(fid, "| load (g) | u (g) | u (N) | u (%% load) | 95%% (N) |\n|---|---|---|---|---|\n");
for i = 1:numel(loadEval)
    fprintf(fid, "| %g | %.3g | %.4g | %.2f | %.4g |\n", loadEval(i), u_load_g(i), ...
        u_load_g(i)*G_TO_N, 100*u_load_g(i)/loadEval(i), 2*u_load_g(i)*G_TO_N);
end
fprintf(fid, "\n_Instrument floor only; thrust-generation repeatability (motor on) stacks on top._\n");
fclose(fid);

fprintf("\nReport written:\n  %s\n", reportPath);
fprintf("\nDone.\n");
