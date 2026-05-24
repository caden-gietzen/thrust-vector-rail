%% analyze_servo_prbs_bode.m
% Analyze servo PRBS data from Pico CSV log.
%
% Goals:
%   1. Approximate servo bandwidth
%   2. Approximate servo delay
%   3. Decide whether behavior looks first-order, second-order, or rate-limited
%   4. Recommend a frequency range for later PRPS excitation
%
% Expected compact CSV columns:
%   t_ms,t_s,run_name,segment,servo_us,command_delta_us,
%   count_delta,theta_rad,sample_index
%
% Main empirical transfer function:
%
%       G(jw) = theta_rad(jw) / command_delta_us(jw)
%
% Units:
%   Input:  command_delta_us, microseconds
%   Output: theta_rad, radians
%   Gain:   rad/us
%
% Notes:
%   PRBS = Pseudo-Random Binary Signal.
%   FRF  = Frequency Response Function.
%
% This is a scouting analyzer. Use it to choose the cleaner PRPS frequency range.

clear; clc; close all;

%% ============================================================
% User options
% ============================================================

DATASET_STATUS = "candidate";   % "candidate", "accepted", "rejected", or "diagnostics"

USE_LATEST_CSV = true;

% Used only when USE_LATEST_CSV = false
dataFile = "servo_prbs_seed3001.csv";

SAVE_FIGURES = false;

% Analyze all PRBS runs in the file.
% Leave empty string array to auto-detect all run_name values.
runNames = strings(0, 1);

% Use only PRBS segment for frequency-domain identification.
USE_ONLY_PRBS_SEGMENT = true;

% Nominal command/sample period from Pico script.
USE_NOMINAL_DT = true;
NOMINAL_DT_S = 0.020;

% FRF estimation options.
WINDOW_LENGTH_S = 12.0;
WINDOW_OVERLAP_FRACTION = 0.50;

% Frequency range to inspect.
FREQ_MIN_HZ = 0.05;
FREQ_MAX_HZ = 20.0;

% Reject weak/noisy frequency points.
MIN_COHERENCE_FOR_FIT = 0.55;
MIN_INPUT_RELATIVE_MAG = 0.02;

% Bandwidth threshold.
BANDWIDTH_DROP_DB = -3.0;

% Model fitting toggles.
FIT_FIRST_ORDER = true;
FIT_SECOND_ORDER_LAG = true;
FIT_FIRST_ORDER_DELAY = true;
FIT_SECOND_ORDER_DELAY = true;

% Initial guesses.
INITIAL_K_RAD_PER_US = pi / 2000;   % From 450->2450 us = 180 deg
INITIAL_TAU_S = 0.08;
INITIAL_TAU2_S = 0.02;
INITIAL_DELAY_S = 0.02;

% Soft parameter limits.
MIN_TAU_S = 0.005;
MAX_TAU_S = 2.0;
MIN_DELAY_S = 0.0;
MAX_DELAY_S = 0.50;
MIN_GAIN_ABS = 1e-5;
MAX_GAIN_ABS = 0.01;

% Rate-limit diagnostic.
% If measured angular velocity repeatedly clusters near a max slope,
% the data may be large-signal/rate-limited rather than linear small-signal.
RATE_LIMIT_PERCENTILE = 95;
RATE_LIMIT_FLATNESS_THRESHOLD = 0.30;

%% ============================================================
% Locate project paths
% ============================================================

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
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

%% ============================================================
% Load data
% ============================================================

T = readtable(dataPath);

requiredVars = [
    "t_s"
    "run_name"
    "segment"
    "servo_us"
    "command_delta_us"
    "theta_rad"
];

for k = 1:numel(requiredVars)
    if ~ismember(requiredVars(k), string(T.Properties.VariableNames))
        error("Missing required CSV column: %s", requiredVars(k));
    end
end

T.run_name = string(T.run_name);
T.segment = string(T.segment);

if isempty(runNames)
    runNames = unique(T.run_name, "stable");
end

fprintf("\nDetected runs:\n");
for k = 1:numel(runNames)
    fprintf("  %s\n", runNames(k));
end

%% ============================================================
% Main loop
% ============================================================

summaryRows = cell(0, 14);

for r = 1:numel(runNames)
    runName = runNames(r);

    fprintf("\n============================================================\n");
    fprintf("Run: %s\n", runName);
    fprintf("============================================================\n");

    D = T(T.run_name == runName, :);

    if USE_ONLY_PRBS_SEGMENT
        D = D(D.segment == "prbs", :);
    end

    if height(D) < 50
        warning("Skipping %s: not enough samples.", runName);
        continue;
    end

    % Force clean numeric vectors.
    t = double(D.t_s);
    u = double(D.command_delta_us);
    y = double(D.theta_rad);
    y_deg = y * 180/pi;

    valid = isfinite(t) & isfinite(u) & isfinite(y);
    t = t(valid);
    u = u(valid);
    y = y(valid);
    y_deg = y_deg(valid);

    % Reset time per run.
    t = t - t(1);

    if USE_NOMINAL_DT
        dt = NOMINAL_DT_S;
    else
        dt = median(diff(t), "omitnan");
    end

    if ~isfinite(dt) || dt <= 0
        warning("Skipping %s: invalid dt.", runName);
        continue;
    end

    Fs = 1 / dt;

    % Resample onto a uniform grid. This makes the FFT-based estimator safer.
    tUniform = (0:dt:t(end)).';
    uUniform = interp1(t, u, tUniform, "previous", "extrap");
    yUniform = interp1(t, y, tUniform, "linear", "extrap");
    yDegUniform = interp1(t, y_deg, tUniform, "linear", "extrap");

    % Remove means for dynamic analysis.
    u0 = uUniform - mean(uUniform, "omitnan");
    y0 = yUniform - mean(yUniform, "omitnan");

    %% ========================================================
    % Empirical FRF estimate using windowed cross spectra
    % ========================================================

    windowLengthSamples = max(32, round(WINDOW_LENGTH_S / dt));
    windowLengthSamples = min(windowLengthSamples, numel(u0));

    overlapSamples = round(WINDOW_OVERLAP_FRACTION * windowLengthSamples);
    stepSamples = max(1, windowLengthSamples - overlapSamples);

    [frf, fHz, coherence, Suu] = estimateWindowedFrf(u0, y0, Fs, windowLengthSamples, stepSamples);

    mag = abs(frf);
    phaseDeg = unwrap(angle(frf)) * 180/pi;
    magDb = 20*log10(mag);

    % Keep useful range.
    validFreq = fHz >= FREQ_MIN_HZ & ...
                fHz <= FREQ_MAX_HZ & ...
                isfinite(frf) & ...
                isfinite(coherence) & ...
                isfinite(Suu);

    if ~any(validFreq)
        warning("No valid frequency points for %s.", runName);
        continue;
    end

    maxSuu = max(Suu(validFreq), [], "omitnan");
    inputStrong = Suu >= maxSuu * MIN_INPUT_RELATIVE_MAG;

    fitMask = validFreq & ...
              inputStrong & ...
              coherence >= MIN_COHERENCE_FOR_FIT;

    if nnz(fitMask) < 3
        warning("Fewer than 3 good FRF points for %s. Fit may be unreliable.", runName);
    end

    %% ========================================================
    % Approximate bandwidth
    % ========================================================

    lowFreqMask = validFreq & inputStrong & coherence >= MIN_COHERENCE_FOR_FIT;

    if any(lowFreqMask)
        lowFreqCandidates = fHz(lowFreqMask);
        lowFreqGainDb = magDb(lowFreqMask);

        % Use median of lowest few good points as DC-ish gain estimate.
        [lowFreqCandidatesSorted, sortIdx] = sort(lowFreqCandidates);
        lowGainSorted = lowFreqGainDb(sortIdx);

        nLow = min(3, numel(lowGainSorted));
        dcGainDbApprox = median(lowGainSorted(1:nLow), "omitnan");

        dropDb = magDb - dcGainDbApprox;
        bwIdx = find(validFreq & inputStrong & coherence >= MIN_COHERENCE_FOR_FIT & ...
                     dropDb <= BANDWIDTH_DROP_DB, 1, "first");

        if isempty(bwIdx)
            bandwidthHz = NaN;
        else
            bandwidthHz = fHz(bwIdx);
        end
    else
        dcGainDbApprox = NaN;
        bandwidthHz = NaN;
    end

    %% ========================================================
    % Approximate delay from phase slope
    % ========================================================

    % Delay contributes phase:
    %
    %   phase_rad = -w * delay
    %
    % Therefore:
    %
    %   delay = -slope(phase_rad vs w)
    %
    % Use moderate-good points below/near bandwidth when possible.

    delayFitMask = validFreq & inputStrong & coherence >= MIN_COHERENCE_FOR_FIT;

    if isfinite(bandwidthHz)
        delayFitMask = delayFitMask & fHz <= max(1.5 * bandwidthHz, FREQ_MIN_HZ);
    else
        delayFitMask = delayFitMask & fHz <= 5.0;
    end

    if nnz(delayFitMask) >= 3
        wDelay = 2*pi*fHz(delayFitMask);
        phaseRadDelay = unwrap(angle(frf(delayFitMask)));

        pDelay = polyfit(wDelay, phaseRadDelay, 1);
        delayApproxS = max(0, -pDelay(1));
    else
        delayApproxS = NaN;
    end

    %% ========================================================
    % Fit simple models
    % ========================================================

    models = struct([]);

    if nnz(fitMask) >= 3
        fFit = fHz(fitMask);
        GFit = frf(fitMask);
        cohFit = coherence(fitMask);

        if FIT_FIRST_ORDER
            models = appendModel(models, fitFrequencyModel( ...
                "first_order", fFit, GFit, cohFit, ...
                INITIAL_K_RAD_PER_US, INITIAL_TAU_S, INITIAL_TAU2_S, INITIAL_DELAY_S, ...
                MIN_TAU_S, MAX_TAU_S, MIN_DELAY_S, MAX_DELAY_S, MIN_GAIN_ABS, MAX_GAIN_ABS));
        end

        if FIT_FIRST_ORDER_DELAY
            models = appendModel(models, fitFrequencyModel( ...
                "first_order_delay", fFit, GFit, cohFit, ...
                INITIAL_K_RAD_PER_US, INITIAL_TAU_S, INITIAL_TAU2_S, INITIAL_DELAY_S, ...
                MIN_TAU_S, MAX_TAU_S, MIN_DELAY_S, MAX_DELAY_S, MIN_GAIN_ABS, MAX_GAIN_ABS));
        end

        if FIT_SECOND_ORDER_LAG
            models = appendModel(models, fitFrequencyModel( ...
                "second_order_lag", fFit, GFit, cohFit, ...
                INITIAL_K_RAD_PER_US, INITIAL_TAU_S, INITIAL_TAU2_S, INITIAL_DELAY_S, ...
                MIN_TAU_S, MAX_TAU_S, MIN_DELAY_S, MAX_DELAY_S, MIN_GAIN_ABS, MAX_GAIN_ABS));
        end

        if FIT_SECOND_ORDER_DELAY
            models = appendModel(models, fitFrequencyModel( ...
                "second_order_lag_delay", fFit, GFit, cohFit, ...
                INITIAL_K_RAD_PER_US, INITIAL_TAU_S, INITIAL_TAU2_S, INITIAL_DELAY_S, ...
                MIN_TAU_S, MAX_TAU_S, MIN_DELAY_S, MAX_DELAY_S, MIN_GAIN_ABS, MAX_GAIN_ABS));
        end
    end

    if ~isempty(models)
        [~, bestIdx] = min([models.weighted_error]);
        bestModel = models(bestIdx);
    else
        bestModel = struct();
        bestModel.model_type = "none";
        bestModel.K = NaN;
        bestModel.tau1_s = NaN;
        bestModel.tau2_s = NaN;
        bestModel.delay_s = NaN;
        bestModel.weighted_error = NaN;
    end

    %% ========================================================
    % Rate-limit diagnostic
    % ========================================================

    dy = gradient(yDegUniform, dt); % deg/s

    absDy = abs(dy);
    rateP95 = prctile(absDy, RATE_LIMIT_PERCENTILE);
    rateP50 = prctile(absDy, 50);

    if rateP95 > 0
        rateFlatness = rateP50 / rateP95;
    else
        rateFlatness = NaN;
    end

    % Also inspect how much time is spent near the high-rate region.
    nearRateLimitFraction = mean(absDy > 0.85 * rateP95, "omitnan");

    likelyRateLimited = nearRateLimitFraction > RATE_LIMIT_FLATNESS_THRESHOLD;

    %% ========================================================
    % Recommend PRPS frequency range
    % ========================================================

    % Conservative recommendation:
    %   lower bound: low enough to capture quasi-static gain
    %   upper bound: around 2x-3x measured bandwidth, unless coherence dies earlier
    %
    % If bandwidth is not found, use highest good coherent point as upper clue.

    goodFreqs = fHz(validFreq & inputStrong & coherence >= MIN_COHERENCE_FOR_FIT);

    if isempty(goodFreqs)
        prpsMinHz = 0.05;
        prpsMaxHz = 3.0;
    else
        prpsMinHz = max(0.05, min(goodFreqs));

        if isfinite(bandwidthHz)
            prpsMaxHz = min(max(goodFreqs), 3.0 * bandwidthHz);
        else
            prpsMaxHz = min(max(goodFreqs), 5.0);
        end

        prpsMaxHz = max(prpsMaxHz, 1.0);
    end

    %% ========================================================
    % Minimal plots
    % ========================================================

    % Plot 1: time-domain command and measured angle
    figure;
    yyaxis left;
    plot(tUniform, uUniform, "LineWidth", 1.0);
    ylabel("Command Delta (\mus)");

    yyaxis right;
    plot(tUniform, yDegUniform, "LineWidth", 1.0);
    ylabel("Measured Angle (deg)");

    grid on;
    xlabel("Time (s)");
    title("Servo PRBS Time Response - " + runName);

    % Plot 2: empirical Bode with best fit
    figure;

    subplot(2,1,1);
    semilogx(fHz(validFreq), magDb(validFreq), "o", "LineWidth", 1.0);
    hold on;
    plotModelIfAvailable(bestModel, fHz(validFreq), "mag");
    if isfinite(bandwidthHz)
        xline(bandwidthHz, "--", sprintf("BW %.2f Hz", bandwidthHz));
    end
    grid on;
    ylabel("Magnitude (dB rad/\mus)");
    title("Empirical Servo FRF - " + runName);

    subplot(2,1,2);
    semilogx(fHz(validFreq), phaseDeg(validFreq), "o", "LineWidth", 1.0);
    hold on;
    plotModelIfAvailable(bestModel, fHz(validFreq), "phase");
    grid on;
    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");

    % Plot 3: coherence and rate diagnostic
    figure;

    subplot(2,1,1);
    semilogx(fHz(validFreq), coherence(validFreq), "o-", "LineWidth", 1.0);
    hold on;
    yline(MIN_COHERENCE_FOR_FIT, "--", "Coherence fit threshold");
    grid on;
    xlabel("Frequency (Hz)");
    ylabel("Coherence");
    title("Coherence - " + runName);

    subplot(2,1,2);
    plot(tUniform, dy, "LineWidth", 1.0);
    hold on;
    yline(rateP95, "--", "95th percentile");
    yline(-rateP95, "--", "-95th percentile");
    grid on;
    xlabel("Time (s)");
    ylabel("Angular Velocity (deg/s)");
    title("Rate-Limit Diagnostic - " + runName);

    %% ========================================================
    % Print summary
    % ========================================================

    fprintf("\nRun summary: %s\n", runName);
    fprintf("  Samples used: %d\n", numel(tUniform));
    fprintf("  Nominal dt: %.4f s\n", dt);
    fprintf("  Sample rate: %.2f Hz\n", Fs);

    fprintf("\nEmpirical bandwidth estimate:\n");
    fprintf("  Approx low-frequency gain: %.3f dB rad/us\n", dcGainDbApprox);
    if isfinite(bandwidthHz)
        fprintf("  Approx -3 dB bandwidth: %.3f Hz\n", bandwidthHz);
    else
        fprintf("  Approx -3 dB bandwidth: not found in usable range\n");
    end

    fprintf("\nApprox delay estimate:\n");
    if isfinite(delayApproxS)
        fprintf("  Phase-slope delay estimate: %.4f s\n", delayApproxS);
    else
        fprintf("  Phase-slope delay estimate: unavailable\n");
    end

    fprintf("\nBest simple model:\n");
    fprintf("  Type: %s\n", bestModel.model_type);
    fprintf("  K: %.9g rad/us\n", bestModel.K);
    fprintf("  tau1: %.5f s\n", bestModel.tau1_s);
    fprintf("  tau2: %.5f s\n", bestModel.tau2_s);
    fprintf("  delay: %.5f s\n", bestModel.delay_s);
    fprintf("  weighted error: %.5f\n", bestModel.weighted_error);

    fprintf("\nRate-limit diagnostic:\n");
    fprintf("  95th percentile abs angular velocity: %.2f deg/s\n", rateP95);
    fprintf("  near-rate-limit fraction: %.3f\n", nearRateLimitFraction);

    if likelyRateLimited
        fprintf("  WARNING: response may be rate-limited at this amplitude.\n");
        fprintf("  Consider rerunning with smaller amplitude, e.g. 50 us.\n");
    else
        fprintf("  No strong rate-limit flag from this simple diagnostic.\n");
    end

    fprintf("\nRecommended PRPS scouting range:\n");
    fprintf("  FREQ_MIN_HZ = %.3f\n", prpsMinHz);
    fprintf("  FREQ_MAX_HZ = %.3f\n", prpsMaxHz);

    fprintf("\nInterpretation:\n");
    if bestModel.model_type == "first_order" || bestModel.model_type == "first_order_delay"
        fprintf("  Servo looks roughly first-order over the reliable frequency range.\n");
    elseif bestModel.model_type == "second_order_lag" || bestModel.model_type == "second_order_lag_delay"
        fprintf("  Servo likely needs at least a second-order lag model over this range.\n");
    else
        fprintf("  Model classification unavailable.\n");
    end

    summaryRows = [summaryRows; {
        runName, ...
        bandwidthHz, ...
        delayApproxS, ...
        string(bestModel.model_type), ...
        bestModel.K, ...
        bestModel.tau1_s, ...
        bestModel.tau2_s, ...
        bestModel.delay_s, ...
        bestModel.weighted_error, ...
        rateP95, ...
        nearRateLimitFraction, ...
        likelyRateLimited, ...
        prpsMinHz, ...
        prpsMaxHz
    }];
end

%% ============================================================
% Summary table
% ============================================================

if ~isempty(summaryRows)

    summaryVariableNames = { ...
        'run_name', ...
        'bandwidth_Hz', ...
        'phase_slope_delay_s', ...
        'best_model_type', ...
        'K_rad_per_us', ...
        'tau1_s', ...
        'tau2_s', ...
        'model_delay_s', ...
        'weighted_error', ...
        'rate_p95_deg_per_s', ...
        'near_rate_limit_fraction', ...
        'likely_rate_limited', ...
        'recommended_prps_min_Hz', ...
        'recommended_prps_max_Hz' ...
    };

    summaryTable = cell2table(summaryRows, ...
        'VariableNames', summaryVariableNames);

    fprintf("\n============================================================\n");
    fprintf("Overall summary\n");
    fprintf("============================================================\n");
    disp(summaryTable);
end

%% ============================================================
% Save figures
% ============================================================

if SAVE_FIGURES
    if ~exist(plotDir, "dir")
        mkdir(plotDir);
    end

    saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);

    fprintf("\nSaved figures to:\n  %s\n", plotDir);
end

%% ============================================================
% Local helper functions
% ============================================================

function [G, fHz, coherence, Suu] = estimateWindowedFrf(u, y, Fs, windowLengthSamples, stepSamples)
    u = u(:);
    y = y(:);

    N = numel(u);

    windowLengthSamples = min(windowLengthSamples, N);
    stepSamples = max(1, stepSamples);

    starts = 1:stepSamples:(N - windowLengthSamples + 1);

    if isempty(starts)
        starts = 1;
    end

    nfft = windowLengthSamples;
    positiveBins = (1:floor(nfft/2)).';

    Syu_acc = zeros(numel(positiveBins), 1);
    Suu_acc = zeros(numel(positiveBins), 1);
    Syy_acc = zeros(numel(positiveBins), 1);

    nAvg = 0;

    w = hann(windowLengthSamples);

    for k = 1:numel(starts)
        idx = starts(k):(starts(k) + windowLengthSamples - 1);

        uSeg = u(idx);
        ySeg = y(idx);

        uSeg = uSeg - mean(uSeg, "omitnan");
        ySeg = ySeg - mean(ySeg, "omitnan");

        uSeg = uSeg .* w;
        ySeg = ySeg .* w;

        U = fft(uSeg, nfft);
        Y = fft(ySeg, nfft);

        U = U(positiveBins);
        Y = Y(positiveBins);

        Syu_acc = Syu_acc + Y .* conj(U);
        Suu_acc = Suu_acc + abs(U).^2;
        Syy_acc = Syy_acc + abs(Y).^2;

        nAvg = nAvg + 1;
    end

    Syu = Syu_acc / max(nAvg, 1);
    Suu = Suu_acc / max(nAvg, 1);
    Syy = Syy_acc / max(nAvg, 1);

    G = Syu ./ max(Suu, eps);
    coherence = abs(Syu).^2 ./ max(Suu .* Syy, eps);

    fHz = positiveBins * Fs / nfft;
end

function model = fitFrequencyModel( ...
    modelType, fHz, Gemp, coherence, ...
    initialK, initialTau1, initialTau2, initialDelay, ...
    minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs)

    w = 2*pi*fHz(:);
    Gemp = Gemp(:);
    coherence = coherence(:);

    p0 = packParams(modelType, initialK, initialTau1, initialTau2, initialDelay);

    gainFloor = 0.20 * median(abs(Gemp), "omitnan");
    if ~isfinite(gainFloor) || gainFloor <= 0
        gainFloor = minGainAbs;
    end

    costFun = @(p) frequencyFitCost( ...
        p, modelType, w, Gemp, coherence, gainFloor, ...
        minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    opts = optimset( ...
        "Display", "off", ...
        "MaxIter", 5000, ...
        "MaxFunEvals", 10000, ...
        "TolX", 1e-10, ...
        "TolFun", 1e-12);

    [pBest, bestCost] = fminsearch(costFun, p0, opts);

    params = unpackParams(modelType, pBest, minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    Gfit = evalModel(modelType, params, w);

    err = weightedError(Gfit, Gemp, coherence, gainFloor);
    weightedErr = sqrt(mean(abs(err).^2, "omitnan"));

    model.model_type = string(modelType);
    model.K = params.K;
    model.tau1_s = params.tau1_s;
    model.tau2_s = params.tau2_s;
    model.delay_s = params.delay_s;
    model.weighted_error = weightedErr;
    model.best_cost = bestCost;
    model.params = params;
end

function J = frequencyFitCost( ...
    p, modelType, w, Gemp, coherence, gainFloor, ...
    minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs)

    params = unpackParams(modelType, p, minTau, maxTau, minDelay, maxDelay, minGainAbs, maxGainAbs);

    Gfit = evalModel(modelType, params, w);

    err = weightedError(Gfit, Gemp, coherence, gainFloor);

    J = mean(abs(err).^2, "omitnan");

    if ~isfinite(J)
        J = 1e30;
    end
end

function err = weightedError(Gfit, Gemp, coherence, gainFloor)
    denom = max(abs(Gemp), gainFloor);
    cohWeight = sqrt(max(coherence, 0));
    err = cohWeight .* (Gfit - Gemp) ./ denom;
end

function p = packParams(modelType, K, tau1, tau2, delay)
    K = max(abs(K), 1e-12);
    tau1 = max(tau1, 1e-12);
    tau2 = max(tau2, 1e-12);
    delay = max(delay, 1e-6);

    switch string(modelType)
        case "first_order"
            p = [log(K), log(tau1)];

        case "first_order_delay"
            p = [log(K), log(tau1), log(delay)];

        case "second_order_lag"
            p = [log(K), log(tau1), log(tau2)];

        case "second_order_lag_delay"
            p = [log(K), log(tau1), log(tau2), log(delay)];

        otherwise
            error("Unknown modelType: %s", modelType);
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

function G = evalModel(modelType, params, w)
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

function x = expClamp(p, lo, hi)
    x = exp(p);
    x = min(max(x, lo), hi);
end

function models = appendModel(models, newModel)
    if isempty(models)
        models = newModel;
    else
        models(end+1) = newModel;
    end
end

function plotModelIfAvailable(model, fHz, plotType)
    if ~isfield(model, "model_type") || model.model_type == "none"
        return;
    end

    w = 2*pi*fHz(:);

    params.K = model.K;
    params.tau1_s = model.tau1_s;
    params.tau2_s = model.tau2_s;
    params.delay_s = model.delay_s;

    G = evalModel(model.model_type, params, w);

    switch string(plotType)
        case "mag"
            semilogx(fHz, 20*log10(abs(G)), "-", "LineWidth", 1.5);

        case "phase"
            semilogx(fHz, unwrap(angle(G))*180/pi, "-", "LineWidth", 1.5);
    end
end