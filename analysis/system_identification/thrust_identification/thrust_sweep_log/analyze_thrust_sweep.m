%% analyze_thrust_sweep.m
% Static thrust characterization from stepped PWM sweep data.
%
% Expected CSV columns from thrust_sweep_log.py:
%   t_ms,t_s,sweep_index,run_name,segment,pwm_us,
%   raw_count,tared_count,force_N,
%   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
%   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
%   phase,step_index,direction
%
% Identification goal:
%   Characterize the static (steady-state) thrust curve:
%
%       F = f(u, V)
%
%   where:
%       F = thrust in Newtons
%       u = PWM command in microseconds
%       V = battery voltage in Volts (if available)
%
%   Polynomial fit (linear region, 1100-1950 us):
%       F = p1*u + p2*u^2 + ... + c   (degree 2 and degree 4)
%
%   Saturation behavior:
%       u < 1100 us: motor stalls, no thrust generated
%       u > 1950 us: ESC saturates, no further thrust gain
%
%   Hysteresis check:
%       Compare up-sweep vs down-sweep mean thrust at each setpoint.

clearvars -except DATASET_STATUS FILE_SELECTION DATA_FILES SAVE_FIGURES;
clc; close all;

%% User options

if ~exist("DATASET_STATUS", "var"), DATASET_STATUS = "candidate"; end
if ~exist("FILE_SELECTION", "var"), FILE_SELECTION = "latest"; end  % "latest", "all", or "manual"
if ~exist("DATA_FILES", "var"), DATA_FILES = strings(0, 1); end
if ~exist("SAVE_FIGURES", "var"), SAVE_FIGURES = true; end

% Older CSVs did not log throttle_pct. These bounds reproduce the firmware
% mapping when throttle_pct must be reconstructed from pwm_us.
PWM_SAFE_US = 1000;
PWM_MAX_US = 2000;

% Run names are auto-detected from the selected CSVs after loading.
runNames = strings(0, 1);

%% Saturation limits (informational — used for plot markers only)

PWM_SAT_LOW_US  = 1075;   % motor stalls below this
PWM_SAT_HIGH_US = 1950;   % ESC saturated above this

%% Polynomial fit options

% PWM range over which polynomial fits are computed.
FIT_MIN_US = 1100;
FIT_MAX_US = 1950;

% Polynomial degrees to fit and compare.
FIT_DEGREES = [2, 4];

% Sub-ranges for linearity check: each row is [lo_us, hi_us].
% RMSE of a degree-1 fit over each range is compared against the global poly fits.
LINEARITY_RANGES = [1075, 1650; 1650, 1950];

%% Plot toggles

PLOT_STATIC_CURVE           = true;   % mean ± std thrust vs PWM
PLOT_STATIC_CURVE_THROTTLE  = true;   % mean and uncertainty vs throttle_pct
PLOT_RAW_SCATTER            = true;   % raw thrust samples colored by PWM
PLOT_UNCERTAINTY_WIDTH      = true;   % uncertainty half-width vs throttle_pct
PLOT_HYSTERESIS             = true;   % up vs down sweep overlay
PLOT_POLYNOMIAL_FIT         = true;   % polynomial fit over linear region
PLOT_REPEATABILITY_OVERLAY  = true;   % one curve per CSV file
PLOT_VOLTAGE_VS_THRUST      = true;   % voltage effects scatter
PLOT_LINEARITY_CHECK        = true;   % local linear fits vs global poly over sub-ranges
PLOT_SAMPLE_TIME_DIAGNOSTIC = false;

%% Locate mirrored folders

scriptPath = mfilename("fullpath");
repoRoot = findRepoRoot(scriptPath);
addpath(fullfile(repoRoot, "analysis", "utils"));

dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);
processedDir = getMirroredProcessedDir(scriptPath);

csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No CSV files found in:\n%s", dataDir);
end

dataFiles = selectDataFiles(csvFiles, FILE_SELECTION, DATA_FILES);

fprintf("\nThrust sweep analysis\n");
fprintf("  Data folder: %s\n", dataDir);
fprintf("  Dataset status: %s\n", DATASET_STATUS);
fprintf("  File selection: %s\n", FILE_SELECTION);
fprintf("  Selected CSV file(s): %d\n", numel(dataFiles));
for k = 1:numel(dataFiles)
    fprintf("  %s\n", fullfile(dataDir, dataFiles(k)));
end

%% Load and combine all CSVs

Tall = readCombinedCsvTables(dataDir, dataFiles, PWM_SAFE_US, PWM_MAX_US);

if isempty(runNames)
    runNames = unique(string(Tall.run_name), "stable");
    runNames(strlength(runNames) == 0) = [];
end

if isempty(runNames)
    runNames = "all_runs";
    Tall.run_name = repmat(runNames, height(Tall), 1);
end

%% Main analysis loop over run names

for r = 1:numel(runNames)
    runName = runNames(r);

    fprintf("\n============================================================\n");
    fprintf("Run: %s\n", runName);
    fprintf("============================================================\n");

    D = getSweepData(Tall, runName);

    if height(D) < 5
        warning("Skipping %s: not enough sweep samples.", runName);
        continue;
    end

    hasVolt = hasVoltage(D);

    %% Per-setpoint statistics (combined up + down)

    statsAll  = computeSetpointStats(D, "all");
    statsUp   = computeSetpointStats(D, "up");
    statsDown = computeSetpointStats(D, "down");

    fprintf("\nSetpoint statistics for %s:\n", runName);
    fprintf("  Total sweep setpoints (combined): %d\n", height(statsAll));
    fprintf("  Up-sweep setpoints: %d\n", height(statsUp));
    fprintf("  Down-sweep setpoints: %d\n", height(statsDown));
    fprintf("  PWM range: %.0f to %.0f us\n", min(D.pwm_us), max(D.pwm_us));
    fprintf("  Force range: %.3f to %.3f N\n", min(D.force_N), max(D.force_N));

    if hasVolt
        fprintf("  Voltage range: %.2f to %.2f V\n", ...
            min(D.battery_voltage_V, [], "omitnan"), ...
            max(D.battery_voltage_V, [], "omitnan"));
    end

    %% Polynomial fit in linear region

    fitResults = struct();

    inLinear = statsAll.pwm_us >= FIT_MIN_US & statsAll.pwm_us <= FIT_MAX_US;

    uFit = statsAll.pwm_us(inLinear);
    yFit = statsAll.mean_force_N(inLinear);

    if numel(uFit) >= 3
        for di = 1:numel(FIT_DEGREES)
            deg = FIT_DEGREES(di);
            % Use 3-output form: mu(1)=mean(uFit), mu(2)=std(uFit).
            % polyfit centers and scales internally; polyval must receive
            % the same mu to evaluate correctly at raw us values.
            [p, S, mu] = polyfit(uFit, yFit, deg);

            yHat = polyval(p, uFit, S, mu);
            resid = yFit - yHat;
            rmse = sqrt(mean(resid.^2, "omitnan"));
            mae  = mean(abs(resid), "omitnan");

            key = sprintf("deg%d", deg);
            fitResults.(key).degree = deg;
            fitResults.(key).p  = p;
            fitResults.(key).S  = S;
            fitResults.(key).mu = mu;
            fitResults.(key).rmse_N = rmse;
            fitResults.(key).mae_N  = mae;
            fitResults.(key).u_fit  = uFit;
            fitResults.(key).y_fit  = yFit;

            fprintf("\nDegree-%d polynomial fit (%d to %d us):\n", deg, FIT_MIN_US, FIT_MAX_US);
            fprintf("  Normalization: mean=%.1f us, std=%.1f us\n", mu(1), mu(2));
            fprintf("  Normalized coefficients (highest power first): ");
            fprintf("%.6g  ", p);
            fprintf("\n");
            fprintf("  RMSE = %.6f N\n", rmse);
            fprintf("  MAE  = %.6f N\n", mae);
        end
    else
        warning("Not enough setpoints in linear region [%d, %d] us for polynomial fit.", ...
            FIT_MIN_US, FIT_MAX_US);
    end

    %% Linearity check: local degree-1 fit vs global poly fits per sub-range

    linearityResults = struct();
    globalFitKeys = fieldnames(fitResults);

    fprintf("\nLinearity check for %s:\n", runName);

    for li = 1:size(LINEARITY_RANGES, 1)
        lo = LINEARITY_RANGES(li, 1);
        hi = LINEARITY_RANGES(li, 2);

        inRange = statsAll.pwm_us >= lo & statsAll.pwm_us <= hi;
        uSub = statsAll.pwm_us(inRange);
        ySub = statsAll.mean_force_N(inRange);

        if numel(uSub) < 2
            warning("Not enough setpoints in linearity range [%d, %d] us.", lo, hi);
            continue;
        end

        % Degree-1 fit with normalization.
        [pLin, SLin, muLin] = polyfit(uSub, ySub, 1);
        yHatLin = polyval(pLin, uSub, SLin, muLin);
        residLin = ySub - yHatLin;
        rmseLin = sqrt(mean(residLin.^2, "omitnan"));
        maeLin  = mean(abs(residLin), "omitnan");

        % Denormalized slope K_T = p1 / sigma.
        KT = pLin(1) / muLin(2);

        % Evaluate each global poly fit over the same sub-range for comparison.
        globalRmse = struct();
        for di = 1:numel(globalFitKeys)
            gkey = globalFitKeys{di};
            fr = fitResults.(gkey);
            inGlobal = fr.u_fit >= lo & fr.u_fit <= hi;
            if any(inGlobal)
                yHatG = polyval(fr.p, fr.u_fit(inGlobal), fr.S, fr.mu);
                residG = fr.y_fit(inGlobal) - yHatG;
                globalRmse.(gkey) = sqrt(mean(residG.^2, "omitnan"));
            end
        end

        rkey = sprintf("r%d_%d", lo, hi);
        linearityResults.(rkey).lo         = lo;
        linearityResults.(rkey).hi         = hi;
        linearityResults.(rkey).p          = pLin;
        linearityResults.(rkey).S          = SLin;
        linearityResults.(rkey).mu         = muLin;
        linearityResults.(rkey).KT_N_per_us = KT;
        linearityResults.(rkey).rmse_N     = rmseLin;
        linearityResults.(rkey).mae_N      = maeLin;
        linearityResults.(rkey).globalRmse = globalRmse;

        fprintf("\n  Range %d-%d us  (K_T = %.5f N/us):\n", lo, hi, KT);
        fprintf("    Local linear RMSE = %.6f N  MAE = %.6f N\n", rmseLin, maeLin);
        for di = 1:numel(globalFitKeys)
            gkey = globalFitKeys{di};
            if isfield(globalRmse, gkey)
                fprintf("    Global deg-%d RMSE over same range = %.6f N\n", ...
                    fitResults.(gkey).degree, globalRmse.(gkey));
            end
        end
    end

    %% Static thrust curve (all data, mean ± std)

    if PLOT_RAW_SCATTER && height(D) > 0
        figure("Color", "k");
        scatter(D.throttle_pct, D.force_N, 14, D.pwm_us, "filled", ...
            "MarkerFaceAlpha", 0.45, "MarkerEdgeAlpha", 0.45);
        grid on;
        cb = colorbar;
        cb.Label.String = "PWM command (\mus)";
        colormap("turbo");
        xlabel("Throttle command (%)");
        ylabel("Thrust (N)");
        title("Raw Thrust Samples - " + runName, "Interpreter", "none");
        styleDarkAxes(gca);
    end

    if PLOT_STATIC_CURVE && height(statsAll) > 0
        figure("Color", "k");
        hold on; grid on;

        drawUncertaintyBand(statsAll.pwm_us, ...
            statsAll.lower_prediction95_N, statsAll.upper_prediction95_N, ...
            [0.0 0.9 1.0], 0.16, "95% sample spread");
        drawUncertaintyBand(statsAll.pwm_us, ...
            statsAll.lower_mean_ci95_N, statsAll.upper_mean_ci95_N, ...
            [0.45 1.0 0.45], 0.24, "95% mean CI");

        errorbar(statsAll.pwm_us, statsAll.mean_force_N, statsAll.std_force_N, ...
            "o", "Color", [0.85 0.85 0.85], "LineWidth", 1.2, "DisplayName", "Mean ± std (all sweeps)");

        xline(PWM_SAT_LOW_US, "--r", "LineWidth", 1.1, ...
            "DisplayName", sprintf("Stall threshold (%d us)", PWM_SAT_LOW_US));
        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.1, ...
            "DisplayName", sprintf("ESC max (%d us)", PWM_SAT_HIGH_US));

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Static Thrust Curve - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none", "TextColor", "w");
        styleDarkAxes(gca);
    end

    if PLOT_STATIC_CURVE_THROTTLE && height(statsAll) > 0
        figure("Color", "k");
        hold on; grid on;

        drawUncertaintyBand(statsAll.throttle_pct, ...
            statsAll.lower_prediction95_N, statsAll.upper_prediction95_N, ...
            [0.0 0.9 1.0], 0.16, "95% sample spread");
        drawUncertaintyBand(statsAll.throttle_pct, ...
            statsAll.lower_mean_ci95_N, statsAll.upper_mean_ci95_N, ...
            [0.45 1.0 0.45], 0.24, "95% mean CI");
        errorbar(statsAll.throttle_pct, statsAll.mean_force_N, statsAll.mean_ci95_halfwidth_N, ...
            "o-", "Color", [1.0 0.65 0.0], "MarkerFaceColor", [1.0 0.65 0.0], ...
            "LineWidth", 1.4, "DisplayName", "Mean thrust");

        xlabel("Throttle command (%)");
        ylabel("Thrust (N)");
        title("Static Thrust Lookup vs Throttle - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none", "TextColor", "w");
        styleDarkAxes(gca);
    end

    if PLOT_UNCERTAINTY_WIDTH && height(statsAll) > 0
        figure("Color", "k");
        hold on; grid on;

        plot(statsAll.throttle_pct, statsAll.mean_ci95_halfwidth_N, "o-", ...
            "Color", [0.45 1.0 0.45], "LineWidth", 1.4, ...
            "DisplayName", "95% mean CI half-width");
        plot(statsAll.throttle_pct, statsAll.prediction95_halfwidth_N, "s-", ...
            "Color", [0.0 0.9 1.0], "LineWidth", 1.4, ...
            "DisplayName", "95% sample spread half-width");

        xlabel("Throttle command (%)");
        ylabel("Half-width (N)");
        title("Static Lookup Uncertainty Width - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none", "TextColor", "w");
        styleDarkAxes(gca);
    end

    %% Hysteresis plot

    if PLOT_HYSTERESIS && height(statsUp) > 0 && height(statsDown) > 0
        figure;
        hold on; grid on;

        plot(statsUp.pwm_us, statsUp.mean_force_N, "o-", "LineWidth", 1.5, ...
            "DisplayName", "Up sweep");
        plot(statsDown.pwm_us, statsDown.mean_force_N, "s--", "LineWidth", 1.5, ...
            "DisplayName", "Down sweep");

        xline(PWM_SAT_LOW_US, "--r", "LineWidth", 1.0, "HandleVisibility", "off");
        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.0, "HandleVisibility", "off");

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Hysteresis Check (Up vs Down) - " + runName, "Interpreter", "none");
        legend("Location", "best");

        % Quantify hysteresis at common setpoints.
        commonPwm = intersect(statsUp.pwm_us, statsDown.pwm_us);

        if ~isempty(commonPwm)
            hystErrors = zeros(numel(commonPwm), 1);

            for i = 1:numel(commonPwm)
                u = commonPwm(i);
                upForce   = statsUp.mean_force_N(statsUp.pwm_us == u);
                downForce = statsDown.mean_force_N(statsDown.pwm_us == u);

                if ~isempty(upForce) && ~isempty(downForce)
                    hystErrors(i) = abs(upForce(1) - downForce(1));
                end
            end

            validHyst = hystErrors(isfinite(hystErrors) & hystErrors > 0);

            fprintf("\nHysteresis summary for %s:\n", runName);
            fprintf("  Mean |up - down| = %.4f N\n", mean(validHyst, "omitnan"));
            fprintf("  Max  |up - down| = %.4f N\n", max(validHyst, [], "omitnan"));
        end
    end

    %% Polynomial fit plot

    if PLOT_POLYNOMIAL_FIT && numel(uFit) >= 3
        uFine = linspace(FIT_MIN_US, FIT_MAX_US, 500).';

        figure;
        hold on; grid on;

        plot(statsAll.pwm_us, statsAll.mean_force_N, "o", "Color", [0.85 0.85 0.85], ...
            "LineWidth", 1.2, "DisplayName", "Measured mean thrust");

        xline(PWM_SAT_LOW_US, "--r", "LineWidth", 1.0, "HandleVisibility", "off");
        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.0, "HandleVisibility", "off");

        fitKeys = fieldnames(fitResults);
        allDegrees = cellfun(@(k) fitResults.(k).degree, fitKeys);
        bestDeg = max(allDegrees);

        for di = 1:numel(fitKeys)
            key = fitKeys{di};
            fr = fitResults.(key);

            yFine = polyval(fr.p, uFine, fr.S, fr.mu);

            if fr.degree == bestDeg
                lineStyle = "-";
                lineWidth = 1.5;
            else
                lineStyle = "--";
                lineWidth = 1.2;
            end

            plot(uFine, yFine, lineStyle, "LineWidth", lineWidth, ...
                "DisplayName", sprintf("Degree-%d fit (RMSE=%.4f N)", fr.degree, fr.rmse_N));
        end

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Polynomial Fit to Static Thrust Curve - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");
    end

    %% Linearity check plot

    linRangeKeys = fieldnames(linearityResults);

    if PLOT_LINEARITY_CHECK && ~isempty(linRangeKeys)
        % Bright palette for dark-mode figures.
        brightColors = [
            0.00  0.90  1.00;   % cyan
            1.00  0.65  0.00;   % orange
            0.45  1.00  0.45;   % lime
            1.00  0.40  0.80;   % pink
        ];
        nColors = min(numel(linRangeKeys), size(brightColors, 1));

        figure;
        hold on; grid on;

        plot(statsAll.pwm_us, statsAll.mean_force_N, "o", "Color", [0.85 0.85 0.85], ...
            "LineWidth", 1.2, "DisplayName", "Measured mean thrust");

        % Global degree-4 reference: bright but de-emphasised with alpha.
        if isfield(fitResults, "deg4")
            uFine = linspace(FIT_MIN_US, FIT_MAX_US, 500).';
            yFineG = polyval(fitResults.deg4.p, uFine, fitResults.deg4.S, fitResults.deg4.mu);
            plot(uFine, yFineG, "-", "LineWidth", 1.5, "Color", [0.55 0.75 0.30 1.00], ...
                "DisplayName", sprintf("Global deg-4 (RMSE=%.4f N)", fitResults.deg4.rmse_N));
        end

        % Local linear fits, drawn only over their own sub-range.
        for li = 1:nColors
            rkey = linRangeKeys{li};
            lr = linearityResults.(rkey);
            c = brightColors(li, :);

            uSubFine = linspace(lr.lo, lr.hi, 200).';
            ySubFine = polyval(lr.p, uSubFine, lr.S, lr.mu);

            plot(uSubFine, ySubFine, "--", "LineWidth", 2.0, "Color", c, ...
                "DisplayName", sprintf("Linear %d-%d us  K_T=%.5f N/µs  RMSE=%.4f N", ...
                    lr.lo, lr.hi, lr.KT_N_per_us, lr.rmse_N));

            xline(lr.lo, ":", "Color", c, "LineWidth", 0.8, "HandleVisibility", "off");
        end

        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.0, "HandleVisibility", "off");

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Linearity Check - Local Linear Fits vs Global Poly - " + runName, ...
            "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");
    end

    %% Repeatability overlay (one curve per source CSV file)

    if PLOT_REPEATABILITY_OVERLAY && ...
       ismember("source_file_index", string(D.Properties.VariableNames))

        figure;
        hold on; grid on;

        fileIndices = unique(D.source_file_index, "stable");

        for fi = 1:numel(fileIndices)
            Df = D(D.source_file_index == fileIndices(fi), :);

            statsFile = computeSetpointStats(Df, "all");

            if height(statsFile) < 2
                continue;
            end

            if ismember("source_file", string(Df.Properties.VariableNames))
                fname = string(Df.source_file(1));
            else
                fname = sprintf("file_%d", fileIndices(fi));
            end

            plot(statsFile.pwm_us, statsFile.mean_force_N, "-o", "LineWidth", 1.1, ...
                "DisplayName", erase(fname, ".csv"));
        end

        xline(PWM_SAT_LOW_US, "--r", "LineWidth", 1.0, "HandleVisibility", "off");
        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.0, "HandleVisibility", "off");

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Repeatability Overlay (One Curve per CSV) - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");
    end

    %% Voltage effects

    if PLOT_VOLTAGE_VS_THRUST && hasVolt && height(statsAll) > 0
        % Scatter: mean thrust vs mean voltage per setpoint.
        figure;
        hold on; grid on;

        inLinearAll = D.pwm_us >= FIT_MIN_US & D.pwm_us <= FIT_MAX_US;
        Dlin = D(inLinearAll, :);

        if height(Dlin) > 0 && hasVoltage(Dlin)
            scatter(Dlin.battery_voltage_V, Dlin.force_N, 12, Dlin.pwm_us, "filled");

            cb = colorbar;
            cb.Label.String = "PWM command (\mus)";
            colormap("turbo");

            xlabel("Battery voltage (V)");
            ylabel("Thrust (N)");
            title("Thrust vs Voltage (colored by PWM, linear region) - " + runName, ...
                "Interpreter", "none");
        end
    end

    %% Sample-time diagnostic

    if PLOT_SAMPLE_TIME_DIAGNOSTIC
        t_s = forceNumeric(D.t_ms) / 1000;
        t_s = t_s(isfinite(t_s));

        if numel(t_s) >= 3
            t_s = t_s - t_s(1);
            dt = diff(t_s);
            dt = dt(isfinite(dt) & dt > 0);

            figure;
            hold on; grid on;

            plot(dt, "LineWidth", 1.0);

            xlabel("Sample index");
            ylabel("Sample interval \Deltat (s)");
            title("Sample-Time Diagnostic - " + runName, "Interpreter", "none");
        end
    end

    %% Summary table

    fprintf("\nSetpoint summary table for %s:\n", runName);

    summaryVars = {'pwm_us', 'mean_force_N', 'std_force_N', 'n_samples'};

    if ismember("throttle_pct", string(statsAll.Properties.VariableNames))
        summaryVars = [{'pwm_us', 'throttle_pct'}, setdiff(summaryVars, {'pwm_us'}, 'stable')];
    end

    uncertaintyVars = { ...
        'sem_force_N', ...
        'mean_ci95_halfwidth_N', ...
        'prediction95_halfwidth_N', ...
        'lower_mean_ci95_N', ...
        'upper_mean_ci95_N', ...
        'lower_prediction95_N', ...
        'upper_prediction95_N' ...
    };

    for uv = 1:numel(uncertaintyVars)
        if ismember(uncertaintyVars{uv}, string(statsAll.Properties.VariableNames))
            summaryVars{end+1} = uncertaintyVars{uv};
        end
    end

    if hasVolt && ismember("mean_voltage_V", string(statsAll.Properties.VariableNames))
        summaryVars{end+1} = 'mean_voltage_V';
    end

    disp(statsAll(:, summaryVars));

    writeLookupArtifacts(statsAll, D, processedDir, runName);

end

%% Save figures

saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);

%% Local helper functions

function dataFiles = selectDataFiles(csvFiles, fileSelection, manualFiles)
    fileSelection = string(fileSelection);

    switch fileSelection
        case "latest"
            [~, newestIdx] = max([csvFiles.datenum]);
            dataFiles = string(csvFiles(newestIdx).name);

        case "all"
            [~, sortIdx] = sort(string({csvFiles.name}));
            dataFiles = string({csvFiles(sortIdx).name});

        case "manual"
            dataFiles = string(manualFiles);
            if isempty(dataFiles)
                error("FILE_SELECTION='manual' requires DATA_FILES to contain CSV file names.");
            end

        otherwise
            error("FILE_SELECTION must be 'latest', 'all', or 'manual'.");
    end
end


function D = getSweepData(T, runName)
    if isempty(T)
        D = table();
        return;
    end

    idx = strcmp(string(T.run_name), runName) & ...
          strcmpi(string(T.phase), "sweep");

    D = T(idx, :);

    if isempty(D)
        return;
    end

    D.pwm_us  = forceNumeric(D.pwm_us);
    D.force_N = forceNumeric(D.force_N);
    D.t_ms    = forceNumeric(D.t_ms);

    if ismember("throttle_pct", string(D.Properties.VariableNames))
        D.throttle_pct = forceNumeric(D.throttle_pct);
    else
        D.throttle_pct = 100.0 * (D.pwm_us - 1000) / 1000;
    end

    if ismember("battery_voltage_V", string(D.Properties.VariableNames))
        D.battery_voltage_V = forceNumeric(D.battery_voltage_V);
    end

    if ismember("battery_current_A", string(D.Properties.VariableNames))
        D.battery_current_A = forceNumeric(D.battery_current_A);
    end

    valid = isfinite(D.pwm_us) & isfinite(D.throttle_pct) & isfinite(D.force_N);
    D = D(valid, :);

    if isempty(D)
        return;
    end

    D.t_ms = D.t_ms - D.t_ms(1);
end

function stats = computeSetpointStats(D, directionFilter)
    % Compute per-setpoint mean/std/median of force_N.
    % directionFilter: "all", "up", or "down"

    if isempty(D)
        stats = table();
        return;
    end

    if ~strcmp(directionFilter, "all") && ...
       ismember("direction", string(D.Properties.VariableNames))
        D = D(strcmpi(string(D.direction), directionFilter), :);
    end

    if isempty(D)
        stats = table();
        return;
    end

    pwmValues = unique(D.pwm_us, "sorted");

    nPwm = numel(pwmValues);

    mean_force_N  = NaN(nPwm, 1);
    std_force_N   = NaN(nPwm, 1);
    sem_force_N   = NaN(nPwm, 1);
    mean_ci95_halfwidth_N = NaN(nPwm, 1);
    lower_mean_ci95_N = NaN(nPwm, 1);
    upper_mean_ci95_N = NaN(nPwm, 1);
    prediction95_halfwidth_N = NaN(nPwm, 1);
    lower_prediction95_N = NaN(nPwm, 1);
    upper_prediction95_N = NaN(nPwm, 1);
    p05_force_N = NaN(nPwm, 1);
    p95_force_N = NaN(nPwm, 1);
    min_force_N = NaN(nPwm, 1);
    max_force_N = NaN(nPwm, 1);
    median_force_N = NaN(nPwm, 1);
    n_samples     = zeros(nPwm, 1);
    n_files       = zeros(nPwm, 1);
    throttle_pct  = NaN(nPwm, 1);
    mean_voltage_V = NaN(nPwm, 1);

    hasVolt = hasVoltage(D);

    for i = 1:nPwm
        idx = D.pwm_us == pwmValues(i);
        y = D.force_N(idx);
        y = y(isfinite(y));

        throttle_pct(i) = mean(D.throttle_pct(idx), "omitnan");
        mean_force_N(i)   = mean(y, "omitnan");
        std_force_N(i)    = std(y, "omitnan");
        median_force_N(i) = median(y, "omitnan");
        n_samples(i)      = numel(y);

        if n_samples(i) > 0
            sem_force_N(i) = std_force_N(i) / sqrt(n_samples(i));
            mean_ci95_halfwidth_N(i) = 1.96 * sem_force_N(i);
            lower_mean_ci95_N(i) = mean_force_N(i) - mean_ci95_halfwidth_N(i);
            upper_mean_ci95_N(i) = mean_force_N(i) + mean_ci95_halfwidth_N(i);
            prediction95_halfwidth_N(i) = 1.96 * std_force_N(i);
            lower_prediction95_N(i) = mean_force_N(i) - prediction95_halfwidth_N(i);
            upper_prediction95_N(i) = mean_force_N(i) + prediction95_halfwidth_N(i);
            p05_force_N(i) = percentileNoToolbox(y, 5);
            p95_force_N(i) = percentileNoToolbox(y, 95);
            min_force_N(i) = min(y);
            max_force_N(i) = max(y);
        end

        if ismember("source_file_index", string(D.Properties.VariableNames))
            n_files(i) = numel(unique(D.source_file_index(idx)));
        else
            n_files(i) = 1;
        end

        if hasVolt
            v = D.battery_voltage_V(idx);
            v = v(isfinite(v));
            mean_voltage_V(i) = mean(v, "omitnan");
        end
    end

    stats = table( ...
        pwmValues, throttle_pct, mean_force_N, median_force_N, std_force_N, sem_force_N, ...
        mean_ci95_halfwidth_N, lower_mean_ci95_N, upper_mean_ci95_N, ...
        prediction95_halfwidth_N, lower_prediction95_N, upper_prediction95_N, ...
        p05_force_N, p95_force_N, min_force_N, max_force_N, ...
        n_samples, n_files, mean_voltage_V, ...
        'VariableNames', { ...
            'pwm_us', ...
            'throttle_pct', ...
            'mean_force_N', ...
            'median_force_N', ...
            'std_force_N', ...
            'sem_force_N', ...
            'mean_ci95_halfwidth_N', ...
            'lower_mean_ci95_N', ...
            'upper_mean_ci95_N', ...
            'prediction95_halfwidth_N', ...
            'lower_prediction95_N', ...
            'upper_prediction95_N', ...
            'p05_force_N', ...
            'p95_force_N', ...
            'min_force_N', ...
            'max_force_N', ...
            'n_samples', ...
            'n_files', ...
            'mean_voltage_V' ...
        });
end

function T = readCombinedCsvTables(dataDir, dataFiles, pwmSafeUs, pwmMaxUs)
    tables = cell(numel(dataFiles), 1);
    allVarNames = strings(0, 1);

    for k = 1:numel(dataFiles)
        thisPath = fullfile(dataDir, dataFiles(k));
        if ~isfile(thisPath)
            error("Selected CSV not found:\n%s", thisPath);
        end

        Tk = readtable(thisPath);

        if ~ismember("run_name", string(Tk.Properties.VariableNames))
            Tk.run_name = repmat("all_runs", height(Tk), 1);
        end

        if ~ismember("phase", string(Tk.Properties.VariableNames))
            Tk.phase = repmat("sweep", height(Tk), 1);
        end

        if ~ismember("direction", string(Tk.Properties.VariableNames))
            Tk.direction = repmat("none", height(Tk), 1);
        end

        if ~ismember("throttle_pct", string(Tk.Properties.VariableNames))
            Tk.throttle_pct = 100.0 * (forceNumeric(Tk.pwm_us) - pwmSafeUs) / (pwmMaxUs - pwmSafeUs);
        end

        Tk.source_file = repmat(string(dataFiles(k)), height(Tk), 1);
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
                if any(varName == ["source_file", "run_name", "phase", "direction", "segment"])
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

function tf = hasVoltage(D)
    tf = ismember("battery_voltage_V", string(D.Properties.VariableNames)) && ...
         any(isfinite(forceNumeric(D.battery_voltage_V)));
end

function writeLookupArtifacts(statsAll, D, processedDir, runName)
    safeRunName = matlab.lang.makeValidName(char(runName));

    lookupPath = fullfile(processedDir, "thrust_static_lookup_" + string(safeRunName) + ".csv");
    samplePath = fullfile(processedDir, "thrust_static_samples_" + string(safeRunName) + ".csv");
    matPath = fullfile(processedDir, "thrust_static_lookup_" + string(safeRunName) + ".mat");

    writetable(statsAll, lookupPath);
    writetable(D, samplePath);
    save(matPath, "statsAll", "D");

    fprintf("\nWrote processed lookup artifacts for %s:\n", runName);
    fprintf("  %s\n", lookupPath);
    fprintf("  %s\n", samplePath);
    fprintf("  %s\n", matPath);
end

function drawUncertaintyBand(x, yLo, yHi, color, alphaValue, label)
    valid = isfinite(x) & isfinite(yLo) & isfinite(yHi);
    x = x(valid);
    yLo = yLo(valid);
    yHi = yHi(valid);

    if numel(x) < 2
        return;
    end

    fill([x; flipud(x)], [yLo; flipud(yHi)], color, ...
        "FaceAlpha", alphaValue, "EdgeColor", "none", ...
        "DisplayName", label);
end

function q = percentileNoToolbox(x, pct)
    x = sort(x(isfinite(x)));

    if isempty(x)
        q = NaN;
        return;
    end

    if numel(x) == 1
        q = x(1);
        return;
    end

    rank = 1 + (pct / 100) * (numel(x) - 1);
    lo = floor(rank);
    hi = ceil(rank);

    if lo == hi
        q = x(lo);
    else
        q = x(lo) + (rank - lo) * (x(hi) - x(lo));
    end
end

function processedDir = getMirroredProcessedDir(scriptPath)
    scriptDir = fileparts(scriptPath);
    repoRoot = findRepoRoot(scriptDir);
    relativeAnalysisDir = getRelativeAnalysisDir(scriptPath);
    processedDir = fullfile(repoRoot, "data", "processed", relativeAnalysisDir);

    if ~isfolder(processedDir)
        mkdir(processedDir);
    end
end

function styleDarkAxes(ax)
    ax.Color = "k";
    ax.XColor = "w";
    ax.YColor = "w";
    ax.GridColor = [0.8 0.8 0.8];
    ax.MinorGridColor = [0.5 0.5 0.5];
    ax.GridAlpha = 0.28;
    ax.MinorGridAlpha = 0.18;
    ax.Title.Color = "w";
    ax.XLabel.Color = "w";
    ax.YLabel.Color = "w";
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
