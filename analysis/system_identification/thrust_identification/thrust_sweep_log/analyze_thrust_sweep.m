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

clear; clc; close all;

%% User options

SAVE_FIGURES = true;

runNames = [
    "global_1000_2000"
];

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
PLOT_HYSTERESIS             = true;   % up vs down sweep overlay
PLOT_POLYNOMIAL_FIT         = true;   % polynomial fit over linear region
PLOT_REPEATABILITY_OVERLAY  = true;   % one curve per CSV file
PLOT_VOLTAGE_VS_THRUST      = true;   % voltage effects scatter
PLOT_LINEARITY_CHECK        = true;   % local linear fits vs global poly over sub-ranges
PLOT_SAMPLE_TIME_DIAGNOSTIC = false;

%% Locate mirrored folders

scriptPath = mfilename("fullpath");

dataDir = getMirroredRawDataDir(scriptPath, "candidate");
plotDir = getMirroredPlotDir(scriptPath);

addpath(genpath(fullfile(findRepoRoot(scriptPath), "analysis", "util")));

csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No CSV files found in:\n%s", dataDir);
end

[~, sortIdx] = sort(string({csvFiles.name}));
csvFiles = csvFiles(sortIdx);

fprintf("\nFound %d CSV file(s):\n", numel(csvFiles));
for k = 1:numel(csvFiles)
    fprintf("  %s\n", fullfile(csvFiles(k).folder, csvFiles(k).name));
end

%% Load and combine all CSVs

Tall = readCombinedCsvTables(csvFiles);

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

    if PLOT_STATIC_CURVE && height(statsAll) > 0
        figure;
        hold on; grid on;

        errorbar(statsAll.pwm_us, statsAll.mean_force_N, statsAll.std_force_N, ...
            "o", "Color", [0.85 0.85 0.85], "LineWidth", 1.2, "DisplayName", "Mean ± std (all sweeps)");

        xline(PWM_SAT_LOW_US, "--r", "LineWidth", 1.1, ...
            "DisplayName", sprintf("Stall threshold (%d us)", PWM_SAT_LOW_US));
        xline(PWM_SAT_HIGH_US, "--m", "LineWidth", 1.1, ...
            "DisplayName", sprintf("ESC max (%d us)", PWM_SAT_HIGH_US));

        xlabel("PWM command (\mus)");
        ylabel("Thrust (N)");
        title("Static Thrust Curve - " + runName, "Interpreter", "none");
        legend("Location", "best", "Interpreter", "none");
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

    if hasVolt && ismember("mean_voltage_V", string(statsAll.Properties.VariableNames))
        summaryVars{end+1} = 'mean_voltage_V';
    end

    disp(statsAll(:, summaryVars));

end

%% Save figures

saveAllFiguresIfEnabled(SAVE_FIGURES, plotDir);

%% Local helper functions

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

    if ismember("battery_voltage_V", string(D.Properties.VariableNames))
        D.battery_voltage_V = forceNumeric(D.battery_voltage_V);
    end

    if ismember("battery_current_A", string(D.Properties.VariableNames))
        D.battery_current_A = forceNumeric(D.battery_current_A);
    end

    valid = isfinite(D.pwm_us) & isfinite(D.force_N);
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
    median_force_N = NaN(nPwm, 1);
    n_samples     = zeros(nPwm, 1);
    mean_voltage_V = NaN(nPwm, 1);

    hasVolt = hasVoltage(D);

    for i = 1:nPwm
        idx = D.pwm_us == pwmValues(i);
        y = D.force_N(idx);
        y = y(isfinite(y));

        mean_force_N(i)   = mean(y, "omitnan");
        std_force_N(i)    = std(y, "omitnan");
        median_force_N(i) = median(y, "omitnan");
        n_samples(i)      = numel(y);

        if hasVolt
            v = D.battery_voltage_V(idx);
            v = v(isfinite(v));
            mean_voltage_V(i) = mean(v, "omitnan");
        end
    end

    stats = table(pwmValues, mean_force_N, std_force_N, median_force_N, n_samples, mean_voltage_V, ...
        'VariableNames', { ...
            'pwm_us', ...
            'mean_force_N', ...
            'std_force_N', ...
            'median_force_N', ...
            'n_samples', ...
            'mean_voltage_V' ...
        });
end

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
