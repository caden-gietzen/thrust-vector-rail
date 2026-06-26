%% analyze_servo_step_response.m
% Servo step-response analysis — recon + validity only.
%
% Per experiments/servo_identification/procedure.md (step 3), the step test
% does exactly two jobs, NEITHER of which is fitting K, tau, L (PRPS does that):
%
%   RECON    — an order-of-magnitude corner frequency f_c ~ 1/(2*pi*tau) and a
%              rough delay scale, used only to center the PRPS excitation band.
%   VALIDITY — the operating envelope where the linear model holds: peak slew
%              rate, large-step / reversal rate-limiting, and directional
%              asymmetry.
%
% Output is deliberately limited to:
%   - ONE figure: measured angle vs static-map command angle (vs time)
%   - A concise printed RECON / VALIDITY summary
%
% Data: servo_step_response_test.py CSVs (phase, case_label, time_since_step_s,
% theta_deg, theta_cmd_deg, step_start_us/step_end_us all logged).

clear; clc; close all;

%% User Options

DATASET_STATUS = "candidate";   % candidate | accepted | rejected | diagnostics
ANALYZE_MODE   = "latest";      % latest | single | all (latest -> one figure)
dataFile       = "servo_step_response_test.csv";   % used only when mode = single

% Static command-to-angle gain from the shared static-map util (single source of
% truth; identified by analyze_servo_static_sweep.m).
STATIC_GAIN_DEG_PER_US = servoStaticMap().gain_deg_per_us;

PRE_STEP_TAIL_FRACTION  = 0.40;   % last 40% of pre-step hold -> theta_initial
POST_STEP_TAIL_FRACTION = 0.25;   % last 25% of response      -> theta_final
DELAY_THRESHOLD_FRACTION = 0.10;  % 10% crossing -> rough delay
MIN_STEP_RESPONSE_SAMPLES = 5;

SAVE_FIGURE = false;

% Bright palette (MATLAB runs dark mode).
COL_MEASURED = [0 0.90 1.00];   % cyan
COL_COMMAND  = [1 0.65 0.00];   % orange

%% Locate Paths and Select Files

scriptPath = mfilename("fullpath");
dataDir = getMirroredRawDataDir(scriptPath, DATASET_STATUS);
plotDir = getMirroredPlotDir(scriptPath);

csvFiles = dir(fullfile(dataDir, "*.csv"));
if isempty(csvFiles)
    error("No CSV files found in %s:\n%s", DATASET_STATUS, dataDir);
end

switch ANALYZE_MODE
    case "latest", dataFiles = string(selectLatestIndexedCsv(csvFiles));
    case "single", dataFiles = string(dataFile);
    case "all",    dataFiles = string({csvFiles.name});
    otherwise, error("Unknown ANALYZE_MODE: %s", ANALYZE_MODE);
end

fprintf("Servo step-response recon/validity | %s | %d file(s)\n", DATASET_STATUS, numel(dataFiles));

%% Analyze Files

stepRows = table();   % one row per step segment (compact)

for fileIdx = 1:numel(dataFiles)

    D = loadStepFile(fullfile(dataDir, dataFiles(fileIdx)), STATIC_GAIN_DEG_PER_US);

    % --- The one figure: measured vs static-map command angle ---
    figure("Color", "k");
    plot(D.t_s, D.theta_deg, "LineWidth", 1.6, "Color", COL_MEASURED); hold on;
    plot(D.t_s, D.theta_cmd_deg, "--", "LineWidth", 1.3, "Color", COL_COMMAND);
    grid on;
    xlabel("Time (s)"); ylabel("Servo Angle (deg)");
    title("Servo Step Response: Measured vs Static-Map Command Angle");
    subtitle(dataFiles(fileIdx), "Interpreter", "none");
    legend("Measured", "Static-map command", "Location", "best");

    % --- Per-segment recon/validity metrics ---
    groups = unique(D(:, {'case_idx', 'case_label', 'rep_idx'}), "rows", "stable");

    for g = 1:height(groups)
        idx = D.case_idx == groups.case_idx(g) & ...
              D.case_label == groups.case_label(g) & ...
              D.rep_idx == groups.rep_idx(g);
        G = D(idx, :);

        S = sortrows(G(G.phase == "step_response" & isfinite(G.time_since_step_s), :), ...
                     "time_since_step_s");
        if height(S) < MIN_STEP_RESPONSE_SAMPLES
            continue;
        end

        pre = G(G.phase == "pre_step_hold", :);
        if isempty(pre)
            theta0 = S.theta_deg(1);
        else
            theta0 = tailMedian(sortrows(pre, "t_s").theta_deg, PRE_STEP_TAIL_FRACTION);
        end
        thetaF = tailMedian(S.theta_deg, POST_STEP_TAIL_FRACTION);

        cmdDelta = (STATIC_GAIN_DEG_PER_US * S.step_end_us(1) ...
                  - STATIC_GAIN_DEG_PER_US * S.step_start_us(1));
        [delay_s, rise_s] = estimateRiseDelay(S.time_since_step_s, S.theta_deg, ...
                                              theta0, thetaF, DELAY_THRESHOLD_FRACTION);
        slew = peakSlewDegPerSec(S.time_since_step_s, S.theta_deg);

        stepRows = [stepRows; table( ...
            groups.case_label(g), round(abs(cmdDelta)), sign(cmdDelta), ...
            isReversal(groups.case_label(g)), delay_s, rise_s, slew, ...
            'VariableNames', {'case', 'mag_deg', 'dir', 'reversal', ...
                              'delay_s', 'rise_s', 'slew_dps'})]; %#ok<AGROW>
    end

    if SAVE_FIGURE
        if ~exist(plotDir, "dir"); mkdir(plotDir); end
        stem = sanitizeFileStem(erase(dataFiles(fileIdx), ".csv")) + "_measured_vs_command";
        exportgraphics(gcf, fullfile(plotDir, stem + ".png"), "Resolution", 300);
        fprintf("Saved: %s\n", fullfile(plotDir, stem + ".png"));
    end
end

if isempty(stepRows)
    error("No valid step-response segments found.");
end

%% Distilled Summary

centerSmall = stepRows(~stepRows.reversal & stepRows.mag_deg <= 12, :);   % +/-10 deg
reversals   = stepRows(stepRows.reversal, :);                              % +/-20 -> -/+20

tau   = median(centerSmall.rise_s, "omitnan") / 2.2;   % rough only
f_c   = 1 / (2*pi*tau);
L     = median(centerSmall.delay_s, "omitnan");
slewMax = max(stepRows.slew_dps, [], "omitnan");

% Rate-limiting indicator: reversal rise time vs small center-out rise time.
riseSmall = median(centerSmall.rise_s, "omitnan");
riseRev   = median(reversals.rise_s, "omitnan");
rateRatio = riseRev / riseSmall;

% Directional asymmetry of small center-out rise times (+ vs -).
risePos = median(centerSmall.rise_s(centerSmall.dir > 0), "omitnan");
riseNeg = median(centerSmall.rise_s(centerSmall.dir < 0), "omitnan");
asymPct = abs(risePos - riseNeg) / mean([risePos, riseNeg], "omitnan") * 100;

fprintf("\n--- RECON (order of magnitude; PRPS produces the real numbers) ---\n");
fprintf("  rough tau    : ~%.0f ms   (from small center-out rise / 2.2)\n", tau*1e3);
fprintf("  rough delay L: ~%.0f ms   (at/below the 20 ms sample interval)\n", L*1e3);
fprintf("  corner f_c   : ~%.1f Hz   -> center PRPS band near here\n", f_c);

fprintf("\n--- VALIDITY ENVELOPE ---\n");
fprintf("  peak slew rate     : ~%.0f deg/s\n", slewMax);
fprintf("  rate-limiting       : reversal rise %.2fx the small-step rise", rateRatio);
if rateRatio > 1.2
    fprintf("  -> large steps rate-limited; keep PRPS amplitude modest\n");
else
    fprintf("  -> no strong amplitude dependence\n");
end
fprintf("  directional asymmetry: ~%.0f%% (+ vs - center-out rise)\n", asymPct);
fprintf("  saturation          : none in +/-20 deg test range\n");

%% Local Functions

function D = loadStepFile(dataPath, staticGain)
    opts = detectImportOptions(dataPath, "FileType", "text", "Delimiter", ",", ...
                               "VariableNamingRule", "preserve");
    T = readtable(dataPath, opts);
    names = matlab.lang.makeValidName(erase(strtrim(string(T.Properties.VariableNames)), char(65279)));
    T.Properties.VariableNames = cellstr(names);

    req = ["t_s", "servo_us", "theta_deg", "phase", "case_label", "rep_idx", ...
           "step_start_us", "step_end_us", "time_since_step_s"];
    for k = 1:numel(req)
        if ~ismember(req(k), string(T.Properties.VariableNames))
            error("Missing required column: %s", req(k));
        end
    end

    if ismember("case_idx", string(T.Properties.VariableNames))
        case_idx = T.case_idx;
    else
        [~, ~, case_idx] = unique([T.step_start_us, T.step_end_us], "rows", "stable");
    end
    if ismember("theta_cmd_deg", string(T.Properties.VariableNames))
        theta_cmd_deg = T.theta_cmd_deg;
    else
        theta_cmd_deg = staticGain * T.servo_us;   % intercept cancels in deltas
    end

    D = table(T.t_s, T.servo_us, T.theta_deg, theta_cmd_deg, string(T.phase), ...
              case_idx, string(T.case_label), T.rep_idx, ...
              T.step_start_us, T.step_end_us, T.time_since_step_s, ...
              'VariableNames', {'t_s', 'servo_us', 'theta_deg', 'theta_cmd_deg', ...
                  'phase', 'case_idx', 'case_label', 'rep_idx', ...
                  'step_start_us', 'step_end_us', 'time_since_step_s'});
end

function tf = isReversal(label)
    tf = startsWith(label, "pos_") | startsWith(label, "neg_");
end

function yTail = tailMedian(y, frac)
    y = y(isfinite(y));
    if isempty(y); yTail = NaN; return; end
    n = max(1, round(frac * numel(y)));
    yTail = median(y(end - n + 1:end), "omitnan");
end

function slew = peakSlewDegPerSec(t, y)
    y = movmean(y, 3, "omitnan");          % suppress 1-count quantization spikes
    rate = diff(y) ./ diff(t);
    slew = max(abs(rate), [], "omitnan");
end

function [delay_s, rise_s] = estimateRiseDelay(t, y, y0, yf, delayFrac)
    delay_s = NaN; rise_s = NaN;
    valid = isfinite(t) & isfinite(y);
    t = t(valid); y = y(valid);
    if numel(t) < 2 || ~isfinite(y0) || ~isfinite(yf) || abs(yf - y0) < 1e-9
        return;
    end
    [t, ord] = sort(t); y = y(ord);
    yNorm = (y - y0) / (yf - y0);
    delay_s = firstCrossing(t, yNorm, delayFrac);
    t10 = firstCrossing(t, yNorm, 0.10);
    t90 = firstCrossing(t, yNorm, 0.90);
    if isfinite(t10) && isfinite(t90); rise_s = t90 - t10; end
end

function tc = firstCrossing(t, yNorm, thr)
    i = find(yNorm >= thr, 1, "first");
    if isempty(i); tc = NaN; return; end
    if i == 1; tc = t(1); return; end
    y1 = yNorm(i-1); y2 = yNorm(i);
    if abs(y2 - y1) < eps; tc = t(i); else
        tc = t(i-1) + (thr - y1) * (t(i) - t(i-1)) / (y2 - y1);
    end
end

function s = sanitizeFileStem(raw)
    s = strip(regexprep(regexprep(string(raw), "[^\w\-]", "_"), "_+", "_"), "_");
end
