%% analyze_thrust_angle_bandwidth_trade.m
% Requirement trade between thrust magnitude, required vector-angle amplitude,
% and vectoring bandwidth.
%
% This answers the PRPS planning question:
%   If the motor can produce T newtons, what servo angle amplitude must be
%   identified, and what bandwidth must be verified at that amplitude?

clear; clc; close all;

%% User options

SAVE_FIGURES = false;
WRITE_TABLE = false;

%% Paths

scriptPath = mfilename("fullpath");
feasibilityDir = fileparts(scriptPath);
analysisDir = fileparts(feasibilityDir);
repoRoot = fileparts(analysisDir);
plotDir = fullfile(repoRoot, "plots", "feasibility");
dataDir = fullfile(repoRoot, "data", "processed", "feasibility");

if ~exist(plotDir, "dir")
    mkdir(plotDir);
end
if WRITE_TABLE && ~exist(dataDir, "dir")
    mkdir(dataDir);
end

%% Configuration: requirements, assumptions, and measured context

G = 9.81;                         % m/s^2

% Top-level motion envelope.
A_REF_MM = 100.0;                 % REQUIREMENT: R1
F_REF_HZ = 0.5;                   % REQUIREMENT: R2
M_LIGHT_KG = 0.45;                % qualification_test_plan.md Sec.2 design mass range
M_HEAVY_KG = 0.75;                % qualification_test_plan.md Sec.2 design mass range

% Feasibility assumptions and margins. SPEC_PEAK_MM and the friction over-bound
% must match analyze_hardware_margin.m so both scripts agree on the same
% required-bandwidth headline.
SPEC_PEAK_MM = 7.5;                % hardware-selection margin (2x under R4's 15 mm)
E_MODEL_MM = 1.0;                 % model-mismatch reserve
% Friction / disturbance model (see analyze_hardware_margin.m for full rationale).
% AUTHORITY carries the FULL friction (motor must produce breakaway); BANDWIDTH
% carries only the FEEDBACK disturbance (injected R13 step + friction residual
% feedforward cannot cancel), since predictable friction is feedforwarded.
MU_C_IDENT_N = 1.0;               % identified Coulomb friction (not finalized)
FRICTION_ASYMMETRY = 0.30;        % measured directional asymmetry
FRICTION_SLACK = 1.25;            % allowance for unsettled friction ID
F_FRICTION_AUTHORITY_N = MU_C_IDENT_N * (1.0 + FRICTION_ASYMMETRY) * FRICTION_SLACK;  % ~1.62 N
F_INJECTED_N = 1.0;               % R13 / M3 injected force step
FF_RESIDUAL_FRAC = 0.30;          % fraction of identified friction FF can't cancel
F_DISTURBANCE_FB_N = F_INJECTED_N + FF_RESIDUAL_FRAC * MU_C_IDENT_N;                  % ~1.30 N
THETA_MAX_DEG = 45.0;             % assumed usable vectoring deflection

% Encoder and controller-sizing assumptions.
ENC_COUNTS_PER_MM = 64.810;       % measured encoder calibration
K_WC = 0.333;                     % crossover fraction of vectoring bandwidth
FF_RENDER_FACTOR = 5.0;           % vectoring bandwidth >= this times reference
AUTO_OPTIMIZE_SPLIT = true;
FF_BUDGET_SHARE = 0.50;           % used only if AUTO_OPTIMIZE_SPLIT = false

% Servo command conversion and already-tested PRPS context.
SERVO_GAIN_DEG_PER_US = 0.093996; % current servo static map magnitude
THRUST_MARKER_N = 12.0;
PRPS_TESTED_SMALL_DEG = [4.1, 4.7];
PRPS_TESTED_LARGE_DEG = [10.2, 11.4];

% Plot/table sweep ranges.
THRUST_MIN_N = 7.0;
THRUST_MAX_N = 25.0;
N_THRUST = 300;
THETA_MIN_DEG = 3.0;
THETA_MAX_PLOT_DEG = 50.0;
N_THETA = 300;

%% Derived requirement forces

frictionLightN = F_DISTURBANCE_FB_N;    % bandwidth + FB disturbance angle (loop rejects)
frictionHeavyN = F_FRICTION_AUTHORITY_N; % authority: full friction the motor must produce
forceFfHeavyN = referenceForceN(M_HEAVY_KG, A_REF_MM, F_REF_HZ);
forceTotalHeavyN = forceFfHeavyN + frictionHeavyN;

%% Error-budget split and bandwidth requirement

eQuantMm = 1.0 / ENC_COUNTS_PER_MM;

if AUTO_OPTIMIZE_SPLIT
    [usedFfShare, bwReqHz] = optimalSplitAndBandwidth( ...
        M_LIGHT_KG, frictionLightN, SPEC_PEAK_MM, eQuantMm, E_MODEL_MM, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);
else
    usedFfShare = FF_BUDGET_SHARE;
    [eFfBudgetMm, eFbBudgetMm] = budgetSharesMm( ...
        usedFfShare, SPEC_PEAK_MM, eQuantMm, E_MODEL_MM);
    bwReqHz = requiredVectoringBandwidthHz( ...
        M_LIGHT_KG, frictionLightN, eFfBudgetMm, eFbBudgetMm, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);
end

[eFfBudgetMm, eFbBudgetMm] = budgetSharesMm( ...
    usedFfShare, SPEC_PEAK_MM, eQuantMm, E_MODEL_MM);
[bwReqHz, bwFbHz, bwFfHz, bwRenderHz, fcReqHz, bwDriver] = ...
    requiredVectoringBandwidthHz( ...
        M_LIGHT_KG, frictionLightN, eFfBudgetMm, eFbBudgetMm, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);

%% Thrust and angle sweeps

thrustN = linspace(THRUST_MIN_N, THRUST_MAX_N, N_THRUST)';
thetaFfDeg = thetaForForceDeg(forceFfHeavyN, thrustN);
thetaFbDeg = thetaForForceDeg(frictionLightN, thrustN);
thetaTotalDeg = thetaForForceDeg(forceTotalHeavyN, thrustN);

thetaAxisDeg = linspace(THETA_MIN_DEG, THETA_MAX_PLOT_DEG, N_THETA)';
thrustFfN = thrustForAngleN(forceFfHeavyN, thetaAxisDeg);
thrustFbN = thrustForAngleN(frictionLightN, thetaAxisDeg);
thrustTotalN = thrustForAngleN(forceTotalHeavyN, thetaAxisDeg);

thetaFfAtMarkerDeg = thetaForForceDeg(forceFfHeavyN, THRUST_MARKER_N);
thetaFbAtMarkerDeg = thetaForForceDeg(frictionLightN, THRUST_MARKER_N);
thetaTotalAtMarkerDeg = thetaForForceDeg(forceTotalHeavyN, THRUST_MARKER_N);

%% Optional table output

tablePath = fullfile(dataDir, "thrust_angle_bandwidth_trade.csv");
tradeTable = table( ...
    thrustN, ...
    thrustN ./ G, ...
    thetaFfDeg, ...
    thetaFbDeg, ...
    thetaTotalDeg, ...
    pwmAmplitudeUs(thetaFfDeg, SERVO_GAIN_DEG_PER_US), ...
    pwmAmplitudeUs(thetaFbDeg, SERVO_GAIN_DEG_PER_US), ...
    pwmAmplitudeUs(thetaTotalDeg, SERVO_GAIN_DEG_PER_US), ...
    repmat(bwReqHz, size(thrustN)), ...
    repmat(bwFfHz, size(thrustN)), ...
    repmat(bwFbHz, size(thrustN)), ...
    'VariableNames', { ...
        'thrust_N', ...
        'thrust_kgf', ...
        'theta_ff_heavy_deg', ...
        'theta_fb_light_deg', ...
        'theta_total_heavy_deg', ...
        'pwm_ff_heavy_us', ...
        'pwm_fb_light_us', ...
        'pwm_total_heavy_us', ...
        'bw_required_Hz', ...
        'bw_ff_bound_Hz', ...
        'bw_fb_bound_Hz'});

if WRITE_TABLE
    writetable(tradeTable, tablePath);
end

%% Console report

fprintf("\n");
fprintf("===============================================================================\n");
fprintf("THRUST / VECTOR-ANGLE / BANDWIDTH REQUIREMENT TRADE\n");
fprintf("===============================================================================\n");
fprintf("Reference requirement          : %.0f mm at %.2f Hz\n", A_REF_MM, F_REF_HZ);
fprintf("Mass range                     : %.2f to %.2f kg\n", M_LIGHT_KG, M_HEAVY_KG);
fprintf("Friction over-bound            : %.2f N light / %.2f N heavy\n", ...
    frictionLightN, frictionHeavyN);
fprintf("Heavy feedforward force        : %.2f N\n", forceFfHeavyN);
fprintf("Heavy simultaneous force       : %.2f N\n", forceTotalHeavyN);
fprintf("\n");
fprintf("Selected FF / FB budget split  : %.2f%% / %.2f%%\n", ...
    100 * usedFfShare, 100 * (1.0 - usedFfShare));
fprintf("Required vectoring bandwidth   : %.2f Hz\n", bwReqHz);
fprintf("  bandwidth driver             : %s\n", bwDriver);
fprintf("  feedback bound               : %.2f Hz (crossover %.2f Hz)\n", ...
    bwFbHz, fcReqHz);
fprintf("  feedforward residual bound   : %.2f Hz\n", bwFfHz);
fprintf("  command-rendering floor      : %.2f Hz\n", bwRenderHz);
fprintf("\n");
fprintf("At T = %.1f N:\n", THRUST_MARKER_N);
fprintf("  feedforward angle            : %.2f deg (%.0f us)\n", ...
    thetaFfAtMarkerDeg, pwmAmplitudeUs(thetaFfAtMarkerDeg, SERVO_GAIN_DEG_PER_US));
fprintf("  feedback/disturbance angle   : %.2f deg (%.0f us)\n", ...
    thetaFbAtMarkerDeg, pwmAmplitudeUs(thetaFbAtMarkerDeg, SERVO_GAIN_DEG_PER_US));
fprintf("  simultaneous envelope        : %.2f deg (%.0f us)\n", ...
    thetaTotalAtMarkerDeg, pwmAmplitudeUs(thetaTotalAtMarkerDeg, SERVO_GAIN_DEG_PER_US));
fprintf("\n");
fprintf("Interpretation:\n");
fprintf("  Measure/verify servo bandwidth at the angle amplitude actually used.\n");
fprintf("  The current PRPS set covered about +/-4-5 deg and +/-10-11 deg, so\n");
fprintf("  the 12 N feedforward point around +/-22 deg is still untested.\n");

%% Figures

bright = struct();
bright.cyan = [0.00 0.65 0.95];
bright.orange = [1.00 0.55 0.00];
bright.magenta = [0.95 0.10 0.35];
bright.lime = [0.30 0.90 0.00];
bright.gray = [0.35 0.37 0.42];

fig1 = figure("Name", "thrust_angle_bandwidth_trade", "Color", "w");
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(thrustN, thetaFfDeg, "LineWidth", 2.2, "Color", bright.cyan, ...
    "DisplayName", "FF reference, heavy mass");
hold on;
plot(thrustN, thetaFbDeg, "LineWidth", 2.2, "Color", bright.orange, ...
    "DisplayName", "FB disturbance, light mass");
plot(thrustN, thetaTotalDeg, "LineWidth", 2.6, "Color", bright.magenta, ...
    "DisplayName", "Simultaneous envelope, heavy mass");
xline(THRUST_MARKER_N, "--", "Color", bright.lime, "LineWidth", 1.8, ...
    "DisplayName", sprintf("%.0f N marker", THRUST_MARKER_N));
yline(THETA_MAX_DEG, ":", "Color", [0.25 0.25 0.25], "LineWidth", 1.2, ...
    "DisplayName", sprintf("%.0f deg assumed usable", THETA_MAX_DEG));
addTestedBand(gca, PRPS_TESTED_SMALL_DEG, bright.gray, 0.12, "tested PRPS small");
addTestedBand(gca, PRPS_TESTED_LARGE_DEG, bright.gray, 0.22, "tested PRPS large");
grid on;
xlim([THRUST_MIN_N, THRUST_MAX_N]);
ylim([0, max(THETA_MAX_PLOT_DEG, 1.05 * max(thetaTotalDeg, [], "omitnan"))]);
xlabel("available thrust magnitude $T$ [N]", "Interpreter", "latex");
ylabel("required servo angle amplitude [deg]");
title("Thrust buys smaller vector-angle amplitude");
legend("Location", "northeast", "FontSize", 8);

yyaxis right;
ylim(pwmAmplitudeUs(ylim, SERVO_GAIN_DEG_PER_US));
ylabel("equivalent servo command amplitude [$\mu$s]", "Interpreter", "latex");
ax = gca;
ax.YAxis(2).Color = [0.15 0.15 0.15];

nexttile;
plot(thetaAxisDeg, thrustFfN, "LineWidth", 2.2, "Color", bright.cyan, ...
    "DisplayName", "T for FF reference");
hold on;
plot(thetaAxisDeg, thrustFbN, "LineWidth", 2.2, "Color", bright.orange, ...
    "DisplayName", "T for FB disturbance");
plot(thetaAxisDeg, thrustTotalN, "LineWidth", 2.6, "Color", bright.magenta, ...
    "DisplayName", "T for simultaneous envelope");
yline(THRUST_MARKER_N, "--", "Color", bright.lime, "LineWidth", 1.8, ...
    "DisplayName", sprintf("%.0f N marker", THRUST_MARKER_N));
addTestedBand(gca, PRPS_TESTED_SMALL_DEG, bright.gray, 0.12, "");
addTestedBand(gca, PRPS_TESTED_LARGE_DEG, bright.gray, 0.22, "");
grid on;
xlim([THETA_MIN_DEG, THETA_MAX_PLOT_DEG]);
ylim([0, 1.05 * THRUST_MAX_N]);
xlabel("servo PRPS / vector angle amplitude [deg]");
ylabel("required thrust magnitude $T$ [N]", "Interpreter", "latex");
title("Angle choice determines required thrust");
legend("Location", "northeast", "FontSize", 8);

sgtitle(sprintf( ...
    "Requirement trade: T, vector angle amplitude, and bandwidth (need measured f_vec(theta) >= %.1f Hz)", ...
    bwReqHz));

fig2 = figure("Name", "bandwidth_required_vs_angle_amplitude", "Color", "w");
plot(thetaFfDeg, bwReqHz .* ones(size(thetaFfDeg)), "LineWidth", 2.2, ...
    "Color", bright.cyan, "DisplayName", "FF amplitude path");
hold on;
plot(thetaFbDeg, bwReqHz .* ones(size(thetaFbDeg)), "LineWidth", 2.2, ...
    "Color", bright.orange, "DisplayName", "FB amplitude path");
plot(thetaTotalDeg, bwReqHz .* ones(size(thetaTotalDeg)), "LineWidth", 2.2, ...
    "Color", bright.magenta, "DisplayName", "Envelope amplitude path");
yline(bwFfHz, ":", "Color", bright.cyan, "LineWidth", 1.4, ...
    "DisplayName", sprintf("FF residual bound %.1f Hz", bwFfHz));
yline(bwFbHz, ":", "Color", bright.orange, "LineWidth", 1.4, ...
    "DisplayName", sprintf("FB disturbance bound %.1f Hz", bwFbHz));
yline(bwRenderHz, ":", "Color", [0.35 0.35 0.35], "LineWidth", 1.4, ...
    "DisplayName", sprintf("rendering floor %.1f Hz", bwRenderHz));
addTestedBand(gca, PRPS_TESTED_SMALL_DEG, bright.gray, 0.12, "tested PRPS small");
addTestedBand(gca, PRPS_TESTED_LARGE_DEG, bright.gray, 0.22, "tested PRPS large");

markerThrustsN = [10.0, 12.0, 15.0, 20.0];
for i = 1:numel(markerThrustsN)
    tMark = markerThrustsN(i);
    if tMark >= THRUST_MIN_N && tMark <= THRUST_MAX_N
        thMark = thetaForForceDeg(forceFfHeavyN, tMark);
        plot(thMark, bwReqHz, "ko", "MarkerFaceColor", "k", "MarkerSize", 4, ...
            "HandleVisibility", "off");
        text(thMark, bwReqHz * 1.012, sprintf("%.0f N", tMark), ...
            "HorizontalAlignment", "center", "VerticalAlignment", "bottom", ...
            "FontSize", 8);
    end
end

grid on;
xlim([0, THETA_MAX_PLOT_DEG]);
ylim([0, 1.18 * max([bwReqHz, bwFfHz, bwFbHz, bwRenderHz])]);
xlabel("required angle amplitude at chosen thrust [deg]");
ylabel("required vectoring bandwidth [Hz]");
title("What must be verified at each PRPS amplitude");
legend("Location", "southeast", "FontSize", 8);

if SAVE_FIGURES
    saveNamedFigure(fig1, plotDir, "thrust_angle_bandwidth_trade");
    saveNamedFigure(fig2, plotDir, "bandwidth_required_vs_angle_amplitude");
end

if WRITE_TABLE
    fprintf("\nSaved table:\n  %s\n", tablePath);
end

%% Local functions

function F = referenceForceN(mKg, aRefMm, fRefHz)
    aPeak = (aRefMm .* 1e-3) .* (2.0 .* pi .* fRefHz).^2;
    F = mKg .* aPeak;
end

function thetaDeg = thetaForForceDeg(forceN, thrustN)
    ratio = forceN ./ thrustN;
    thetaDeg = NaN(size(ratio));
    valid = abs(ratio) <= 1.0;
    thetaDeg(valid) = asind(ratio(valid));
end

function thrustN = thrustForAngleN(forceN, thetaDeg)
    thrustN = forceN ./ sind(thetaDeg);
end

function pwmUs = pwmAmplitudeUs(thetaDeg, gainDegPerUs)
    pwmUs = thetaDeg ./ gainDegPerUs;
end

function [eFfMm, eFbMm, remMm] = budgetSharesMm(ffShare, specMm, eQuantMm, eModelMm)
    rem2 = specMm.^2 - eQuantMm.^2 - eModelMm.^2;
    rem2 = max(rem2, 1e-12);
    eFfMm = sqrt(ffShare .* rem2);
    eFbMm = sqrt((1.0 - ffShare) .* rem2);
    remMm = sqrt(rem2);
end

function bwFfHz = feedforwardBandwidthHz(aRefMm, fRefHz, eFfMm)
    bwFfHz = fRefHz .* sqrt(aRefMm ./ (2.0 .* eFfMm));
end

function fcHz = feedbackCrossoverHz(mKg, disturbanceN, eFbMm)
    wcRadS = sqrt(disturbanceN ./ (mKg .* (eFbMm .* 1e-3)));
    fcHz = wcRadS ./ (2.0 .* pi);
end

function [bwReqHz, bwFbHz, bwFfHz, bwRenderHz, fcReqHz, driver] = ...
        requiredVectoringBandwidthHz(mKg, disturbanceN, eFfMm, eFbMm, ...
                                     aRefMm, fRefHz, kWc, renderFactor)
    bwFfHz = feedforwardBandwidthHz(aRefMm, fRefHz, eFfMm);
    fcReqHz = feedbackCrossoverHz(mKg, disturbanceN, eFbMm);
    bwFbHz = fcReqHz ./ kWc;
    bwRenderHz = renderFactor .* fRefHz;

    [bwReqHz, idx] = max([bwFbHz, bwFfHz, bwRenderHz]);
    names = ["feedback disturbance rejection", "feedforward residual", "command rendering"];
    driver = names(idx);
end

function [bestShare, bestBwHz] = optimalSplitAndBandwidth( ...
        mKg, disturbanceN, specMm, eQuantMm, eModelMm, aRefMm, fRefHz, kWc, renderFactor)
    shareGrid = linspace(0.002, 0.95, 3000);
    bwGrid = zeros(size(shareGrid));
    for i = 1:numel(shareGrid)
        [eFfMm, eFbMm] = budgetSharesMm(shareGrid(i), specMm, eQuantMm, eModelMm);
        bwGrid(i) = requiredVectoringBandwidthHz( ...
            mKg, disturbanceN, eFfMm, eFbMm, aRefMm, fRefHz, kWc, renderFactor);
    end
    [bestBwHz, idx] = min(bwGrid);
    bestShare = shareGrid(idx);
end

function addTestedBand(ax, bandDeg, color, alphaVal, displayName)
    axes(ax); %#ok<LAXES>
    yLimits = ylim(ax);
    h = patch(ax, ...
        [bandDeg(1), bandDeg(2), bandDeg(2), bandDeg(1)], ...
        [yLimits(1), yLimits(1), yLimits(2), yLimits(2)], ...
        color, ...
        "FaceAlpha", alphaVal, ...
        "EdgeColor", color, ...
        "EdgeAlpha", alphaVal);
    if strlength(string(displayName)) > 0
        h.DisplayName = displayName;
    else
        h.HandleVisibility = "off";
    end
    uistack(h, "bottom");
end

function saveNamedFigure(fig, plotDir, stem)
    pngPath = fullfile(plotDir, stem + ".png");
    exportgraphics(fig, pngPath, "Resolution", 300);
    fprintf("Saved figure:\n  %s\n", pngPath);
end
