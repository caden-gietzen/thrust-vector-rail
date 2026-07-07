%% analyze_hardware_margin.m
% Compatibility wrapper. The active feasibility workflow is now
% analyze_requirement_feasibility.m. This early return prevents the obsolete
% heuristic implementation below from running through older commands.

scriptDir = fileparts(mfilename("fullpath"));
run(fullfile(scriptDir, "analyze_requirement_feasibility.m"));
return;

%% Obsolete implementation retained below for history only.
% Requirement-to-hardware feasibility flowdown for the thrust-vector rail.
%
% Answers the from-scratch buy question: "what motor thrust and what
% vectoring bandwidth must the hardware provide to hold a margin under the
% qualification tracking spec (R4/R5), across the design mass range?"
% Nothing is bought yet -- the candidate table below is a set of (thrust,
% bandwidth) purchase targets, each labelled by its own (kgf, Hz) pair; there
% is no "current hardware" row.
%
% Inputs trace to docs/requirements.md (R1, R2) and
% docs/qualification_test_plan.md Sec.2 (mass range); the design target below
% is a dedicated hardware-selection margin, tighter than R4/R5 on purpose so
% the qualification spec is met with room to spare (see MARGIN_PEAK_MM).

clear; clc; close all;

%% User options

SAVE_FIGURES = true;

%% Paths

scriptPath = mfilename("fullpath");
feasibilityDir = fileparts(scriptPath);
analysisDir = fileparts(feasibilityDir);
repoRoot = fileparts(analysisDir);
plotDir = fullfile(repoRoot, "plots", "feasibility");

if ~exist(plotDir, "dir")
    mkdir(plotDir);
end

%% Configuration: requirements, margin, and measured context

G = 9.81;                         % m/s^2

% Top-level motion envelope (requirement, not a margin).
A_REF_MM = 100.0;                 % REQUIREMENT: R1 (requirements.md)
F_REF_HZ = 0.5;                   % REQUIREMENT: R2 (requirements.md)
M_LIGHT_KG = 0.45;                % qualification_test_plan.md Sec.2 design mass range
M_HEAVY_KG = 0.75;                % qualification_test_plan.md Sec.2 design mass range

% Hardware-selection margin: sized TIGHTER than the qualification spec
% (R4 = 15 mm peak / R5 = 7 mm RMS) on purpose so the hardware we spec clears
% R4/R5 with room to spare. Dedicated margin, independent of the L1 sim budget
% in project_metrics.md. 2x under R4 keeps the required pair modest enough that
% a reasonable single-purchase actuator (~1-1.5 kgf, ~10 Hz class) clears it.
MARGIN_PEAK_MM = 7.5;             % ASSUMED -- REPLACE (2x under R4's 15 mm)
E_MODEL_MM = 1.0;                 % model-mismatch reserve, mm

% Usable vectoring deflection.
THETA_MAX_DEG = 45.0;             % ASSUMED -- REPLACE

% ------------------------------------------------------------------------
% Friction / disturbance model -- AUTHORITY and BANDWIDTH see DIFFERENT forces.
% ------------------------------------------------------------------------
% Identified Coulomb friction, inflated for the measured ~30% directional
% asymmetry and a slack allowance for the unsettled post-rebuild mechanism.
MU_C_IDENT_N = 1.0;               % MEASURED (identified Coulomb friction; not finalized)
FRICTION_ASYMMETRY = 0.30;        % MEASURED (~30% directional asymmetry)
FRICTION_SLACK = 1.25;            % ASSUMED -- REPLACE (allowance for unsettled ID)
%
% AUTHORITY friction: the motor must PHYSICALLY PRODUCE the breakaway force,
% whether it is cancelled by feedforward or rejected by feedback -- so the
% thrust sizing carries the FULL friction over-bound.
F_FRICTION_AUTHORITY_N = MU_C_IDENT_N * (1.0 + FRICTION_ASYMMETRY) * FRICTION_SLACK;  % ~1.62 N
%
% FEEDBACK disturbance: what the loop must reject WITH BANDWIDTH. The predictable
% part of friction (Coulomb + viscous + asymmetry, a static velocity function)
% is cancelled by FEEDFORWARD and therefore costs NO feedback bandwidth. The
% loop owns only (1) the exogenous injected qualification disturbance (R13/M3 --
% the 1 N vectoring-bias force step), and (2) a friction RESIDUAL feedforward
% cannot cancel -- concentrated at velocity reversals (stiction/Stribeck, where
% sign(v) is ill-defined), plus nominal-model error and drift. Sizing feedback
% against the FULL friction would double-count what feedforward already erases.
F_INJECTED_N = 1.0;               % REQUIREMENT: R13 / M3 injected force step
FF_RESIDUAL_FRAC = 0.30;          % ASSUMED -- REPLACE (fraction of ident. friction FF can't cancel)
F_FRICTION_RESIDUAL_N = FF_RESIDUAL_FRAC * MU_C_IDENT_N;              % ~0.30 N
F_DISTURBANCE_FB_N = F_INJECTED_N + F_FRICTION_RESIDUAL_N;            % ~1.30 N (summed, coincident)

% Encoder and controller-sizing assumptions.
ENC_COUNTS_PER_MM = 64.810;       % MEASURED (encoder_calibration/results.md)
K_WC = 0.333;                     % ASSUMED -- REPLACE (servo = 1/K_WC ~ 3x crossover)
FF_RENDER_FACTOR = 5.0;           % ASSUMED -- REPLACE (vectoring bw >= 5x f_ref)
AUTO_OPTIMIZE_SPLIT = true;       % split the error budget to MINIMIZE required bandwidth

% Candidate (thrust, vectoring-bandwidth) spec pairs to screen against the
% required pair. These are the concrete numbers a vendor part must meet --
% NOTHING is bought yet, so there is no "current hardware" row; each pair is a
% purchase target labelled by its own (kgf, Hz) so the plots read directly.
candidatePairsKgfHz = [ ...
    0.5, 9.0; ...
    1.0, 10.0; ...
    1.5, 15.0];
candidate = table( ...
    compose("%.1f kgf / %.0f Hz", candidatePairsKgfHz(:, 1), candidatePairsKgfHz(:, 2)), ...
    candidatePairsKgfHz(:, 1), ...
    candidatePairsKgfHz(:, 2), ...
    'VariableNames', {'name', 'thrust_kgf', 'bw_Hz'});

% Representative servo slew rates for the "thrust as a bandwidth lever" plot.
% The slew-limited vectoring bandwidth ceiling is slew/(2*pi*theta_amp), so a
% smaller required deflection (bought with MORE thrust) raises it. Datasheet-
% class values -- ASSUMED, REPLACE with the candidate servo's real slew spec.
SERVO_SLEW_CLASSES_DEG_S = [300.0, 600.0, 900.0];  % ASSUMED -- REPLACE
THRUST_LEVER_MAX_N = 25.0;                          % top of the thrust sweep

% Sweeps and plotting ranges.
A_MIN_MM = 1.0;
A_MAX_MM = 300.0;
F_MIN_HZ = 0.05;
F_MAX_HZ = 5.0;
N_A = 260;
N_F = 240;

%% Derived constants

thetaMaxRad = deg2rad(THETA_MAX_DEG);
qMm = 1.0 / ENC_COUNTS_PER_MM;
eQuantMm = qMm;

% Bandwidth is sized against the FEEDBACK disturbance (injected + residual, at
% the light mass where e_fb ~ 1/m is worst); authority against the FULL friction
% (at the heavy mass). Both forces are mass-independent here.
feedbackDisturbanceN = F_DISTURBANCE_FB_N;
authorityFrictionN = F_FRICTION_AUTHORITY_N;

%% Error-budget split (minimize required bandwidth, or use a fixed 50/50 split)

if AUTO_OPTIMIZE_SPLIT
    [usedFfShare, bwReqHz] = optimalSplitAndBandwidth( ...
        M_LIGHT_KG, feedbackDisturbanceN, MARGIN_PEAK_MM, eQuantMm, E_MODEL_MM, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);
else
    usedFfShare = 0.50;
    [eFfBudgetMm, eFbBudgetMm] = budgetSharesMm( ...
        usedFfShare, MARGIN_PEAK_MM, eQuantMm, E_MODEL_MM);
    bwReqHz = requiredVectoringBandwidthHz( ...
        M_LIGHT_KG, feedbackDisturbanceN, eFfBudgetMm, eFbBudgetMm, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);
end

[eFfBudgetMm, eFbBudgetMm] = budgetSharesMm( ...
    usedFfShare, MARGIN_PEAK_MM, eQuantMm, E_MODEL_MM);
[bwReqHz, bwFbHz, bwFfHz, bwRenderHz, fcReqHz, bwDriver] = ...
    requiredVectoringBandwidthHz( ...
        M_LIGHT_KG, feedbackDisturbanceN, eFfBudgetMm, eFbBudgetMm, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);

%% Headline authority and realized error ledger

TReqN = requiredThrustN(A_REF_MM, F_REF_HZ, M_HEAVY_KG, thetaMaxRad, authorityFrictionN);
TReqKgf = TReqN / G;

eFfActualMm = feedforwardResidualMm(A_REF_MM, F_REF_HZ, bwReqHz);
eFbActualMm = feedbackResidualMm(feedbackDisturbanceN, M_LIGHT_KG, fcReqHz);
eTotalMm = sqrt(eFfActualMm^2 + eFbActualMm^2 + eQuantMm^2 + E_MODEL_MM^2);
slewRequiredDegS = peakVectoringSlewDegS(A_REF_MM, F_REF_HZ, M_HEAVY_KG, TReqN);

%% Candidate hardware screen (thrust, bandwidth, pass/fail)

candidate.theta_amp_deg = min(THETA_MAX_DEG, requiredThetaAmpDeg( ...
    A_REF_MM, F_REF_HZ, M_HEAVY_KG, candidate.thrust_kgf .* G));
candidate.thrust_pass = candidate.thrust_kgf >= TReqKgf;
candidate.bandwidth_pass = candidate.bw_Hz >= bwReqHz;
candidate.overall_pass = candidate.thrust_pass & candidate.bandwidth_pass;

%% Console report

fprintf("\n");
fprintf("===============================================================================\n");
fprintf("THRUST-VECTORING RAIL -- HARDWARE-SELECTION MARGIN ANALYSIS\n");
fprintf("===============================================================================\n");
fprintf("Reference envelope (R1/R2)      : %.0f mm at %.2f Hz\n", A_REF_MM, F_REF_HZ);
fprintf("Mass range                      : %.2f to %.2f kg\n", M_LIGHT_KG, M_HEAVY_KG);
fprintf("Hardware-selection margin       : %.2f mm peak (R4 = 15 mm; margin factor %.1fx)\n", ...
    MARGIN_PEAK_MM, 15.0 / MARGIN_PEAK_MM);
fprintf("Usable vectoring deflection      : %.1f deg\n", THETA_MAX_DEG);
fprintf("Authority friction (motor must   : %.2f N " + ...
    "= mu_c %.1f x (1+%.0f%% asym) x %.2f slack\n", ...
    authorityFrictionN, MU_C_IDENT_N, 100 * FRICTION_ASYMMETRY, FRICTION_SLACK);
fprintf("  produce; sizes THRUST)\n");
fprintf("Feedback disturbance (loop must  : %.2f N " + ...
    "= injected %.1f N (R13) + %.2f N friction residual (%.0f%% of mu_c)\n", ...
    feedbackDisturbanceN, F_INJECTED_N, F_FRICTION_RESIDUAL_N, 100 * FF_RESIDUAL_FRAC);
fprintf("  reject; sizes BANDWIDTH)         predictable friction is feedforwarded, not rejected\n");
fprintf("Budget split FF / FB             : %.2f%% / %.2f%%\n", ...
    100 * usedFfShare, 100 * (1.0 - usedFfShare));

fprintf("\nHARDWARE REQUIREMENT PAIR\n");
fprintf("  Required motor thrust          : %.3f kgf (%.2f N)\n", TReqKgf, TReqN);
fprintf("  Required vectoring bandwidth   : %.2f Hz\n", bwReqHz);
fprintf("  Feedback crossover             : %.2f Hz\n", fcReqHz);
fprintf("  Bandwidth driver               : %s\n", bwDriver);
fprintf("  Component bounds               : FB %.2f Hz | FF %.2f Hz | render %.2f Hz\n", ...
    bwFbHz, bwFfHz, bwRenderHz);

fprintf("\nERROR LEDGER\n");
fprintf("  %-24s %10s %10s\n", "contributor", "budget", "actual");
fprintf("  %-24s %10.4f %10.4f mm\n", "encoder quantization", eQuantMm, eQuantMm);
fprintf("  %-24s %10.3f %10.3f mm\n", "model reserve", E_MODEL_MM, E_MODEL_MM);
fprintf("  %-24s %10.3f %10.3f mm\n", "feedforward", eFfBudgetMm, eFfActualMm);
fprintf("  %-24s %10.3f %10.3f mm\n", "feedback", eFbBudgetMm, eFbActualMm);
fprintf("  %-24s %10.3f %10.3f mm\n", "RSS total", MARGIN_PEAK_MM, eTotalMm);

fprintf("\nSHARED-ACTUATOR CHECK\n");
forceFfHeavyN = M_HEAVY_KG * (A_REF_MM * 1e-3) * (2 * pi * F_REF_HZ)^2;
fprintf("  Reference peak lateral force   : %.2f N\n", forceFfHeavyN);
fprintf("  Heavy-mass friction reserve    : %.2f N\n", authorityFrictionN);
fprintf("  Required vectoring slew        : %.1f deg/s\n", slewRequiredDegS);

fprintf("\nCANDIDATE HARDWARE SCREEN (need thrust >= %.2f kgf AND bandwidth >= %.2f Hz)\n", ...
    TReqKgf, bwReqHz);
fprintf("  %-20s %9s %9s %9s %9s %9s\n", ...
    "candidate", "kgf", "theta deg", "bw Hz", "thrust", "bw");
for i = 1:height(candidate)
    fprintf("  %-20s %9.3f %9.1f %9.2f %9s %9s\n", ...
        candidate.name(i), candidate.thrust_kgf(i), ...
        candidate.theta_amp_deg(i), candidate.bw_Hz(i), ...
        passFail(candidate.thrust_pass(i)), passFail(candidate.bandwidth_pass(i)));
end

fprintf("\nSaved figures will be written to:\n  %s\n", plotDir);

%% Figures

bright = struct();
bright.cyan = [0.00 0.90 1.00];
bright.orange = [1.00 0.62 0.00];
bright.lime = [0.45 1.00 0.45];
bright.magenta = [1.00 0.20 0.90];
bright.blue = [0.15 0.55 1.00];
bright.gray = [0.45 0.45 0.45];

% Figure 1: required-thrust map over (A, f), candidate contours, R1/R2 marker.
Agrid = logspace(log10(A_MIN_MM), log10(A_MAX_MM), N_A);
Fgrid = logspace(log10(F_MIN_HZ), log10(F_MAX_HZ), N_F);
[AA, FF] = meshgrid(Agrid, Fgrid);
TmapKgf = requiredThrustN(AA, FF, M_HEAVY_KG, thetaMaxRad, authorityFrictionN) ./ G;

fig1 = figure("Name", "select_authority_map", "Color", "w");
surf(Agrid, Fgrid, TmapKgf, "EdgeColor", "none");
view(2);
set(gca, "XScale", "log", "YScale", "log", "ColorScale", "log");
colormap(turbo);
cb = colorbar;
cb.Label.String = "required motor thrust [kgf]";
hold on; grid on;
contour3(Agrid, Fgrid, TmapKgf, candidate.thrust_kgf', "LineColor", "w", "LineWidth", 1.4);
plot3(A_REF_MM, F_REF_HZ, TReqKgf, "d", ...
    "MarkerFaceColor", bright.magenta, "MarkerEdgeColor", "k", "MarkerSize", 9);
xlabel("reference amplitude [mm]");
ylabel("reference frequency [Hz]");
title("Required motor thrust [kgf]");
subtitle(sprintf("heavy mass %.2f kg, theta max %.0f deg, R1/R2 = %.0f mm @ %.2f Hz", ...
    M_HEAVY_KG, THETA_MAX_DEG, A_REF_MM, F_REF_HZ));

% Figure 2: required vectoring bandwidth vs. the FEEDBACK disturbance, with the
% chosen disturbance (injected + residual) and candidate bandwidths marked. The
% full-friction over-bound is shown for contrast -- feedforward cancellation is
% what moves the operating point LEFT from it.
FdAxis = linspace(0.5, 3.0, 220);
bwVsFriction = zeros(size(FdAxis));
for k = 1:numel(FdAxis)
    [~, bwVsFriction(k)] = optimalSplitAndBandwidth( ...
        M_LIGHT_KG, FdAxis(k), MARGIN_PEAK_MM, eQuantMm, E_MODEL_MM, ...
        A_REF_MM, F_REF_HZ, K_WC, FF_RENDER_FACTOR);
end

fig2 = figure("Name", "select_bandwidth_drivers", "Color", "w");
plot(FdAxis, bwVsFriction, "LineWidth", 2.4, "Color", bright.cyan);
hold on; grid on;
xline(F_INJECTED_N, ":", "Color", bright.gray, "LineWidth", 1.3, ...
    "Label", sprintf("injected only %.1f N (R13)", F_INJECTED_N), ...
    "LabelVerticalAlignment", "bottom");
xline(feedbackDisturbanceN, "-", "Color", bright.magenta, "LineWidth", 1.6, ...
    "Label", sprintf("chosen F_fb = %.2f N (injected + residual)", feedbackDisturbanceN), ...
    "LabelVerticalAlignment", "bottom");
xline(authorityFrictionN, "--", "Color", bright.orange, "LineWidth", 1.3, ...
    "Label", sprintf("full friction %.2f N (feedforwarded away)", authorityFrictionN), ...
    "LabelVerticalAlignment", "bottom");
for k = 1:height(candidate)
    yline(candidate.bw_Hz(k), "--", candidate.name(k), ...
        "Color", bright.gray, "LabelHorizontalAlignment", "left");
end
xlabel("feedback disturbance F_{fb}, light mass [N]");
ylabel("required vectoring bandwidth [Hz]");
title("Required bandwidth vs. feedback disturbance");
subtitle("feedforward cancels predictable friction -> loop rejects injected + residual only");

% Figure 3: candidate pass/fail matrix.
fig3 = figure("Name", "select_feasibility_matrix", "Color", "w");
passMatrix = [candidate.thrust_pass, candidate.bandwidth_pass, candidate.overall_pass];
imagesc(double(passMatrix));
colormap([0.90 0.18 0.18; 0.25 0.80 0.25]);
clim([0 1]);
set(gca, "XTick", 1:3, "XTickLabel", ["thrust", "bandwidth", "overall"]);
set(gca, "YTick", 1:height(candidate), "YTickLabel", candidate.name);
title(sprintf("Candidate hardware screen: need %.2f kgf and %.1f Hz", ...
    TReqKgf, bwReqHz));
for r = 1:height(candidate)
    for c = 1:3
        text(c, r, passFail(passMatrix(r, c)), ...
            "HorizontalAlignment", "center", "FontWeight", "bold", "Color", "w");
    end
end

% Figure 4: thrust as a vectoring-bandwidth lever.
% More thrust -> smaller required deflection (theta = asin(F/T)) -> higher
% slew-limited vectoring bandwidth ceiling (slew/(2*pi*theta)). Uses the
% simultaneous worst-case rail force (heavy FF + friction) as the peak
% deflection the servo must swing to. The sweep starts at the authority
% minimum thrust (where theta = theta_max); every extra newton buys smaller
% deflection and more slew headroom. Thrust lifts ONLY the slew ceiling, not
% the servo's small-signal corner.
F_env_N = M_HEAVY_KG * (A_REF_MM * 1e-3) * (2 * pi * F_REF_HZ)^2 + authorityFrictionN;
Tlever = linspace(TReqN, THRUST_LEVER_MAX_N, 400)';
thetaReqDeg = asind(min(F_env_N ./ Tlever, 1.0));   % required peak deflection [deg]

fig4 = figure("Name", "select_thrust_bandwidth_lever", "Color", "w", ...
    "Position", [100 100 1150 460]);
tiledlayout(1, 2, "TileSpacing", "compact");

% Panel A: thrust shrinks the required deflection.
nexttile;
plot(Tlever, thetaReqDeg, "LineWidth", 2.6, "Color", bright.cyan);
hold on; grid on;
yline(THETA_MAX_DEG, ":", sprintf("theta_{max} %.0f deg", THETA_MAX_DEG), ...
    "Color", [0.3 0.3 0.3], "LineWidth", 1.2);
xline(TReqN, "--", sprintf("authority min %.1f N", TReqN), ...
    "Color", bright.magenta, "LineWidth", 1.4, "LabelOrientation", "horizontal");
xlabel("available motor thrust T [N]");
ylabel("required peak vector deflection [deg]");
title("Thrust shrinks the required deflection");
subtitle("theta = asin(F/T), F = heavy FF + friction");
ylim([0, THETA_MAX_DEG * 1.1]);
xlim([TReqN, THRUST_LEVER_MAX_N]);

% Panel B: smaller deflection lifts the slew-limited bandwidth ceiling.
nexttile;
hold on; grid on;
slewCols = [bright.orange; bright.lime; bright.blue];
for i = 1:numel(SERVO_SLEW_CLASSES_DEG_S)
    s = SERVO_SLEW_CLASSES_DEG_S(i);
    fbwCeil = s ./ (2.0 * pi * thetaReqDeg);        % slew-limited ceiling [Hz]
    plot(Tlever, fbwCeil, "LineWidth", 2.2, "Color", slewCols(i, :), ...
        "DisplayName", sprintf("slew %.0f deg/s", s));
end
yline(bwReqHz, "--", sprintf("required %.1f Hz", bwReqHz), ...
    "Color", bright.magenta, "LineWidth", 1.8, "DisplayName", "required bandwidth");
xlabel("available motor thrust T [N]");
ylabel("slew-limited vectoring bandwidth ceiling [Hz]");
title("Smaller deflection lifts the bandwidth ceiling");
subtitle("slew ceiling only (not the small-signal corner)");
ylim([0, max(bwReqHz * 2.5, 30)]);
xlim([TReqN, THRUST_LEVER_MAX_N]);
legend("Location", "northwest", "FontSize", 8);

if SAVE_FIGURES
    saveNamedFigure(fig1, plotDir, "select_authority_map");
    saveNamedFigure(fig2, plotDir, "select_bandwidth_drivers");
    saveNamedFigure(fig3, plotDir, "select_feasibility_matrix");
    saveNamedFigure(fig4, plotDir, "select_thrust_bandwidth_lever");
end

fprintf("\nDECISION READOUT\n");
fprintf("  Select motor-prop hardware with static thrust >= %.2f kgf.\n", TReqKgf);
fprintf("  Select vectoring hardware with usable bandwidth >= %.1f Hz.\n", bwReqHz);
fprintf("  Both criteria sized against a %.1f mm margin, %.1fx tighter than R4 (15 mm).\n\n", ...
    MARGIN_PEAK_MM, 15.0 / MARGIN_PEAK_MM);

%% Local functions

function Tn = requiredThrustN(Amm, fHz, mKg, thetaMaxRad, reserveN)
    lateralForceN = mKg .* (Amm .* 1e-3) .* (2.0 .* pi .* fHz).^2;
    Tn = (lateralForceN + reserveN) ./ sin(thetaMaxRad);
end

function [eFfMm, eFbMm, remMm] = budgetSharesMm(ffShare, specMm, eQuantMm, eModelMm)
    rem2 = specMm.^2 - eQuantMm.^2 - eModelMm.^2;
    rem2 = max(rem2, 1e-9);
    eFfMm = sqrt(ffShare .* rem2);
    eFbMm = sqrt((1.0 - ffShare) .* rem2);
    remMm = sqrt(rem2);
end

function e = feedforwardResidualMm(Amm, fRefHz, fVecHz)
    e = 0.5 .* (fRefHz ./ fVecHz).^2 .* Amm;
end

function bw = vectorBwFromFfResidualHz(Amm, fRefHz, eFfBudgetMm)
    bw = fRefHz .* sqrt(Amm ./ (2.0 .* eFfBudgetMm));
end

function fcHz = crossoverFromFbBudgetHz(mKg, FdN, eFbBudgetMm)
    wcRad = sqrt(FdN ./ (mKg .* (eFbBudgetMm .* 1e-3)));
    fcHz = wcRad ./ (2.0 .* pi);
end

function e = feedbackResidualMm(FdN, mKg, fcHz)
    wcRad = 2.0 .* pi .* fcHz;
    e = FdN ./ (mKg .* wcRad.^2) .* 1e3;
end

function [bw, bwFb, bwFf, bwRender, fcHz, driver] = requiredVectoringBandwidthHz( ...
        mKg, FdN, eFfBudgetMm, eFbBudgetMm, ARefMm, fRefHz, kWc, renderFactor)
    fcHz = crossoverFromFbBudgetHz(mKg, FdN, eFbBudgetMm);
    bwFb = fcHz ./ kWc;
    bwFf = vectorBwFromFfResidualHz(ARefMm, fRefHz, eFfBudgetMm);
    bwRender = renderFactor .* fRefHz;
    [bw, idx] = max([bwFb, bwFf, bwRender]);
    names = ["feedback disturbance rejection", "feedforward residual", "command rendering"];
    driver = names(idx);
end

function [bestShare, bestBw] = optimalSplitAndBandwidth( ...
        mKg, FdN, specMm, eQuantMm, eModelMm, ARefMm, fRefHz, kWc, renderFactor)
    shareGrid = linspace(0.002, 0.95, 2000);
    bwGrid = zeros(size(shareGrid));
    for i = 1:numel(shareGrid)
        [eff, efb] = budgetSharesMm(shareGrid(i), specMm, eQuantMm, eModelMm);
        bwGrid(i) = requiredVectoringBandwidthHz( ...
            mKg, FdN, eff, efb, ARefMm, fRefHz, kWc, renderFactor);
    end
    [bestBw, idx] = min(bwGrid);
    bestShare = shareGrid(idx);
end

function slewDegS = peakVectoringSlewDegS(Amm, fHz, mKg, thrustN)
    t = linspace(0, 1.0 ./ fHz, 2000);
    w = 2.0 .* pi .* fHz;
    accel = (Amm .* 1e-3) .* w.^2 .* sin(w .* t);
    s = max(min(mKg .* accel ./ thrustN, 1.0), -1.0);
    theta = asin(s);
    slewDegS = max(abs(gradient(theta, t))) .* 180.0 ./ pi;
end

function thetaDeg = requiredThetaAmpDeg(Amm, fHz, mKg, thrustN)
    aPeak = (Amm .* 1e-3) .* (2.0 .* pi .* fHz).^2;
    s = max(min(mKg .* aPeak ./ thrustN, 1.0), -1.0);
    thetaDeg = asin(s) .* 180.0 ./ pi;
end

function label = passFail(tf)
    if tf
        label = "PASS";
    else
        label = "FAIL";
    end
end

function saveNamedFigure(fig, plotDir, stem)
    pngPath = fullfile(plotDir, stem + ".png");
    figPath = fullfile(plotDir, stem + ".fig");
    exportgraphics(fig, pngPath, "Resolution", 300);
    savefig(fig, figPath);
    fprintf("Saved figure:\n  %s\n", pngPath);
end
