%% analyze_requirement_feasibility.m
% Requirements-first feasibility flowdown for thrust-vectoring rail hardware.
%
% This is a day-zero hardware-selection analysis: it uses only
% docs/requirements.md and docs/qualification_test_plan.md plus explicit
% design assumptions. It does not use measured thrust, measured servo dynamics,
% or identified friction. Those are later validation/ID steps.
%
% Architecture:
%   M1/M2 tracking are feedforward-dominated. Screen the vectoring actuator by
%   asking whether G_act(s) can render the inverse-dynamics theta_ff command.
%   M3 disturbance rejection is feedback-dominated. Derive the required
%   crossover from the 1 N force-step excursion limit, then ask whether the
%   actuator leaves enough gain/phase headroom at that crossover.

clear; clc; close all;

%% User options

SAVE_FIGURES = true;
WRITE_TABLES = true;

%% Paths

scriptPath = mfilename("fullpath");
feasibilityDir = fileparts(scriptPath);
analysisDir = fileparts(feasibilityDir);
repoRoot = fileparts(analysisDir);
plotDir = fullfile(repoRoot, "plots", "feasibility");
dataDir = fullfile(repoRoot, "data", "processed", "feasibility");

ensureDir(plotDir);
if WRITE_TABLES
    ensureDir(dataDir);
end

%% Configuration: requirements and design assumptions

G = 9.81;
M_LIGHT_KG = 0.45;                % qualification_test_plan.md Sec. 2
M_HEAVY_KG = 0.75;                % qualification_test_plan.md Sec. 2

% R4/R5 tracking limits used by M1/M2.
TRACKING_PEAK_LIMIT_MM = 15.0;    % R4
TRACKING_RMS_LIMIT_MM = 7.0;      % R5

% M1: minimum-jerk slew, -100 mm to +100 mm in 2.5 s.
M1_MOVE_MM = 200.0;
M1_DURATION_S = 2.5;
M1_DT_S = 0.005;

% M2: 100 mm amplitude, 0.5 Hz sine.
M2_AMPLITUDE_MM = 100.0;          % R1 / qualification M2
M2_FREQUENCY_HZ = 0.5;            % R2 / qualification M2

% M3: center-hold, injected input-referred force step.
M3_FORCE_STEP_N = 1.0;            % R13 / qualification M3
M3_PEAK_EXCURSION_MM = 15.0;      % R13
M3_RECOVER_MM = 3.0;              % R13
M3_RECOVER_S = 2.0;               % R13
M3_STEADY_STATE_MM = 1.0;         % R13

% M4: closed-loop ID band, shown as context only.
M4_BAND_HZ = [0.05, 2.5];

% A selected motor thrust is not assumed. The thrust axis below shows the
% tradeoff between available thrust and required vector angle. This marker is
% only a readout point for tables/plots, not an input requirement.
THRUST_MARKER_N = 12.0;
THETA_LIMIT_DEG = 45.0;

% Controller-agnostic vectoring-actuator support thresholds. A later controller
% can add lead or integral action, but it cannot undo actuator delay.
MIN_ACTUATOR_MAG_DB_AT_CROSSOVER = -3.0;
MAX_ACTUATOR_PHASE_LAG_DEG_AT_CROSSOVER = 45.0;

% Candidate actuator classes for screening, not real measured hardware.
candidate = table( ...
    ["3 Hz / 80 ms"; "5 Hz / 50 ms"; "10 Hz / 25 ms"; "20 Hz / 15 ms"; "30 Hz / 10 ms"], ...
    [3.0; 5.0; 10.0; 20.0; 30.0], ...
    [80.0; 50.0; 25.0; 15.0; 10.0], ...
    'VariableNames', {'name', 'actuator_bw_Hz', 'delay_ms'});

% Sweeps for hardware trade plots.
THRUST_MIN_N = 1.0;
THRUST_MAX_N = 25.0;
N_THRUST = 400;
BW_GRID_HZ = linspace(2.0, 30.0, 113);
DELAY_GRID_MS = linspace(0.0, 100.0, 101);
SERVO_SLEW_CLASSES_DEG_S = [300.0, 600.0, 900.0];

%% Requirement force and angle flowdown

m1 = makeM1MinimumJerk(M_HEAVY_KG, M1_MOVE_MM, M1_DURATION_S, M1_DT_S);
m2 = makeM2Sine(M_HEAVY_KG, M2_AMPLITUDE_MM, M2_FREQUENCY_HZ);

m1ForceN = max(abs(M_HEAVY_KG .* m1.accelRef));
m2ForceN = max(abs(M_HEAVY_KG .* m2.accelRef));
m3ForceN = M3_FORCE_STEP_N;
forceEnvelopeN = max([m1ForceN, m2ForceN, m3ForceN]);

thetaM1AtMarkerDeg = thetaForForceDeg(m1ForceN, THRUST_MARKER_N);
thetaM2AtMarkerDeg = thetaForForceDeg(m2ForceN, THRUST_MARKER_N);
thetaM3AtMarkerDeg = thetaForForceDeg(m3ForceN, THRUST_MARKER_N);
thetaEnvelopeAtMarkerDeg = thetaForForceDeg(forceEnvelopeN, THRUST_MARKER_N);

authorityMinThrustN = forceEnvelopeN / sind(THETA_LIMIT_DEG);
authorityMinThrustKgf = authorityMinThrustN / G;

m3FcPeakHz = sqrt(M3_FORCE_STEP_N / ...
    (M_LIGHT_KG * (M3_PEAK_EXCURSION_MM * 1e-3))) / (2.0 * pi);
m3FcRecoverHz = 1.0 / M3_RECOVER_S;
m3FcReqHz = max(m3FcPeakHz, m3FcRecoverHz);

%% Candidate actuator screening

m1Spec = makeM1MinimumJerkCommandSpectrum(M_HEAVY_KG, THRUST_MARKER_N, ...
    M1_MOVE_MM, M1_DURATION_S, M1_DT_S);

candidateRows = table();
for i = 1:height(candidate)
    screen = screenActuatorParams(candidate.actuator_bw_Hz(i), candidate.delay_ms(i), ...
        m1Spec, M_HEAVY_KG, THRUST_MARKER_N, ...
        M2_AMPLITUDE_MM, M2_FREQUENCY_HZ, m3FcReqHz, ...
        TRACKING_PEAK_LIMIT_MM, MIN_ACTUATOR_MAG_DB_AT_CROSSOVER, ...
        MAX_ACTUATOR_PHASE_LAG_DEG_AT_CROSSOVER);

    candidateRows = [candidateRows; table( ...
        candidate.name(i), candidate.actuator_bw_Hz(i), candidate.delay_ms(i), ...
        screen.m1ErrPkMm, screen.m2ErrPkMm, screen.m3MagDb, screen.m3PhaseLagDeg, ...
        screen.m1Pass, screen.m2Pass, screen.m3MagPass, screen.m3PhasePass, screen.overallPass, ...
        'VariableNames', {'candidate', 'actuator_bw_Hz', 'delay_ms', ...
                          'M1_peak_error_mm', 'M2_peak_error_mm', ...
                          'M3_Gact_mag_dB_at_fc', 'M3_Gact_phase_lag_deg_at_fc', ...
                          'M1_pass', 'M2_pass', 'M3_gain_pass', ...
                          'M3_phase_pass', 'overall_pass'})]; %#ok<AGROW>
end

%% Bandwidth-delay pass/fail grid

[BW, DELAY] = meshgrid(BW_GRID_HZ, DELAY_GRID_MS);
passGrid = false(size(BW));
m1ErrGrid = zeros(size(BW));
m2ErrGrid = zeros(size(BW));
m3PhaseGrid = zeros(size(BW));
m3MagGrid = zeros(size(BW));

for r = 1:size(BW, 1)
    for c = 1:size(BW, 2)
        screen = screenActuatorParams(BW(r, c), DELAY(r, c), ...
            m1Spec, M_HEAVY_KG, THRUST_MARKER_N, ...
            M2_AMPLITUDE_MM, M2_FREQUENCY_HZ, m3FcReqHz, ...
            TRACKING_PEAK_LIMIT_MM, MIN_ACTUATOR_MAG_DB_AT_CROSSOVER, ...
            MAX_ACTUATOR_PHASE_LAG_DEG_AT_CROSSOVER);
        passGrid(r, c) = screen.overallPass;
        m1ErrGrid(r, c) = screen.m1ErrPkMm;
        m2ErrGrid(r, c) = screen.m2ErrPkMm;
        m3PhaseGrid(r, c) = screen.m3PhaseLagDeg;
        m3MagGrid(r, c) = screen.m3MagDb;
    end
end

maxDelayFromM3Ms = maxAllowableDelayMs(BW_GRID_HZ, m3FcReqHz, ...
    MAX_ACTUATOR_PHASE_LAG_DEG_AT_CROSSOVER);

%% Thrust/angle trade

thrustN = linspace(THRUST_MIN_N, THRUST_MAX_N, N_THRUST)';
thetaM1Deg = thetaForForceDeg(m1ForceN, thrustN);
thetaM2Deg = thetaForForceDeg(m2ForceN, thrustN);
thetaM3Deg = thetaForForceDeg(m3ForceN, thrustN);
thetaEnvelopeDeg = thetaForForceDeg(forceEnvelopeN, thrustN);

%% Console report

fprintf("\n");
fprintf("===============================================================================\n");
fprintf("REQUIREMENTS-FIRST HARDWARE FEASIBILITY\n");
fprintf("===============================================================================\n");
fprintf("Sources: docs/requirements.md and docs/qualification_test_plan.md only.\n");
fprintf("Mass range                       : %.2f to %.2f kg\n", M_LIGHT_KG, M_HEAVY_KG);
fprintf("Tracking spec R4/R5              : peak <= %.1f mm, RMS <= %.1f mm\n", ...
    TRACKING_PEAK_LIMIT_MM, TRACKING_RMS_LIMIT_MM);
fprintf("M1 theta_ff 95%% energy frequency : %.2f Hz\n", m1Spec.f95Hz);
fprintf("M3 required crossover            : %.2f Hz (peak bound %.2f Hz, recovery scale %.2f Hz)\n", ...
    m3FcReqHz, m3FcPeakHz, m3FcRecoverHz);
fprintf("M3 actuator support threshold    : gain >= %.1f dB, phase lag <= %.0f deg at crossover\n", ...
    MIN_ACTUATOR_MAG_DB_AT_CROSSOVER, MAX_ACTUATOR_PHASE_LAG_DEG_AT_CROSSOVER);

fprintf("\nAUTHORITY FLOWDOWN\n");
fprintf("  M1 min-jerk peak inertial force : %.3f N\n", m1ForceN);
fprintf("  M2 sine peak inertial force     : %.3f N\n", m2ForceN);
fprintf("  M3 force-step counter-force     : %.3f N\n", m3ForceN);
fprintf("  Envelope force                  : %.3f N\n", forceEnvelopeN);
fprintf("  Minimum thrust at theta %.0f deg : %.3f N (%.3f kgf)\n", ...
    THETA_LIMIT_DEG, authorityMinThrustN, authorityMinThrustKgf);
fprintf("  At %.1f N thrust: theta envelope %.2f deg (M1 %.2f, M2 %.2f, M3 %.2f)\n", ...
    THRUST_MARKER_N, thetaEnvelopeAtMarkerDeg, thetaM1AtMarkerDeg, ...
    thetaM2AtMarkerDeg, thetaM3AtMarkerDeg);

fprintf("\nCANDIDATE ACTUATOR SCREEN\n");
fprintf("  %-14s %8s %8s %9s %9s %9s %9s\n", ...
    "candidate", "bw Hz", "delay", "M1 mm", "M2 mm", "M3 lag", "overall");
for i = 1:height(candidateRows)
    fprintf("  %-14s %8.1f %8.1f %9.2f %9.2f %9.2f %9s\n", ...
        candidateRows.candidate(i), candidateRows.actuator_bw_Hz(i), ...
        candidateRows.delay_ms(i), candidateRows.M1_peak_error_mm(i), ...
        candidateRows.M2_peak_error_mm(i), ...
        candidateRows.M3_Gact_phase_lag_deg_at_fc(i), ...
        passFail(candidateRows.overall_pass(i)));
end

fprintf("\nRECOMMENDED PURCHASE-FACING TARGET\n");
fprintf("  Vectoring actuator useful bandwidth >= 10 Hz with delay <= 25 ms.\n");
fprintf("  This class clears M1/M2 feedforward rendering and M3 crossover support in the screen.\n");
fprintf("  Thrust should be selected high enough that required deflection stays small; e.g. %.1f N gives %.1f deg envelope.\n", ...
    THRUST_MARKER_N, thetaEnvelopeAtMarkerDeg);

%% Figures

bright = getBrightPalette();

fig1 = figure("Name", "feasibility_maneuver_force_angle_flowdown", "Color", "w", ...
    "Position", [100 100 1150 460]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
forceVals = [m1ForceN; m2ForceN; m3ForceN; forceEnvelopeN];
bForce = bar(categorical(["M1"; "M2"; "M3"; "envelope"]), forceVals, ...
    "FaceColor", "flat", "DisplayName", "required rail-axis force");
bForce.CData = [bright.cyan; bright.cyan; bright.orange; bright.magenta];
grid on;
ylabel("rail-axis force [N]");
title("Qualification maneuver force requirements");
legend("Location", "northwest");

nexttile;
thetaVals = [thetaM1AtMarkerDeg; thetaM2AtMarkerDeg; thetaM3AtMarkerDeg; thetaEnvelopeAtMarkerDeg];
bTheta = bar(categorical(["M1"; "M2"; "M3"; "envelope"]), thetaVals, ...
    "FaceColor", "flat", "DisplayName", sprintf("required angle at %.1f N", THRUST_MARKER_N));
bTheta.CData = [bright.cyan; bright.cyan; bright.orange; bright.magenta];
hold on; grid on;
hTheta = yline(THETA_LIMIT_DEG, "--", sprintf("theta limit %.0f deg", THETA_LIMIT_DEG), ...
    "Color", bright.magenta, "LineWidth", 1.4);
ylabel("vector angle [deg]");
title("Required vector angle at marker thrust");
legend([bTheta, hTheta], "Location", "northwest");

fig2 = figure("Name", "feasibility_actuator_bandwidth_delay_map", "Color", "w", ...
    "Position", [120 120 1050 650]);
imagesc(BW_GRID_HZ, DELAY_GRID_MS, double(passGrid));
set(gca, "YDir", "normal");
colormap([0.90 0.18 0.18; 0.25 0.80 0.25]);
clim([0 1]);
hold on; grid on;
plot(BW_GRID_HZ, maxDelayFromM3Ms, "w--", "LineWidth", 1.8, ...
    "DisplayName", "M3 phase-lag delay bound");
for i = 1:height(candidate)
    markerColor = bright.blue;
    if candidateRows.overall_pass(i)
        markerColor = bright.lime;
    end
    plot(candidate.actuator_bw_Hz(i), candidate.delay_ms(i), "o", ...
        "MarkerFaceColor", markerColor, "MarkerEdgeColor", "k", ...
        "MarkerSize", 7, "DisplayName", candidate.name(i));
end
xlabel("candidate actuator useful bandwidth [Hz]");
ylabel("candidate delay [ms]");
title("Vectoring actuator feasibility over bandwidth and delay");
subtitle("green: M1 FFT feedforward + M2 sine rendering + M3 crossover gain/phase pass");
legend("Location", "northeastoutside", "FontSize", 8);

fig3 = figure("Name", "feasibility_thrust_deflection_slew_trade", "Color", "w", ...
    "Position", [140 140 1150 460]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(thrustN, thetaM1Deg, "LineWidth", 2.0, "Color", bright.cyan, ...
    "DisplayName", "M1 feedforward");
hold on; grid on;
plot(thrustN, thetaM2Deg, "LineWidth", 2.0, "Color", bright.blue, ...
    "DisplayName", "M2 feedforward");
plot(thrustN, thetaM3Deg, "LineWidth", 2.0, "Color", bright.orange, ...
    "DisplayName", "M3 counter-force");
plot(thrustN, thetaEnvelopeDeg, "LineWidth", 2.6, "Color", bright.magenta, ...
    "DisplayName", "authority envelope");
xline(THRUST_MARKER_N, "--", sprintf("%.0f N marker", THRUST_MARKER_N), ...
    "Color", [0.25 0.25 0.25], "LineWidth", 1.2, "HandleVisibility", "off");
yline(THETA_LIMIT_DEG, ":", sprintf("%.0f deg limit", THETA_LIMIT_DEG), ...
    "Color", [0.25 0.25 0.25], "LineWidth", 1.2, "HandleVisibility", "off");
xlabel("available thrust T [N]");
ylabel("required vector angle [deg]");
title("More thrust shrinks required deflection");
legend("Location", "northeast", "FontSize", 8);
ylim([0, min(90, max(10, 1.15 * max(thetaEnvelopeDeg, [], "omitnan")))]);

nexttile;
hold on; grid on;
slewColors = [bright.orange; bright.lime; bright.blue];
for k = 1:numel(SERVO_SLEW_CLASSES_DEG_S)
    slew = SERVO_SLEW_CLASSES_DEG_S(k);
    fSlewHz = slew ./ (2.0 * pi .* thetaEnvelopeDeg);
    plot(thrustN, fSlewHz, "LineWidth", 2.0, "Color", slewColors(k, :), ...
        "DisplayName", sprintf("slew %.0f deg/s", slew));
end
yline(10.0, "--", "10 Hz target class", "Color", bright.magenta, ...
    "LineWidth", 1.8, "DisplayName", "10 Hz target class");
xlabel("available thrust T [N]");
ylabel("slew-limited vectoring ceiling [Hz]");
title("Smaller deflection raises slew-limited bandwidth");
subtitle("slew ceiling only; small-signal bandwidth/delay must still be identified");
legend("Location", "northwest", "FontSize", 8);
ylim([0, 40]);

if SAVE_FIGURES
    saveNamedFigure(fig1, plotDir, "feasibility_maneuver_force_angle_flowdown");
    saveNamedFigure(fig2, plotDir, "feasibility_actuator_bandwidth_delay_map");
    saveNamedFigure(fig3, plotDir, "feasibility_thrust_deflection_slew_trade");
end

if WRITE_TABLES
    maneuverTable = table( ...
        ["M1"; "M2"; "M3"; "envelope"], ...
        ["minimum-jerk feedforward"; "0.5 Hz sine feedforward"; "1 N force-step rejection"; "maximum of M1-M3"], ...
        forceVals, thetaVals, ...
        'VariableNames', {'case_id', 'role', 'required_force_N', ...
                          'theta_at_marker_thrust_deg'});
    writetable(maneuverTable, fullfile(dataDir, "feasibility_maneuver_flowdown.csv"));
    writetable(candidateRows, fullfile(dataDir, "feasibility_candidate_actuator_screen.csv"));
    writetable(m1Spec.table, fullfile(dataDir, "feasibility_m1_theta_ff_spectrum.csv"));
end

%% Local functions

function ensureDir(pathName)
    if ~exist(pathName, "dir")
        mkdir(pathName);
    end
end

function data = makeM1MinimumJerk(massKg, moveMm, durationS, dtS)
    t = (0:dtS:durationS)';
    tau = t ./ durationS;
    dpM = moveMm * 1e-3;
    sigma = 10.0 .* tau.^3 - 15.0 .* tau.^4 + 6.0 .* tau.^5;
    accelRef = dpM ./ durationS.^2 .* (60.0 .* tau - 180.0 .* tau.^2 + 120.0 .* tau.^3);
    data = struct();
    data.t = t;
    data.posRef = -0.5 .* dpM + dpM .* sigma;
    data.accelRef = accelRef;
    data.forceRef = massKg .* accelRef;
end

function data = makeM2Sine(massKg, ampMm, freqHz)
    t = linspace(0, 1.0 / freqHz, 2000)';
    w = 2.0 * pi * freqHz;
    ampM = ampMm * 1e-3;
    data = struct();
    data.t = t;
    data.posRef = ampM .* sin(w .* t);
    data.accelRef = -ampM .* w.^2 .* sin(w .* t);
    data.forceRef = massKg .* data.accelRef;
end

function spec = makeM1MinimumJerkCommandSpectrum(massKg, thrustN, moveMm, durationS, dtS)
    traj = makeM1MinimumJerk(massKg, moveMm, durationS, dtS);
    thetaCmd = traj.forceRef ./ thrustN;
    n = numel(traj.t);
    thetaFft = fft(thetaCmd);
    fs = 1.0 / dtS;
    freqHz = (0:n-1)' .* fs ./ n;
    halfIdx = 1:floor(n / 2) + 1;
    posFreqHz = freqHz(halfIdx);
    thetaAmp = abs(thetaFft(halfIdx)) ./ n;
    if numel(thetaAmp) > 2
        thetaAmp(2:end-1) = 2.0 .* thetaAmp(2:end-1);
    end
    energy = abs(thetaFft(halfIdx)).^2;
    cumulativeEnergy = cumsum(energy) ./ max(sum(energy), eps);
    idx95 = find(cumulativeEnergy >= 0.95, 1, "first");

    spec = struct();
    spec.t = traj.t;
    spec.dtS = dtS;
    spec.posRef = traj.posRef;
    spec.thetaCmd = thetaCmd;
    spec.thetaFft = thetaFft;
    spec.freqHz = freqHz;
    spec.f95Hz = posFreqHz(idx95);
    spec.table = table(posFreqHz, thetaAmp, cumulativeEnergy, ...
        'VariableNames', {'frequency_Hz', 'theta_command_amplitude_rad', ...
                          'cumulative_theta_command_energy'});
end

function screen = screenActuatorParams(actuatorBwHz, delayMs, m1Spec, massKg, thrustN, ampMm, freqHz, ...
        fcReqHz, errLimitMm, minMagDb, maxPhaseLagDeg)
    [m1ErrPkMm, m1ResidualRatio] = m1FftRenderingErrorParams( ...
        actuatorBwHz, delayMs, m1Spec, massKg, thrustN);
    Hm2 = actuatorResponse(actuatorBwHz, delayMs, freqHz);
    m2ErrPkMm = ampMm * abs(1.0 - Hm2);

    Hm3 = actuatorResponse(actuatorBwHz, delayMs, fcReqHz);
    m3MagDb = mag2db(abs(Hm3));
    m3PhaseLagDeg = -rad2deg(angle(Hm3));

    screen = struct();
    screen.m1ErrPkMm = m1ErrPkMm;
    screen.m1ResidualRatio = m1ResidualRatio;
    screen.m2ErrPkMm = m2ErrPkMm;
    screen.m3MagDb = m3MagDb;
    screen.m3PhaseLagDeg = m3PhaseLagDeg;
    screen.m1Pass = m1ErrPkMm <= errLimitMm;
    screen.m2Pass = m2ErrPkMm <= errLimitMm;
    screen.m3MagPass = m3MagDb >= minMagDb;
    screen.m3PhasePass = m3PhaseLagDeg <= maxPhaseLagDeg;
    screen.overallPass = screen.m1Pass && screen.m2Pass && ...
        screen.m3MagPass && screen.m3PhasePass;
end

function [errPkMm, thetaResidualRatio] = m1FftRenderingErrorParams(actuatorBwHz, delayMs, spec, massKg, thrustN)
    n = numel(spec.thetaFft);
    fs = 1.0 / spec.dtS;
    signedFreqHz = spec.freqHz;
    negIdx = (1:n)' > floor(n / 2) + 1;
    signedFreqHz(negIdx) = signedFreqHz(negIdx) - fs;
    H = actuatorResponse(actuatorBwHz, delayMs, signedFreqHz);

    thetaAct = real(ifft(spec.thetaFft .* H));
    accelAct = thetaAct .* thrustN ./ massKg;
    velAct = cumtrapz(spec.t, accelAct);
    posAct = spec.posRef(1) + cumtrapz(spec.t, velAct);
    errPkMm = max(abs(posAct - spec.posRef)) .* 1e3;

    thetaResidual = spec.thetaCmd - thetaAct;
    thetaResidualRatio = sqrt(mean(thetaResidual.^2)) ./ ...
        max(sqrt(mean(spec.thetaCmd.^2)), eps);
end

function H = actuatorResponse(bwHz, delayMs, freqHz)
    delayS = delayMs ./ 1000.0;
    H = 1.0 ./ (1.0 + 1i .* (freqHz ./ bwHz)) .* ...
        exp(-1i .* 2.0 .* pi .* freqHz .* delayS);
end

function thetaDeg = thetaForForceDeg(forceN, thrustN)
    ratio = forceN ./ thrustN;
    thetaDeg = NaN(size(ratio));
    valid = abs(ratio) <= 1.0;
    thetaDeg(valid) = asind(ratio(valid));
end

function maxDelayMs = maxAllowableDelayMs(actuatorBwHz, crossoverHz, phaseBudgetDeg)
    lagPhaseDeg = atan2d(crossoverHz, actuatorBwHz);
    remainingDeg = phaseBudgetDeg - lagPhaseDeg;
    maxDelayMs = 1000.0 .* max(remainingDeg, 0.0) ./ (360.0 .* crossoverHz);
end

function bright = getBrightPalette()
    bright = struct();
    bright.cyan = [0.00 0.90 1.00];
    bright.orange = [1.00 0.62 0.00];
    bright.lime = [0.45 1.00 0.45];
    bright.magenta = [1.00 0.20 0.90];
    bright.blue = [0.15 0.55 1.00];
end

function label = passFail(tfValue)
    if tfValue
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
