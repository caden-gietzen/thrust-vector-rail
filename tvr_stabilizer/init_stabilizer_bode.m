%% Crude stabilizer Bode and margin sweep
% This script documents the Phase 1 crude-stabilizer loop-shaping logic.
% The ideal double-integrator model is retained for intuition; the rough
% actuator model and implemented Simulink loop are the margin checks that
% justify the first hardware gains.

clear; close all; clc;

p0 = params();
omegaGrid = [0.3 0.6 1.0 1.8 3.0 6.0];

fprintf('\nCrude stabilizer Bode/margin sweep\n');
fprintf('Soft integral convention: k1 = 3*w^2, k2 = 3*w, ki = 0.5*w^3\n');
fprintf('Servo truth model: tau = %.4f s, delay = %.4f s\n\n', ...
    p0.truth.tau_theta, p0.truth.L_theta);

idealRows = repmat(emptyMarginRow(), numel(omegaGrid), 1);
roughRows = repmat(emptyMarginRow(), numel(omegaGrid), 1);

for i = 1:numel(omegaGrid)
    w = omegaGrid(i);
    gains = gainsFromOmega(w);

    idealLoop = buildIdealLoop(gains);
    roughLoop = buildRoughActuatorLoop(gains, p0);

    idealRows(i) = analyzeLoop("ideal_double_integrator", w, idealLoop);
    roughRows(i) = analyzeLoop("rough_servo_delay", w, roughLoop);
end

printMarginTable("Ideal feedback-linearized double integrator", idealRows);
printMarginTable("Rough identified servo lag/delay loop", roughRows);

implementedRows = analyzeImplementedLoopSweep(omegaGrid);
if ~isempty(implementedRows)
    printMarginTable("Implemented Simulink loop with velocity filter", implementedRows);
end

selectedOmega = 0.6;
selectedGains = gainsFromOmega(selectedOmega);
selectedLoop = buildRoughActuatorLoop(selectedGains, p0);
selectedClosedLoop = feedback(selectedLoop, 1);

fprintf('\nSelected first-hardware tune\n');
fprintf('omega = %.3f rad/s\n', selectedOmega);
fprintf('k1 = %.6g, k2 = %.6g, ki = %.6g\n', ...
    selectedGains.k1, selectedGains.k2, selectedGains.ki);
printClosedLoopMetrics(selectedClosedLoop);

figure('Name', 'Selected rough-model open-loop margin');
margin(selectedLoop); grid on;
title(sprintf('Crude stabilizer rough loop: omega=%.2f rad/s', selectedOmega));

figure('Name', 'Selected rough-model closed-loop step');
step(selectedClosedLoop); grid on;
title(sprintf('Closed-loop step, rough model: omega=%.2f rad/s', selectedOmega));

figure('Name', 'Selected rough-model closed-loop Bode');
bode(selectedClosedLoop); grid on;
title(sprintf('Closed-loop Bode, rough model: omega=%.2f rad/s', selectedOmega));

function gains = gainsFromOmega(w)
    gains = struct();
    gains.omega = w;
    gains.k1 = 3*w^2;
    gains.k2 = 3*w;
    gains.ki = 0.5*w^3;
end

function L = buildIdealLoop(gains)
    s = tf('s');
    P = 1/s^2;
    C = gains.k2*s + gains.k1 + gains.ki/s;
    L = minreal(series(C, P));
end

function L = buildRoughActuatorLoop(gains, p)
    s = tf('s');
    C = gains.k2*s + gains.k1 + gains.ki/s;
    P = (1/s^2) * (1/(1 + p.truth.tau_theta*s));
    P.InputDelay = p.truth.L_theta;
    L = minreal(series(C, P));
end

function rows = analyzeImplementedLoopSweep(omegaGrid)
    rows = repmat(emptyMarginRow(), numel(omegaGrid), 1);
    try
        mdl = 'tvr_sim';
        if ~bdIsLoaded(mdl)
            load_system(mdl);
        end

        dblk = find_system(mdl, 'BlockType', 'TransportDelay');
        for j = 1:numel(dblk)
            set_param(dblk{j}, 'PadeOrder', '3');
        end

        for i = 1:numel(omegaGrid)
            w = omegaGrid(i);
            gains = gainsFromOmega(w);
            L = getImplementedLoop(mdl, w, gains);
            rows(i) = analyzeLoop("implemented_simulink", w, L);
        end
    catch ME
        warning('Skipping implemented-loop sweep: %s', ME.message);
        rows = [];
    end
end

function L = getImplementedLoop(mdl, w, gains)
    p = params();
    p.des.omega = w;
    p.des.k1 = gains.k1;
    p.des.k2 = gains.k2;
    p.des.ki = gains.ki;
    p.tog.friction = 0;
    p.tog.meas_noise = 0;
    p.ref.amp = 0;
    assignin('base', 'p', p);

    io = linio([mdl '/ctrl_fblin'], 1, 'output');
    sllin = slLinearizer(mdl, io);
    pts = getPoints(sllin);
    if iscell(pts)
        pt = pts{1};
    else
        pt = pts;
    end
    L = -getLoopTransfer(sllin, pt);
    L = minreal(tf(L));
end

function row = analyzeLoop(label, w, L)
    [gm, pm, wcg, wcp] = margin(L);
    row = emptyMarginRow();
    row.label = label;
    row.omega = w;
    row.gmDb = gainToDb(gm);
    row.pmDeg = pm;
    row.wcg = wcg;
    row.wcp = wcp;
    row.wcpHz = wcp/(2*pi);
    row.closedLoopStable = assessClosedLoopStability(L);
    row.warning = marginWarning(L, gm, pm, row.closedLoopStable);
end

function stable = assessClosedLoopStability(L)
    T = feedback(L, 1);
    try
        stable = isstable(T);
    catch
        stable = isstable(pade(T, 3));
    end
end

function warningText = marginWarning(L, gm, pm, closedLoopStable)
    warningParts = strings(0, 1);
    try
        margins = allmargin(L);
        if isfield(margins, 'Stable') && ~margins.Stable
            warningParts(end+1) = "allmargin_unstable";
        end
        if isfield(margins, 'GainMargin') && numel(margins.GainMargin) > 1
            warningParts(end+1) = "multiple_gain_margins";
        end
        if isfield(margins, 'PhaseMargin') && numel(margins.PhaseMargin) > 1
            warningParts(end+1) = "multiple_phase_margins";
        end
    catch ME
        warningParts(end+1) = "allmargin_failed:" + string(ME.identifier);
    end

    if ~closedLoopStable
        warningParts(end+1) = "closed_loop_unstable";
    end
    if isfinite(gm) && gm < 1
        warningParts(end+1) = "gain_margin_below_0_dB";
    end
    if isfinite(pm) && pm < 45
        warningParts(end+1) = "phase_margin_below_45_deg";
    end

    if isempty(warningParts)
        warningText = "";
    else
        warningText = strjoin(unique(warningParts, 'stable'), ", ");
    end
end

function printMarginTable(titleText, rows)
    fprintf('\n%s\n', titleText);
    fprintf('%7s %10s %9s %11s %11s %8s %s\n', ...
        'omega', 'wc(rad/s)', 'wc(Hz)', 'PM(deg)', 'GM(dB)', 'stable', 'warnings');
    for i = 1:numel(rows)
        fprintf('%7.3f %10.3f %9.3f %11.2f %11.2f %8s %s\n', ...
            rows(i).omega, rows(i).wcp, rows(i).wcpHz, rows(i).pmDeg, ...
            rows(i).gmDb, string(rows(i).closedLoopStable), rows(i).warning);
    end
end

function printClosedLoopMetrics(T)
    bw = bandwidth(T);
    info = stepinfo(T);
    fprintf('Closed-loop bandwidth = %.3f rad/s (%.3f Hz)\n', bw, bw/(2*pi));
    fprintf('Settling time = %.3f s\n', info.SettlingTime);
    fprintf('Overshoot = %.2f%%\n', info.Overshoot);
end

function row = emptyMarginRow()
    row = struct( ...
        'label', "", ...
        'omega', NaN, ...
        'gmDb', NaN, ...
        'pmDeg', NaN, ...
        'wcg', NaN, ...
        'wcp', NaN, ...
        'wcpHz', NaN, ...
        'closedLoopStable', false, ...
        'warning', "");
end

function db = gainToDb(gm)
    if isempty(gm) || isnan(gm)
        db = NaN;
    elseif isinf(gm)
        db = Inf;
    else
        db = 20*log10(gm);
    end
end
