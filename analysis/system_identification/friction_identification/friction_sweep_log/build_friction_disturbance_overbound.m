%% build_friction_disturbance_overbound.m
% Build a conservative, simulation-facing friction disturbance family.
%
% This is a V&V overbound, not a finalized friction identification. It uses
% the current candidate friction sweep data to extract:
%   1. direction-dependent dynamic friction coefficients,
%   2. static breakaway/stiction bounds,
%   3. residual-vs-position bounds for spatial disturbance bumps.

clear; clc; close all;

%% User options

SAVE_FIGURES = true;
SPATIAL_SEED = 1;
N_SPATIAL_REALIZATIONS = 20;
SPATIAL_BUMP_COUNT_RANGE = [2, 5];
SPATIAL_WIDTH_RANGE_M = [0.015, 0.080];
RESIDUAL_PERCENTILE = 95;
V_EPSILON_MPS = 0.01;

%% Models and processing constants copied from analyze_friction_sweep.m

M_kg = 0.4536;
K_servo = 0.001556;
tau_servo = 0.0244;
L_servo = 0.0288;
u_neutral = 1431;
K_T = 0.00414;
tau_T = 0.0781;
L_T = 0.0252;
u_T_min = 1075;
C_m = 64810.4;
SG_WINDOW_S = 0.20;
SG_POLY = 3;
V_MIN_MPS = 0.01;

%% Locate data and output folders

scriptPath = mfilename("fullpath");
repoRoot = findRepoRoot(fileparts(scriptPath));
addpath(fullfile(repoRoot, "analysis", "utils"));

dataDir = getMirroredRawDataDir(scriptPath, "candidate");
csvFiles = dir(fullfile(dataDir, "*.csv"));

if isempty(csvFiles)
    error("No candidate friction CSV files found in:\n%s", dataDir);
end

[~, sortIdx] = sort(string({csvFiles.name}));
csvFiles = csvFiles(sortIdx);

plotDir = fullfile(getMirroredPlotDir(scriptPath), "friction_disturbance_overbound");
reportDir = strrep(plotDir, fullfile(repoRoot, "plots"), fullfile(repoRoot, "reports"));

if ~exist(plotDir, "dir")
    mkdir(plotDir);
end
if ~exist(reportDir, "dir")
    mkdir(reportDir);
end

fprintf("\nFriction disturbance overbound\n");
fprintf("  Dataset: %s\n", dataDir);
fprintf("  Candidate files: %d\n", numel(csvFiles));

%% Load runs

runs = loadFrictionRuns(csvFiles, dataDir, u_neutral);
nRuns = numel(runs);

if nRuns == 0
    error("No valid friction runs loaded.");
end

%% Dynamic models

[n_servo_pad, d_servo_pad] = pade(L_servo, 3);
G_servo = tf(K_servo, [tau_servo, 1]) * tf(n_servo_pad, d_servo_pad);

[n_thrust_pad, d_thrust_pad] = pade(L_T, 3);
G_thrust = tf(K_T, [tau_T, 1]) * tf(n_thrust_pad, d_thrust_pad);

%% Stiction and residual extraction

runs = addStaticRailForce(runs, K_T, u_T_min);
stictionTable = buildStictionTable(runs);

[residualSamples, runs] = computeResidualSamples( ...
    runs, ...
    G_servo, ...
    G_thrust, ...
    M_kg, ...
    C_m, ...
    SG_WINDOW_S, ...
    SG_POLY, ...
    V_MIN_MPS, ...
    u_neutral, ...
    u_T_min);

if isempty(residualSamples)
    error("No valid residual samples generated.");
end

fprintf("  Valid residual samples: %d\n", height(residualSamples));

%% Fit direction-dependent friction

fits = fitDirectionalFriction(residualSamples);
residualSamples = addResidualRemainder(residualSamples, fits);

spatial = deriveSpatialBounds(residualSamples, RESIDUAL_PERCENTILE, SPATIAL_BUMP_COUNT_RANGE, SPATIAL_WIDTH_RANGE_M, SPATIAL_SEED);
dynamicCases = buildDynamicCases(fits, residualSamples);
stiction = collapseStictionBounds(stictionTable, V_EPSILON_MPS);

frictionOverbound = struct();
frictionOverbound.meta.generated = datestr(now, "yyyy-mm-dd HH:MM:SS");
frictionOverbound.meta.source_data_dir = strrep(char(dataDir), "\", "/");
frictionOverbound.meta.status = "provisional_candidate_overbound";
frictionOverbound.meta.note = "Use for controller V&V stress testing; this is not an earned friction model.";
frictionOverbound.dynamic = dynamicCases;
frictionOverbound.stiction = stiction;
frictionOverbound.spatial = spatial;

summary = buildSummaryTable(fits, dynamicCases, stiction, spatial, residualSamples, stictionTable, nRuns);

%% Spatial realizations for plotting / examples

spatialRealizations = sampleSpatialRealizations(spatial, N_SPATIAL_REALIZATIONS);

%% Save outputs

matPath = fullfile(plotDir, "friction_overbound_params.mat");
jsonPath = fullfile(plotDir, "friction_overbound_params.json");
residualCsvPath = fullfile(plotDir, "friction_residual_samples.csv");
summaryCsvPath = fullfile(plotDir, "friction_overbound_summary.csv");
reportPath = fullfile(reportDir, "friction_disturbance_overbound.report.md");

writetable(residualSamples, residualCsvPath);
writetable(summary, summaryCsvPath);
save(matPath, "frictionOverbound", "residualSamples", "stictionTable", "fits", "spatialRealizations", "summary");

fid = fopen(jsonPath, "w");
fprintf(fid, "%s", jsonencode(frictionOverbound, "PrettyPrint", true));
fclose(fid);

if SAVE_FIGURES
    plotForceVsVelocityEnvelope(residualSamples, fits, dynamicCases, fullfile(plotDir, "friction_force_vs_velocity_envelope.png"));
    plotResidualVsPosition(residualSamples, spatial, fullfile(plotDir, "friction_residual_vs_position.png"));
    plotSpatialBumpRealizations(spatial, spatialRealizations, fullfile(plotDir, "friction_spatial_bump_realizations.png"));
end

writeReport(reportPath, frictionOverbound, fits, dynamicCases, stiction, spatial, summary, stictionTable, nRuns, residualSamples, dataDir);

fprintf("\nFriction overbound outputs written to:\n  %s\n", plotDir);
fprintf("Report written to:\n  %s\n", reportPath);

%% Local functions

function runs = loadFrictionRuns(csvFiles, dataDir, uNeutral)
    runs = struct([]);

    for k = 1:numel(csvFiles)
        fpath = fullfile(dataDir, csvFiles(k).name);
        T = readtable(fpath, "TextType", "string");

        required = ["t_s", "phase", "servo_us", "esc_pwm_us", "count", "count_from_start"];
        if ~all(ismember(required, string(T.Properties.VariableNames)))
            fprintf("Skipping %s: unexpected columns.\n", csvFiles(k).name);
            continue;
        end

        idxRun = strcmp(string(T.phase), "run");

        if any(idxRun)
            servoUs = median(double(T.servo_us(idxRun)), "omitnan");
            escUs = median(double(T.esc_pwm_us(idxRun)), "omitnan");
        else
            servoUs = median(double(T.servo_us), "omitnan");
            escUs = median(double(T.esc_pwm_us), "omitnan");
        end

        thetaCmdDeg = -(servoUs - uNeutral) * 0.091092;
        angleSign = sign(thetaCmdDeg);
        if angleSign == 0
            angleSign = 1;
        end

        hasStiction = any(strcmp(string(T.phase), "stiction_halt"));
        hasEndstop = any(strcmp(string(T.phase), "endstop_halt"));
        hasTimeout = any(strcmp(string(T.phase), "timeout_end"));

        r.filename = string(csvFiles(k).name);
        r.T = T;
        r.servo_us = servoUs;
        r.esc_us = escUs;
        r.theta_cmd_deg = thetaCmdDeg;
        r.theta_cmd_rad = thetaCmdDeg * pi / 180;
        r.angle_sign = angleSign;
        r.has_stiction = hasStiction;
        r.has_endstop = hasEndstop;
        r.has_timeout = hasTimeout;
        r.is_stiction = hasStiction && ~hasEndstop && ~hasTimeout;
        r.is_motion = hasEndstop || hasTimeout || ...
            (any(idxRun) && max(abs(double(T.count_from_start(idxRun)))) >= 15);

        if isempty(runs)
            runs = r;
        else
            runs(end+1) = r; %#ok<AGROW>
        end
    end
end

function runs = addStaticRailForce(runs, kT, uTMin)
    for k = 1:numel(runs)
        deltaU = max(0, runs(k).esc_us - uTMin);
        runs(k).F_thrust_ss = kT * deltaU;
        runs(k).F_rail_ss = runs(k).F_thrust_ss * sin(runs(k).theta_cmd_rad);
    end
end

function stictionTable = buildStictionTable(runs)
    angleRounded = arrayfun(@(r) round(r.theta_cmd_deg), runs);
    angles = unique(angleRounded);

    rows = cell(0, 8);

    for i = 1:numel(angles)
        angle = angles(i);
        idx = find(angleRounded == angle);

        noMotionIdx = idx(arrayfun(@(j) ~runs(j).is_motion, idx));
        motionIdx = idx(arrayfun(@(j) runs(j).is_motion, idx));

        lowerN = NaN;
        upperN = NaN;
        lowerEsc = NaN;
        upperEsc = NaN;
        lowerFile = "";
        upperFile = "";

        if ~isempty(noMotionIdx)
            [~, local] = max(arrayfun(@(j) runs(j).esc_us, noMotionIdx));
            rr = runs(noMotionIdx(local));
            lowerN = abs(rr.F_rail_ss);
            lowerEsc = rr.esc_us;
            lowerFile = rr.filename;
        end

        if ~isempty(motionIdx)
            [~, local] = min(arrayfun(@(j) runs(j).esc_us, motionIdx));
            rr = runs(motionIdx(local));
            upperN = abs(rr.F_rail_ss);
            upperEsc = rr.esc_us;
            upperFile = rr.filename;
        end

        rows(end+1, :) = {angle, sign(angle), lowerN, upperN, lowerEsc, upperEsc, lowerFile, upperFile}; %#ok<AGROW>
    end

    stictionTable = cell2table(rows, 'VariableNames', { ...
        'angle_deg', ...
        'direction_sign', ...
        'breakaway_lower_N', ...
        'breakaway_upper_N', ...
        'lower_esc_us', ...
        'upper_esc_us', ...
        'lower_source_file', ...
        'upper_source_file' ...
    });
end

function [samples, runs] = computeResidualSamples(runs, GServo, GThrust, massKg, countsPerM, sgWindowS, sgPoly, vMinMps, uNeutral, uTMin)
    rows = cell(0, 13);

    for k = 1:numel(runs)
        if ~runs(k).is_motion
            continue;
        end

        T = runs(k).T;
        mask = strcmp(string(T.phase), "run") | ...
               strcmp(string(T.phase), "endstop_halt") | ...
               strcmp(string(T.phase), "timeout_end");

        if sum(mask) < 20
            continue;
        end

        t_s = double(T.t_s(mask));
        cnt = double(T.count_from_start(mask));
        svUs = double(T.servo_us(mask));
        escUs = double(T.esc_pwm_us(mask));
        x_m = cnt / countsPerM;

        dt = median(diff(t_s), "omitnan");
        if ~isfinite(dt) || dt <= 0 || dt > 0.1
            continue;
        end

        nSamp = numel(t_s);
        tUniform = (0:nSamp-1)' * dt;

        winN = round(sgWindowS / dt);
        if mod(winN, 2) == 0
            winN = winN + 1;
        end
        winN = max(winN, sgPoly + 2);

        xFilt = sgolayfilt(x_m, sgPoly, winN);
        xdot = gradient(xFilt, dt);
        xddot = gradient(xdot, dt);

        thetaRad = lsim(GServo, svUs - uNeutral, tUniform);
        thrustDev = max(0, escUs - uTMin);
        FThrust = lsim(GThrust, thrustDev, tUniform);
        FThrust = max(0, FThrust);
        FRail = FThrust .* sin(thetaRad);
        FFric = FRail - massKg .* xddot;

        nTrim = round(0.5 / dt);
        valid = abs(xdot) >= vMinMps;
        valid(1:min(nTrim, end)) = false;
        valid(max(1, end-nTrim):end) = false;

        if sum(valid) < 5
            continue;
        end

        runs(k).x_m = x_m;
        runs(k).xdot = xdot;
        runs(k).xddot = xddot;
        runs(k).F_fric = FFric;
        runs(k).F_rail = FRail;
        runs(k).idx_valid = valid;

        validIdx = find(valid);
        for ii = 1:numel(validIdx)
            j = validIdx(ii);
            rows(end+1, :) = { ...
                runs(k).filename, ...
                runs(k).theta_cmd_deg, ...
                runs(k).angle_sign, ...
                runs(k).esc_us, ...
                tUniform(j), ...
                x_m(j), ...
                xdot(j), ...
                xddot(j), ...
                thetaRad(j), ...
                FThrust(j), ...
                FRail(j), ...
                FFric(j), ...
                sign(xdot(j)) ...
            }; %#ok<AGROW>
        end
    end

    samples = cell2table(rows, 'VariableNames', { ...
        'source_file', ...
        'theta_cmd_deg', ...
        'direction_sign', ...
        'esc_us', ...
        't_s', ...
        'position_m', ...
        'velocity_mps', ...
        'acceleration_mps2', ...
        'theta_rad', ...
        'thrust_N', ...
        'rail_force_N', ...
        'friction_residual_N', ...
        'velocity_sign' ...
    });
end

function fits = fitDirectionalFriction(samples)
    fits = struct();
    fits.pooled = fitOne(samples.velocity_mps, samples.friction_residual_N);

    pos = samples.velocity_mps > 0;
    neg = samples.velocity_mps < 0;
    fits.positive = fitOne(samples.velocity_mps(pos), samples.friction_residual_N(pos));
    fits.negative = fitOne(samples.velocity_mps(neg), samples.friction_residual_N(neg));
end

function out = fitOne(v, f)
    A = [v(:), sign(v(:))];
    p = A \ f(:);
    pred = A * p;
    resid = f(:) - pred;
    out.b_Ns_per_m = abs(p(1));
    out.mu_N = abs(p(2));
    out.signed_b = p(1);
    out.signed_mu = p(2);
    out.rmse_N = rms(resid);
    out.n = numel(v);
    out.residual_abs_p95_N = percentileLocal(abs(resid), 95);
    out.residual_abs_p99_N = percentileLocal(abs(resid), 99);
end

function samples = addResidualRemainder(samples, fits)
    pred = NaN(height(samples), 1);

    pos = samples.velocity_mps > 0;
    neg = samples.velocity_mps < 0;
    pred(pos) = fits.positive.signed_b * samples.velocity_mps(pos) + ...
        fits.positive.signed_mu * sign(samples.velocity_mps(pos));
    pred(neg) = fits.negative.signed_b * samples.velocity_mps(neg) + ...
        fits.negative.signed_mu * sign(samples.velocity_mps(neg));

    samples.dynamic_fit_N = pred;
    samples.spatial_residual_N = samples.friction_residual_N - pred;
end

function cases = buildDynamicCases(fits, samples)
    padN = percentileLocal(abs(samples.spatial_residual_N), 95);

    muNomPos = fits.positive.mu_N;
    muNomNeg = fits.negative.mu_N;
    bNomPos = fits.positive.b_Ns_per_m;
    bNomNeg = fits.negative.b_Ns_per_m;

    cases.nominal = makeCase(bNomPos, muNomPos, bNomNeg, muNomNeg, "direction-dependent fitted nominal");
    cases.low = makeCase( ...
        0.80*bNomPos, max(0, muNomPos - 0.50*padN), ...
        0.80*bNomNeg, max(0, muNomNeg - 0.50*padN), ...
        "low friction stress case");
    cases.high = makeCase( ...
        1.20*bNomPos, muNomPos + padN, ...
        1.20*bNomNeg, muNomNeg + padN, ...
        "high friction overbound using p95 residual padding");

    maxB = max([bNomPos, bNomNeg]);
    maxMu = max([muNomPos, muNomNeg]) + padN;
    minB = min([bNomPos, bNomNeg]);
    minMu = max(0, min([muNomPos, muNomNeg]) - 0.25*padN);
    cases.asymmetric_worst = makeCase( ...
        1.25*maxB, maxMu, ...
        0.75*minB, minMu, ...
        "worst observed direction asymmetry plus residual padding");
end

function c = makeCase(bPos, muPos, bNeg, muNeg, description)
    c.description = description;
    c.positive_direction.b_Ns_per_m = bPos;
    c.positive_direction.mu_N = muPos;
    c.negative_direction.b_Ns_per_m = bNeg;
    c.negative_direction.mu_N = muNeg;
end

function stiction = collapseStictionBounds(stictionTable, vEpsilon)
    lowers = stictionTable.breakaway_lower_N(isfinite(stictionTable.breakaway_lower_N));
    uppers = stictionTable.breakaway_upper_N(isfinite(stictionTable.breakaway_upper_N));

    if isempty(lowers)
        low = 0;
    else
        low = min(lowers);
    end

    if isempty(uppers)
        high = NaN;
    else
        high = max(uppers);
    end

    stiction.breakaway_low_N = low;
    stiction.breakaway_high_N = high;
    stiction.v_epsilon_mps = vEpsilon;
    stiction.note = "Use high threshold for conservative breakaway stress tests.";
end

function spatial = deriveSpatialBounds(samples, pct, bumpCountRange, widthRange, seed)
    x = samples.position_m;
    r = samples.spatial_residual_N;
    x = x(isfinite(x) & isfinite(r));
    r = samples.spatial_residual_N(isfinite(samples.position_m) & isfinite(samples.spatial_residual_N));

    spatial.bump_count_range = bumpCountRange;
    spatial.amplitude_abs_max_N = percentileLocal(abs(r), pct);
    spatial.amplitude_abs_p99_N = percentileLocal(abs(r), 99);
    spatial.width_range_m = widthRange;
    spatial.position_range_m = [min(x), max(x)];
    spatial.seed = seed;
    spatial.percentile_basis = pct;
    spatial.note = "Gaussian bump amplitudes are bounded by residual-vs-position remainder after dynamic friction fit.";
end

function realizations = sampleSpatialRealizations(spatial, nRealizations)
    rng(spatial.seed, "twister");
    xGrid = linspace(spatial.position_range_m(1), spatial.position_range_m(2), 250).';
    realizations = struct("x_grid_m", xGrid, "force_N", [], "centers_m", [], "amplitudes_N", [], "widths_m", []);

    for i = 1:nRealizations
        nBumps = randi(spatial.bump_count_range);
        centers = spatial.position_range_m(1) + diff(spatial.position_range_m) * rand(nBumps, 1);
        widths = spatial.width_range_m(1) + diff(spatial.width_range_m) * rand(nBumps, 1);
        amplitudes = spatial.amplitude_abs_max_N * (2*rand(nBumps, 1) - 1);
        force = zeros(size(xGrid));

        for b = 1:nBumps
            force = force + amplitudes(b) * exp(-((xGrid - centers(b)).^2) ./ (2*widths(b)^2));
        end

        realizations(i).x_grid_m = xGrid;
        realizations(i).force_N = force;
        realizations(i).centers_m = centers;
        realizations(i).amplitudes_N = amplitudes;
        realizations(i).widths_m = widths;
    end
end

function summary = buildSummaryTable(fits, dynamicCases, stiction, spatial, samples, stictionTable, nRuns)
    metric = [
        "candidate_runs"
        "valid_residual_samples"
        "positive_b_Ns_per_m"
        "positive_mu_N"
        "negative_b_Ns_per_m"
        "negative_mu_N"
        "pooled_rmse_N"
        "spatial_amplitude_abs_max_N"
        "stiction_breakaway_low_N"
        "stiction_breakaway_high_N"
    ];

    value = [
        nRuns
        height(samples)
        fits.positive.b_Ns_per_m
        fits.positive.mu_N
        fits.negative.b_Ns_per_m
        fits.negative.mu_N
        fits.pooled.rmse_N
        spatial.amplitude_abs_max_N
        stiction.breakaway_low_N
        stiction.breakaway_high_N
    ];

    summary = table(metric, value);

    %#ok<NASGU> dynamicCases stictionTable
end

function plotForceVsVelocityEnvelope(samples, fits, cases, pngPath)
    fig = figure("Name", "friction_force_vs_velocity_envelope", "Color", "w");
    scatter(samples.velocity_mps, samples.friction_residual_N, 6, [0.7 0.7 0.7], "filled");
    hold on;
    v = linspace(min(samples.velocity_mps), max(samples.velocity_mps), 300).';
    plotCase(v, cases.nominal, [0 0.65 0.95], "nominal");
    plotCase(v, cases.high, [1 0.45 0], "high");
    plotCase(v, cases.asymmetric_worst, [0.45 1 0.45], "asymmetric worst");
    xlabel("Velocity (m/s)");
    ylabel("Friction residual force (N)");
    title("Friction Force vs Velocity Overbound");
    legend("Location", "best");
    grid on;
    exportgraphics(fig, pngPath, "Resolution", 300);
    %#ok<NASGU> fits
end

function plotCase(v, c, color, name)
    f = zeros(size(v));
    pos = v >= 0;
    neg = ~pos;
    f(pos) = c.positive_direction.b_Ns_per_m .* v(pos) + c.positive_direction.mu_N;
    f(neg) = c.negative_direction.b_Ns_per_m .* v(neg) - c.negative_direction.mu_N;
    plot(v, f, "LineWidth", 2, "Color", color, "DisplayName", name);
end

function plotResidualVsPosition(samples, spatial, pngPath)
    fig = figure("Name", "friction_residual_vs_position", "Color", "w");
    scatter(samples.position_m, samples.spatial_residual_N, 7, samples.velocity_mps, "filled");
    hold on;
    yline(spatial.amplitude_abs_max_N, "--", "Color", [1 0.45 0], "LineWidth", 1.6, "DisplayName", "p95 abs bound");
    yline(-spatial.amplitude_abs_max_N, "--", "Color", [1 0.45 0], "LineWidth", 1.6, "HandleVisibility", "off");
    xlabel("Rail position (m)");
    ylabel("Residual after dynamic friction fit (N)");
    title("Spatial Friction Remainder vs Position");
    colorbar;
    grid on;
    exportgraphics(fig, pngPath, "Resolution", 300);
end

function plotSpatialBumpRealizations(spatial, realizations, pngPath)
    fig = figure("Name", "friction_spatial_bump_realizations", "Color", "w");
    hold on;
    nPlot = min(numel(realizations), 12);

    for i = 1:nPlot
        plot(realizations(i).x_grid_m, realizations(i).force_N, "LineWidth", 1.1);
    end

    yline(spatial.amplitude_abs_max_N, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.4);
    yline(-spatial.amplitude_abs_max_N, "--", "Color", [0.5 0.5 0.5], "LineWidth", 1.4);
    xlabel("Rail position (m)");
    ylabel("Spatial disturbance force (N)");
    title("Example Spatial Friction Bump Realizations");
    grid on;
    exportgraphics(fig, pngPath, "Resolution", 300);
end

function writeReport(path, overbound, fits, cases, stiction, spatial, summary, stictionTable, nRuns, samples, dataDir)
    fid = fopen(path, "w");
    if fid < 0
        error("Could not open report for writing:\n%s", path);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, "# Friction Disturbance Overbound Report\n\n");
    fprintf(fid, "**Generated:** %s  \n", datestr(now, "yyyy-mm-dd HH:MM:SS"));
    fprintf(fid, "**Dataset:** `%s`  \n", strrep(char(dataDir), "\", "/"));
    fprintf(fid, "**Runs loaded:** %d  \n", nRuns);
    fprintf(fid, "**Valid residual samples:** %d  \n", height(samples));
    fprintf(fid, "**Status:** provisional candidate-data V&V overbound\n\n");

    fprintf(fid, "This artifact is for controller robustness testing. It is not a finalized friction identification result.\n\n");

    fprintf(fid, "## Direction-Dependent Dynamic Friction\n\n");
    fprintf(fid, "| Direction | b (N*s/m) | mu (N) | RMSE (N) | n |\n");
    fprintf(fid, "|---|---:|---:|---:|---:|\n");
    fprintf(fid, "| positive velocity | %.4f | %.4f | %.4f | %d |\n", fits.positive.b_Ns_per_m, fits.positive.mu_N, fits.positive.rmse_N, fits.positive.n);
    fprintf(fid, "| negative velocity | %.4f | %.4f | %.4f | %d |\n", fits.negative.b_Ns_per_m, fits.negative.mu_N, fits.negative.rmse_N, fits.negative.n);
    fprintf(fid, "| pooled | %.4f | %.4f | %.4f | %d |\n\n", fits.pooled.b_Ns_per_m, fits.pooled.mu_N, fits.pooled.rmse_N, fits.pooled.n);

    fprintf(fid, "## Simulation Cases\n\n");
    fprintf(fid, "| Case | b+ | mu+ | b- | mu- |\n");
    fprintf(fid, "|---|---:|---:|---:|---:|\n");
    writeCaseRow(fid, "low", cases.low);
    writeCaseRow(fid, "nominal", cases.nominal);
    writeCaseRow(fid, "high", cases.high);
    writeCaseRow(fid, "asymmetric_worst", cases.asymmetric_worst);

    fprintf(fid, "\n## Stiction Bounds\n\n");
    fprintf(fid, "| Angle (deg) | lower breakaway (N) | upper breakaway (N) | lower ESC | upper ESC |\n");
    fprintf(fid, "|---:|---:|---:|---:|---:|\n");
    for i = 1:height(stictionTable)
        fprintf(fid, "| %.0f | %.4f | %.4f | %.0f | %.0f |\n", ...
            stictionTable.angle_deg(i), ...
            stictionTable.breakaway_lower_N(i), ...
            stictionTable.breakaway_upper_N(i), ...
            stictionTable.lower_esc_us(i), ...
            stictionTable.upper_esc_us(i));
    end
    fprintf(fid, "\nCollapsed stiction range for V&V:\n\n");
    fprintf(fid, "$$\n");
    fprintf(fid, "F_\\text{breakaway} \\in [%.3f,\\ %.3f]~\\text{N}\n", stiction.breakaway_low_N, stiction.breakaway_high_N);
    fprintf(fid, "$$\n\n");

    fprintf(fid, "## Spatial Disturbance Bounds\n\n");
    fprintf(fid, "- Position range: %.4f to %.4f m\n", spatial.position_range_m(1), spatial.position_range_m(2));
    fprintf(fid, "- Bump count range: %d to %d\n", spatial.bump_count_range(1), spatial.bump_count_range(2));
    fprintf(fid, "- Bump width range: %.4f to %.4f m\n", spatial.width_range_m(1), spatial.width_range_m(2));
    fprintf(fid, "- Bump amplitude bound: %.4f N (p%d of absolute residual remainder)\n\n", spatial.amplitude_abs_max_N, spatial.percentile_basis);

    fprintf(fid, "The exported JSON/MAT struct contains `dynamic`, `stiction`, and `spatial` fields for simulation.\n\n");
    fprintf(fid, "## Summary Table\n\n");
    fprintf(fid, "| Metric | Value |\n");
    fprintf(fid, "|---|---:|\n");
    for i = 1:height(summary)
        fprintf(fid, "| `%s` | %.6g |\n", summary.metric(i), summary.value(i));
    end

    %#ok<NASGU> overbound cleanup
end

function writeCaseRow(fid, name, c)
    fprintf(fid, "| `%s` | %.4f | %.4f | %.4f | %.4f |\n", ...
        name, ...
        c.positive_direction.b_Ns_per_m, ...
        c.positive_direction.mu_N, ...
        c.negative_direction.b_Ns_per_m, ...
        c.negative_direction.mu_N);
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
