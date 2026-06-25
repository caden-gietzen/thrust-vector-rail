function fig = plot_mc_trajectories(resultsOrPath, opts)
%PLOT_MC_TRAJECTORIES Plot Monte Carlo trajectory overlays from saved results.
%   plot_mc_trajectories("campaign_id") overlays position,
%   tracking error, command angle, and saturation flags. Large ensembles are
%   shown as percentile envelopes plus a small spaghetti subset.

    if nargin < 1 || isempty(resultsOrPath)
        error("Pass a results struct, results.mat path, or campaign_id.");
    end
    if nargin < 2
        opts = struct();
    end
    opts = withDefault(opts, "CaseIndices", []);
    opts = withDefault(opts, "ShowReference", true);
    opts = withDefault(opts, "OutputPath", "");
    opts = withDefault(opts, "MaxSpaghettiCases", 50);
    opts = withDefault(opts, "MaxPlotPoints", 2000);
    opts = withDefault(opts, "ShowEnvelope", true);
    opts = withDefault(opts, "EnvelopePercentiles", [5 50 95]);
    opts = withDefault(opts, "RngSeed", 1);
    defaultVisible = "on";
    if strlength(string(opts.OutputPath)) > 0
        defaultVisible = "off";
    end
    opts = withDefault(opts, "Visible", defaultVisible);

    results = loadResults(resultsOrPath);
    caseMask = true(height(results.scores), 1);
    if ~isempty(opts.CaseIndices)
        caseMask = ismember(results.scores.case_index, opts.CaseIndices);
    end
    selectedRows = find(caseMask);
    if isempty(selectedRows)
        error("No Monte Carlo cases selected for plotting.");
    end
    rng(opts.RngSeed, "twister");
    spaghettiRows = selectedRows;
    if numel(spaghettiRows) > opts.MaxSpaghettiCases
        spaghettiRows = spaghettiRows(randperm(numel(spaghettiRows), opts.MaxSpaghettiCases));
    end

    fig = figure("Visible", opts.Visible, "Color", [1 1 1]);
    tiledlayout(fig, 4, 1, "TileSpacing", "compact");

    brightBlue = [0 0.55 1];
    brightOrange = [1 0.50 0];
    brightGreen = [0.1 0.75 0.25];
    brightPink = [1 0.15 0.65];
    failGray = [0.45 0.45 0.45];

    nexttile;
    hold on;
    plotEnvelope(results, selectedRows, opts, @(d) d.position_m, brightBlue, 1.0);
    for row = spaghettiRows(:)'
        detail = results.detail{row};
        if isempty(fieldnames(detail))
            continue;
        end
        [t, y] = decimateTrace(detail.t_s, detail.position_m, opts.MaxPlotPoints);
        color = passFailColor(results.scores.success(row), brightBlue, failGray);
        plot(t, y, "Color", [color 0.35], "LineWidth", 0.8);
    end
    if opts.ShowReference
        refDetail = results.detail{selectedRows(1)};
        [tRef, yRef] = decimateTrace(refDetail.t_s, refDetail.reference_m, opts.MaxPlotPoints);
        plot(tRef, yRef, "--", ...
            "Color", brightOrange, "LineWidth", 1.6);
    end
    ylabel("p [m]");
    title("Position Trajectories");
    grid on;

    nexttile;
    hold on;
    plotEnvelope(results, selectedRows, opts, @(d) 1000*d.error_m, brightGreen, 1.0);
    for row = spaghettiRows(:)'
        detail = results.detail{row};
        if isempty(fieldnames(detail))
            continue;
        end
        [t, y] = decimateTrace(detail.t_s, 1000*detail.error_m, opts.MaxPlotPoints);
        color = passFailColor(results.scores.success(row), brightGreen, failGray);
        plot(t, y, "Color", [color 0.35], "LineWidth", 0.8);
    end
    yline(2, "--", "Color", brightOrange);
    yline(-2, "--", "Color", brightOrange);
    ylabel("error [mm]");
    title("Tracking Error");
    grid on;

    nexttile;
    hold on;
    plotEnvelope(results, selectedRows, opts, @(d) rad2deg(d.theta_cmd_rad), brightPink, 1.0);
    for row = spaghettiRows(:)'
        detail = results.detail{row};
        if isempty(fieldnames(detail))
            continue;
        end
        [t, y] = decimateTrace(detail.t_s, rad2deg(detail.theta_cmd_rad), opts.MaxPlotPoints);
        color = passFailColor(results.scores.success(row), brightPink, failGray);
        plot(t, y, "Color", [color 0.35], "LineWidth", 0.8);
    end
    yline(rad2deg(asin(0.999)), "--", "Color", brightOrange);
    yline(-rad2deg(asin(0.999)), "--", "Color", brightOrange);
    ylabel("\theta cmd [deg]");
    title("Servo Command");
    grid on;

    nexttile;
    hold on;
    plotEnvelope(results, selectedRows, opts, @(d) double(d.saturation_flag), brightBlue, 0.25);
    for row = spaghettiRows(:)'
        detail = results.detail{row};
        if isempty(fieldnames(detail))
            continue;
        end
        [t, y] = decimateTrace(detail.t_s, double(detail.saturation_flag), opts.MaxPlotPoints);
        stairs(t, y, ...
            "Color", [passFailColor(results.scores.success(row), brightBlue, failGray) 0.35], ...
            "LineWidth", 0.8);
    end
    xlabel("time [s]");
    ylabel("sat");
    ylim([-0.05 1.05]);
    title("Saturation Flag");
    grid on;

    if strlength(string(opts.OutputPath)) > 0
        exportgraphics(fig, opts.OutputPath, "Resolution", 180);
        if string(opts.Visible) == "off"
            close(fig);
        end
    end
end

function opts = withDefault(opts, name, value)
    if ~isfield(opts, name) || isempty(opts.(name))
        opts.(name) = value;
    end
end

function results = loadResults(resultsOrPath)
    if ischar(resultsOrPath) || isstring(resultsOrPath)
        resultsPath = resolveResultsPath(resultsOrPath);
        loaded = load(resultsPath, "results");
        results = loaded.results;
    elseif isstruct(resultsOrPath) && isfield(resultsOrPath, "detail")
        results = resultsOrPath;
    else
        error("Input must be a results struct, results.mat path, or campaign_id.");
    end
end

function path = resolveResultsPath(inputText)
    path = string(inputText);
    if isfile(path)
        return;
    end
    repoRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
    candidate = fullfile(repoRoot, "data", "processed", ...
        "controller_validation", "tvr_stabilizer", "monte_carlo", ...
        char(path), "results.mat");
    if isfile(candidate)
        path = string(candidate);
        return;
    end
    error("Could not resolve Monte Carlo result input: %s", path);
end

function color = passFailColor(success, passColor, failColor)
    if success
        color = passColor;
    else
        color = failColor;
    end
end

function plotEnvelope(results, selectedRows, opts, valueFcn, color, alpha)
    if ~opts.ShowEnvelope || numel(selectedRows) < 2
        return;
    end
    ref = results.detail{selectedRows(1)};
    if isempty(fieldnames(ref))
        return;
    end
    [tPlot, ~] = decimateTrace(ref.t_s, ref.t_s, opts.MaxPlotPoints);
    y = nan(numel(tPlot), numel(selectedRows));
    for i = 1:numel(selectedRows)
        detail = results.detail{selectedRows(i)};
        if isempty(fieldnames(detail))
            continue;
        end
        raw = valueFcn(detail);
        y(:, i) = interp1(detail.t_s, raw(:), tPlot, "linear", "extrap");
    end
    pct = prctile(y, opts.EnvelopePercentiles, 2);
    lo = pct(:, 1);
    med = pct(:, 2);
    hi = pct(:, 3);
    fill([tPlot; flipud(tPlot)], [lo; flipud(hi)], color, ...
        "FaceAlpha", alpha, "EdgeColor", "none");
    plot(tPlot, med, "Color", color, "LineWidth", 1.4);
end

function [tOut, yOut] = decimateTrace(t, y, maxPoints)
    t = t(:);
    y = y(:);
    if numel(t) <= maxPoints
        tOut = t;
        yOut = y;
        return;
    end
    idx = unique(round(linspace(1, numel(t), maxPoints)));
    tOut = t(idx);
    yOut = y(idx);
end
