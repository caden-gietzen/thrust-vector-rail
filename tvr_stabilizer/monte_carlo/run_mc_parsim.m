function results = run_mc_parsim(nCases, scenario, opts)
%RUN_MC_PARSIM Run and score Monte Carlo controller validation cases.
%   results = run_mc_parsim(20, "step") builds sampled truth plants, runs the
%   existing tvr_sim model, scores each response, and writes campaign
%   artifacts under data/processed, plots, and reports.

    if nargin < 1 || isempty(nCases)
        nCases = 20;
    end
    if nargin < 2 || isempty(scenario)
        scenario = "step";
    end
    if nargin < 3
        opts = struct();
    end

    opts = withDefault(opts, "UseParallel", false);
    opts = withDefault(opts, "ShowProgress", true);
    opts = withDefault(opts, "SaveOutputs", true);
    opts = withDefault(opts, "SaveFullResultsMat", opts.SaveOutputs);
    opts = withDefault(opts, "RngSeed", 1);
    opts = withDefault(opts, "WriteSummaryCsv", true);
    opts = withDefault(opts, "WriteReport", true);
    opts = withDefault(opts, "WriteSummaryFigure", true);

    thisDir = fileparts(mfilename("fullpath"));
    stabilizerDir = fileparts(thisDir);
    repoRoot = fileparts(stabilizerDir);
    addpath(stabilizerDir);
    addpath(thisDir);

    opts = localResolveOutputDirs(opts, repoRoot, scenario, nCases);
    ensureDir(opts.DataOutputDir);
    ensureDir(opts.PlotOutputDir);
    ensureDir(opts.ReportOutputDir);
    if opts.WriteSummaryCsv
        ensureDir(opts.DataOutputDir);
    end
    if opts.WriteSummaryFigure
        ensureDir(opts.PlotOutputDir);
    end
    if opts.WriteReport
        ensureDir(opts.ReportOutputDir);
    end

    [simInputs, cases, caseParams] = make_mc_cases(nCases, scenario, opts);

    fprintf("Running %d Monte Carlo cases for scenario '%s'...\n", nCases, string(scenario));
    runTimer = tic;
    try
        if opts.UseParallel
            simOut = parsim(simInputs, ...
                "UseParallel", true, ...
                "ShowProgress", opts.ShowProgress, ...
                "TransferBaseWorkspaceVariables", "on");
        else
            simOut = localRunSerialSims(simInputs, opts.ShowProgress);
        end
    catch ME
        warning("Monte Carlo simulation batch failed (%s). Falling back to serial sim loop.", ME.message);
        simOut = localRunSerialSims(simInputs, opts.ShowProgress);
    end
    fprintf("Monte Carlo simulations complete in %.1f s. Scoring runs...\n", toc(runTimer));

    scoreTimer = tic;
    emptyScore = score_mc_run([], cases(1, :), struct("ReturnEmpty", true));
    scoreRows = repmat(emptyScore, nCases, 1);
    detail = cell(nCases, 1);
    for i = 1:nCases
        try
            [scoreRows(i), detail{i}] = score_mc_run(simOut(i), cases(i, :));
        catch ME
            scoreRows(i) = score_mc_run([], cases(i, :), struct( ...
                "ErrorMessage", string(ME.message)));
            detail{i} = struct();
        end
    end
    scores = struct2table(scoreRows);
    fprintf("Scoring complete in %.1f s. Writing summary artifacts...\n", toc(scoreTimer));

    summaryTimer = tic;
    summary = summarize_mc_results(scores, cases, opts.DataOutputDir, ...
        string(scenario), opts);
    fprintf("Summary artifacts complete in %.1f s.\n", toc(summaryTimer));

    results = struct();
    results.cases = cases;
    results.scores = scores;
    results.summary = summary;
    results.simOut = simOut;
    results.detail = detail;
    results.caseParams = caseParams;
    results.campaignId = opts.CampaignId;
    results.dataOutputDir = opts.DataOutputDir;
    results.plotOutputDir = opts.PlotOutputDir;
    results.reportOutputDir = opts.ReportOutputDir;

    if ~opts.SaveFullResultsMat
        results.simOut = [];
        simOut = []; %#ok<NASGU>
    end

    if opts.SaveOutputs
        saveTimer = tic;
        writetable(cases, fullfile(opts.DataOutputDir, "cases.csv"));
        writetable(scores, fullfile(opts.DataOutputDir, "scores.csv"));
        if lower(string(scenario)) == "prps" && ~isempty(caseParams)
            localWritePrpsPlan(caseParams{1}.ref.prps, opts);
        end
        save(fullfile(opts.DataOutputDir, "results.mat"), "results", "-v7.3");
        localWriteManifest(opts, scenario, nCases);
        fprintf("Result files complete in %.1f s.\n", toc(saveTimer));
    end
end

function opts = withDefault(opts, name, value)
    if ~isfield(opts, name) || isempty(opts.(name))
        opts.(name) = value;
    end
end

function simOut = localRunSerialSims(simInputs, showProgress)
    nCases = numel(simInputs);
    simOut(nCases, 1) = Simulink.SimulationOutput;
    if showProgress
        fprintf("Running serial sim loop because UseParallel=false...\n");
    end
    progressEvery = max(1, floor(nCases / 20));
    for i = 1:nCases
        simOut(i) = sim(simInputs(i));
        if showProgress && (i == 1 || i == nCases || mod(i, progressEvery) == 0)
            fprintf("  Completed %d / %d simulations\n", i, nCases);
        end
    end
end

function opts = localResolveOutputDirs(opts, repoRoot, scenario, nCases)
    opts = withDefault(opts, "CampaignTag", "");
    opts = withDefault(opts, "CampaignId", makeCampaignId(scenario, nCases, opts));

    base = fullfile("controller_validation", "tvr_stabilizer", ...
        "monte_carlo", char(opts.CampaignId));
    opts = withDefault(opts, "DataOutputDir", ...
        fullfile(repoRoot, "data", "processed", base));
    opts = withDefault(opts, "PlotOutputDir", ...
        fullfile(repoRoot, "plots", base));
    opts = withDefault(opts, "ReportOutputDir", ...
        fullfile(repoRoot, "reports", base));
end

function id = makeCampaignId(scenario, nCases, opts)
    stamp = string(datetime("now", "Format", "yyyy_MM_dd_HHmm"));
    stopTime = "default";
    if isfield(opts, "StopTime") && ~isempty(opts.StopTime)
        stopTime = sprintf("%.15g", opts.StopTime);
        stopTime = regexprep(stopTime, "\.", "p");
    end
    tag = string(opts.CampaignTag);
    if strlength(tag) > 0
        tag = "_" + sanitizeToken(tag);
    end
    id = stamp + "_" + sanitizeToken(string(scenario)) + "_N" + ...
        string(nCases) + "_T" + stopTime + "s_seed" + ...
        string(opts.RngSeed) + tag;
end

function token = sanitizeToken(token)
    token = lower(string(token));
    token = regexprep(token, "[^a-z0-9]+", "_");
    token = regexprep(token, "^_+|_+$", "");
end

function ensureDir(pathText)
    if ~exist(pathText, "dir")
        mkdir(pathText);
    end
end

function localWriteManifest(opts, scenario, nCases)
    manifest = struct();
    manifest.campaign_id = char(opts.CampaignId);
    manifest.generated = char(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
    manifest.scenario = char(string(scenario));
    manifest.n_cases = nCases;
    manifest.stop_time_s = getFieldOr(opts, "StopTime", []);
    manifest.rng_seed = opts.RngSeed;
    manifest.initial_position_range_m = getFieldOr(opts, "InitialPositionRangeM", []);
    manifest.initial_velocity_range_mps = getFieldOr(opts, "InitialVelocityRangeMps", []);
    manifest.rail_length_m = getFieldOr(opts, "RailLengthM", 0.29);
    manifest.centered_rail_limit_m = 0.5 * manifest.rail_length_m;
    if lower(string(scenario)) == "prps"
        manifest.prps_plan_json = "prps_plan.json";
    end
    manifest.data_output_dir = char(string(opts.DataOutputDir));
    manifest.plot_output_dir = char(string(opts.PlotOutputDir));
    manifest.report_output_dir = char(string(opts.ReportOutputDir));
    text = jsonencode(manifest, "PrettyPrint", true);
    fid = fopen(fullfile(opts.DataOutputDir, "campaign_manifest.json"), "w");
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, "%s\n", text);
end

function localWritePrpsPlan(plan, opts)
    text = jsonencode(plan, "PrettyPrint", true);

    fid = fopen(fullfile(opts.DataOutputDir, "prps_plan.json"), "w");
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, "%s\n", text);

    if isfield(opts, "ReportOutputDir")
        ensureDir(opts.ReportOutputDir);
        fid2 = fopen(fullfile(opts.ReportOutputDir, "prps_plan.json"), "w");
        cleaner2 = onCleanup(@() fclose(fid2)); %#ok<NASGU>
        fprintf(fid2, "%s\n", text);
    end
end

function value = getFieldOr(s, name, fallback)
    if isfield(s, name)
        value = s.(name);
    else
        value = fallback;
    end
end
