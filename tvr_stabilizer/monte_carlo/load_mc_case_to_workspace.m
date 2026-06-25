function [p, caseRow, simOut] = load_mc_case_to_workspace(resultsOrCasePath, caseIndex, opts)
%LOAD_MC_CASE_TO_WORKSPACE Replay one Monte Carlo case in the Simulink model.
%   [p, caseRow] = load_mc_case_to_workspace("campaign_id", 1)
%   reconstructs the sampled truth parameters for case 1, assigns p in the
%   base workspace, and leaves tvr_sim ready to run from Simulink.
%
%   Use caseIndex = "worst" to replay the worst case selected by
%   summarize_mc_results.m.

    if nargin < 1 || isempty(resultsOrCasePath)
        error("Pass a results struct, results.mat path, cases.csv path, or campaign_id.");
    end
    if nargin < 2 || isempty(caseIndex)
        caseIndex = "worst";
    end
    if nargin < 3
        opts = struct();
    end
    opts = withDefault(opts, "ModelName", "tvr_sim");
    opts = withDefault(opts, "RunSimulation", false);
    opts = withDefault(opts, "AssignBase", true);
    opts = withDefault(opts, "OpenModel", true);

    thisDir = fileparts(mfilename("fullpath"));
    stabilizerDir = fileparts(thisDir);
    addpath(stabilizerDir);
    addpath(thisDir);

    [cases, scores, summary, caseParams] = loadCaseData(resultsOrCasePath);
    selectedCaseIndex = resolveCaseIndex(caseIndex, scores, summary);
    rowIdx = find(cases.case_index == selectedCaseIndex, 1, "first");
    if isempty(rowIdx)
        error("Case index %g was not found in the loaded case table.", selectedCaseIndex);
    end
    caseRow = cases(rowIdx, :);

    if ~isempty(caseParams)
        p = caseParams{rowIdx};
    else
        p = params();
        p = applyScenario(p, string(caseRow.scenario));
        p.ic.p = caseRow.initial_position_m;
        p.ic.v = caseRow.initial_velocity_mps;
        p.seed.friction = caseRow.friction_seed;
        p.seed.meas = caseRow.measurement_seed;
        p.truth.T = caseRow.truth_thrust_N;
        p.truth.k_theta = caseRow.truth_k_theta_rad_per_us;
        p.truth.tau_theta = caseRow.truth_tau_theta_s;
        p.truth.L_theta = caseRow.truth_delay_theta_s;
        if any(strcmp("truth_friction_d_max_mps2", caseRow.Properties.VariableNames))
            p.truth.d_max = caseRow.truth_friction_d_max_mps2;
        end
        if any(strcmp("truth_friction_asym", caseRow.Properties.VariableNames))
            p.truth.asym = caseRow.truth_friction_asym;
        end
    end

    p.mc.scenario = char(string(caseRow.scenario));
    p.mc.stop_time_s = caseRow.stop_time_s;
    p.mc.theta_clip_rad = caseRow.theta_clip_rad;
    if any(strcmp("rail_length_m", caseRow.Properties.VariableNames))
        p.mc.rail_length_m = caseRow.rail_length_m;
    else
        p.mc.rail_length_m = 0.29;
    end
    if any(strcmp("rail_limit_m", caseRow.Properties.VariableNames))
        p.mc.rail_limit_m = caseRow.rail_limit_m;
    else
        p.mc.rail_limit_m = 0.5 * p.mc.rail_length_m;
    end
    p.mc.case_index = selectedCaseIndex;
    p.mc.case_source = localCaseSource(resultsOrCasePath);

    if opts.AssignBase
        assignin("base", "p", p);
        fprintf("Loaded Monte Carlo case %g into base workspace variable p.\n", selectedCaseIndex);
    end

    if opts.OpenModel
        load_system(opts.ModelName);
        set_param(opts.ModelName, "StopTime", sprintf("%.12g", caseRow.stop_time_s));
        open_system(opts.ModelName);
    end

    simOut = [];
    if opts.RunSimulation
        in = Simulink.SimulationInput(opts.ModelName);
        in = in.setVariable("p", p);
        in = in.setModelParameter("StopTime", sprintf("%.12g", caseRow.stop_time_s));
        in = in.setModelParameter("ReturnWorkspaceOutputs", "on");
        in = in.setModelParameter("SignalLogging", "on");
        simOut = sim(in);
    end
end

function opts = withDefault(opts, name, value)
    if ~isfield(opts, name) || isempty(opts.(name))
        opts.(name) = value;
    end
end

function [cases, scores, summary, caseParams] = loadCaseData(resultsOrCasePath)
    scores = table();
    summary = struct();
    caseParams = {};
    if isstruct(resultsOrCasePath)
        if isfield(resultsOrCasePath, "cases")
            cases = resultsOrCasePath.cases;
        else
            error("Results struct must contain a cases table.");
        end
        if isfield(resultsOrCasePath, "scores")
            scores = resultsOrCasePath.scores;
        end
        if isfield(resultsOrCasePath, "summary")
            summary = resultsOrCasePath.summary;
        end
        if isfield(resultsOrCasePath, "caseParams")
            caseParams = resultsOrCasePath.caseParams;
        end
        return;
    end

    path = resolveResultsPath(resultsOrCasePath);
    if endsWith(path, "_results.mat")
        loaded = load(path, "results");
        cases = loaded.results.cases;
        scores = loaded.results.scores;
        summary = loaded.results.summary;
        if isfield(loaded.results, "caseParams")
            caseParams = loaded.results.caseParams;
        end
    elseif endsWith(path, "results.mat")
        loaded = load(path, "results");
        cases = loaded.results.cases;
        scores = loaded.results.scores;
        summary = loaded.results.summary;
        if isfield(loaded.results, "caseParams")
            caseParams = loaded.results.caseParams;
        end
    elseif endsWith(path, "_cases.csv") || endsWith(path, "cases.csv")
        cases = readtable(path, "TextType", "string", "Delimiter", ",");
    else
        error("Input must be a results struct, results MAT path, cases CSV path, or campaign_id.");
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

function idx = resolveCaseIndex(caseIndex, scores, summary)
    if ischar(caseIndex) || isstring(caseIndex)
        label = lower(string(caseIndex));
        switch label
            case "worst"
                if isfield(summary, "worst_case_index") && isfinite(summary.worst_case_index)
                    idx = summary.worst_case_index;
                    return;
                end
                if ~isempty(scores)
                    penalty = scores.rmse_tracking_m + 0.25*scores.saturation_fraction + ...
                        10*double(scores.rail_violation) + 10*double(scores.unstable);
                    [~, row] = max(penalty);
                    idx = scores.case_index(row);
                    return;
                end
                error("Cannot resolve 'worst' without scores or summary data.");
            otherwise
                idx = str2double(label);
                if ~isfinite(idx)
                    error("caseIndex must be numeric or 'worst'.");
                end
        end
    else
        idx = caseIndex;
    end
end

function p = applyScenario(p, scenario)
    switch lower(string(scenario))
        case "step"
            p.ref.type = 4;
            p.ref.amp = 0.10;
            p.ref.freq = 4.0;
        case "track"
            p.ref.type = 2;
            p.ref.amp = 0.08;
            p.ref.freq = 2*pi*0.35;
        case "hold"
            p.ref.type = 1;
            p.ref.amp = 0.05;
            p.ref.freq = 0.0;
        case "prps"
            p.ref.type = 5;
            p.ref.amp = 0.05;
            p.ref.freq = 5001;
            [~, ~, ~, p.ref.prps] = make_prps_reference([], p.ref.amp, p.ref.freq);
        otherwise
            warning("Unknown scenario '%s'; leaving params.m reference settings unchanged.", scenario);
    end
end

function source = localCaseSource(resultsOrCasePath)
    if isstruct(resultsOrCasePath)
        if isfield(resultsOrCasePath, "campaignId")
            source = char(string(resultsOrCasePath.campaignId));
        else
            source = "results_struct";
        end
    else
        source = char(string(resultsOrCasePath));
    end
end
