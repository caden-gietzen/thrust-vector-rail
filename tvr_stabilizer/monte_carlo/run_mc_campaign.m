%% TVR stabilizer Monte Carlo runner
% Edit the toggles below, then press Run/F5. No MATLAB command-line typing
% needed. This script can either run a fresh Monte Carlo campaign or load an
% existing *_results.mat file and regenerate plots.

clearvars;
close all;

%% User Toggles

% What to run.
SCENARIO = "prps";           % "prps", "track", "step", or "hold"
N_CASES = 500;
STOP_TIME_S = [];            % [] uses scenario default; PRPS default is 120 s
RNG_SEED = 1;
CAMPAIGN_TAG = "cl_id_prps_0p05m";
CAMPAIGN_ID = "";            % leave blank to auto-generate from toggles

% Initial condition stress. For clean tracking validation, [0 0] is best.
INITIAL_POSITION_RANGE_M = [-0.02 0.02];
INITIAL_VELOCITY_RANGE_MPS = [0 0];

% Rail geometry. Position is modeled relative to rail center, so the usable
% safety limit is +/- RAIL_LENGTH_M/2.
RAIL_LENGTH_M = 0.29;

% Execution controls.
RUN_MONTE_CARLO = true;      % true: run Simulink/parsim now
LOAD_EXISTING_CAMPAIGN = false;
LOAD_EXISTING_CAMPAIGN_ID = "";
USE_PARALLEL = true;
SHOW_PROGRESS = true;

% Core result exports from run_mc_parsim.
SAVE_RESULTS_MAT_AND_CSV = true;
SAVE_FULL_RESULTS_MAT = false;  % false saves compact results without raw Simulink outputs
WRITE_REPORT = true;
WRITE_SUMMARY_CSV = true;
WRITE_SUMMARY_FIGURE = true;

% Trajectory plot export. For N=500, keep this off-screen.
EXPORT_TRAJECTORY_PLOT = true;
SHOW_TRAJECTORY_FIGURE = false;
MAX_SPAGHETTI_CASES = 50;
MAX_PLOT_POINTS = 2000;
SHOW_PERCENTILE_ENVELOPE = true;

% Optional debug replay after the run. Useful when the report identifies a
% bad case. Set REPLAY_CASE_INDEX = "worst" or a numeric case index.
LOAD_CASE_TO_WORKSPACE = false;
REPLAY_CASE_INDEX = "worst";
RUN_REPLAY_SIMULATION = false;

%% Paths

STABILIZER_DIR = "c:/dev/thrust-vector-rail/tvr_stabilizer";
REPO_ROOT = "c:/dev/thrust-vector-rail";
MC_DIR = STABILIZER_DIR + "/monte_carlo";
if strlength(CAMPAIGN_ID) == 0
    CAMPAIGN_ID = makeCampaignId(SCENARIO, N_CASES, STOP_TIME_S, RNG_SEED, CAMPAIGN_TAG);
end
if LOAD_EXISTING_CAMPAIGN && strlength(LOAD_EXISTING_CAMPAIGN_ID) > 0
    CAMPAIGN_ID = LOAD_EXISTING_CAMPAIGN_ID;
end

DATA_OUTPUT_DIR = REPO_ROOT + "/data/processed/controller_validation/tvr_stabilizer/monte_carlo/" + CAMPAIGN_ID;
PLOT_OUTPUT_DIR = REPO_ROOT + "/plots/controller_validation/tvr_stabilizer/monte_carlo/" + CAMPAIGN_ID;
REPORT_OUTPUT_DIR = REPO_ROOT + "/reports/controller_validation/tvr_stabilizer/monte_carlo/" + CAMPAIGN_ID;
RESULTS_FILE = DATA_OUTPUT_DIR + "/results.mat";
TRAJECTORY_PNG = PLOT_OUTPUT_DIR + "/trajectories.png";

cd(STABILIZER_DIR);
addpath(MC_DIR);

%% Run or Load

opts = struct();
opts.StopTime = STOP_TIME_S;
opts.InitialPositionRangeM = INITIAL_POSITION_RANGE_M;
opts.InitialVelocityRangeMps = INITIAL_VELOCITY_RANGE_MPS;
opts.RailLengthM = RAIL_LENGTH_M;
opts.UseParallel = USE_PARALLEL;
opts.ShowProgress = SHOW_PROGRESS;
opts.RngSeed = RNG_SEED;
opts.CampaignTag = CAMPAIGN_TAG;
opts.CampaignId = CAMPAIGN_ID;
opts.DataOutputDir = DATA_OUTPUT_DIR;
opts.PlotOutputDir = PLOT_OUTPUT_DIR;
opts.ReportOutputDir = REPORT_OUTPUT_DIR;
opts.SaveOutputs = SAVE_RESULTS_MAT_AND_CSV;
opts.SaveFullResultsMat = SAVE_FULL_RESULTS_MAT;
opts.WriteReport = WRITE_REPORT;
opts.WriteSummaryCsv = WRITE_SUMMARY_CSV;
opts.WriteSummaryFigure = WRITE_SUMMARY_FIGURE;

if RUN_MONTE_CARLO
    results = run_mc_parsim(N_CASES, SCENARIO, opts);
elseif LOAD_EXISTING_CAMPAIGN
    loaded = load(RESULTS_FILE, "results");
    results = loaded.results;
else
    error("Nothing to do. Enable RUN_MONTE_CARLO or LOAD_EXISTING_CAMPAIGN.");
end

%% Optional Trajectory Plot

if EXPORT_TRAJECTORY_PLOT || SHOW_TRAJECTORY_FIGURE
    fprintf("Generating trajectory plot...\n");
    plotOpts = struct();
    plotOpts.Visible = ternary(SHOW_TRAJECTORY_FIGURE, "on", "off");
    plotOpts.OutputPath = ternary(EXPORT_TRAJECTORY_PLOT, TRAJECTORY_PNG, "");
    plotOpts.MaxSpaghettiCases = MAX_SPAGHETTI_CASES;
    plotOpts.MaxPlotPoints = MAX_PLOT_POINTS;
    plotOpts.ShowEnvelope = SHOW_PERCENTILE_ENVELOPE;
    plot_mc_trajectories(results, plotOpts);
    fprintf("Trajectory plot complete.\n");
end

%% Optional Replay Setup

if LOAD_CASE_TO_WORKSPACE
    fprintf("Loading replay case '%s'...\n", string(REPLAY_CASE_INDEX));
    replayOpts = struct();
    replayOpts.RunSimulation = RUN_REPLAY_SIMULATION;
    replayOpts.OpenModel = true;
    load_mc_case_to_workspace(results, REPLAY_CASE_INDEX, replayOpts);
    fprintf("Replay setup complete.\n");
end

fprintf("\nMonte Carlo script complete.\n");
fprintf("Scenario: %s, N=%d\n", SCENARIO, N_CASES);
fprintf("Campaign: %s\n", CAMPAIGN_ID);
fprintf("Results file: %s\n", RESULTS_FILE);
if EXPORT_TRAJECTORY_PLOT
    fprintf("Trajectory plot: %s\n", TRAJECTORY_PNG);
end

function value = ternary(condition, trueValue, falseValue)
    if condition
        value = trueValue;
    else
        value = falseValue;
    end
end

function id = makeCampaignId(scenario, nCases, stopTimeS, rngSeed, tag)
    stamp = string(datetime("now", "Format", "yyyy_MM_dd_HHmm"));
    if isempty(stopTimeS)
        stopText = "default";
    else
        stopText = regexprep(sprintf("%.15g", stopTimeS), "\.", "p");
    end
    tag = sanitizeToken(tag);
    if strlength(tag) > 0
        tag = "_" + tag;
    end
    id = stamp + "_" + sanitizeToken(scenario) + "_N" + string(nCases) + ...
        "_T" + stopText + "s_seed" + string(rngSeed) + tag;
end

function token = sanitizeToken(token)
    token = lower(string(token));
    token = regexprep(token, "[^a-z0-9]+", "_");
    token = regexprep(token, "^_+|_+$", "");
end
