function [simInputs, cases, caseParams] = make_mc_cases(nCases, scenario, opts)
%MAKE_MC_CASES Build Simulink.SimulationInput objects for controller Monte Carlo.
%   The controller design parameters p.des.* stay fixed. Only p.truth.*,
%   seeds, initial conditions, and the selected reference scenario vary.

    if nargin < 1 || isempty(nCases)
        nCases = 20;
    end
    if nargin < 2 || isempty(scenario)
        scenario = "step";
    end
    if nargin < 3
        opts = struct();
    end

    opts = withDefault(opts, "ModelName", "tvr_sim");
    opts = withDefault(opts, "RngSeed", 1);
    opts = withDefault(opts, "UseArtifacts", true);
    opts = withDefault(opts, "StopTime", []);
    opts = withDefault(opts, "InitialPositionRangeM", [-0.02, 0.02]);
    opts = withDefault(opts, "InitialVelocityRangeMps", [-0.02, 0.02]);
    opts = withDefault(opts, "ThetaClipRad", asin(0.999));
    opts = withDefault(opts, "RailLengthM", 0.29);

    thisDir = fileparts(mfilename("fullpath"));
    stabilizerDir = fileparts(thisDir);
    repoRoot = fileparts(stabilizerDir);
    addpath(stabilizerDir);
    addpath(thisDir);

    p0 = params();
    scenarioSpec = localScenarioSpec(scenario);
    if ~isempty(opts.StopTime)
        scenarioSpec.stopTime = opts.StopTime;
    end

    artifacts = localLoadArtifacts(repoRoot, opts.UseArtifacts);
    frictionOverbound = artifacts.frictionOverbound;

    rng(opts.RngSeed, "twister");

    simInputs(nCases, 1) = Simulink.SimulationInput(opts.ModelName);
    caseRows = repmat(localEmptyCaseRow(), nCases, 1);
    caseParams = cell(nCases, 1);

    for i = 1:nCases
        p = p0;
        p.ref.type = scenarioSpec.refType;
        p.ref.amp = scenarioSpec.refAmp;
        p.ref.freq = scenarioSpec.refFreq;
        if isfield(scenarioSpec, "prps")
            p.ref.prps = scenarioSpec.prps;
        end
        p.seed.friction = 20000 + i;
        p.seed.meas = 22000 + i;
        p.ic.p = uniformSample(opts.InitialPositionRangeM);
        p.ic.v = uniformSample(opts.InitialVelocityRangeMps);

        servo = localSampleServo(artifacts.servoSamples, p0);
        thrust = localSampleThrust(artifacts.thrustSamples, p0);
        p.truth.k_theta = servo.k_theta;
        p.truth.tau_theta = servo.tau_theta;
        p.truth.L_theta = servo.L_theta;
        p.truth.T = thrust.T_N;

        friction = sample_friction_realization(frictionOverbound, p.truth.m);
        p.truth.friction = friction;
        p.truth.d_max = friction.d_max_mps2;
        p.truth.asym = friction.asym;

        p.mc.scenario = char(scenarioSpec.name);
        p.mc.stop_time_s = scenarioSpec.stopTime;
        p.mc.theta_clip_rad = opts.ThetaClipRad;
        p.mc.position_tolerance_m = 0.002;
        p.mc.velocity_tolerance_mps = 0.01;
        p.mc.rail_length_m = opts.RailLengthM;
        p.mc.rail_limit_m = 0.5 * opts.RailLengthM;
        p.mc.track_rmse_success_m = 0.020;
        p.mc.prps_rmse_success_m = 0.025;
        p.mc.prps_max_abs_error_success_m = 0.080;
        p.mc.step_max_abs_error_success_m = 0.12;
        p.mc.max_saturation_fraction = 0.20;

        in = Simulink.SimulationInput(opts.ModelName);
        in = in.setVariable("p", p);
        in = in.setModelParameter("StopTime", sprintf("%.12g", scenarioSpec.stopTime));
        in = in.setModelParameter("ReturnWorkspaceOutputs", "on");
        in = in.setModelParameter("SignalLogging", "on");
        simInputs(i) = in;

        caseRows(i).case_index = i;
        caseRows(i).scenario = string(scenarioSpec.name);
        caseRows(i).stop_time_s = scenarioSpec.stopTime;
        caseRows(i).rng_seed = opts.RngSeed;
        caseRows(i).friction_seed = p.seed.friction;
        caseRows(i).measurement_seed = p.seed.meas;
        caseRows(i).initial_position_m = p.ic.p;
        caseRows(i).initial_velocity_mps = p.ic.v;
        caseRows(i).truth_thrust_N = p.truth.T;
        caseRows(i).truth_k_theta_rad_per_us = p.truth.k_theta;
        caseRows(i).truth_tau_theta_s = p.truth.tau_theta;
        caseRows(i).truth_delay_theta_s = p.truth.L_theta;
        caseRows(i).friction_case = string(friction.case_name);
        caseRows(i).friction_spatial_force_N = friction.spatial_force_N;
        caseRows(i).theta_clip_rad = opts.ThetaClipRad;
        caseRows(i).rail_length_m = p.mc.rail_length_m;
        caseRows(i).rail_limit_m = p.mc.rail_limit_m;
        if isfield(p.ref, "prps")
            caseRows(i).prps_seed = p.ref.prps.seed;
            caseRows(i).prps_amp_m = p.ref.prps.amp_m;
            caseRows(i).prps_period_s = p.ref.prps.period_s;
            caseRows(i).prps_num_periods = p.ref.prps.num_periods;
            caseRows(i).prps_update_dt_s = p.ref.prps.update_dt_s;
            caseRows(i).prps_freq_min_hz = min(p.ref.prps.snapped_freqs_hz);
            caseRows(i).prps_freq_max_hz = max(p.ref.prps.snapped_freqs_hz);
            caseRows(i).prps_num_freqs = numel(p.ref.prps.snapped_freqs_hz);
            caseRows(i).prps_peak_raw = p.ref.prps.peak_raw;
            caseRows(i).prps_id_start_s = p.ref.prps.id_start_s;
            caseRows(i).prps_id_end_s = p.ref.prps.id_end_s;
        end
        caseParams{i} = p;
    end

    cases = struct2table(caseRows);
end

function opts = withDefault(opts, name, value)
    if ~isfield(opts, name) || isempty(opts.(name))
        opts.(name) = value;
    end
end

function x = uniformSample(bounds)
    x = bounds(1) + rand() * (bounds(2) - bounds(1));
end

function row = localEmptyCaseRow()
    row = struct( ...
        "case_index", NaN, ...
        "scenario", "", ...
        "stop_time_s", NaN, ...
        "rng_seed", NaN, ...
        "friction_seed", NaN, ...
        "measurement_seed", NaN, ...
        "initial_position_m", NaN, ...
        "initial_velocity_mps", NaN, ...
        "truth_thrust_N", NaN, ...
        "truth_k_theta_rad_per_us", NaN, ...
        "truth_tau_theta_s", NaN, ...
        "truth_delay_theta_s", NaN, ...
        "friction_case", "", ...
        "friction_spatial_force_N", NaN, ...
        "theta_clip_rad", NaN, ...
        "rail_length_m", NaN, ...
        "rail_limit_m", NaN, ...
        "prps_seed", NaN, ...
        "prps_amp_m", NaN, ...
        "prps_period_s", NaN, ...
        "prps_num_periods", NaN, ...
        "prps_update_dt_s", NaN, ...
        "prps_freq_min_hz", NaN, ...
        "prps_freq_max_hz", NaN, ...
        "prps_num_freqs", NaN, ...
        "prps_peak_raw", NaN, ...
        "prps_id_start_s", NaN, ...
        "prps_id_end_s", NaN);
end

function spec = localScenarioSpec(scenario)
    name = lower(string(scenario));
    switch name
        case "step"
            spec.name = "step";
            spec.refType = 4;
            spec.refAmp = 0.10;
            spec.refFreq = 4.0;
            spec.stopTime = 5.0;
        case {"track", "sine"}
            spec.name = "track";
            spec.refType = 2;
            spec.refAmp = 0.08;
            spec.refFreq = 2*pi*0.35;
            spec.stopTime = 8.0;
        case "hold"
            spec.name = "hold";
            spec.refType = 1;
            spec.refAmp = 0.05;
            spec.refFreq = 0.0;
            spec.stopTime = 5.0;
        case "prps"
            spec.name = "prps";
            spec.refType = 5;
            spec.refAmp = 0.05;
            spec.refFreq = 5001;
            [~, ~, ~, spec.prps] = make_prps_reference([], spec.refAmp, spec.refFreq);
            spec.stopTime = spec.prps.total_duration_s;
        otherwise
            error("Unknown Monte Carlo scenario '%s'. Use step, track, prps, or hold.", scenario);
    end
end

function artifacts = localLoadArtifacts(repoRoot, useArtifacts)
    artifacts.servoSamples = table();
    artifacts.thrustSamples = table();
    artifacts.frictionOverbound = struct();
    if ~useArtifacts
        return;
    end

    servoPath = fullfile(repoRoot, "plots", "system_identification", ...
        "servo_identification", "servo_prps_log", "bootstrap_fopd", ...
        "bootstrap_parameter_samples.csv");
    if isfile(servoPath)
        servo = readtable(servoPath, "TextType", "string", ...
            "Delimiter", ",", "VariableNamingRule", "preserve");
        if any(strcmp("fit_status", servo.Properties.VariableNames))
            servo = servo(servo.fit_status == "ok", :);
        end
        artifacts.servoSamples = servo;
    end

    thrustPath = fullfile(repoRoot, "plots", "system_identification", ...
        "thrust_identification", "thrust_prps_daq_voltage", ...
        "bootstrap_fixed_pwm_1825", "fixed_pwm_bootstrap_samples.csv");
    if isfile(thrustPath)
        artifacts.thrustSamples = readtable(thrustPath, "TextType", "string", ...
            "Delimiter", ",", "VariableNamingRule", "preserve");
    end

    frictionPath = fullfile(repoRoot, "plots", "system_identification", ...
        "friction_identification", "friction_sweep_log", ...
        "friction_disturbance_overbound", "friction_overbound_params.json");
    if isfile(frictionPath)
        artifacts.frictionOverbound = jsondecode(fileread(frictionPath));
    end
end

function servo = localSampleServo(samples, p0)
    if isempty(samples)
        servo = struct("k_theta", p0.truth.k_theta, ...
            "tau_theta", p0.truth.tau_theta, ...
            "L_theta", p0.truth.L_theta);
        return;
    end
    idx = randi(height(samples));
    servo = struct("k_theta", samples.K_rad_per_us(idx), ...
        "tau_theta", samples.tau_s(idx), ...
        "L_theta", samples.delay_s(idx));
end

function thrust = localSampleThrust(samples, p0)
    if isempty(samples)
        thrust = struct("T_N", p0.truth.T);
        return;
    end
    idx = randi(height(samples));
    thrust = struct("T_N", samples.mean_thrust_N(idx));
end
