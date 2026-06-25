function [score, detail] = score_mc_run(simOut, caseRow, opts)
%SCORE_MC_RUN Compute controller validation metrics from one Simulink run.

    if nargin < 3
        opts = struct();
    end
    if isfield(opts, "ReturnEmpty") && opts.ReturnEmpty
        score = localEmptyScore();
        detail = struct();
        return;
    end

    score = localEmptyScore();
    detail = struct();
    score.case_index = caseRow.case_index;
    score.scenario = string(caseRow.scenario);

    if isfield(opts, "ErrorMessage")
        score.sim_ok = false;
        score.error_message = string(opts.ErrorMessage);
        return;
    end

    errMsg = "";
    try
        if ~isempty(simOut.ErrorMessage)
            errMsg = string(simOut.ErrorMessage);
        end
    catch
    end
    if strlength(errMsg) > 0
        score.sim_ok = false;
        score.error_message = errMsg;
        return;
    end

    logs = simOut.logsout;
    [t, pReal] = localSignal(logs, ["p_real [m]", "p_real", "Actual Position"]);
    [~, vReal] = localSignal(logs, ["v_real [m/s]", "v_real", "Actual Velocity"], t);
    [~, pRef] = localSignal(logs, ["p_ref", "r", "Reference Position"], t);
    [~, vRef] = localSignal(logs, ["v_ref", "rdot", "Reference Velocity"], t);
    [~, thetaCmdRaw] = localSignal(logs, ["ang_cmd", "theta_cmd"], t);
    [thetaCmd, saturationFlag] = localAngleCommand(thetaCmdRaw, caseRow.theta_clip_rad);

    e = pReal - pRef;
    ev = vReal - vRef;
    dt = max(t(end) - t(1), eps);

    saturationDuration = trapz(t, double(saturationFlag));

    score.sim_ok = true;
    score.rmse_tracking_m = sqrt(mean(e.^2, "omitnan"));
    score.max_abs_error_m = max(abs(e), [], "omitnan");
    score.iae_m_s = trapz(t, abs(e));
    score.final_error_m = e(end);
    score.final_velocity_error_mps = ev(end);
    score.theta_cmd_rms_rad = sqrt(mean(thetaCmd.^2, "omitnan"));
    score.theta_cmd_max_abs_rad = max(abs(thetaCmd), [], "omitnan");
    score.control_effort_l2_rad2_s = trapz(t, thetaCmd.^2);
    score.saturation_fraction = saturationDuration / dt;
    score.saturation_duration_s = saturationDuration;
    score.saturation_count = localCountRisingEdges(saturationFlag);
    railLimitM = localRailLimit(caseRow);
    score.rail_limit_m = railLimitM;
    score.max_abs_position_m = max(abs(pReal), [], "omitnan");
    score.rail_margin_min_m = railLimitM - score.max_abs_position_m;
    score.rail_violation = any(abs(pReal) > railLimitM);
    score.unstable = any(~isfinite(pReal)) || any(abs(pReal) > 1.0) || ...
        any(~isfinite(thetaCmd));

    [score.settled, score.settling_time_s] = localSettling(t, e, ev, ...
        0.002, 0.01);
    score.overshoot_m = localOvershoot(pReal, pRef);
    score.oscillation_count = localOscillationCount(e, 0.002);
    if score.scenario == "prps"
        score = localPrpsIdMetrics(score, t, pRef, pReal, caseRow);
    end

    isTrack = score.scenario == "track";
    isPrps = score.scenario == "prps";
    if isPrps
        trackingPass = score.rmse_tracking_m <= 0.025 && ...
            score.max_abs_error_m <= 0.080;
        settlePass = true;
    elseif isTrack
        trackingPass = score.rmse_tracking_m <= 0.020;
        settlePass = true;
    else
        trackingPass = score.max_abs_error_m <= 0.12;
        settlePass = score.settled;
    end
    score.success = score.sim_ok && ~score.unstable && ~score.rail_violation && ...
        trackingPass && settlePass && score.saturation_fraction <= 0.20;

    detail.t_s = t;
    detail.position_m = pReal;
    detail.reference_m = pRef;
    detail.error_m = e;
    detail.theta_cmd_rad = thetaCmd;
    detail.saturation_flag = saturationFlag;
end

function [thetaCmdRad, saturationFlag] = localAngleCommand(thetaCmdRaw, thetaClipRad)
    thetaCmdRaw = thetaCmdRaw(:);
    if max(abs(thetaCmdRaw), [], "omitnan") > 2*pi
        thetaCmdRad = deg2rad(thetaCmdRaw);
        thetaClipLogged = rad2deg(thetaClipRad);
        saturationFlag = abs(thetaCmdRaw) >= (thetaClipLogged - 0.05);
    else
        thetaCmdRad = thetaCmdRaw;
        saturationFlag = abs(thetaCmdRaw) >= (thetaClipRad - 1e-3);
    end
end

function score = localEmptyScore()
    score = struct( ...
        "case_index", NaN, ...
        "scenario", "", ...
        "sim_ok", false, ...
        "success", false, ...
        "settled", false, ...
        "settling_time_s", NaN, ...
        "rmse_tracking_m", NaN, ...
        "max_abs_error_m", NaN, ...
        "iae_m_s", NaN, ...
        "final_error_m", NaN, ...
        "final_velocity_error_mps", NaN, ...
        "overshoot_m", NaN, ...
        "oscillation_count", NaN, ...
        "theta_cmd_rms_rad", NaN, ...
        "theta_cmd_max_abs_rad", NaN, ...
        "control_effort_l2_rad2_s", NaN, ...
        "saturation_fraction", NaN, ...
        "saturation_duration_s", NaN, ...
        "saturation_count", NaN, ...
        "rail_limit_m", NaN, ...
        "max_abs_position_m", NaN, ...
        "rail_margin_min_m", NaN, ...
        "rail_violation", false, ...
        "unstable", false, ...
        "prps_id_start_s", NaN, ...
        "prps_id_end_s", NaN, ...
        "prps_ref_excited_bin_min_rms_m", NaN, ...
        "prps_pos_excited_bin_min_rms_m", NaN, ...
        "prps_pos_excited_bin_median_rms_m", NaN, ...
        "prps_response_to_ref_min_ratio", NaN, ...
        "prps_response_to_ref_median_ratio", NaN, ...
        "error_message", "");
end

function score = localPrpsIdMetrics(score, t, pRef, pReal, caseRow)
    if any(strcmp("prps_id_start_s", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.prps_id_start_s)
        idStart = caseRow.prps_id_start_s;
    else
        idStart = 40.0;
    end
    if any(strcmp("prps_id_end_s", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.prps_id_end_s)
        idEnd = caseRow.prps_id_end_s;
    else
        idEnd = t(end);
    end
    if any(strcmp("prps_amp_m", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.prps_amp_m)
        amp = caseRow.prps_amp_m;
    else
        amp = 0.05;
    end
    if any(strcmp("prps_seed", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.prps_seed)
        seed = caseRow.prps_seed;
    else
        seed = 5001;
    end

    [~, ~, ~, plan] = make_prps_reference([], amp, seed);
    keep = t >= idStart & t < idEnd;
    if nnz(keep) < 8
        return;
    end

    ti = t(keep);
    ref = pRef(keep);
    pos = pReal(keep);
    dt = median(diff(ti), "omitnan");
    if ~isfinite(dt) || dt <= 0
        return;
    end
    fs = 1 / dt;
    n = numel(ti);
    ref = ref - mean(ref, "omitnan");
    pos = pos - mean(pos, "omitnan");
    refFft = fft(ref);
    posFft = fft(pos);
    binRmsRef = nan(size(plan.snapped_freqs_hz));
    binRmsPos = nan(size(plan.snapped_freqs_hz));
    for i = 1:numel(plan.snapped_freqs_hz)
        k = round(plan.snapped_freqs_hz(i) * n / fs) + 1;
        if k >= 1 && k <= numel(refFft)
            binRmsRef(i) = sqrt(2) * abs(refFft(k)) / n;
            binRmsPos(i) = sqrt(2) * abs(posFft(k)) / n;
        end
    end
    ratio = binRmsPos ./ max(binRmsRef, eps);

    score.prps_id_start_s = idStart;
    score.prps_id_end_s = idEnd;
    score.prps_ref_excited_bin_min_rms_m = min(binRmsRef, [], "omitnan");
    score.prps_pos_excited_bin_min_rms_m = min(binRmsPos, [], "omitnan");
    score.prps_pos_excited_bin_median_rms_m = median(binRmsPos, "omitnan");
    score.prps_response_to_ref_min_ratio = min(ratio, [], "omitnan");
    score.prps_response_to_ref_median_ratio = median(ratio, "omitnan");
end

function [t, y] = localSignal(logs, names, tTarget)
    if nargin < 3
        tTarget = [];
    end
    elt = [];
    for name = names
        try
            elt = logs.get(char(name));
            if ~isempty(elt)
                break;
            end
        catch
        end
    end
    if isempty(elt)
        error("Required logged signal missing. Tried: %s", strjoin(string(names), ", "));
    end
    values = elt.Values;
    t = values.Time(:);
    y = squeeze(values.Data);
    y = y(:);
    if ~isempty(tTarget) && (numel(t) ~= numel(tTarget) || any(abs(t - tTarget) > 1e-10))
        y = interp1(t, y, tTarget, "linear", "extrap");
        t = tTarget;
    end
end

function [settled, settlingTime] = localSettling(t, e, ev, posTol, velTol)
    settled = false;
    settlingTime = NaN;
    startIdx = find(t >= 0.20 * t(end), 1, "first");
    if isempty(startIdx)
        startIdx = 1;
    end
    within = abs(e) <= posTol & abs(ev) <= velTol;
    for k = startIdx:numel(t)
        if all(within(k:end))
            settled = true;
            settlingTime = t(k);
            return;
        end
    end
end

function overshoot = localOvershoot(pReal, pRef)
    target = pRef(end);
    start = pRef(1);
    direction = sign(target - start);
    if direction == 0
        overshoot = max(abs(pReal - target));
    else
        overshoot = max(direction * (pReal - target));
        overshoot = max(overshoot, 0);
    end
end

function n = localOscillationCount(e, deadband)
    e = e(:);
    active = abs(e) > deadband;
    signs = sign(e(active));
    if isempty(signs)
        n = 0;
    else
        n = sum(abs(diff(signs)) > 0);
    end
end

function n = localCountRisingEdges(flag)
    flag = logical(flag(:));
    n = sum(diff([false; flag]) == 1);
end

function railLimitM = localRailLimit(caseRow)
    railLimitM = 0.145;
    if any(strcmp("rail_limit_m", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.rail_limit_m)
        railLimitM = caseRow.rail_limit_m;
    elseif any(strcmp("rail_length_m", caseRow.Properties.VariableNames)) && ...
            isfinite(caseRow.rail_length_m)
        railLimitM = 0.5 * caseRow.rail_length_m;
    end
end
