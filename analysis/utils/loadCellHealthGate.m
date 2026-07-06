function health = loadCellHealthGate(csvPath, thresholds)
%LOADCELLHEALTHGATE Gate a thrust run on its weightless load-cell health probe.
%
%   health = loadCellHealthGate(csvPath) reads a per-run health sidecar CSV
%   written by firmware/pico_micropython/lib/load_cell_health.py (columns
%   t_ms,sample_idx,ok,raw_count -- one row per read ATTEMPT), recomputes the
%   authoritative instrument-health statistics, compares them against
%   thresholds, and prints a PASS/FAIL report. The firmware's on-bench summary
%   is advisory only; this function is the authority.
%
%   health = loadCellHealthGate(csvPath, thresholds) overrides the default
%   thresholds. thresholds is a struct with any of these fields:
%       maxDropoutRate  fraction of failed reads allowed        (default 0.00)
%       maxNoiseStd     max zero-load std, counts               (default Inf)
%       maxDriftCps     max |least-squares drift|, counts/s     (default Inf)
%       maxPtp          max zero-load peak-to-peak, counts      (default Inf)
%
%   IMPORTANT: the numeric thresholds default to Inf (report-only) on purpose.
%   The real limits are set from the load_cell_characterization baseline study
%   (docs) -- populate them there once, then pass the same struct here. Only the
%   dropout gate is armed by default, because ANY dropout is a connection fault.
%
%   Returned struct fields:
%       stats   .n .dropouts .attempts .dropoutRate
%               .mean .std .ptp .driftCps .seconds
%       pass    logical overall verdict
%       reasons string array of failed-check descriptions (empty if pass)
%       file    the resolved CSV path
%
%   Example:
%       th = struct('maxNoiseStd', 40, 'maxDriftCps', 5, 'maxPtp', 300);
%       h  = loadCellHealthGate("2026_07_05_thrust_..._load_cell_health_00.csv", th);
%       if ~h.pass, warning("Rejecting run: %s", strjoin(h.reasons, "; ")); end

    if nargin < 2 || isempty(thresholds)
        thresholds = struct();
    end
    thresholds = applyThresholdDefaults(thresholds);

    T = readtable(csvPath);
    T.Properties.VariableNames = lower(T.Properties.VariableNames);

    ok  = logical(T.ok);
    raw = double(T.raw_count);
    t_s = double(T.t_ms) / 1000.0;

    attempts = height(T);
    good     = raw(ok);
    tGood    = t_s(ok);
    nGood    = numel(good);
    dropouts = attempts - nGood;

    stats = struct();
    stats.attempts    = attempts;
    stats.n           = nGood;
    stats.dropouts    = dropouts;
    stats.dropoutRate = ternary(attempts > 0, dropouts / attempts, 0);
    stats.seconds     = ternary(attempts > 0, max(t_s), 0);

    if nGood > 0
        stats.mean = mean(good);
        stats.std  = std(good, 1);          % population std, matches firmware
        stats.ptp  = max(good) - min(good);
    else
        stats.mean = NaN; stats.std = NaN; stats.ptp = NaN;
    end

    % Least-squares slope of raw vs time over good samples (counts/s).
    if nGood >= 2 && (nGood * sum(tGood.^2) - sum(tGood)^2) ~= 0
        p = polyfit(tGood, good, 1);
        stats.driftCps = p(1);
    else
        stats.driftCps = 0;
    end

    % --- Gate ---
    reasons = string.empty(0, 1);
    if stats.dropoutRate > thresholds.maxDropoutRate
        reasons(end+1,1) = sprintf("dropout rate %.3f > %.3f (%d/%d reads failed)", ...
            stats.dropoutRate, thresholds.maxDropoutRate, dropouts, attempts);
    end
    if stats.std > thresholds.maxNoiseStd
        reasons(end+1,1) = sprintf("noise std %.1f > %.1f counts", ...
            stats.std, thresholds.maxNoiseStd);
    end
    if abs(stats.driftCps) > thresholds.maxDriftCps
        reasons(end+1,1) = sprintf("|drift| %.2f > %.2f counts/s", ...
            abs(stats.driftCps), thresholds.maxDriftCps);
    end
    if stats.ptp > thresholds.maxPtp
        reasons(end+1,1) = sprintf("peak-to-peak %.0f > %.0f counts", ...
            stats.ptp, thresholds.maxPtp);
    end

    health = struct();
    health.stats   = stats;
    health.pass    = isempty(reasons);
    health.reasons = reasons;
    health.file    = string(csvPath);

    printReport(health, thresholds);
end


function thresholds = applyThresholdDefaults(thresholds)
    defaults = struct( ...
        'maxDropoutRate', 0.00, ...   % any dropout is a connection fault
        'maxNoiseStd',    Inf,  ...   % report-only until set from baseline
        'maxDriftCps',    Inf,  ...
        'maxPtp',         Inf);
    f = fieldnames(defaults);
    for k = 1:numel(f)
        if ~isfield(thresholds, f{k}) || isempty(thresholds.(f{k}))
            thresholds.(f{k}) = defaults.(f{k});
        end
    end
end


function printReport(health, thresholds)
    s = health.stats;
    fprintf('\nLoad-cell health gate: %s\n', health.file);
    fprintf('  reads ok / attempts : %d / %d (dropouts: %d, rate %.3f)\n', ...
        s.n, s.attempts, s.dropouts, s.dropoutRate);
    fprintf('  probe duration (s)  : %.1f\n', s.seconds);
    fprintf('  zero-load mean      : %.1f counts\n', s.mean);
    fprintf('  noise std           : %.1f counts   (limit %s)\n', ...
        s.std, limStr(thresholds.maxNoiseStd));
    fprintf('  peak-to-peak        : %.0f counts   (limit %s)\n', ...
        s.ptp, limStr(thresholds.maxPtp));
    fprintf('  drift               : %.2f counts/s (limit %s)\n', ...
        s.driftCps, limStr(thresholds.maxDriftCps));
    if health.pass
        fprintf('  VERDICT             : PASS\n');
    else
        fprintf('  VERDICT             : FAIL\n');
        for k = 1:numel(health.reasons)
            fprintf('      - %s\n', health.reasons(k));
        end
    end
end


function str = limStr(v)
    if isinf(v)
        str = 'off';
    else
        str = sprintf('%.3g', v);
    end
end


function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
