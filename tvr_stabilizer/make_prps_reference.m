function [r, rdot, rddot, plan] = make_prps_reference(t, amp, seed, opts)
%MAKE_PRPS_REFERENCE Deterministic periodic multisine reference for CL ID.
%   [r,rdot,rddot,plan] = MAKE_PRPS_REFERENCE(t, amp, seed) returns a
%   peak-normalized PRPS position reference and its analytic derivatives.
%
%   The default plan is intentionally conservative for first closed-loop ID:
%   0.05--1.0 Hz requested frequencies, 40 s period, and three periods.

    if nargin < 1
        t = [];
    end
    if nargin < 2 || isempty(amp)
        amp = 0.05;
    end
    if nargin < 3 || isempty(seed)
        seed = 5001;
    end
    if nargin < 4
        opts = struct();
    end

    plan = localBuildPlan(amp, seed, opts);

    if isempty(t)
        r = [];
        rdot = [];
        rddot = [];
        return;
    end

    t = max(t - plan.pre_hold_s, 0);
    if plan.period_s > 0
        t = mod(t, plan.period_s);
    end

    raw = 0;
    rawDot = 0;
    rawDdot = 0;
    for i = 1:numel(plan.bins)
        w = 2*pi*plan.snapped_freqs_hz(i);
        a = w*t + plan.phase_rad(i);
        raw = raw + sin(a);
        rawDot = rawDot + w*cos(a);
        rawDdot = rawDdot - w^2*sin(a);
    end

    scale = plan.amp_m / plan.peak_raw;
    r = scale * raw;
    rdot = scale * rawDot;
    rddot = scale * rawDdot;
end

function plan = localBuildPlan(amp, seed, opts)
    periodS = localGet(opts, 'period_s', 40.0);
    numPeriods = localGet(opts, 'num_periods', 3);
    updateDtS = localGet(opts, 'update_dt_s', 0.020);
    preHoldS = localGet(opts, 'pre_hold_s', 0.0);
    postHoldS = localGet(opts, 'post_hold_s', 0.0);
    requested = localGet(opts, 'requested_freqs_hz', localDefaultFreqs());

    [requestedKept, snapped, bins] = localSnapFrequencies(requested, periodS);
    phaseRad = localGet(opts, 'phase_rad', []);
    if isempty(phaseRad)
        phaseRad = localDeterministicPhases(seed, numel(bins));
    end

    samplesPerPeriod = max(4, round(periodS / updateDtS));
    peakRaw = localEstimatePeak(samplesPerPeriod, bins, phaseRad);

    plan = struct();
    plan.amp_m = amp;
    plan.seed = seed;
    plan.period_s = periodS;
    plan.num_periods = numPeriods;
    plan.update_dt_s = updateDtS;
    plan.pre_hold_s = preHoldS;
    plan.post_hold_s = postHoldS;
    plan.total_duration_s = preHoldS + numPeriods*periodS + postHoldS;
    plan.id_warmup_periods = 1;
    plan.id_start_s = preHoldS + periodS;
    plan.id_end_s = preHoldS + numPeriods*periodS;
    plan.requested_freqs_hz = requestedKept;
    plan.snapped_freqs_hz = snapped;
    plan.bins = bins;
    plan.phase_rad = phaseRad;
    plan.peak_raw = peakRaw;
    plan.samples_per_period = samplesPerPeriod;
    plan.fundamental_hz = 1.0 / periodS;
end

function freqs = localDefaultFreqs()
    freqs = exp(linspace(log(0.05), log(1.0), 15));
end

function value = localGet(s, name, fallback)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = fallback;
    end
end

function [requestedKept, snapped, bins] = localSnapFrequencies(requested, periodS)
    f0 = 1.0 / periodS;
    requested = requested(:).';
    bins = zeros(size(requested));
    snapped = zeros(size(requested));
    requestedKept = zeros(size(requested));
    n = 0;
    used = zeros(1, 512);
    for i = 1:numel(requested)
        if requested(i) <= 0
            continue;
        end
        k = max(1, round(requested(i) / f0));
        if k <= numel(used) && used(k)
            continue;
        end
        n = n + 1;
        bins(n) = k;
        snapped(n) = k * f0;
        requestedKept(n) = requested(i);
        if k <= numel(used)
            used(k) = 1;
        end
    end
    bins = bins(1:n);
    snapped = snapped(1:n);
    requestedKept = requestedKept(1:n);
end

function phases = localDeterministicPhases(seed, n)
    phases = zeros(1, n);
    state = double(seed);
    for i = 1:n
        state = mod(1664525*state + 1013904223, 2^32);
        phases(i) = 2*pi*state / 2^32;
    end
end

function peak = localEstimatePeak(samplesPerPeriod, bins, phases)
    peak = 0;
    for n = 0:(samplesPerPeriod - 1)
        x = 0;
        for i = 1:numel(bins)
            x = x + sin(2*pi*bins(i)*n/samplesPerPeriod + phases(i));
        end
        peak = max(peak, abs(x));
    end
    if peak <= 0
        peak = 1;
    end
end
