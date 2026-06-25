function d = friction_mc(v, p_real, friction, jitter_in)
%FRICTION_MC Acceleration-equivalent friction disturbance for MC validation.
%   Returns d [m/s^2] with the same sign as velocity, so plant.m subtracts it
%   from acceleration. The input friction struct is sampled in make_mc_cases.

    if nargin < 4
        jitter_in = 0;
    end
    if isempty(friction) || ~isstruct(friction)
        d = friction_wc(v, p_real, 1.5, 0.3, jitter_in);
        return;
    end

    massKg = getFieldOr(friction, "mass_kg", 0.5);
    vEps = getFieldOr(friction, "v_epsilon_mps", 5e-3);
    jitterAmpN = getFieldOr(friction, "jitter_amp_N", 0.05);

    u = tanh(v / max(vEps, eps));
    speed = abs(v);
    if v >= 0
        b = getFieldOr(friction, "b_pos_Ns_per_m", 0);
        mu = getFieldOr(friction, "mu_pos_N", 0);
    else
        b = getFieldOr(friction, "b_neg_Ns_per_m", 0);
        mu = getFieldOr(friction, "mu_neg_N", 0);
    end

    dynamicForceN = mu + b * speed;
    spatialForceN = spatialBumpForce(p_real, friction);
    stictionForceN = stictionBlend(speed, friction);
    jitterForceN = jitterAmpN * jitter_in;

    forceMagN = max(dynamicForceN + spatialForceN + stictionForceN + jitterForceN, 0);
    d = u * forceMagN / max(massKg, eps);
end

function y = spatialBumpForce(p_real, friction)
    if ~isfield(friction, "spatial_centers_m") || isempty(friction.spatial_centers_m)
        y = 0;
        return;
    end
    centers = friction.spatial_centers_m(:);
    widths = friction.spatial_widths_m(:);
    amps = friction.spatial_amplitudes_N(:);
    n = min([numel(centers), numel(widths), numel(amps)]);
    if n == 0
        y = 0;
        return;
    end
    centers = centers(1:n);
    widths = max(widths(1:n), eps);
    amps = amps(1:n);
    y = sum(amps .* exp(-((p_real - centers).^2) ./ (2 * widths.^2)));
end

function y = stictionBlend(speed, friction)
    low = getFieldOr(friction, "breakaway_low_N", 0);
    high = getFieldOr(friction, "breakaway_high_N", low);
    vEps = getFieldOr(friction, "v_epsilon_mps", 5e-3);
    breakaway = 0.5 * (low + high);
    y = breakaway * exp(-(speed / max(vEps, eps))^2);
end

function value = getFieldOr(s, name, fallback)
    if isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = fallback;
    end
end
