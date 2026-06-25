function d = friction_mc_fields(v, p_real, jitter_in, ...
        b_pos, b_neg, mu_pos, mu_neg, break_low, break_high, v_eps, ...
        mass_kg, centers, widths, amplitudes, jitter_amp)
%FRICTION_MC_FIELDS Codegen-friendly friction disturbance from sampled fields.

    u = tanh(v / max(v_eps, eps));
    speed = abs(v);
    if v >= 0
        b = b_pos;
        mu = mu_pos;
    else
        b = b_neg;
        mu = mu_neg;
    end

    dynamicForceN = mu + b * speed;
    spatialForceN = sum(amplitudes(:) .* exp(-((p_real - centers(:)).^2) ./ ...
        (2 * max(widths(:), eps).^2)));
    breakaway = 0.5 * (break_low + break_high);
    stictionForceN = breakaway * exp(-(speed / max(v_eps, eps))^2);
    jitterForceN = jitter_amp * jitter_in;

    forceMagN = max(dynamicForceN + spatialForceN + stictionForceN + jitterForceN, 0);
    d = u * forceMagN / max(mass_kg, eps);
end
