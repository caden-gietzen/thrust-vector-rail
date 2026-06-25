function d = friction_wc(v, p_real, d_max, asym, jitter_in)
    v_eps   = 5e-3;    % smoothing width for sign(v), m/s
    jit_amp = 0.15;    % jitter amplitude, m/s^2 equiv (now absolute, not relative)

    u = tanh(v/v_eps);                  % smooth sign(v)

    base  = d_max * (1 + asym*u);       % directional baseline
    patch = 0.8*exp(-((p_real-0.10)/0.01)^2) ...
          + 0.6*exp(-((p_real-0.25)/0.008)^2) ...
          + 0.4*exp(-((p_real-0.40)/0.015)^2 );   % additive, absolute bumps
    jitter = jit_amp * jitter_in;            % additive noise

    d = u * (base + patch + jitter);
end