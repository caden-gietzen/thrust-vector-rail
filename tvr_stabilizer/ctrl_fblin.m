function [theta_cmd, sigma_dot] = ctrl_fblin(p_meas, v_meas, sigma, r, rdot, rddot, m, T, ki, k1, k2)
    % Input-output linearization, relative degree 2, instantaneous-theta design
    e1 = p_meas - r;
    e2 = v_meas - rdot;
    u_raw = (m/T) * (rddot -ki*sigma - k1*e1 - k2*e2); % u = sin(theta) command
    u = max(min(u_raw, 0.999), -0.999); % actuator saturation, avoid asin domain issues
    saturated = (u ~= u_raw);
    % conditional integration: freeze integrator when saturated and it would push further
    if saturated && sign(u_raw) == sign(e1)
        sigma_dot = 0;
    else
        sigma_dot = e1;
    end
    theta_cmd = asin(u);
end