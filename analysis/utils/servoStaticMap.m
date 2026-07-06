function m = servoStaticMap()
%SERVOSTATICMAP Single source of truth for the servo static command-to-angle map.
%
% The map is identified by analyze_servo_static_sweep.m (combined linear fit over
% both static sweep files) and documented in
% experiments/servo_identification/results.md (Section 3 — Static Command-to-Angle
% Map). Any script that converts a servo PWM command (us) to angle should pull the
% gain from here rather than hardcoding it, so there is exactly one place to update
% if the static sweep is re-run.
%
% Model:
%   theta_deg = gain_deg_per_us * servo_us + intercept_deg
%             = gain_deg_per_us * (servo_us - neutral_pwm_us)
%
% Returned struct fields:
%   gain_deg_per_us  - static gain (deg/us); negative (angle decreases with command)
%   gain_rad_per_us  - same gain in rad/us
%   intercept_deg    - fit intercept (deg)
%   neutral_pwm_us   - command giving zero angle, = -intercept/gain
%   neutral_command_us - rounded zero-angle command used by firmware/PWM tests
%
% Example:
%   m = servoStaticMap();
%   theta_deg = m.gain_deg_per_us * (servo_us - m.neutral_pwm_us);

    % Upgraded high-speed digital servo, re-identified 2026-06-29 (two repeats).
    % Branch-mean (lash-free) gain -0.093996 deg/us, neutral 1427.8 us (->1428),
    % responsive range 481..2531 us, backlash 0.39 deg (prior servo: 2.40 deg).
    % Prior servo: gain -0.091092, intercept 130.329728.
    m.gain_deg_per_us = -0.093996;
    m.intercept_deg   = 134.209370;

    m.gain_rad_per_us = m.gain_deg_per_us * pi / 180;
    m.neutral_pwm_us  = -m.intercept_deg / m.gain_deg_per_us;
    m.neutral_command_us = round(m.neutral_pwm_us);
end
