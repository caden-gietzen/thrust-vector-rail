%% analyze_propulsion_sizing.m
% Propulsion-chain and compute sizing for the thrust-vector rail component
% specification sheet (docs/component_specification.md).
%
% This is a day-zero, over-bound-first sizing pass. It takes the requirement
% force envelope (identical inputs to analyze_requirement_feasibility.m) and
% carries it down the propulsion chain -- thrust -> required vector deflection
% -> slew-limited vectoring ceiling -> electrical power/current -> ESC/battery --
% and reports the compute (Pico) sizing. Every input is a conservative a priori
% over-bound; nothing here uses an identified transfer function as an input.
%
% Companion: analyze_requirement_feasibility.m owns the servo bandwidth/delay
% screen. This script owns the thrust-authority-to-battery chain and the MCU.

clear; clc; close all;

%% Configuration: shared over-bound inputs (docs/component_specification.md Sec.2)

G = 9.81;                          % m/s^2
M_HEAVY_KG = 0.75;                 % REQUIREMENT: authority worst case
A_REF_MM = 100.0;                  % REQUIREMENT: R1
F_REF_HZ = 0.5;                    % REQUIREMENT: R2
F_INJ_N = 1.0;                     % REQUIREMENT: R13 injected force step

% Friction is a DISTURBANCE the motor must overcome, characterized on the rail
% mechanism BEFORE actuator selection (it is a property of the bearings/pulley,
% not of the motor/servo being chosen). The over-bound is the worst-DIRECTION
% breakaway -- the identified nominal mu_c inflated once by the directional
% asymmetry. No extra "slack" factor: the double-inflation that produced 1.625 N
% is removed. PROVISIONAL, pending re-identification on the 2026-06-26 rebuilt
% mechanism (which locks this input).
MU_C_N = 1.0;                      % identified nominal (pooled), pre-rebuild
FRICTION_ASYM = 0.30;              % directional asymmetry -> worst-direction
THETA_MAX_DEG = 45.0;              % ASSUMPTION: geometric deflection ceiling

% Vectoring bandwidth the thrust spec must not slew-limit (from the servo
% screen in analyze_requirement_feasibility.m).
F_VEC_REQ_HZ = 10.0;               % required useful vectoring bandwidth
SERVO_SLEW_DEG_S = 900.0;          % ASSUMPTION: datasheet-class servo slew

% Propulsion electrical sizing bands (a priori -- replace with vendor data).
STATIC_EFF_GF_PER_W = [6.0, 8.0];  % ASSUMPTION: static thrust efficiency band
CELL_VOLTAGE_V = [11.1, 14.8];     % 3S / 4S nominal
DRIVE_EFFICIENCY = 0.80;           % ASSUMPTION: ESC+motor electrical efficiency
ESC_MARGIN = 1.5;                  % continuous rating vs peak current

% Compute (Pico) sizing.
ACTUATOR_BW_HZ = 10.0;             % actuator useful bandwidth
SAMPLE_MULTIPLE = 20.0;            % f_s >= SAMPLE_MULTIPLE x bandwidth
SERVO_GAIN_DEG_PER_US = 0.09;      % ASSUMPTION: typical hobby servo command gain
ANGLE_RES_DEG = 0.05;              % required servo command angle resolution
V_PEAK_M_S = 0.15;                 % M1 peak cart velocity
ENC_COUNTS_PER_MM = 64.810;        % installed encoder scale

% Current (upgraded) motor/prop context -- shown only as a status callout.
% The constant-thrust-hold campaign (2026-07-03/05) measures ~9.5 N at 40%
% command; the old swept static map (~4.2 N) is stale (hardware changed).
CURRENT_MOTOR_HOLD_N = 9.5;        % hold campaign, 40% command, V~22.3 V

%% Force over-bound (component_specification.md Sec.2)

F_fric_N = MU_C_N * (1 + FRICTION_ASYM);                          % ~1.30 N worst-direction
F_inertial_N = M_HEAVY_KG * (A_REF_MM * 1e-3) * (2 * pi * F_REF_HZ)^2;
F_track_N = F_inertial_N + F_fric_N;                              % tracking coincident
F_hold_N = F_INJ_N + F_fric_N;                                    % M3 hold coincident
Fx_max_N = max(F_track_N, F_hold_N);                             % governing rail force

%% Thrust: authority floor and small-deflection target

T_floor_N = Fx_max_N / sind(THETA_MAX_DEG);
T_floor_kgf = T_floor_N / G;

% Small-deflection target: keep worst-case deflection below the angle at which
% the servo slew rate would drop the slew-limited ceiling to F_VEC_REQ_HZ.
% f_slew = slew / (2*pi*theta)  ->  theta_max_for_bw = slew / (2*pi*f_vec_req)
theta_target_deg = SERVO_SLEW_DEG_S / (2 * pi * F_VEC_REQ_HZ);
T_target_N = Fx_max_N / sind(theta_target_deg);
T_target_kgf = T_target_N / G;

% Current (upgraded) hardware status at its 40%-command hold thrust.
theta_current_deg = asind(min(Fx_max_N / CURRENT_MOTOR_HOLD_N, 1.0));
f_slew_current_hz = SERVO_SLEW_DEG_S / (2 * pi * theta_current_deg);
f_slew_target_hz = SERVO_SLEW_DEG_S / (2 * pi * theta_target_deg);

%% Electrical: power, current, ESC, battery

thrust_target_gf = T_target_kgf * 1000.0;
power_W = thrust_target_gf ./ STATIC_EFF_GF_PER_W;                % [hi eff -> lo P, lo eff -> hi P]
power_W = sort(power_W);                                         % [min, max]

% Peak current across cell options and the power band.
[PW, VV] = meshgrid(power_W, CELL_VOLTAGE_V);
current_A = PW ./ (VV * DRIVE_EFFICIENCY);
peak_current_A = max(current_A(:));
min_current_A = min(current_A(:));
esc_cont_A = ceil(ESC_MARGIN * peak_current_A / 5) * 5;           % round up to 5 A

%% Compute (Pico) sizing

f_s_min_hz = SAMPLE_MULTIPLE * ACTUATOR_BW_HZ;
pwm_res_us = ANGLE_RES_DEG / SERVO_GAIN_DEG_PER_US;
enc_edge_rate = V_PEAK_M_S * (ENC_COUNTS_PER_MM * 1000.0) * 4.0;  % quadrature edges/s

%% Console report

fprintf("\n");
fprintf("===============================================================================\n");
fprintf("PROPULSION-CHAIN AND COMPUTE SIZING (over-bound first)\n");
fprintf("===============================================================================\n");
fprintf("Source: docs/requirements.md + qualification_test_plan.md + a priori over-bounds.\n\n");

fprintf("FORCE OVER-BOUND\n");
fprintf("  Friction over-bound            : %.3f N (worst-dir: mu_c %.1f x (1+%.0f%%))\n", ...
    F_fric_N, MU_C_N, 100 * FRICTION_ASYM);
fprintf("  Peak inertial (heavy)          : %.3f N\n", F_inertial_N);
fprintf("  Tracking coincident            : %.3f N\n", F_track_N);
fprintf("  M3 hold coincident             : %.3f N\n", F_hold_N);
fprintf("  Governing rail force Fx_max    : %.3f N\n\n", Fx_max_N);

fprintf("THRUST SPEC\n");
fprintf("  Authority floor (theta %.0f deg) : %.2f N (%.3f kgf)\n", ...
    THETA_MAX_DEG, T_floor_N, T_floor_kgf);
fprintf("  Small-deflection target        : %.2f N (%.2f kgf), keeps theta <= %.1f deg\n", ...
    T_target_N, T_target_kgf, theta_target_deg);
fprintf("    -> slew-limited ceiling      : %.1f Hz (need >= %.1f Hz) with %.0f deg/s servo\n", ...
    f_slew_target_hz, F_VEC_REQ_HZ, SERVO_SLEW_DEG_S);
fprintf("  Current (upgraded) hardware    : ~%.1f N at 40%% cmd -> theta = %.1f deg -> ceiling %.1f Hz (CLEARS)\n", ...
    CURRENT_MOTOR_HOLD_N, theta_current_deg, f_slew_current_hz);
fprintf("    (old swept map ~4.2 N is stale; re-sweep current hardware to lock the absolute map)\n\n");

fprintf("ELECTRICAL SPEC\n");
fprintf("  Static thrust                  : %.0f gf (%.2f kgf)\n", thrust_target_gf, T_target_kgf);
fprintf("  Power band (%.0f-%.0f gf/W)       : %.0f - %.0f W\n", ...
    STATIC_EFF_GF_PER_W(1), STATIC_EFF_GF_PER_W(2), power_W(1), power_W(2));
fprintf("  Peak current (3S/4S)           : %.1f - %.1f A\n", min_current_A, peak_current_A);
fprintf("  ESC continuous (>= %.1fx peak)   : >= %.0f A\n", ESC_MARGIN, esc_cont_A);
fprintf("  Battery                        : 3-4S, >= 25C, >= 1.5 Ah (or bench DC supply)\n\n");

fprintf("COMPUTE (PICO) SPEC\n");
fprintf("  Control/log loop rate          : >= %.0f Hz (%.0fx the %.0f Hz actuator BW); Pico >= 1 kHz\n", ...
    f_s_min_hz, SAMPLE_MULTIPLE, ACTUATOR_BW_HZ);
fprintf("  Servo PWM resolution           : <= %.2f us (for %.2f deg angle step)\n", ...
    pwm_res_us, ANGLE_RES_DEG);
fprintf("  Encoder edge rate at v_peak    : %.0f edge/s -> HW (PIO) quadrature decode\n", enc_edge_rate);
fprintf("  I/O                            : >= 2 PWM, >= 1 PIO SM, GPIO (HX711), >= 1 ADC, >= 1 UART\n\n");
