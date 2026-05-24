import time
import math
from machine import Pin, PWM
import encoder

# ============================================================
# Servo step-response test
# ============================================================
# Purpose:
#   Identify dynamic servo command-to-angle behavior using encoder feedback.
#
# CSV output is intentionally similar to the static sweep script:
#   - t_ms / t_s timing columns
#   - servo_us command
#   - count, count_zero, count_delta
#   - theta_deg, theta_rad
#   - segment / phase labels
#
# External configuration:
#   This file will load /run_config.json first, then run_config.json.
#   If the JSON has a top-level "config" key, only that object is applied.
# ============================================================

ENC_A_PIN = 18
ENC_B_PIN = 19
SERVO_PIN = 15

MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SERVO_CENTER_US = 1450
SERVO_MIN_US = 450
SERVO_MAX_US = 2450
SERVO_FREQ_HZ = 50

COUNTS_PER_REV = 2400.0

# Static map from 2026_05_24 static sweep.
# theta_deg = STATIC_GAIN_DEG_PER_US * servo_us + STATIC_INTERCEPT_DEG
STATIC_GAIN_DEG_PER_US = -0.091092
STATIC_INTERCEPT_DEG = 130.329728

PRE_ZERO_CENTER_SETTLE_MS = 2000
POST_ZERO_SETTLE_MS = 300
BASELINE_HOLD_MS = 1000

# Default timing.
SAMPLE_PERIOD_MS = 20
PRE_STEP_HOLD_MS = 800
POST_STEP_HOLD_MS = 1500
BETWEEN_STEP_CENTER_HOLD_MS = 800

# CSV flushing. Use 1 for maximum data safety; larger values reduce overhead.
FLUSH_EVERY_N_ROWS = 1

FILENAME = "servo_step_response_test.csv"

# Each case is a commanded step from start_us to end_us.
# Keep these inside the control-relevant vectoring envelope unless intentionally testing endpoints.
STEP_CASES = [
    {"label": "center_to_pos_small", "start_us": 1450, "end_us": 1300, "reps": 2},
    {"label": "center_to_neg_small", "start_us": 1450, "end_us": 1600, "reps": 2},
    {"label": "pos_small_to_neg_small", "start_us": 1300, "end_us": 1600, "reps": 2},
    {"label": "neg_small_to_pos_small", "start_us": 1600, "end_us": 1300, "reps": 2}
]


def load_json_file_if_exists(path):
    try:
        import ujson as json
    except ImportError:
        import json

    try:
        with open(path, "r") as f:
            return json.load(f)
    except OSError:
        return None


def apply_external_config_if_present():
    external = load_json_file_if_exists("/run_config.json")

    if external is None:
        external = load_json_file_if_exists("run_config.json")

    if external is None:
        return

    if isinstance(external, dict) and "config" in external:
        external = external["config"]

    if not isinstance(external, dict):
        raise ValueError("External run config must be a JSON object/dictionary.")

    print("Loaded external run config.")

    for key in external:
        globals()[key] = external[key]


def ensure_csv_filename():
    global FILENAME

    if FILENAME is None or FILENAME == "":
        FILENAME = "servo_step_response_test.csv"

    if not FILENAME.endswith(".csv"):
        FILENAME = FILENAME + ".csv"


def clamp_servo_us(pulse_us):
    pulse_us = int(pulse_us)

    if pulse_us < SERVO_MIN_US:
        return int(SERVO_MIN_US)

    if pulse_us > SERVO_MAX_US:
        return int(SERVO_MAX_US)

    return pulse_us


def write_pwm_us(pwm, pulse_us):
    pulse_us = clamp_servo_us(pulse_us)
    pwm.duty_ns(int(pulse_us * 1000))
    return pulse_us


def count_to_theta_deg(count_delta):
    return count_delta / COUNTS_PER_REV * 360.0


def count_to_theta_rad(count_delta):
    return count_delta / COUNTS_PER_REV * 2.0 * math.pi


def command_to_theta_deg(servo_us):
    return STATIC_GAIN_DEG_PER_US * servo_us + STATIC_INTERCEPT_DEG


def command_to_theta_rad(servo_us):
    return command_to_theta_deg(servo_us) * math.pi / 180.0


def safe_case_value(case, key, default_value):
    if isinstance(case, dict) and key in case:
        return case[key]
    return default_value


def write_header(f):
    f.write(
        "t_ms,t_s,trial_idx,case_idx,case_label,rep_idx,"
        "phase,segment,"
        "step_start_us,step_end_us,step_delta_us,servo_us,"
        "theta_cmd_deg,theta_cmd_rad,"
        "count,count_zero,count_delta,theta_deg,theta_rad,"
        "time_since_step_ms,time_since_step_s\n"
    )


def write_sample(
    f,
    start_ms,
    trial_idx,
    case_idx,
    case_label,
    rep_idx,
    phase,
    segment,
    step_start_us,
    step_end_us,
    servo_us,
    count_zero,
    step_start_time_ms,
):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, start_ms)
    t_s = t_ms / 1000.0

    count = encoder.get_count()
    count_delta = count - count_zero
    theta_deg = count_to_theta_deg(count_delta)
    theta_rad = count_to_theta_rad(count_delta)

    step_delta_us = int(step_end_us) - int(step_start_us)
    theta_cmd_deg = command_to_theta_deg(servo_us)
    theta_cmd_rad = command_to_theta_rad(servo_us)

    if step_start_time_ms is None:
        time_since_step_ms = -1
    else:
        time_since_step_ms = time.ticks_diff(now_ms, step_start_time_ms)

    time_since_step_s = time_since_step_ms / 1000.0

    f.write("{},{:.3f},{},{},{},{},{},{},{},{},{},{},{:.6f},{:.8f},{},{},{},{:.6f},{:.8f},{},{}\n".format(
        t_ms,
        t_s,
        trial_idx,
        case_idx,
        case_label,
        rep_idx,
        phase,
        segment,
        step_start_us,
        step_end_us,
        step_delta_us,
        servo_us,
        theta_cmd_deg,
        theta_cmd_rad,
        count,
        count_zero,
        count_delta,
        theta_deg,
        theta_rad,
        time_since_step_ms,
        time_since_step_s
    ))


def log_for_duration(
    f,
    start_ms,
    duration_ms,
    sample_period_ms,
    trial_idx,
    case_idx,
    case_label,
    rep_idx,
    phase,
    segment,
    step_start_us,
    step_end_us,
    servo_us,
    count_zero,
    step_start_time_ms,
    row_counter,
):
    duration_ms = int(duration_ms)
    sample_period_ms = int(sample_period_ms)

    if sample_period_ms <= 0:
        raise ValueError("SAMPLE_PERIOD_MS must be positive.")

    phase_start_ms = time.ticks_ms()
    next_sample_ms = phase_start_ms

    while time.ticks_diff(time.ticks_ms(), phase_start_ms) < duration_ms:
        now_ms = time.ticks_ms()

        if time.ticks_diff(now_ms, next_sample_ms) >= 0:
            write_sample(
                f,
                start_ms,
                trial_idx,
                case_idx,
                case_label,
                rep_idx,
                phase,
                segment,
                step_start_us,
                step_end_us,
                servo_us,
                count_zero,
                step_start_time_ms
            )

            row_counter += 1

            if FLUSH_EVERY_N_ROWS <= 1 or (row_counter % FLUSH_EVERY_N_ROWS) == 0:
                f.flush()

            next_sample_ms = time.ticks_add(next_sample_ms, sample_period_ms)

        time.sleep_ms(1)

    return row_counter


def run_step_case(
    f,
    pwm,
    start_ms,
    count_zero,
    trial_idx,
    case_idx,
    case,
    row_counter,
):
    label = str(safe_case_value(case, "label", "case_{}".format(case_idx)))
    start_us = clamp_servo_us(safe_case_value(case, "start_us", SERVO_CENTER_US))
    end_us = clamp_servo_us(safe_case_value(case, "end_us", SERVO_CENTER_US))
    reps = int(safe_case_value(case, "reps", 1))

    pre_hold_ms = int(safe_case_value(case, "pre_step_hold_ms", PRE_STEP_HOLD_MS))
    post_hold_ms = int(safe_case_value(case, "post_step_hold_ms", POST_STEP_HOLD_MS))
    sample_period_ms = int(safe_case_value(case, "sample_period_ms", SAMPLE_PERIOD_MS))
    return_to_center = bool(safe_case_value(case, "return_to_center", True))
    center_hold_ms = int(safe_case_value(case, "between_step_center_hold_ms", BETWEEN_STEP_CENTER_HOLD_MS))

    for rep_idx in range(1, reps + 1):
        print("Case", case_idx, label, "rep", rep_idx, ":", start_us, "->", end_us)

        actual_start_us = write_pwm_us(pwm, start_us)

        row_counter = log_for_duration(
            f,
            start_ms,
            pre_hold_ms,
            sample_period_ms,
            trial_idx,
            case_idx,
            label,
            rep_idx,
            "pre_step_hold",
            "pre_step_hold",
            start_us,
            end_us,
            actual_start_us,
            count_zero,
            None,
            row_counter
        )

        step_start_time_ms = time.ticks_ms()
        actual_end_us = write_pwm_us(pwm, end_us)

        row_counter = log_for_duration(
            f,
            start_ms,
            post_hold_ms,
            sample_period_ms,
            trial_idx,
            case_idx,
            label,
            rep_idx,
            "step_response",
            "step_response",
            start_us,
            end_us,
            actual_end_us,
            count_zero,
            step_start_time_ms,
            row_counter
        )

        if return_to_center:
            actual_center_us = write_pwm_us(pwm, SERVO_CENTER_US)

            row_counter = log_for_duration(
                f,
                start_ms,
                center_hold_ms,
                sample_period_ms,
                trial_idx,
                case_idx,
                label,
                rep_idx,
                "return_to_center",
                "return_to_center",
                end_us,
                SERVO_CENTER_US,
                actual_center_us,
                count_zero,
                None,
                row_counter
            )

    return row_counter


apply_external_config_if_present()
ensure_csv_filename()

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

try:
    print("Initializing encoder...")
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)

    print("Centering servo...")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(PRE_ZERO_CENTER_SETTLE_MS)

    print("Zeroing encoder at center...")
    encoder.zero()
    time.sleep_ms(POST_ZERO_SETTLE_MS)

    count_zero = encoder.get_count()
    print("Count zero:", count_zero)

    start_ms = time.ticks_ms()
    row_counter = 0

    with open(FILENAME, "w") as f:
        write_header(f)

        print("Logging center baseline...")
        write_pwm_us(servo, SERVO_CENTER_US)
        row_counter = log_for_duration(
            f,
            start_ms,
            BASELINE_HOLD_MS,
            SAMPLE_PERIOD_MS,
            0,
            0,
            "baseline_center",
            0,
            "baseline_center",
            "baseline_center",
            SERVO_CENTER_US,
            SERVO_CENTER_US,
            SERVO_CENTER_US,
            count_zero,
            None,
            row_counter
        )

        trial_idx = 1

        for case_idx in range(1, len(STEP_CASES) + 1):
            row_counter = run_step_case(
                f,
                servo,
                start_ms,
                count_zero,
                trial_idx,
                case_idx,
                STEP_CASES[case_idx - 1],
                row_counter
            )

        f.flush()

    print("Step response test complete.")
    print("Rows written:", row_counter)
    print("Saved:", FILENAME)


finally:
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()
    print("Done.")
