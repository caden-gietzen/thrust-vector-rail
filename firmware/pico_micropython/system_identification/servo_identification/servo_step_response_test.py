import time
import math
from array import array
from machine import Pin, PWM
import encoder


# ------------------------------------------------------------
# Servo truth values (single source of truth).
#
# The zero-angle center command and the static gain live in
# lib/servo_static_map.py (the device-side mirror of
# analysis/utils/servoStaticMap.m). Pull them from there so this test never
# hardcodes a stale center/gain. If the module isn't flashed to /lib yet -- e.g.
# a standalone run before the orchestrator's "uploads" step has populated it --
# fall back to the last-known literals so the script still runs.
try:
    from servo_static_map import NEUTRAL_US as _TRUTH_CENTER_US
    from servo_static_map import GAIN_DEG_PER_US as _TRUTH_GAIN_DEG_PER_US
    _TRUTH_SOURCE = "lib/servo_static_map.py"
except ImportError:
    _TRUTH_CENTER_US = 1431
    _TRUTH_GAIN_DEG_PER_US = -0.091092
    _TRUTH_SOURCE = "fallback literals (servo_static_map not on /lib)"

# ============================================================
# Servo step-response test
# ============================================================
# Purpose:
#   Identify dynamic servo command-to-angle behavior using encoder feedback.
#
# CSV output is intentionally compact:
#   - timing: t_s, time_since_step_s
#   - grouping: case_idx, case_label, rep_idx, phase
#   - command: step_start_us, step_end_us, servo_us, theta_cmd_deg
#   - measurement: count_delta
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

# Zero-angle center from the static-map truth source (lib/servo_static_map.py).
SERVO_CENTER_US = _TRUTH_CENTER_US
SERVO_MIN_US = 450
SERVO_MAX_US = 2450

# PWM carrier = the servo's actuation/command-refresh rate (200 Hz -> one frame
# every 5 ms). Kept at 200 Hz ON PURPOSE: the PRPS model this step test feeds was
# identified in the 200 Hz actuation regime (clean coherence to ~10.7 Hz), so the
# recon corner and validity envelope must be measured in that same regime. Raising
# the carrier would characterise delay/slew in a regime the model is not fit for,
# and not every servo tracks a faster frame cleanly. This is independent of the
# encoder SAMPLE rate below, which we DO push high.
SERVO_FREQ_HZ = 330

# Encoder count-to-angle scale: spec-derived and exact (1:1 GT2 16T belt, 600 PPR
# x 4 quadrature = 2400 counts/rev = 0.15 deg/count). Single source of truth on
# the analysis side is analysis/utils/encoderAngleScale.m -- keep this in sync.
COUNTS_PER_REV = 2400.0

# Static command-to-angle gain from the truth source (signed; angle decreases with
# command). theta_deg = STATIC_GAIN_DEG_PER_US * (servo_us - SERVO_CENTER_US).
STATIC_GAIN_DEG_PER_US = _TRUTH_GAIN_DEG_PER_US

PRE_ZERO_CENTER_SETTLE_MS = 2000
POST_ZERO_SETTLE_MS = 300
BASELINE_HOLD_MS = 200

# Default timing.
#
# Holds (pre-step settle, post-step tail, return-to-center) only need to confirm
# the servo has settled, so they sample at the slow SAMPLE_PERIOD_MS via the
# inline format-and-write path.
#
# The step transient is where peak slew rate, delay, and the rate-limit plateau
# are measured -- the numbers that found the PRPS velocity ceiling and
# amplitude/band limits. At 20 ms only 2-3 points land on the steep part of a
# ~30 ms rise, biasing slew low. So the transient is captured at a much higher
# rate (STEP_SAMPLE_PERIOD_MS) by a dedicated RAM-buffered, microsecond-timestamped
# busy-poll loop (capture_step_response_fast): samples are stored as raw
# (t_us, count) and only formatted/written AFTER the burst, so the sample rate is
# decoupled from flash-write/format cost (the same reason the PRPS logger buffers).
# Microsecond timestamps are required so sub-millisecond samples have distinct t
# values (a 1 ms-resolution clock would alias adjacent samples and corrupt the
# velocity estimate). 1 ms (1 kHz) is ~5x oversampling the 5 ms PWM frame and well
# within RP2040 RAM/CPU; lower it if flash space is tight. May be fractional.
SAMPLE_PERIOD_MS = 20
STEP_SAMPLE_PERIOD_MS = 1
ESTIMATED_SERVO_SLEW_DEG_PER_S = 240
SETTLE_MARGIN_MS = 150
# Only the first STEP_FAST_WINDOW_MS of the post-step hold is captured at the fast
# rate; the rest is sampled at SAMPLE_PERIOD_MS. The transient (move + initial
# settle) of the largest step here is ~200 ms, so ~350 ms of fast capture covers
# it with margin while keeping the per-run CSV inside the Pico flash. Set <= 0 to
# capture the entire post-step hold fast.
# Dwell times are kept SHORT: the fast window already captures the move plus its
# settle, so the post-step hold only needs a brief steady tail for the final-value
# estimate (POST_STEP_HOLD_MS just past STEP_FAST_WINDOW_MS), and the pre-step and
# return-to-center dwells only need to re-settle before the next step. Long dwells
# add nothing but rows and fill the Pico flash before the campaign finishes.
STEP_FAST_WINDOW_MS = 350
PRE_STEP_HOLD_MS = 250
POST_STEP_HOLD_MS = 450
BETWEEN_STEP_CENTER_HOLD_MS = 250

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


def command_to_theta_deg(servo_us):
    return STATIC_GAIN_DEG_PER_US * (servo_us - SERVO_CENTER_US)


def safe_case_value(case, key, default_value):
    if isinstance(case, dict) and key in case:
        return case[key]
    return default_value


def estimated_servo_move_ms(from_us, to_us):
    slew = float(ESTIMATED_SERVO_SLEW_DEG_PER_S)
    if slew <= 0:
        raise ValueError("ESTIMATED_SERVO_SLEW_DEG_PER_S must be positive.")

    angle_delta_deg = abs(command_to_theta_deg(to_us) - command_to_theta_deg(from_us))
    return int(math.ceil(angle_delta_deg / slew * 1000.0))


def hold_ms_for_move(from_us, to_us):
    return estimated_servo_move_ms(from_us, to_us) + int(SETTLE_MARGIN_MS)


def write_header(f):
    f.write(
        "t_s,case_idx,case_label,rep_idx,phase,"
        "step_start_us,step_end_us,servo_us,theta_cmd_deg,"
        "count_delta,time_since_step_s\n"
    )


def write_sample(
    f,
    start_ms,
    case_idx,
    case_label,
    rep_idx,
    phase,
    step_start_us,
    step_end_us,
    servo_us,
    count_zero,
    step_start_time_ms,
):
    now_ms = time.ticks_ms()
    t_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    count = encoder.get_count()
    count_delta = count - count_zero

    theta_cmd_deg = command_to_theta_deg(servo_us)

    if step_start_time_ms is None:
        time_since_step_s = -1.0
    else:
        time_since_step_s = time.ticks_diff(now_ms, step_start_time_ms) / 1000.0

    f.write("{:.6f},{},{},{},{},{},{},{},{:.6f},{},{:.6f}\n".format(
        t_s,
        case_idx,
        case_label,
        rep_idx,
        phase,
        step_start_us,
        step_end_us,
        servo_us,
        theta_cmd_deg,
        count_delta,
        time_since_step_s
    ))


def log_for_duration(
    f,
    start_ms,
    duration_ms,
    sample_period_ms,
    case_idx,
    case_label,
    rep_idx,
    phase,
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
                case_idx,
                case_label,
                rep_idx,
                phase,
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


def capture_step_response_fast(start_us, duration_ms, sample_period_ms):
    """Capture the step transient into RAM at a high, microsecond-timed rate.

    Returns (t_us_buf, count_buf, n): per-sample microsecond timestamp (relative
    to start_us) and raw encoder count. NOTHING is formatted or written here --
    that happens after the burst -- so the realized sample rate is bounded by the
    encoder read + array store (a few us), not by str.format/flash. A busy-poll on
    ticks_us gives sub-millisecond pacing; the 1 ms-resolution ticks_ms clock is
    used only for the coarse phase-duration check.
    """
    duration_ms = int(duration_ms)
    period_us = int(round(float(sample_period_ms) * 1000.0))
    if period_us < 1:
        period_us = 1

    capacity = duration_ms * 1000 // period_us + 16
    t_us_buf = array("i", [0] * capacity)
    count_buf = array("i", [0] * capacity)
    n = 0

    phase_start_ms = time.ticks_ms()
    next_us = time.ticks_us()

    while time.ticks_diff(time.ticks_ms(), phase_start_ms) < duration_ms:
        if time.ticks_diff(time.ticks_us(), next_us) >= 0 and n < capacity:
            t_us_buf[n] = time.ticks_diff(time.ticks_us(), start_us)
            count_buf[n] = encoder.get_count()
            n += 1
            next_us = time.ticks_add(next_us, period_us)

    return t_us_buf, count_buf, n


def write_captured_step_rows(
    f,
    t_us_buf,
    count_buf,
    n,
    start_us,
    case_idx,
    case_label,
    rep_idx,
    step_start_us,
    step_end_us,
    servo_us,
    count_zero,
    step_start_time_us,
    row_counter,
):
    """Format and write the buffered step-response samples after the burst."""
    theta_cmd_deg = command_to_theta_deg(servo_us)

    for i in range(n):
        t_us = t_us_buf[i]
        t_s = t_us / 1_000_000.0

        count = count_buf[i]
        count_delta = count - count_zero

        # Reconstruct the absolute sample time (start_us + t_us) to difference
        # against the step instant at microsecond precision.
        now_us = time.ticks_add(start_us, t_us)
        time_since_step_us = time.ticks_diff(now_us, step_start_time_us)
        time_since_step_s = time_since_step_us / 1_000_000.0

        f.write("{:.6f},{},{},{},{},{},{},{},{:.6f},{},{:.6f}\n".format(
            t_s,
            case_idx,
            case_label,
            rep_idx,
            "step_response",
            step_start_us,
            step_end_us,
            servo_us,
            theta_cmd_deg,
            count_delta,
            time_since_step_s
        ))

        row_counter += 1
        if FLUSH_EVERY_N_ROWS <= 1 or (row_counter % FLUSH_EVERY_N_ROWS) == 0:
            f.flush()

    f.flush()
    return row_counter


def run_step_case(
    f,
    pwm,
    start_ms,
    start_us,
    count_zero,
    case_idx,
    case,
    row_counter,
):
    label = str(safe_case_value(case, "label", "case_{}".format(case_idx)))
    case_start_us = clamp_servo_us(safe_case_value(case, "start_us", SERVO_CENTER_US))
    end_us = clamp_servo_us(safe_case_value(case, "end_us", SERVO_CENTER_US))
    reps = int(safe_case_value(case, "reps", 1))

    pre_hold_ms = int(safe_case_value(case, "pre_step_hold_ms", PRE_STEP_HOLD_MS))
    post_hold_ms = int(safe_case_value(case, "post_step_hold_ms", POST_STEP_HOLD_MS))
    sample_period_ms = int(safe_case_value(case, "sample_period_ms", SAMPLE_PERIOD_MS))
    step_sample_period_ms = float(safe_case_value(case, "step_sample_period_ms", STEP_SAMPLE_PERIOD_MS))
    fast_window_ms = int(safe_case_value(case, "step_fast_window_ms", STEP_FAST_WINDOW_MS))
    return_to_center = bool(safe_case_value(case, "return_to_center", True))
    center_hold_ms = int(safe_case_value(case, "between_step_center_hold_ms", BETWEEN_STEP_CENTER_HOLD_MS))

    min_pre_hold_ms = hold_ms_for_move(SERVO_CENTER_US, case_start_us)
    min_post_hold_ms = hold_ms_for_move(case_start_us, end_us)
    min_center_hold_ms = hold_ms_for_move(end_us, SERVO_CENTER_US)
    min_fast_window_ms = min_post_hold_ms

    pre_hold_ms = max(pre_hold_ms, min_pre_hold_ms)
    post_hold_ms = max(post_hold_ms, min_post_hold_ms)
    center_hold_ms = max(center_hold_ms, min_center_hold_ms)
    if fast_window_ms > 0:
        fast_window_ms = max(fast_window_ms, min_fast_window_ms)

    for rep_idx in range(1, reps + 1):
        print("Case", case_idx, label, "rep", rep_idx, ":", case_start_us, "->", end_us,
              "holds ms pre/post/center:", pre_hold_ms, post_hold_ms, center_hold_ms,
              "fast:", fast_window_ms)

        actual_start_us = write_pwm_us(pwm, case_start_us)

        row_counter = log_for_duration(
            f,
            start_ms,
            pre_hold_ms,
            sample_period_ms,
            case_idx,
            label,
            rep_idx,
            "pre_step_hold",
            case_start_us,
            end_us,
            actual_start_us,
            count_zero,
            None,
            row_counter
        )

        # Apply the step and capture the TRANSIENT at high rate, then drop to the
        # slow rate for the settle tail. The fast window is where slew, delay, and
        # the rate-limit plateau live; the long tail only feeds the final-value
        # estimate, so sampling it at 1 kHz would just bloat the CSV (and can
        # overflow the Pico flash). The step instant is timestamped in us for
        # clean sub-ms velocity, as tightly around the PWM write as possible.
        step_start_time_ms = time.ticks_ms()
        step_start_time_us = time.ticks_us()
        actual_end_us = write_pwm_us(pwm, end_us)

        if fast_window_ms <= 0:
            fast_ms = post_hold_ms
        else:
            fast_ms = min(post_hold_ms, fast_window_ms)

        t_us_buf, count_buf, n = capture_step_response_fast(
            start_us,
            fast_ms,
            step_sample_period_ms
        )

        row_counter = write_captured_step_rows(
            f,
            t_us_buf,
            count_buf,
            n,
            start_us,
            case_idx,
            label,
            rep_idx,
            case_start_us,
            end_us,
            actual_end_us,
            count_zero,
            step_start_time_us,
            row_counter
        )

        tail_ms = post_hold_ms - fast_ms
        if tail_ms > 0:
            row_counter = log_for_duration(
                f,
                start_ms,
                tail_ms,
                sample_period_ms,
                case_idx,
                label,
                rep_idx,
                "step_response",
                case_start_us,
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
                case_idx,
                label,
                rep_idx,
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
    print("Servo truth values from:", _TRUTH_SOURCE)
    print("  center:", SERVO_CENTER_US, "us  gain:", STATIC_GAIN_DEG_PER_US, "deg/us")
    print("  PWM carrier:", SERVO_FREQ_HZ, "Hz (command refresh)")
    print("  hold sample:", SAMPLE_PERIOD_MS, "ms  fast transient:", STEP_SAMPLE_PERIOD_MS,
          "ms for first", STEP_FAST_WINDOW_MS, "ms")

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
    start_us = time.ticks_us()   # microsecond reference for high-rate step capture
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
            "baseline_center",
            0,
            "baseline_center",
            SERVO_CENTER_US,
            SERVO_CENTER_US,
            SERVO_CENTER_US,
            count_zero,
            None,
            row_counter
        )

        for case_idx in range(1, len(STEP_CASES) + 1):
            row_counter = run_step_case(
                f,
                servo,
                start_ms,
                start_us,
                count_zero,
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
