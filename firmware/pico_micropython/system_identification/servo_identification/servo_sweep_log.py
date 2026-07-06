# servo_sweep_log.py
#
# Purpose:
#   Static servo command-to-angle sweep that DISCOVERS the usable range instead
#   of assuming declared endpoints. The servo is walked OUTWARD from center in
#   both directions; each direction stops as soon as the encoder stops
#   responding to further command (true mechanical / control saturation).
#
#   This replaces the prior blind MIN->MAX sweep, which jumped straight to a
#   hand-declared endpoint and held there even if it was past the mechanical
#   stop. Walking outward approaches the stops gradually and backs off the
#   instant motion flatlines, so probing out to (and past) the standard hobby
#   range (~1000-2000 us, extended ~500-2500 us) does not slam a stalled servo
#   against a hard limit.
#
#   Goal of the data: map theta_deg = f(servo_us) across the FULL responsive
#   range, locate both saturation commands, and let the analysis pick the
#   center + linear operating band from data rather than by hand.
#
# CSV columns:
#   t_ms,t_s,sweep_idx,direction,servo_us,count,count_zero,count_delta,
#   theta_deg,theta_rad,segment,step_delta,is_stalled
#
# Hardware:
#   Servo PWM: GP15
#   Encoder A: GP18
#   Encoder B: GP19

import time
import math
from machine import Pin, PWM
import encoder

ENC_A_PIN = 18
ENC_B_PIN = 19
SERVO_PIN = 15

MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SERVO_FREQ_HZ = 50

# Data-derived neutral (static-map fit gave 1430.75 us; was a round 1450).
SERVO_CENTER_US = 1431

# Absolute safety clamps. The command is NEVER driven outside this window, even
# if stall detection fails. Set to the widest pulse you are willing to risk on
# this servo; 400/2600 brackets the standard (1000-2000) and extended
# (~500-2500) hobby ranges so saturation onset is captured on both ends.
SERVO_ABS_MIN_US = 400
SERVO_ABS_MAX_US = 2600

COUNTS_PER_REV = 2400.0

# ---- Stall / saturation detection -----------------------------------------
# A step is "stalled" if the encoder moves fewer than this many counts.
# Expected per-step motion ~ STEP_US * (COUNTS_PER_REV/360) * |gain_deg_per_us|
#   ~ 25 * 6.667 * 0.091 ~ 15 counts, so 4 counts is ~25% of nominal.
STALL_DELTA_COUNTS = 4
# Declare saturation after this many consecutive stalled steps, then stop
# advancing in that direction.
STALL_STEPS_LIMIT = 3
# Shorter settle once a step looks stalled, to limit time spent pushing a stop.
STALL_HOLD_MS = 120

PRE_ZERO_CENTER_SETTLE_MS = 2000
POST_ZERO_SETTLE_MS = 300
BASELINE_HOLD_MS = 1000

FILENAME = "servo_static_pwm_to_angle_sweep.csv"

STEP_US = 25
HOLD_MS = 250
N_SWEEPS = 2


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
        FILENAME = "servo_sweep_{}_to_{}us_trial.csv".format(
            SERVO_ABS_MIN_US,
            SERVO_ABS_MAX_US
        )

    if not FILENAME.endswith(".csv"):
        FILENAME = FILENAME + ".csv"


def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def count_to_theta_deg(count_delta):
    return count_delta / COUNTS_PER_REV * 360.0


def count_to_theta_rad(count_delta):
    return count_delta / COUNTS_PER_REV * 2.0 * math.pi


def log_row(f, start_ms, sweep_idx, direction, servo_us,
            count, count_zero, segment, step_delta, is_stalled):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, start_ms)
    t_s = t_ms / 1000.0

    count_delta = count - count_zero
    theta_deg = count_to_theta_deg(count_delta)
    theta_rad = count_to_theta_rad(count_delta)

    f.write("{},{:.3f},{},{},{},{},{},{},{:.6f},{:.8f},{},{},{}\n".format(
        t_ms,
        t_s,
        sweep_idx,
        direction,
        servo_us,
        count,
        count_zero,
        count_delta,
        theta_deg,
        theta_rad,
        segment,
        step_delta,
        1 if is_stalled else 0,
    ))
    f.flush()


def walk_to_saturation(servo, f, start_ms, sweep_idx, count_zero,
                       direction_sign, limit_us):
    """Walk the servo outward from center until the encoder stops responding.

    Steps by direction_sign*STEP_US toward limit_us. Returns the last command
    that still produced motion (the saturation command for this direction).
    """
    if direction_sign > 0:
        direction = "up_from_center"
    else:
        direction = "down_from_center"

    prev_count = encoder.get_count()
    last_moving_us = SERVO_CENTER_US
    servo_us = SERVO_CENTER_US
    consecutive_stalled = 0

    while True:
        servo_us = clamp(servo_us + direction_sign * STEP_US,
                         SERVO_ABS_MIN_US, SERVO_ABS_MAX_US)

        write_pwm_us(servo, servo_us)
        # Provisional full hold; shortened on the confirmation steps once we
        # suspect a stall so we do not sit on a hard stop.
        if consecutive_stalled > 0:
            time.sleep_ms(STALL_HOLD_MS)
        else:
            time.sleep_ms(HOLD_MS)

        count = encoder.get_count()
        step_delta = count - prev_count
        prev_count = count

        is_stalled = abs(step_delta) < STALL_DELTA_COUNTS

        log_row(f, start_ms, sweep_idx, direction, servo_us,
                count, count_zero, "sweep", step_delta, is_stalled)

        print("dir:", direction, "servo_us:", servo_us,
              "step_delta:", step_delta, "stalled:", is_stalled)

        if is_stalled:
            consecutive_stalled += 1
        else:
            consecutive_stalled = 0
            last_moving_us = servo_us

        if consecutive_stalled >= STALL_STEPS_LIMIT:
            print("  -> saturation detected. Last moving command:",
                  last_moving_us, "us")
            return last_moving_us

        if servo_us == limit_us:
            print("  -> reached absolute clamp", limit_us,
                  "us without stalling.")
            return servo_us


def return_to_center(servo):
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(HOLD_MS)


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
    print("Absolute command clamps:", SERVO_ABS_MIN_US, "to",
          SERVO_ABS_MAX_US, "us")

    start_ms = time.ticks_ms()

    with open(FILENAME, "w") as f:
        f.write(
            "t_ms,t_s,sweep_idx,direction,servo_us,"
            "count,count_zero,count_delta,theta_deg,theta_rad,segment,"
            "step_delta,is_stalled\n"
        )

        # Log center baseline.
        baseline_start_ms = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), baseline_start_ms) < BASELINE_HOLD_MS:
            count = encoder.get_count()
            log_row(f, start_ms, 0, "center", SERVO_CENTER_US,
                    count, count_zero, "baseline_center", 0, False)
            time.sleep_ms(HOLD_MS)

        pos_sat_us = None
        neg_sat_us = None

        for sweep_idx in range(1, N_SWEEPS + 1):
            print("Sweep", sweep_idx, "of", N_SWEEPS)

            # Up (toward SERVO_ABS_MAX_US) from center.
            print("Walking up from center...")
            return_to_center(servo)
            pos_sat_us = walk_to_saturation(
                servo, f, start_ms, sweep_idx, count_zero,
                +1, SERVO_ABS_MAX_US)

            # Down (toward SERVO_ABS_MIN_US) from center.
            print("Walking down from center...")
            return_to_center(servo)
            neg_sat_us = walk_to_saturation(
                servo, f, start_ms, sweep_idx, count_zero,
                -1, SERVO_ABS_MIN_US)

        print("Sweep complete.")
        print("Positive saturation command:", pos_sat_us, "us")
        print("Negative saturation command:", neg_sat_us, "us")
        print("Saved:", FILENAME)

finally:
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()
    print("Done.")
