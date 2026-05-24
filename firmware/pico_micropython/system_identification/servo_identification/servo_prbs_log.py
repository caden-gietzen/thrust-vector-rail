# servo_prbs_log.py
#
# Purpose:
#   Run a PRBS-style servo command test and log encoder-measured servo angle.
#
# Goal:
#   Generate data that can estimate:
#       - approximate servo bandwidth
#       - approximate delay
#       - first-order vs second-order behavior
#       - rate limiting / saturation
#       - useful frequency range for later PRPS testing
#
# Hardware:
#   Servo PWM: GP15
#   Encoder A: GP18
#   Encoder B: GP19
#
# Mechanical setup:
#   Servo pulley and encoder pulley are both 20T GT2.
#   Therefore, assume 1:1 servo-to-encoder rotation.
#
# Encoder assumption:
#   600 PPR encoder with 4x quadrature decoding:
#
#       600 * 4 = 2400 counts/rev
#
# Angle conversion:
#
#       theta_rad = count_delta / 2400 * 2*pi
#       theta_deg = count_delta / 2400 * 360
#
# CSV columns:
#   t_ms,t_s,run_name,segment,servo_us,command_delta_us,count_delta,theta_rad,sample_index
#
# Notes:
#   PRBS = Pseudo-Random Binary Signal.
#   This commands the servo between center +/- amplitude_us.
#
#   This is NOT the final best identification signal.
#   It is a scouting test before PRPS.

import time
from machine import Pin, PWM
import encoder
import urandom
import math


# ============================================================
# Hardware settings
# ============================================================

ENC_A_PIN = 18
ENC_B_PIN = 19
SERVO_PIN = 15

MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SERVO_FREQ_HZ = 50


# ============================================================
# Servo / encoder calibration
# ============================================================

COUNTS_PER_REV = 2400.0

RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV

# From prior static calibration:
#   450 us  -> approx 0 deg
#   2450 us -> approx 180 deg
#
# Therefore:
#   2000 us -> 180 deg
#   1 us    -> 0.09 deg
#
# In radians:
#   0.09 deg/us * pi/180 = 0.0015708 rad/us
K_THETA_RAD_PER_US = math.pi / 2000.0
K_THETA_DEG_PER_US = 180.0 / 2000.0

SERVO_CENTER_US = 1450

# Hard safety bounds.
# These are intentionally narrower than the full 450-2450 us range.
# For dynamics ID, avoid endpoint saturation and mechanical hard stops.
SERVO_HARD_MIN_US = 900
SERVO_HARD_MAX_US = 2000


# ============================================================
# PRBS test settings
# ============================================================

LOG_FILE_BASE = "servo_prbs"

# For repeatability.
PRBS_SEED = 3001

# Each run:
#   (run_name, center_us, amplitude_us, bit_hold_samples)
#
# COMMAND_DT_MS defines the sample/update period.
#
# bit_hold_samples defines how long each PRBS bit is held:
#
#   bit_hold_time_s = COMMAND_DT_MS/1000 * bit_hold_samples
#
# Smaller bit_hold_samples injects faster switching.
# Larger bit_hold_samples gives the servo more time to respond.
#
# Recommended first pass:
#   Start with 20 ms command/update period.
#   Use multiple bit hold times to excite different frequency regions.
#
# Warning:
#   Do not use too large of an amplitude at first. If the servo rate-limits,
#   the Bode estimate will look like "servo dynamics", but it is actually
#   large-signal nonlinear behavior.
TEST_RUNS = [
    # run_name, center_us, amplitude_us, bit_hold_samples
    # command switches between center_us - amplitude_us and center_us + amplitude_us

    ("local_amp100_hold10", 1450, 100, 10),  # 1350 <-> 1550 us
    ("local_amp100_hold5",  1450, 100, 5),   # 1350 <-> 1550 us
    ("local_amp100_hold3",  1450, 100, 3),   # 1350 <-> 1550 us
]

COMMAND_DT_MS = 20

# Duration per run.
# 120 s gives enough data for a rough frequency response estimate.
RUN_DURATION_S = 60

# Settle at center before each run.
PRE_RUN_SETTLE_MS = 1000

# Baseline logs before and after each PRBS run.
BASELINE_HOLD_MS = 500

# Startup safety.
INITIAL_COUNTDOWN_S = 5

# Print progress.
PRINT_EVERY_N_SAMPLES = 100

# If True, the run order is fixed.
# If False, runs are shuffled so thermal/mechanical drift is less biased.
FIXED_RUN_ORDER = True


# ============================================================
# Helper functions
# ============================================================

def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def write_pwm_us(pwm, pulse_us):
    pulse_us = int(clamp(pulse_us, SERVO_HARD_MIN_US, SERVO_HARD_MAX_US))
    pwm.duty_ns(int(pulse_us * 1000))


def countdown(message, seconds):
    print()
    print(message)
    for remaining in range(seconds, 0, -1):
        print(remaining)
        time.sleep(1)
    print("Starting now.")


def seed_urandom(seed_value, label):
    try:
        urandom.seed(seed_value)
        print(label, "seed:", seed_value)
    except Exception:
        print("NOTE: urandom.seed unavailable for", label)


def random_bit():
    # Returns -1 or +1.
    if urandom.getrandbits(1):
        return 1
    return -1


def maybe_shuffle_runs(runs, seed_value):
    if FIXED_RUN_ORDER:
        return list(runs)

    seed_urandom(seed_value, "Run-order")

    runs = list(runs)
    for i in range(len(runs) - 1, 0, -1):
        j = urandom.getrandbits(16) % (i + 1)
        runs[i], runs[j] = runs[j], runs[i]

    return runs


def make_log_filename(seed_value):
    return "{}_seed{}.csv".format(LOG_FILE_BASE, int(seed_value))


def count_to_theta_rad(count_delta):
    return count_delta * RAD_PER_COUNT


def count_to_theta_deg(count_delta):
    return count_delta * DEG_PER_COUNT


def command_delta_to_theta_rad(command_delta_us):
    return command_delta_us * K_THETA_RAD_PER_US


def command_delta_to_theta_deg(command_delta_us):
    return command_delta_us * K_THETA_DEG_PER_US


# ============================================================
# Logging helpers
# ============================================================

sample_counter = 0


def write_sample(
    f,
    t0_ms,
    run_name,
    segment,
    servo_us,
    count_zero,
    center_us,
    prbs_state,
    bit_index,
    sample_index,
):
    global sample_counter

    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, t0_ms)
    t_s = t_ms / 1000.0

    count = encoder.get_count()
    count_delta = count - count_zero

    theta_rad = count_to_theta_rad(count_delta)
    command_delta_us = servo_us - center_us

    f.write(
        "{},{:.3f},{},{},{},{},{},{:.9f},{}\n".format(
            t_ms,
            t_s,
            run_name,
            segment,
            int(servo_us),
            int(command_delta_us),
            int(count_delta),
            theta_rad,
            int(sample_index),
        )
    )

    sample_counter += 1

    if sample_counter % PRINT_EVERY_N_SAMPLES == 0:
        print(
            "sample", sample_counter,
            "run", run_name,
            "servo_us", int(servo_us),
            "count_delta", int(count_delta),
            "theta_rad", round(theta_rad, 4),
        )


def hold_and_log(
    f,
    servo,
    t0_ms,
    run_name,
    segment,
    servo_us,
    duration_ms,
    count_zero,
    center_us,
):
    write_pwm_us(servo, servo_us)

    start_ms = time.ticks_ms()
    sample_index = 0

    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        loop_start_ms = time.ticks_ms()

        write_sample(
            f=f,
            t0_ms=t0_ms,
            run_name=run_name,
            segment=segment,
            servo_us=servo_us,
            count_zero=count_zero,
            center_us=center_us,
            prbs_state=0,
            bit_index=-1,
            sample_index=sample_index,
        )

        sample_index += 1

        elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
        remaining_ms = COMMAND_DT_MS - elapsed_ms

        if remaining_ms > 0:
            time.sleep_ms(remaining_ms)


def run_prbs_sequence(
    f,
    servo,
    t0_ms,
    run_name,
    center_us,
    amplitude_us,
    bit_hold_samples,
    count_zero,
    seed_value,
):
    print()
    print("============================================================")
    print("RUN:", run_name)
    print("============================================================")
    print("Center us:", center_us)
    print("Amplitude us:", amplitude_us)
    print("Command bounds:", center_us - amplitude_us, "to", center_us + amplitude_us)
    print("Command dt ms:", COMMAND_DT_MS)
    print("Bit hold samples:", bit_hold_samples)
    print("Bit hold time s:", bit_hold_samples * COMMAND_DT_MS / 1000.0)
    print("Run duration s:", RUN_DURATION_S)

    low = center_us - amplitude_us
    high = center_us + amplitude_us

    if low < SERVO_HARD_MIN_US or high > SERVO_HARD_MAX_US:
        raise ValueError("Requested PRBS command exceeds hard servo safety bounds.")

    print("Pre-run baseline at center...")
    hold_and_log(
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        run_name=run_name,
        segment="baseline_pre",
        servo_us=center_us,
        duration_ms=BASELINE_HOLD_MS,
        count_zero=count_zero,
        center_us=center_us,
    )

    print("Settling at center...")
    hold_and_log(
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        run_name=run_name,
        segment="settle",
        servo_us=center_us,
        duration_ms=PRE_RUN_SETTLE_MS,
        count_zero=count_zero,
        center_us=center_us,
    )

    print("Running PRBS...")

    seed_urandom(seed_value, "PRBS")

    total_samples = int((RUN_DURATION_S * 1000) / COMMAND_DT_MS)

    current_state = random_bit()
    bit_index = 0

    for sample_index in range(total_samples):
        loop_start_ms = time.ticks_ms()

        if sample_index % bit_hold_samples == 0:
            current_state = random_bit()
            bit_index += 1

        servo_us = center_us + current_state * amplitude_us
        servo_us = int(clamp(servo_us, SERVO_HARD_MIN_US, SERVO_HARD_MAX_US))

        write_pwm_us(servo, servo_us)

        write_sample(
            f=f,
            t0_ms=t0_ms,
            run_name=run_name,
            segment="prbs",
            servo_us=servo_us,
            count_zero=count_zero,
            center_us=center_us,
            prbs_state=current_state,
            bit_index=bit_index,
            sample_index=sample_index,
        )

        elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
        remaining_ms = COMMAND_DT_MS - elapsed_ms

        if remaining_ms > 0:
            time.sleep_ms(remaining_ms)

    print("Post-run baseline at center...")
    hold_and_log(
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        run_name=run_name,
        segment="baseline_post",
        servo_us=center_us,
        duration_ms=BASELINE_HOLD_MS,
        count_zero=count_zero,
        center_us=center_us,
    )

    f.flush()

    print("Finished:", run_name)


# ============================================================
# External config override
# ============================================================

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


# ============================================================
# Main
# ============================================================

apply_external_config_if_present()

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

filename = make_log_filename(PRBS_SEED)

try:
    print()
    print("Servo PRBS logger")
    print("Servo pin:", SERVO_PIN)
    print("Encoder pins:", ENC_A_PIN, ENC_B_PIN)
    print("Counts per rev:", COUNTS_PER_REV)
    print("Command dt ms:", COMMAND_DT_MS)
    print("Log file:", filename)

    print()
    print("Initializing encoder...")
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)

    print("Centering servo...")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(1000)

    countdown("Keep mechanism clear. PRBS test begins after countdown.", INITIAL_COUNTDOWN_S)

    print("Zeroing encoder at center.")
    encoder.zero()
    time.sleep_ms(100)

    count_zero = encoder.get_count()
    print("Zero count:", count_zero)

    run_order_seed = PRBS_SEED + 50000
    ordered_runs = maybe_shuffle_runs(TEST_RUNS, run_order_seed)

    t0_ms = time.ticks_ms()

    with open(filename, "w") as f:
        f.write(
            "t_ms,t_s,run_name,segment,servo_us,command_delta_us,"
            "count_delta,theta_rad,sample_index\n"
        )

        for run_idx in range(len(ordered_runs)):
            run_name, center_us, amplitude_us, bit_hold_samples = ordered_runs[run_idx]

            run_seed = PRBS_SEED + run_idx

            run_prbs_sequence(
                f=f,
                servo=servo,
                t0_ms=t0_ms,
                run_name=run_name,
                center_us=center_us,
                amplitude_us=amplitude_us,
                bit_hold_samples=bit_hold_samples,
                count_zero=count_zero,
                seed_value=run_seed,
            )

    print()
    print("All PRBS runs complete.")
    print("Saved:", filename)

finally:
    print()
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()
    print("Done.")