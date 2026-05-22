# servo_half_revolution_sweep_log.py
#
# Purpose:
#   Sweep servo from approximately 0 deg to 180 deg using the known command range:
#
#       450 us  -> approx 0 deg
#       2450 us -> approx 180 deg
#
#   With a 1:1 GT2 belt drive between servo pulley and encoder pulley:
#
#       180 deg = 0.5 revolution
#
#   If the encoder is 600 PPR and quadrature decoding counts all 4 edges:
#
#       600 PPR * 4 = 2400 counts/rev
#
#   Therefore, over 180 degrees, expected count change is:
#
#       2400 * 0.5 = 1200 counts
#
# CSV columns:
#   t_ms,t_s,sweep_idx,direction,servo_us,count,count_delta_from_start,
#   expected_deg_from_command,estimated_counts_per_rev
#
# Hardware:
#   Servo PWM: GP15
#   Encoder A: GP18
#   Encoder B: GP19

import time
from machine import Pin, PWM
import encoder


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
# Sweep settings
# ============================================================

# Prior test result:
#   450 us  -> approx 0 deg
#   2450 us -> approx 180 deg
SERVO_0_DEG_US = 450
SERVO_180_DEG_US = 2450

SERVO_CENTER_US = 1450

# Use small enough steps to avoid violent motion,
# but large enough that the test does not take forever.
STEP_US = 25

# Hold after each command so the servo can settle.
# Increase if count still changes after logging.
HOLD_MS = 300

# Number of up/down sweep cycles.
N_SWEEPS = 2

# Optional extra settle at each endpoint before logging summary.
ENDPOINT_SETTLE_MS = 1000

FILENAME = "servo_450_to_2450_half_rev_sweep.csv"


# ============================================================
# Expected encoder relationship
# ============================================================

EXPECTED_COUNTS_PER_REV = 2400
EXPECTED_COUNTS_HALF_REV = EXPECTED_COUNTS_PER_REV / 2.0
EXPECTED_DEG_RANGE = 180.0


# ============================================================
# Helper functions
# ============================================================

def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def command_to_expected_deg(servo_us):
    """
    Linear approximation based on prior calibration:
        450 us  -> 0 deg
        2450 us -> 180 deg
    """
    alpha = (servo_us - SERVO_0_DEG_US) / float(SERVO_180_DEG_US - SERVO_0_DEG_US)
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * EXPECTED_DEG_RANGE


def make_sweep_values():
    up = list(range(SERVO_0_DEG_US, SERVO_180_DEG_US + 1, STEP_US))
    down = list(range(SERVO_180_DEG_US, SERVO_0_DEG_US - 1, -STEP_US))
    return up, down


def estimate_counts_per_rev(count_delta, expected_deg):
    """
    If expected_deg is 180 deg and count_delta is about 1200,
    this should return about 2400 counts/rev.
    """
    if expected_deg == 0:
        return 0.0

    rev_fraction = expected_deg / 360.0

    if rev_fraction == 0:
        return 0.0

    return abs(count_delta) / rev_fraction


def log_sample(
    f,
    start_ms,
    sweep_idx,
    direction,
    servo_us,
    count_start,
):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, start_ms)
    t_s = t_ms / 1000.0

    count = encoder.get_count()
    count_delta = count - count_start

    expected_deg = command_to_expected_deg(servo_us)
    estimated_cpr = estimate_counts_per_rev(count_delta, expected_deg)

    f.write("{},{:.3f},{},{},{},{},{},{:.3f},{:.3f}\n".format(
        t_ms,
        t_s,
        sweep_idx,
        direction,
        servo_us,
        count,
        count_delta,
        expected_deg,
        estimated_cpr,
    ))

    f.flush()

    print(
        "sweep:", sweep_idx,
        "dir:", direction,
        "servo_us:", servo_us,
        "count:", count,
        "delta:", count_delta,
        "expected_deg:", round(expected_deg, 2),
        "est_counts_per_rev:", round(estimated_cpr, 1),
    )

    return count, count_delta, estimated_cpr


# ============================================================
# Main
# ============================================================

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

try:
    print("Initializing encoder...")
    print("Encoder pins:", ENC_A_PIN, ENC_B_PIN)
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)

    print()
    print("Centering servo...")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(1000)

    print()
    print("Moving to 0-degree endpoint command:", SERVO_0_DEG_US, "us")
    write_pwm_us(servo, SERVO_0_DEG_US)
    time.sleep_ms(ENDPOINT_SETTLE_MS)

    print()
    print("Zeroing encoder at 0-degree endpoint.")
    encoder.zero()
    time.sleep_ms(100)

    count_start = encoder.get_count()

    print("Encoder zeroed.")
    print("Start count:", count_start)

    print()
    print("Expected result:")
    print("  450 us to 2450 us should be approximately 180 degrees.")
    print("  Expected count change over 180 degrees:", EXPECTED_COUNTS_HALF_REV)
    print("  Expected full-revolution count estimate:", EXPECTED_COUNTS_PER_REV)
    print()
    print("Logging to:", FILENAME)
    print("Starting sweep.")

    start_ms = time.ticks_ms()

    up_values, down_values = make_sweep_values()

    with open(FILENAME, "w") as f:
        f.write(
            "t_ms,t_s,sweep_idx,direction,servo_us,count,"
            "count_delta_from_start,expected_deg_from_command,"
            "estimated_counts_per_rev\n"
        )

        for sweep_idx in range(1, N_SWEEPS + 1):
            print()
            print("Sweep", sweep_idx, "of", N_SWEEPS)

            # Up: 0 deg -> 180 deg
            for servo_us in up_values:
                write_pwm_us(servo, servo_us)
                time.sleep_ms(HOLD_MS)

                log_sample(
                    f=f,
                    start_ms=start_ms,
                    sweep_idx=sweep_idx,
                    direction="up",
                    servo_us=servo_us,
                    count_start=count_start,
                )

            # Extra endpoint sample at 180 deg after settling.
            print("Settling at 180-degree endpoint...")
            time.sleep_ms(ENDPOINT_SETTLE_MS)
            count_180, delta_180, cpr_180 = log_sample(
                f=f,
                start_ms=start_ms,
                sweep_idx=sweep_idx,
                direction="up_endpoint_settled",
                servo_us=SERVO_180_DEG_US,
                count_start=count_start,
            )

            print()
            print("180-degree endpoint summary:")
            print("  count_delta:", delta_180)
            print("  expected half-rev counts:", EXPECTED_COUNTS_HALF_REV)
            print("  estimated counts/rev:", cpr_180)

            # Down: 180 deg -> 0 deg
            for servo_us in down_values:
                write_pwm_us(servo, servo_us)
                time.sleep_ms(HOLD_MS)

                log_sample(
                    f=f,
                    start_ms=start_ms,
                    sweep_idx=sweep_idx,
                    direction="down",
                    servo_us=servo_us,
                    count_start=count_start,
                )

            # Extra endpoint sample back at 0 deg after settling.
            print("Settling back at 0-degree endpoint...")
            time.sleep_ms(ENDPOINT_SETTLE_MS)
            count_0, delta_0, cpr_0 = log_sample(
                f=f,
                start_ms=start_ms,
                sweep_idx=sweep_idx,
                direction="down_endpoint_settled",
                servo_us=SERVO_0_DEG_US,
                count_start=count_start,
            )

            print()
            print("Return-to-zero endpoint summary:")
            print("  count_delta:", delta_0)
            print("  Ideally near 0 if there is little backlash/slip/hysteresis.")

    print()
    print("Sweep complete.")
    print("Saved:", FILENAME)

finally:
    print()
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()

    print("Done.")