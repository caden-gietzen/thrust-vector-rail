# servo_pwm_sweep_encoder_mismatch.py
#
# Purpose:
#   Bidirectional servo PWM sweep test to measure encoder final-position mismatch.
#
# Test idea:
#   For each amplitude and dwell time:
#       center -> zero encoder
#       command +A
#       command -A
#       command center
#       record final count mismatch
#
# Important:
#   Very short dwell times may exceed servo actuator bandwidth.
#   In that case, this test no longer cleanly tests encoder reliability because the
#   servo may not physically reach the commanded endpoints before the next command.
#
# Encoder:
#   2400 counts/rev assumed.
#
# Servo:
#   PWM on GP15 by default.

from machine import Pin, PWM
import time
import os

# -----------------------------
# USER CONFIG
# -----------------------------

SERVO_PIN = 15

CENTER_US = 1450
MIN_US = 450
MAX_US = 2450

PWM_FREQ_HZ = 50

COUNTS_PER_REV = 2400

# Recommended:
#   Keep dwell times inside the servo's practical bandwidth.
#   50 ms may be too aggressive if the servo cannot physically reach all three points.
AMPLITUDES_US = [50, 100, 200, 400, 600]
DWELL_TIMES_MS = [1000, 500, 250, 100, 50]

CYCLES_PER_CASE = 5

TEST_NAME = "servo_pwm_sweep_encoder_mismatch"

# Save directly to Pico root directory, not /data.
SAVE_TO_ROOT = True

# Optional frequency-plan export.
# For now, keep this False unless you specifically want an additional plan CSV.
EXPORT_FREQUENCY_PLAN = False
FREQUENCY_PLAN_NAME = "servo_pwm_sweep_frequency_plan"

# If True, script waits for user before each major test case.
PAUSE_BETWEEN_CASES = False

# If True, script prints every sample.
PRINT_EVERY_ROW = True

# Settling after centering before zeroing.
INITIAL_CENTER_SETTLE_MS = 1500

# Extra wait after commanding center at the end of each cycle.
FINAL_CENTER_SETTLE_MS = 500

# Delay between repeated cycles in one case.
INTER_CYCLE_DELAY_MS = 250

# Encoder pins/settings for custom C module.
ENCODER_PIN_A = 18
ENCODER_PIN_B = 19
ENCODER_MAX_STEP_RATE = 1000000
ENCODER_DRAIN_HZ = 10000


# -----------------------------
# ENCODER ADAPTER
# -----------------------------

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

encoder_backend = None

try:
    import pico_encoder as enc
    encoder_backend = "pico_encoder"
except ImportError:
    try:
        import encoder as enc
        encoder_backend = "encoder"
    except ImportError:
        enc = None
        encoder_backend = None


def encoder_init():
    if enc is None:
        raise RuntimeError("No encoder module found. Update encoder adapter section.")

    if hasattr(enc, "init_configured"):
        enc.init_configured(
            ENCODER_PIN_A,
            ENCODER_PIN_B,
            ENCODER_MAX_STEP_RATE,
            ENCODER_DRAIN_HZ,
        )
    elif hasattr(enc, "init"):
        enc.init()
    else:
        raise RuntimeError("Encoder module found, but no init/init_configured function exists.")


def encoder_zero():
    if hasattr(enc, "zero"):
        enc.zero()
    elif hasattr(enc, "zero_count"):
        enc.zero_count()
    elif hasattr(enc, "set_count"):
        enc.set_count(0)
    else:
        raise RuntimeError("Encoder module has no zero function.")


def encoder_get_count():
    if hasattr(enc, "get_count"):
        return int(enc.get_count())
    elif hasattr(enc, "count"):
        return int(enc.count())
    else:
        raise RuntimeError("Encoder module has no get_count/count function.")


# -----------------------------
# SERVO HELPERS
# -----------------------------

apply_external_config_if_present()

if not FILENAME.endswith(".csv"):
    FILENAME = FILENAME + ".csv"

servo = PWM(Pin(SERVO_PIN))
servo.freq(PWM_FREQ_HZ)


def clamp_pwm_us(pwm_us):
    if pwm_us < MIN_US:
        return MIN_US
    if pwm_us > MAX_US:
        return MAX_US
    return int(pwm_us)


def write_servo_us(pwm_us):
    pwm_us = clamp_pwm_us(pwm_us)

    # 50 Hz period = 20,000 us.
    period_us = int(1_000_000 / PWM_FREQ_HZ)
    duty = int((pwm_us / period_us) * 65535)

    servo.duty_u16(duty)
    return pwm_us


def servo_center():
    return write_servo_us(CENTER_US)


# -----------------------------
# FILE HELPERS
# -----------------------------

def make_filename(base_name, ext=".csv"):
    return "{}_{}{}".format(base_name, time.ticks_ms(), ext)


def make_output_path(base_name):
    # Intentionally save to Pico root directory.
    return make_filename(base_name, ".csv")


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def make_unique_output_path(base_name):
    path = make_output_path(base_name)

    # Very unlikely to collide because ticks_ms is included,
    # but this keeps the file creation robust.
    while file_exists(path):
        path = make_output_path(base_name)

    return path


# -----------------------------
# OPTIONAL FREQUENCY PLAN EXPORT
# -----------------------------

def dwell_to_command_frequency_hz(dwell_ms):
    # One bidirectional sweep has three commanded holds:
    #   +A, -A, center
    # This is not a sinusoidal frequency. It is just a rough command-cycle rate.
    cycle_time_s = (3.0 * dwell_ms) / 1000.0
    if cycle_time_s <= 0:
        return 0.0
    return 1.0 / cycle_time_s


def export_frequency_plan_if_enabled():
    if not EXPORT_FREQUENCY_PLAN:
        return None

    path = make_unique_output_path(FREQUENCY_PLAN_NAME)

    header = [
        "amplitude_us",
        "dwell_ms",
        "command_cycle_time_s",
        "approx_command_cycle_hz",
        "note",
    ]

    with open(path, "w") as f:
        f.write(",".join(header) + "\n")

        for dwell_ms in DWELL_TIMES_MS:
            for amplitude_us in AMPLITUDES_US:
                cycle_time_s = (3.0 * dwell_ms) / 1000.0
                approx_hz = dwell_to_command_frequency_hz(dwell_ms)

                note = "not_servo_response_frequency"

                row = [
                    amplitude_us,
                    dwell_ms,
                    "{:.6f}".format(cycle_time_s),
                    "{:.6f}".format(approx_hz),
                    note,
                ]

                f.write(",".join([str(x) for x in row]) + "\n")

        f.flush()

    print("Frequency plan saved:", path)
    return path


# -----------------------------
# TEST HELPERS
# -----------------------------

def wait_ms(ms):
    time.sleep_ms(int(ms))


def read_count_after_settle(dwell_ms):
    wait_ms(dwell_ms)
    return encoder_get_count()


def run_one_cycle(amplitude_us, dwell_ms, cycle_index, csv_file):
    positive_pwm = clamp_pwm_us(CENTER_US + amplitude_us)
    negative_pwm = clamp_pwm_us(CENTER_US - amplitude_us)

    # Start centered before every bidirectional cycle.
    servo_center()
    wait_ms(INITIAL_CENTER_SETTLE_MS)

    # Encoder is zeroed before every individual cycle.
    start_count_before_zero = encoder_get_count()
    encoder_zero()
    wait_ms(50)

    start_count = encoder_get_count()

    # Command first endpoint.
    t0 = time.ticks_ms()
    write_servo_us(positive_pwm)
    count_pos = read_count_after_settle(dwell_ms)
    t_pos = time.ticks_ms()

    # Command opposite endpoint.
    write_servo_us(negative_pwm)
    count_neg = read_count_after_settle(dwell_ms)
    t_neg = time.ticks_ms()

    # Return to center.
    write_servo_us(CENTER_US)
    wait_ms(dwell_ms)
    wait_ms(FINAL_CENTER_SETTLE_MS)
    final_count = encoder_get_count()
    t_final = time.ticks_ms()

    final_mismatch_counts = final_count - start_count
    final_mismatch_revs = final_mismatch_counts / COUNTS_PER_REV

    positive_delta_counts = count_pos - start_count
    negative_delta_counts = count_neg - count_pos
    return_delta_counts = final_count - count_neg

    total_elapsed_ms = time.ticks_diff(t_final, t0)

    approx_command_cycle_hz = dwell_to_command_frequency_hz(dwell_ms)

    row = [
        time.ticks_ms(),
        amplitude_us,
        dwell_ms,
        cycle_index,
        CENTER_US,
        positive_pwm,
        negative_pwm,
        start_count_before_zero,
        start_count,
        count_pos,
        count_neg,
        final_count,
        positive_delta_counts,
        negative_delta_counts,
        return_delta_counts,
        final_mismatch_counts,
        final_mismatch_revs,
        total_elapsed_ms,
        "{:.6f}".format(approx_command_cycle_hz),
    ]

    line = ",".join([str(x) for x in row]) + "\n"
    csv_file.write(line)
    csv_file.flush()

    if PRINT_EVERY_ROW:
        print(
            "A={:4d} us | dwell={:4d} ms | cycle={} | "
            "pos={:7d} neg={:7d} final={:7d} | mismatch={:+6d} counts".format(
                amplitude_us,
                dwell_ms,
                cycle_index,
                count_pos,
                count_neg,
                final_count,
                final_mismatch_counts,
            )
        )

    return final_mismatch_counts


def run_test():
    print("")
    print("Servo PWM sweep encoder mismatch test")
    print("-------------------------------------")
    print("Servo pin:", SERVO_PIN)
    print("Center PWM:", CENTER_US, "us")
    print("Counts/rev:", COUNTS_PER_REV)
    print("Encoder backend:", encoder_backend)
    print("Amplitudes:", AMPLITUDES_US)
    print("Dwell times:", DWELL_TIMES_MS)
    print("Cycles per case:", CYCLES_PER_CASE)
    print("Saving location: Pico root directory")
    print("Export frequency plan:", EXPORT_FREQUENCY_PLAN)
    print("")

    print("Initializing encoder...")
    encoder_init()

    print("Centering servo...")
    servo_center()
    wait_ms(INITIAL_CENTER_SETTLE_MS)

    print("Zeroing encoder at center...")
    encoder_zero()
    wait_ms(100)

    export_frequency_plan_if_enabled()

    output_path = make_unique_output_path(TEST_NAME)
    print("Saving data to:", output_path)
    print("")

    header = [
        "ticks_ms",
        "amplitude_us",
        "dwell_ms",
        "cycle_index",
        "center_pwm_us",
        "positive_pwm_us",
        "negative_pwm_us",
        "start_count_before_zero",
        "start_count",
        "count_pos",
        "count_neg",
        "final_count",
        "positive_delta_counts",
        "negative_delta_counts",
        "return_delta_counts",
        "final_mismatch_counts",
        "final_mismatch_revs",
        "total_elapsed_ms",
        "approx_command_cycle_hz",
    ]

    with open(output_path, "w") as f:
        f.write(",".join(header) + "\n")
        f.flush()

        for dwell_ms in DWELL_TIMES_MS:
            for amplitude_us in AMPLITUDES_US:

                if CENTER_US + amplitude_us > MAX_US or CENTER_US - amplitude_us < MIN_US:
                    print("Skipping amplitude", amplitude_us, "us because it exceeds PWM limits.")
                    continue

                print("")
                print("Case: amplitude =", amplitude_us, "us | dwell =", dwell_ms, "ms")
                print(
                    "Approx command-cycle frequency:",
                    "{:.3f}".format(dwell_to_command_frequency_hz(dwell_ms)),
                    "Hz",
                )

                if PAUSE_BETWEEN_CASES:
                    input("Press Enter to run this case...")

                mismatches = []

                for cycle in range(1, CYCLES_PER_CASE + 1):
                    mismatch = run_one_cycle(amplitude_us, dwell_ms, cycle, f)
                    mismatches.append(mismatch)
                    wait_ms(INTER_CYCLE_DELAY_MS)

                avg_mismatch = sum(mismatches) / len(mismatches)
                max_abs_mismatch = max([abs(x) for x in mismatches])

                print(
                    "Summary | A={} us | dwell={} ms | avg mismatch={:+.2f} counts | max abs mismatch={} counts".format(
                        amplitude_us,
                        dwell_ms,
                        avg_mismatch,
                        max_abs_mismatch,
                    )
                )

    servo_center()
    print("")
    print("Test complete.")
    print("Saved:", output_path)


# -----------------------------
# MAIN
# -----------------------------

try:
    run_test()
finally:
    servo_center()
    print("Servo returned to center.")