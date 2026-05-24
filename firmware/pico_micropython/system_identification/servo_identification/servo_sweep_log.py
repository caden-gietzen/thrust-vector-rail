import time
import math
from machine import Pin, PWM
import encoder

ENC_A_PIN = 18
ENC_B_PIN = 19
SERVO_PIN = 15

MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SERVO_CENTER_US = 1450
SERVO_MIN_US = 800
SERVO_MAX_US = 2125
SERVO_FREQ_HZ = 50

COUNTS_PER_REV = 2400.0

PRE_ZERO_CENTER_SETTLE_MS = 2000
POST_ZERO_SETTLE_MS = 300
PRE_SWEEP_ENDPOINT_SETTLE_MS = 1500
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
            SERVO_MIN_US,
            SERVO_MAX_US
        )

    if not FILENAME.endswith(".csv"):
        FILENAME = FILENAME + ".csv"


def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))


def make_sweep_values():
    up = list(range(SERVO_MIN_US, SERVO_MAX_US + 1, STEP_US))
    down = list(range(SERVO_MAX_US, SERVO_MIN_US - 1, -STEP_US))
    return up, down


def count_to_theta_deg(count_delta):
    return count_delta / COUNTS_PER_REV * 360.0


def count_to_theta_rad(count_delta):
    return count_delta / COUNTS_PER_REV * 2.0 * math.pi


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

    print("Holding center baseline before sweep...")
    start_ms = time.ticks_ms()

    with open(FILENAME, "w") as f:
        f.write(
            "t_ms,t_s,sweep_idx,direction,servo_us,"
            "count,count_zero,count_delta,theta_deg,theta_rad,segment\n"
        )

        # Log center baseline.
        baseline_start_ms = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), baseline_start_ms) < BASELINE_HOLD_MS:
            now_ms = time.ticks_ms()
            t_ms = time.ticks_diff(now_ms, start_ms)
            t_s = t_ms / 1000.0

            count = encoder.get_count()
            count_delta = count - count_zero
            theta_deg = count_to_theta_deg(count_delta)
            theta_rad = count_to_theta_rad(count_delta)

            f.write("{},{:.3f},{},{},{},{},{},{},{:.6f},{:.8f},{}\n".format(
                t_ms,
                t_s,
                0,
                "center",
                SERVO_CENTER_US,
                count,
                count_zero,
                count_delta,
                theta_deg,
                theta_rad,
                "baseline_center"
            ))
            f.flush()

            time.sleep_ms(HOLD_MS)

        # Move to first endpoint and let the mechanics settle before logging sweep.
        print("Moving to first sweep command and settling...")
        write_pwm_us(servo, SERVO_MIN_US)
        time.sleep_ms(PRE_SWEEP_ENDPOINT_SETTLE_MS)

        up_values, down_values = make_sweep_values()

        for sweep_idx in range(1, N_SWEEPS + 1):
            print("Sweep", sweep_idx, "of", N_SWEEPS)

            for direction, sweep_values in [
                ("up", up_values),
                ("down", down_values),
            ]:
                for servo_us in sweep_values:
                    write_pwm_us(servo, servo_us)
                    time.sleep_ms(HOLD_MS)

                    now_ms = time.ticks_ms()
                    t_ms = time.ticks_diff(now_ms, start_ms)
                    t_s = t_ms / 1000.0

                    count = encoder.get_count()
                    count_delta = count - count_zero
                    theta_deg = count_to_theta_deg(count_delta)
                    theta_rad = count_to_theta_rad(count_delta)

                    f.write("{},{:.3f},{},{},{},{},{},{},{:.6f},{:.8f},{}\n".format(
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
                        "sweep"
                    ))

                    f.flush()

    print("Sweep complete.")
    print("Saved:", FILENAME)


finally:
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()

    if FILENAME is not None:
        print("Saved:", FILENAME)
    else:
        print("No file was created.")