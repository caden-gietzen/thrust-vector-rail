# friction_sweep_log.py
#
# Purpose:
#   Fixed-angle, fixed-ESC-command friction identification run.
#   Servo is held at a target angle while a constant ESC command drives the cart
#   from an end stop. The run ends when encoder counts stop changing for
#   HALT_DURATION_S seconds (end stop reached or stiction dominates).
#
#   No load cell is used — thrust force is predicted in analysis from the
#   identified thrust dynamic model (K=0.00414 N/µs, tau=78.1 ms, L=25.2 ms).
#
# Output CSV columns:
#   t_ms,t_s,set_index,segment,phase,
#   servo_us,esc_pwm_us,
#   count,count_delta,count_from_start
#
# Hardware:
#   Servo PWM: GP15
#   Encoder A: GP18
#   Encoder B: GP19
#   ESC PWM:   GP13, GP14
#
# Structure note:
#   The main execution path is:
#
#       main()
#           cfg = finalize_config(make_config())
#           print_startup_banner(cfg)
#           setup_hardware(cfg)
#           run_all_acquisition_sets(cfg, servo, escs, enc_state)
#           safe_shutdown(cfg, servo, escs)
#
# Orchestration note:
#   Run via tools/run_pico_and_pull.py with friction_sweep_log.orchestrate.json.
#   Each segment overrides SERVO_ANGLE_DEG and ESC_COMMAND_US via run_config.json.
#
# Encoder calibration:
#   64.810 counts/mm = 64,810.4 counts/m (measured).
#   See experiments/encoder_calibration/results.md.

import time
from machine import Pin, PWM
import encoder


# ============================================================
# Configuration
# ============================================================


def make_config():
    """
    Return all user-editable settings.

    The orchestrator uploads /run_config.json before each segment to override
    SERVO_ANGLE_DEG and ESC_COMMAND_US (at minimum) for that run.
    """
    cfg = {
        # --------------------------------------------------------
        # File / acquisition
        # --------------------------------------------------------
        "LOG_FILE_BASE": "friction_sweep_log",
        "NUM_ACQUISITION_SETS": 1,
        "COOLDOWN_BETWEEN_SETS_MS": 0,

        # --------------------------------------------------------
        # Servo
        # --------------------------------------------------------
        "SERVO_PIN": 15,
        "SERVO_FREQ_HZ": 50,
        "SERVO_HARD_MIN_US": 1000,
        "SERVO_HARD_MAX_US": 2000,
        # From servo identification (experiments/servo_identification/results.md):
        "SERVO_NEUTRAL_US": 1350,
        "SERVO_DEG_PER_US": 0.091092,   # |static gain|, deg/µs

        # --------------------------------------------------------
        # ESC
        # --------------------------------------------------------
        "ESC_PINS": [13, 14],
        "ESC_FREQ_HZ": 50,
        "PWM_SAFE_US": 1000,
        "PWM_HARD_MIN_US": 1100,
        "PWM_HARD_MAX_US": 1950,

        # --------------------------------------------------------
        # Encoder
        # --------------------------------------------------------
        "ENC_A_PIN": 18,
        "ENC_B_PIN": 19,
        "MAX_STEP_RATE": 1000000,
        "DRAIN_HZ": 10000,

        # --------------------------------------------------------
        # Per-segment run parameters (overridden via run_config.json)
        # --------------------------------------------------------
        "SERVO_ANGLE_DEG": 20.0,        # target vectoring angle; positive or negative
        "ESC_COMMAND_US": 1300,         # fixed ESC command held during run phase

        "MAX_RUN_DURATION_S": 30,       # hard time limit; run stops even if no halt
        "USER_PROMPT_WAIT_S": 10,       # countdown while operator positions cart
        "SERVO_SETTLE_MS": 1000,        # dwell at target angle before applying thrust
        "COMMAND_UPDATE_DT_MS": 20,     # sample interval (50 Hz)

        # --------------------------------------------------------
        # Halt / stiction detection
        # --------------------------------------------------------
        # Halt is declared when the ring buffer of recent encoder counts has a
        # max-minus-min below HALT_THRESHOLD_COUNTS. If the total displacement
        # from run start is also below STICTION_NO_MOTION_COUNTS, the halt is
        # classified as stiction (motor didn't overcome static friction).
        "HALT_DURATION_S": 5,
        "HALT_THRESHOLD_COUNTS": 15,
        "STICTION_NO_MOTION_COUNTS": 15,

        "REPOSITION_TIMEOUT_S": 30,     # max time for end-stop reposition before giving up

        "PRINT_EVERY_N_SAMPLES": 50,
    }

    cfg = apply_external_config_if_present(cfg)
    return cfg


# ============================================================
# External config loading
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


def normalize_external_config(external):
    if external is None:
        return None

    if isinstance(external, dict) and "config" in external:
        maybe_config = external["config"]
        if isinstance(maybe_config, dict):
            return maybe_config

    if isinstance(external, dict):
        return external

    raise ValueError("External run config must be a JSON object/dictionary.")


def apply_external_config_if_present(cfg):
    config_path = "/run_config.json"
    external = load_json_file_if_exists(config_path)

    if external is None:
        config_path = "run_config.json"
        external = load_json_file_if_exists(config_path)

    external = normalize_external_config(external)

    if external is None:
        return cfg

    print("Loaded external run config:", config_path)

    for key in external:
        cfg[key] = external[key]

    return cfg


def finalize_config(cfg):
    # Servo command for target angle.
    # Static map: theta_deg = -0.091092 * u + 130.33
    #   → u = SERVO_NEUTRAL_US - theta_deg / SERVO_DEG_PER_US
    servo_target = cfg["SERVO_NEUTRAL_US"] - cfg["SERVO_ANGLE_DEG"] / cfg["SERVO_DEG_PER_US"]
    servo_target = int(servo_target + 0.5)
    servo_target = max(cfg["SERVO_HARD_MIN_US"], min(cfg["SERVO_HARD_MAX_US"], servo_target))
    cfg["SERVO_TARGET_US"] = servo_target

    cfg["HALT_WINDOW_SAMPLES"] = max(
        1, int(cfg["HALT_DURATION_S"] * 1000 / cfg["COMMAND_UPDATE_DT_MS"])
    )
    cfg["MAX_RUN_SAMPLES"] = int(
        cfg["MAX_RUN_DURATION_S"] * 1000 / cfg["COMMAND_UPDATE_DT_MS"]
    )

    # Human-readable segment label logged in each CSV row.
    angle_sign = "p" if cfg["SERVO_ANGLE_DEG"] >= 0 else "n"
    cfg["SEGMENT_NAME"] = "angle_{}{:.0f}_esc{:04d}".format(
        angle_sign, abs(cfg["SERVO_ANGLE_DEG"]), int(cfg["ESC_COMMAND_US"])
    )

    return cfg


# ============================================================
# Utilities
# ============================================================


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def sleep_to_next_sample(cfg, loop_start_ms):
    elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
    remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms
    if remaining_ms > 0:
        time.sleep_ms(remaining_ms)


# ============================================================
# Servo helpers
# ============================================================


def write_servo_us(cfg, servo, pulse_us):
    pulse_us = int(clamp(pulse_us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))
    servo.duty_ns(pulse_us * 1000)


def setup_servo(cfg):
    print("Initializing servo. Pin:", cfg["SERVO_PIN"])
    servo = PWM(Pin(cfg["SERVO_PIN"]))
    servo.freq(cfg["SERVO_FREQ_HZ"])
    write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
    time.sleep_ms(500)
    return servo


# ============================================================
# ESC helpers
# ============================================================


def pwm_us_to_duty_u16(pwm_us):
    return int(int(pwm_us) * 65535 // 20000)


def set_esc_us(cfg, escs, pwm_us):
    pwm_us = clamp(int(pwm_us), cfg["PWM_SAFE_US"], 2000)
    duty = pwm_us_to_duty_u16(pwm_us)
    for esc in escs:
        esc.duty_u16(duty)


def setup_escs(cfg):
    print("Initializing ESC PWM. Pins:", cfg["ESC_PINS"])
    escs = []
    for pin_num in cfg["ESC_PINS"]:
        esc = PWM(Pin(pin_num, Pin.OUT))
        esc.freq(cfg["ESC_FREQ_HZ"])
        escs.append(esc)
    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    print("ESC at safe PWM:", cfg["PWM_SAFE_US"], "us")
    return escs


# ============================================================
# Encoder helpers
# ============================================================


def setup_encoder_hw(cfg):
    print("Initializing encoder. Pins:", cfg["ENC_A_PIN"], cfg["ENC_B_PIN"])
    encoder.init_configured(
        cfg["ENC_A_PIN"],
        cfg["ENC_B_PIN"],
        cfg["MAX_STEP_RATE"],
        cfg["DRAIN_HZ"],
    )
    time.sleep_ms(100)
    print("Encoder initialized. Initial count:", encoder.get_count())


# ============================================================
# CSV logging
# ============================================================


def make_log_filename(cfg, set_index):
    return "{}_set{:02d}.csv".format(cfg["LOG_FILE_BASE"], int(set_index))


def write_csv_header(f):
    f.write(
        "t_ms,t_s,set_index,segment,phase,"
        "servo_us,esc_pwm_us,"
        "count,count_delta,count_from_start\n"
    )


def write_sample(cfg, state, f, t0_ms, set_index, segment, phase,
                  servo_us, esc_pwm_us, count, count_from_start):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, t0_ms)
    t_s = t_ms / 1000.0

    count_delta = count - state["prev_count"]
    state["prev_count"] = count

    f.write("{},{:.3f},{},{},{},{},{},{},{},{}\n".format(
        int(t_ms),
        t_s,
        int(set_index),
        segment,
        phase,
        int(servo_us),
        int(esc_pwm_us),
        int(count),
        int(count_delta),
        int(count_from_start),
    ))

    state["sample_counter"] += 1

    if state["sample_counter"] % cfg["PRINT_EVERY_N_SAMPLES"] == 0:
        print(
            "sample", state["sample_counter"],
            "phase", phase,
            "servo", int(servo_us), "us",
            "esc", int(esc_pwm_us), "us",
            "count", int(count),
            "from_start", int(count_from_start),
        )


# ============================================================
# User prompt
# ============================================================


def print_user_prompt(cfg):
    angle = cfg["SERVO_ANGLE_DEG"]
    esc = cfg["ESC_COMMAND_US"]
    servo_us = cfg["SERVO_TARGET_US"]

    print()
    print("=" * 60)
    print("friction_sweep_log")
    print("  Servo angle : {:+.1f} deg  (servo_us = {:d})".format(angle, servo_us))
    print("  ESC command : {:d} us".format(esc))
    print()
    if angle >= 0:
        print("  --> Place cart at the MINIMUM encoder count end stop.")
        print("      (Cart should travel toward the maximum count end.)")
    else:
        print("  --> Place cart at the MAXIMUM encoder count end stop.")
        print("      (Cart should travel toward the minimum count end.)")
    print()
    print("  Starting in {:d} seconds...".format(cfg["USER_PROMPT_WAIT_S"]))
    print("=" * 60)

    for remaining in range(cfg["USER_PROMPT_WAIT_S"], 0, -1):
        print(remaining)
        time.sleep(1)

    print("Starting.")


# ============================================================
# Single acquisition set
# ============================================================


def run_single_set(cfg, state, f, servo, escs, t0_ms, set_index):
    segment = cfg["SEGMENT_NAME"]
    servo_neutral = cfg["SERVO_NEUTRAL_US"]
    servo_target = cfg["SERVO_TARGET_US"]
    esc_cmd = cfg["ESC_COMMAND_US"]
    dt_ms = cfg["COMMAND_UPDATE_DT_MS"]

    # ---- user_wait phase: ESC safe, servo at target angle, countdown ----
    # Servo moves to target before the countdown so it is already settled
    # when the operator positions the cart — no jerk after placement.
    print("Setting servo to", servo_target, "us (angle {:+.1f} deg)...".format(
        cfg["SERVO_ANGLE_DEG"]))
    write_servo_us(cfg, servo, servo_target)
    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])

    print_user_prompt(cfg)

    phase = "user_wait"
    count_ref = encoder.get_count()
    state["prev_count"] = count_ref

    for _ in range(10):
        loop_start = time.ticks_ms()
        count = encoder.get_count()
        write_sample(cfg, state, f, t0_ms, set_index, segment, phase,
                     servo_target, cfg["PWM_SAFE_US"], count, count - count_ref)
        sleep_to_next_sample(cfg, loop_start)

    # Record reference count at run start.
    count_at_run_start = encoder.get_count()
    state["prev_count"] = count_at_run_start
    print("Run start count:", count_at_run_start)

    # ---- run phase: apply ESC command ----
    print("Applying ESC command:", esc_cmd, "us")
    set_esc_us(cfg, escs, esc_cmd)

    phase = "run"
    halt_buf = [count_at_run_start] * cfg["HALT_WINDOW_SAMPLES"]
    halt_idx = 0
    run_sample = 0
    halt_phase = "timeout_end"

    while run_sample < cfg["MAX_RUN_SAMPLES"]:
        loop_start = time.ticks_ms()
        count = encoder.get_count()
        count_from_start = count - count_at_run_start

        write_sample(cfg, state, f, t0_ms, set_index, segment, phase,
                     servo_target, esc_cmd, count, count_from_start)

        # Update ring buffer
        halt_buf[halt_idx] = count
        halt_idx = (halt_idx + 1) % cfg["HALT_WINDOW_SAMPLES"]

        # Check halt only after ring buffer is fully populated
        if run_sample >= cfg["HALT_WINDOW_SAMPLES"] - 1:
            count_range = max(halt_buf) - min(halt_buf)
            if count_range < cfg["HALT_THRESHOLD_COUNTS"]:
                total_disp = abs(count - count_at_run_start)
                if total_disp < cfg["STICTION_NO_MOTION_COUNTS"]:
                    halt_phase = "stiction_halt"
                else:
                    halt_phase = "endstop_halt"
                break

        run_sample += 1
        sleep_to_next_sample(cfg, loop_start)

    # ---- halt phase: ESC off, log final samples ----
    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])

    total_disp = abs(encoder.get_count() - count_at_run_start)
    print("Run ended:", halt_phase,
          "| samples:", run_sample,
          "| total displacement:", total_disp, "counts")

    phase = halt_phase
    for _ in range(10):
        loop_start = time.ticks_ms()
        count = encoder.get_count()
        count_from_start = count - count_at_run_start
        write_sample(cfg, state, f, t0_ms, set_index, segment, phase,
                     servo_target, cfg["PWM_SAFE_US"], count, count_from_start)
        sleep_to_next_sample(cfg, loop_start)

    # Return servo to neutral
    write_servo_us(cfg, servo, servo_neutral)
    time.sleep_ms(500)

    run_reposition_phase(cfg, state, f, servo, escs, t0_ms, set_index, count_at_run_start)


def run_reposition_phase(cfg, state, f, servo, escs, t0_ms, set_index, count_at_run_start):
    """
    Drive cart at PWM_HARD_MAX_US with the same servo angle as the current run
    until the end-stop halt fires. Ensures the cart is at the correct starting
    position for the next segment in the alternating pos/neg sweep sequence.
    """
    segment       = cfg["SEGMENT_NAME"]
    servo_target  = cfg["SERVO_TARGET_US"]
    reposition_us = 1750
    dt_ms         = cfg["COMMAND_UPDATE_DT_MS"]

    print()
    print("Repositioning: servo", servo_target, "us  ESC", reposition_us, "us")
    write_servo_us(cfg, servo, servo_target)
    time.sleep_ms(cfg["SERVO_SETTLE_MS"])
    set_esc_us(cfg, escs, reposition_us)

    phase     = "reposition"
    halt_buf  = [encoder.get_count()] * cfg["HALT_WINDOW_SAMPLES"]
    halt_idx  = 0
    n_timeout = int(cfg["REPOSITION_TIMEOUT_S"] * 1000 / dt_ms)

    for rep_sample in range(n_timeout):
        loop_start       = time.ticks_ms()
        count            = encoder.get_count()
        count_from_start = count - count_at_run_start

        write_sample(cfg, state, f, t0_ms, set_index, segment, phase,
                     servo_target, reposition_us, count, count_from_start)

        halt_buf[halt_idx] = count
        halt_idx = (halt_idx + 1) % cfg["HALT_WINDOW_SAMPLES"]

        if rep_sample >= cfg["HALT_WINDOW_SAMPLES"] - 1:
            if max(halt_buf) - min(halt_buf) < cfg["HALT_THRESHOLD_COUNTS"]:
                print("Reposition complete.")
                break

        sleep_to_next_sample(cfg, loop_start)
    else:
        print("Reposition timeout.")

    set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
    write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
    time.sleep_ms(500)


# ============================================================
# Acquisition set loop
# ============================================================


def run_all_acquisition_sets(cfg, servo, escs):
    saved_files = []

    for set_index in range(1, cfg["NUM_ACQUISITION_SETS"] + 1):
        set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
        write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])

        log_file = make_log_filename(cfg, set_index)
        state = {"sample_counter": 0, "prev_count": encoder.get_count()}

        print()
        print("=" * 60)
        print("Acquisition set", set_index, "of", cfg["NUM_ACQUISITION_SETS"])
        print("Log file:", log_file)
        print("Servo angle:", cfg["SERVO_ANGLE_DEG"], "deg ->", cfg["SERVO_TARGET_US"], "us")
        print("ESC command:", cfg["ESC_COMMAND_US"], "us")
        print("=" * 60)

        with open(log_file, "w") as f:
            write_csv_header(f)
            t0_ms = time.ticks_ms()
            run_single_set(cfg, state, f, servo, escs, t0_ms, set_index)

        print("Saved:", log_file)
        saved_files.append(log_file)

        if set_index < cfg["NUM_ACQUISITION_SETS"] and cfg["COOLDOWN_BETWEEN_SETS_MS"] > 0:
            print("Cooldown:", cfg["COOLDOWN_BETWEEN_SETS_MS"] / 1000.0, "s")
            time.sleep_ms(cfg["COOLDOWN_BETWEEN_SETS_MS"])

    return saved_files


# ============================================================
# Hardware setup / shutdown
# ============================================================


def setup_hardware(cfg):
    servo = setup_servo(cfg)
    escs = setup_escs(cfg)
    setup_encoder_hw(cfg)
    return servo, escs


def safe_shutdown(cfg, servo, escs):
    print("Safe shutdown.")
    if escs:
        set_esc_us(cfg, escs, cfg["PWM_SAFE_US"])
        time.sleep_ms(500)
    if servo:
        write_servo_us(cfg, servo, cfg["SERVO_NEUTRAL_US"])
        time.sleep_ms(500)


# ============================================================
# Startup banner
# ============================================================


def print_startup_banner(cfg):
    print()
    print("friction_sweep_log — friction identification")
    print("  LOG_FILE_BASE    :", cfg["LOG_FILE_BASE"])
    print("  NUM_SETS         :", cfg["NUM_ACQUISITION_SETS"])
    print("  SERVO_ANGLE_DEG  :", cfg["SERVO_ANGLE_DEG"])
    print("  SERVO_TARGET_US  :", cfg["SERVO_TARGET_US"])
    print("  ESC_COMMAND_US   :", cfg["ESC_COMMAND_US"])
    print("  MAX_RUN_DURATION :", cfg["MAX_RUN_DURATION_S"], "s")
    print("  HALT_DURATION    :", cfg["HALT_DURATION_S"], "s")
    print("  HALT_THRESHOLD   :", cfg["HALT_THRESHOLD_COUNTS"], "counts")
    print("  STICTION_THRESH  :", cfg["STICTION_NO_MOTION_COUNTS"], "counts")
    print("  REPOSITION_TIMEOUT:", cfg["REPOSITION_TIMEOUT_S"], "s")
    print("  SEGMENT_NAME     :", cfg["SEGMENT_NAME"])
    print()


# ============================================================
# Main
# ============================================================


def main():
    cfg = finalize_config(make_config())

    servo = None
    escs = []

    try:
        print_startup_banner(cfg)
        servo, escs = setup_hardware(cfg)

        saved_files = run_all_acquisition_sets(cfg, servo, escs)

        print()
        print("All sets complete. Saved files:")
        for fname in saved_files:
            print(" -", fname)

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        safe_shutdown(cfg, servo, escs)


main()
