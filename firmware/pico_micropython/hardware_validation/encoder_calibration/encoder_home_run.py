# encoder_home_run.py
#
# Standalone runner for the powered homing utility (lib/encoder_home.py).
#
# This is the *executable* entry point — encoder_home.py itself is an importable
# library with no top-level action, so running that directly does nothing. Use
# this script (or call encoder_home.home() from any test) to actually home.
#
# PREREQUISITE:
#   The library must be on the Pico filesystem first:
#       mpremote connect COM3 fs cp \
#           firmware/pico_micropython/lib/encoder_home.py :lib/encoder_home.py
#
# What it does:
#   1. Sets up servo (GP15), ESCs (GP13/GP14), encoder (GP18/GP19).
#   2. Loads /run_config.json (if present) and passes it as overrides to home(),
#      so all homing knobs live on the laptop (in the orchestrate.json), not here.
#   3. Calls encoder_home.home() — powered homing + inferred rail length.
#   4. Writes a per-leg table CSV (for the orchestrator + MATLAB analysis).
#
# Output CSV columns (one row per driven leg):
#   leg_index,leg_name,direction,angle_deg,status,
#   breakaway_us,start_count,end_count,span_counts,scale_counts_per_mm
#
# The analyzer (analysis/.../analyze_encoder_home.m) derives rail length from
# the span columns and breakaway force from breakaway_us + angle_deg via the
# identified static thrust map.
#
# Knobs:
#   Tune homing in encoder_home_run.orchestrate.json (its "config" block becomes
#   /run_config.json). The orchestrator also auto-uploads the latest
#   lib/encoder_home.py before each run via the plan's "uploads" list.

import time
from machine import Pin, PWM
import encoder
import encoder_home
import servo_static_map


# ---- Hardware constants (match the rest of the firmware) ----
SERVO_PIN = 15
SERVO_FREQ_HZ = 50
SERVO_NEUTRAL_US = servo_static_map.NEUTRAL_US   # single source: lib/servo_static_map.py

ESC_PINS = [13, 14]
ESC_FREQ_HZ = 50
PWM_SAFE_US = 1000

ENC_A_PIN = 18
ENC_B_PIN = 19
MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

LOG_FILE = "encoder_home_run.csv"


def setup_servo():
    servo = PWM(Pin(SERVO_PIN))
    servo.freq(SERVO_FREQ_HZ)
    servo.duty_ns(SERVO_NEUTRAL_US * 1000)
    time.sleep_ms(500)
    return servo


def setup_escs():
    escs = []
    duty = int(PWM_SAFE_US * 65535 // 20000)
    for pin_num in ESC_PINS:
        esc = PWM(Pin(pin_num, Pin.OUT))
        esc.freq(ESC_FREQ_HZ)
        esc.duty_u16(duty)
        escs.append(esc)
    return escs


def setup_encoder():
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)


def load_overrides():
    """
    Load homing knobs from /run_config.json (uploaded by the orchestrator from
    encoder_home_run.orchestrate.json). Returns {} if no config is present, so a
    bare `mpremote run` still works on the library defaults.
    """
    try:
        import ujson as json
    except ImportError:
        import json

    for path in ("/run_config.json", "run_config.json"):
        try:
            with open(path, "r") as f:
                cfg = json.load(f)
        except OSError:
            continue
        # Allow either a bare dict or a {"config": {...}} wrapper.
        if isinstance(cfg, dict) and isinstance(cfg.get("config"), dict):
            cfg = cfg["config"]
        print("Loaded homing overrides from", path, ":", cfg)
        return cfg

    print("No /run_config.json found — using library defaults.")
    return {}


def write_leg_csv(datum):
    """
    One row per driven leg. The MATLAB analyzer derives rail length (from the
    span columns) and breakaway force (breakaway_us + angle_deg -> static thrust
    map -> axial force) from this table. Empty fields are written for missing
    values (None) so they parse as NaN.
    """
    scale = datum["scale_counts_per_mm"]
    ramp_step = datum["ramp_step_us"]
    with open(LOG_FILE, "w") as f:
        f.write("leg_index,leg_name,direction,angle_deg,status,"
                "breakaway_us,start_count,end_count,span_counts,"
                "scale_counts_per_mm,ramp_step_us\n")
        for leg in datum["legs"]:
            brk = "" if leg["breakaway_us"] is None else int(leg["breakaway_us"])
            span = "" if leg["span_counts"] is None else int(leg["span_counts"])
            f.write("{},{},{},{:.3f},{},{},{},{},{},{:.4f},{}\n".format(
                leg["index"], leg["name"], leg["direction"],
                leg["angle_deg"], leg["status"],
                brk, int(leg["start_count"]), int(leg["end_count"]),
                span, scale, int(ramp_step),
            ))
    print("Saved:", LOG_FILE)


def main():
    print("Setting up hardware for homing run...")
    servo = setup_servo()
    escs = setup_escs()
    setup_encoder()

    overrides = load_overrides()

    try:
        datum = encoder_home.home(servo, escs, overrides)
        write_leg_csv(datum)
        if not datum["ok"]:
            print("HOMING FAILED:", datum["status"])
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        # Belt-and-suspenders safe shutdown.
        duty = int(PWM_SAFE_US * 65535 // 20000)
        for esc in escs:
            esc.duty_u16(duty)
        time.sleep_ms(300)
        servo.duty_ns(SERVO_NEUTRAL_US * 1000)
        time.sleep_ms(300)
        print("Homing run done.")


main()
