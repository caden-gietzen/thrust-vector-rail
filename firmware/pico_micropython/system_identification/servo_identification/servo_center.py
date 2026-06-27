# servo_center.py
#
# Pre-run helper: drive the servo to the neutral/center command and HOLD it
# there until interrupted. Use this to mechanically mount, align, or zero
# fixtures at the same center the sweeps/excitation use, before a logging run.
#
# Holds by continuously refreshing the PWM pulse (a servo only holds position
# while it keeps receiving pulses). Stop with Ctrl-C; the servo is left at
# center with PWM still active so it does not go slack on exit.
#
# Hardware:
#   Servo PWM: GP15
#
# Center is the data-derived neutral from the static command-to-angle map
# (1431 us). Override via run_config.json if needed.

import time
from machine import Pin, PWM
import servo_static_map

SERVO_PIN = 15
SERVO_FREQ_HZ = 50

# Single source of truth: lib/servo_static_map.py (static-map fit gave 1430.75 us).
SERVO_CENTER_US = servo_static_map.NEUTRAL_US


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


def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))


apply_external_config_if_present()

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

print("Centering servo at", SERVO_CENTER_US, "us and holding.")
print("Press Ctrl-C to stop (servo stays at center).")

write_pwm_us(servo, SERVO_CENTER_US)

try:
    while True:
        # Keep refreshing the pulse so the servo actively holds center.
        write_pwm_us(servo, SERVO_CENTER_US)
        time.sleep_ms(500)
except KeyboardInterrupt:
    print("Stopped. Servo left at center, PWM still active.")
