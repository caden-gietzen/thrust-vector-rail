# servo_static_map.py
#
# Firmware single source of truth for the servo static command<->angle map.
# This is the device-side mirror of analysis/utils/servoStaticMap.m — keep the
# two in sync. If the static command-to-angle sweep is re-run, update BOTH.
#
# Any firmware that converts a servo PWM command (us) to/from an angle, or that
# needs the zero-angle "center" command, should pull these from here instead of
# hardcoding — exactly one place to change.
#
# Model (experiments/servo_identification/results.md, Section 3):
#   theta_deg = GAIN_DEG_PER_US * (servo_us - NEUTRAL_US)
#
# Deploy: like the encoder C module and hx711_gpio, this lives at /lib on the
# Pico. The homing orchestrator pushes it via its "uploads" list; for scripts
# run standalone, flash it once:
#   mpremote connect COMx fs cp \
#       firmware/pico_micropython/lib/servo_static_map.py :lib/servo_static_map.py

# Zero-angle servo command (us). Static-map fit gave 1430.75 us -> 1431.
NEUTRAL_US = 1431

# Static gain. Signed form mirrors servoStaticMap.m (angle decreases with
# command); DEG_PER_US is the magnitude used by the firmware "NEUTRAL - angle/mag"
# command convention.
GAIN_DEG_PER_US = -0.091092
DEG_PER_US = 0.091092


def command_for_angle(angle_deg):
    """Servo command (us) that produces angle_deg, before any hard clamp."""
    return NEUTRAL_US - angle_deg / DEG_PER_US


def angle_for_command(servo_us):
    """Angle (deg) produced by a servo command (us)."""
    return GAIN_DEG_PER_US * (servo_us - NEUTRAL_US)
