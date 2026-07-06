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

# Zero-angle servo command (us). Upgraded high-speed digital servo
# (re-identified 2026-06-29): branch-mean lash-free neutral 1427.8 us -> 1428.
# Responsive range 481..2531 us; backlash 0.39 deg (down from the prior servo's
# 2.40 deg). Prior servo: 1430.75 -> 1431.
NEUTRAL_US = 1428

# Static gain. Signed form mirrors servoStaticMap.m (angle decreases with
# command); DEG_PER_US is the magnitude used by the firmware "NEUTRAL - angle/mag"
# command convention. Branch-mean (lash-free) value from the 2026-06-29 re-ID
# (two repeats: 0.093991 / 0.094001). Prior servo: 0.091092.
GAIN_DEG_PER_US = -0.093996
DEG_PER_US = 0.093996


def command_for_angle(angle_deg):
    """Servo command (us) that produces angle_deg, before any hard clamp."""
    return NEUTRAL_US - angle_deg / DEG_PER_US


def angle_for_command(servo_us):
    """Angle (deg) produced by a servo command (us)."""
    return GAIN_DEG_PER_US * (servo_us - NEUTRAL_US)
