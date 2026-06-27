import time
from machine import Pin, PWM
import servo_static_map

SERVO_PIN    = 15
SERVO_FREQ_HZ = 50
SERVO_MIN_US  = 450
SERVO_MAX_US  = 2450

# Edit this value and re-run to command a new position.
# Defaults to the zero-angle center (single source: lib/servo_static_map.py).
SERVO_US = servo_static_map.NEUTRAL_US

pwm = PWM(Pin(SERVO_PIN))
pwm.freq(SERVO_FREQ_HZ)

period_us = 1_000_000 // SERVO_FREQ_HZ
duty = int((SERVO_US / period_us) * 65535)
pwm.duty_u16(duty)

print(f"Servo commanded to {SERVO_US} us (duty {duty}/65535)")
print("Running indefinitely — press Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pwm.deinit()
    print("Stopped.")
