# servo_encoder_hold_drift_test.py

import time
from machine import Pin, PWM
import encoder

ENC_A_PIN = 18
ENC_B_PIN = 19
SERVO_PIN = 15

MAX_STEP_RATE = 1_000_000
DRAIN_HZ = 10_000

SERVO_FREQ_HZ = 50
SERVO_CENTER_US = 1450

LOG_FILE = "servo_encoder_hold_drift.csv"

DURATION_S = 60
SAMPLE_DT_MS = 20


def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))


servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

try:
    print("Initializing encoder...")
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)

    print("Centering servo...")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(1000)

    print("Zeroing encoder...")
    encoder.zero()
    time.sleep_ms(100)

    count_zero = encoder.get_count()
    print("Zero count:", count_zero)

    t0 = time.ticks_ms()

    with open(LOG_FILE, "w") as f:
        f.write("t_ms,servo_us,count,count_delta\n")

        while time.ticks_diff(time.ticks_ms(), t0) < DURATION_S * 1000:
            now = time.ticks_ms()
            t_ms = time.ticks_diff(now, t0)

            count = encoder.get_count()
            count_delta = count - count_zero

            f.write("{},{},{},{}\n".format(
                int(t_ms),
                int(SERVO_CENTER_US),
                int(count),
                int(count_delta),
            ))

            if t_ms % 1000 < SAMPLE_DT_MS:
                print("t_ms", t_ms, "count_delta", count_delta)

            time.sleep_ms(SAMPLE_DT_MS)

    print("Saved:", LOG_FILE)

finally:
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()
    print("Done.")