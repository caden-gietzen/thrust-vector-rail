import time
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

FILENAME = None

STEP_US = 25
HOLD_MS = 250
N_SWEEPS = 2

def write_pwm_us(pwm, pulse_us):
    pwm.duty_ns(int(pulse_us * 1000))

def make_sweep_values():
    up = list(range(SERVO_MIN_US, SERVO_MAX_US + 1, STEP_US))
    down = list(range(SERVO_MAX_US, SERVO_MIN_US - 1, -STEP_US))
    return up + down

servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)

try:
    print("Initializing encoder...")
    encoder.init_configured(ENC_A_PIN, ENC_B_PIN, MAX_STEP_RATE, DRAIN_HZ)
    time.sleep_ms(100)

    print("Centering servo...")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(1000)

    print("Set mechanical system to safe starting position.")
    print("Zeroing encoder in 5 seconds...")
    time.sleep_ms(5000)

    encoder.zero()
    time.sleep_ms(100)

    print("Encoder zeroed.")
    print("Starting sweep.")
    print("Logging to:", FILENAME)

    start_ms = time.ticks_ms()

    with open(FILENAME, "w") as f:
        f.write("t_ms,t_s,servo_us,count\n")

        sweep_values = make_sweep_values()

        for sweep_idx in range(N_SWEEPS):
            print("Sweep", sweep_idx + 1, "of", N_SWEEPS)

            for servo_us in sweep_values:
                write_pwm_us(servo, servo_us)
                time.sleep_ms(HOLD_MS)

                now_ms = time.ticks_ms()
                t_ms = time.ticks_diff(now_ms, start_ms)
                t_s = t_ms / 1000.0
                count = encoder.get_count()

                f.write("{},{:.3f},{},{}\n".format(
                    t_ms,
                    t_s,
                    servo_us,
                    count
                ))

                f.flush()

    print("Sweep complete.")

    FILENAME = "servo_sweep_log.csv"

finally:
    print("Centering servo and shutting down.")
    write_pwm_us(servo, SERVO_CENTER_US)
    time.sleep_ms(500)
    servo.deinit()

    if FILENAME is not None:
        print("Saved:", FILENAME)
    else:
        print("No file was created.")