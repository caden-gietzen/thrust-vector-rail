# thrust_prbs_daq.py
# Dynamic thrust data acquisition for actuator/system identification.
#
# Hardware:
#   - Raspberry Pi Pico / RP2040 running MicroPython
#   - HX711 + load cell
#   - One or more ESC signal pins driven with identical PWM command
#
# Output CSV columns:
#   t_ms,t_s,run_name,segment,pwm_us,raw_count,tared_count,force_N,phase
#
# Notes:
#   - This script is designed for automated run-and-pull execution.
#   - No keyboard input is required.
#   - The script prints "Saved: <filename>.csv" for PC-side retrieval.
#   - Calibration constants come from HX711 load cell validation.

from machine import Pin, PWM
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711
import urandom


# ============================================================
# User settings
# ============================================================

# Output file on Pico. The PC run-and-pull tool mirrors this into data/raw
# based on this script's repository path.
LOG_FILE = "thrust_prbs_daq_no_voltage.csv"

# HX711 pins
HX711_DAT_PIN = 20
HX711_SCK_PIN = 21
HX711_GAIN = 128

# Calibration constants from load_cell_calibration validation.
# Identified fit:
#   counts = 415.008037 * mass_g + 148.655610
#   grams_per_count = 0.002409592
SCALE_G_PER_COUNT = 0.002409592
SCALE_N_PER_COUNT = SCALE_G_PER_COUNT * 9.80665 / 1000.0

# Force sign convention.
# If positive thrust produces decreasing HX711 counts, use -1.
# If positive thrust produces increasing HX711 counts, use +1.
FORCE_SIGN = 1

# HX711 sampling
TARE_SAMPLES = 100
SAMPLES_RUN = 1                  # keep low for dynamics; averaging reduces bandwidth
TARE_SAMPLE_DELAY_MS = 20

# ESC pins
ESC_PINS = [13, 14]

# ESC / PWM
ESC_FREQ_HZ = 50
PWM_SAFE_US = 1000
PWM_HARD_MIN_US = 1100
PWM_HARD_MAX_US = 1950           # avoid saturation near 2000 us

# Safety/countdown timing
INITIAL_SAFETY_COUNTDOWN_S = 10
ARM_COUNTDOWN_S = 5
ARM_TIME_MS = 5000
PRE_RUN_SETTLE_MS = 3000
POST_RUN_IDLE_MS = 1500
BASELINE_HOLD_MS = 3000

# Dynamic excitation settings
HOLD_TIME_MS = 150
RUN_DURATION_MS = 60000

# Each entry:
#   (run_name, center_pwm_us, amplitude_pwm_us, duration_ms, hold_ms)
TEST_RUNS = [
    ("global_1100_1950", 1525, 425, RUN_DURATION_MS, HOLD_TIME_MS),
    ("local_1400_1650", 1525, 125, RUN_DURATION_MS, HOLD_TIME_MS),
    ("local_1700_1850", 1775, 75, RUN_DURATION_MS, HOLD_TIME_MS),
]

FIXED_RUN_ORDER = True
RANDOM_SEED = 12345

# Print every N samples to avoid overwhelming serial output.
PRINT_EVERY_N_SAMPLES = 50


# ============================================================
# Helper functions
# ============================================================

def countdown(message, seconds):
    print()
    print(message)
    for remaining in range(seconds, 0, -1):
        print(remaining)
        sleep(1)
    print("Starting now.")


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def pwm_us_to_duty_u16(pwm_us):
    # 50 Hz period = 20,000 us
    return int(int(pwm_us) * 65535 / 20000)


def set_pwm_us(escs, pwm_us):
    pwm_us = clamp(int(pwm_us), PWM_SAFE_US, 2000)
    duty = pwm_us_to_duty_u16(pwm_us)
    for esc in escs:
        esc.duty_u16(duty)


def read_average(hx, n=1, delay_ms=0):
    total = 0
    good = 0

    for _ in range(n):
        try:
            total += hx.read()
            good += 1
        except OSError as e:
            print("HX711 read error:", e)

        if delay_ms > 0:
            sleep_ms(delay_ms)

    if good == 0:
        raise OSError("No valid HX711 samples")

    return total / good


def force_N_from_raw(raw_count, offset_count):
    tared_count = raw_count - offset_count
    force_N = FORCE_SIGN * tared_count * SCALE_N_PER_COUNT
    return tared_count, force_N


def maybe_seed_prbs():
    try:
        urandom.seed(RANDOM_SEED)
        print("PRBS seed:", RANDOM_SEED)
    except Exception:
        print("NOTE: urandom.seed unavailable; PRBS may not be exactly repeatable.")


def maybe_shuffle_runs(runs):
    if FIXED_RUN_ORDER:
        return runs

    runs = list(runs)
    for i in range(len(runs) - 1, 0, -1):
        j = urandom.getrandbits(16) % (i + 1)
        runs[i], runs[j] = runs[j], runs[i]
    return runs


def prbs_next_pwm(center_pwm, amp_pwm):
    if urandom.getrandbits(1):
        return center_pwm + amp_pwm
    return center_pwm - amp_pwm


# ============================================================
# Data logging functions
# ============================================================

sample_counter = 0


def write_sample(f, hx, t0_ms, run_name, segment, pwm_us, offset_count, phase):
    global sample_counter

    raw_count = read_average(hx, SAMPLES_RUN)
    tared_count, thrust_N = force_N_from_raw(raw_count, offset_count)

    t_ms = ticks_diff(ticks_ms(), t0_ms)
    t_s = t_ms / 1000.0

    f.write("{},{:.3f},{},{},{},{:.3f},{:.3f},{:.6f},{}\n".format(
        t_ms,
        t_s,
        run_name,
        segment,
        int(pwm_us),
        raw_count,
        tared_count,
        thrust_N,
        phase,
    ))

    sample_counter += 1
    if sample_counter % PRINT_EVERY_N_SAMPLES == 0:
        print("sample", sample_counter, "phase", phase, "pwm_us", int(pwm_us), "force_N", thrust_N)

    return raw_count, thrust_N


def hold_and_log(f, hx, escs, t0_ms, run_name, segment, pwm_us, duration_ms, offset_count, phase):
    set_pwm_us(escs, pwm_us)
    start_ms = ticks_ms()

    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        write_sample(f, hx, t0_ms, run_name, segment, pwm_us, offset_count, phase)


def run_prbs_sequence(f, hx, escs, t0_ms, run_name, center_pwm, amp_pwm, duration_ms, hold_ms, offset_count):
    low = clamp(center_pwm - amp_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)
    high = clamp(center_pwm + amp_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)

    print()
    print("RUN:", run_name)
    print("Center PWM:", center_pwm)
    print("Amplitude:", amp_pwm)
    print("Command bounds:", low, "to", high, "us")
    print("Duration:", duration_ms / 1000.0, "s")
    print("Hold time:", hold_ms, "ms")

    print("Pre-run baseline...")
    hold_and_log(
        f, hx, escs, t0_ms, run_name, "baseline_pre", center_pwm,
        BASELINE_HOLD_MS, offset_count, "baseline_pre"
    )

    print("Settling at center...")
    hold_and_log(
        f, hx, escs, t0_ms, run_name, "settle", center_pwm,
        PRE_RUN_SETTLE_MS, offset_count, "settle"
    )

    print("PRBS running...")
    start_ms = ticks_ms()
    segment_idx = 0

    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        pwm_cmd = prbs_next_pwm(center_pwm, amp_pwm)
        pwm_cmd = clamp(pwm_cmd, PWM_HARD_MIN_US, PWM_HARD_MAX_US)
        set_pwm_us(escs, pwm_cmd)

        hold_start_ms = ticks_ms()
        while ticks_diff(ticks_ms(), hold_start_ms) < hold_ms:
            write_sample(
                f, hx, t0_ms, run_name, segment_idx,
                pwm_cmd, offset_count, "prbs"
            )

        segment_idx += 1

    print("Post-run baseline...")
    hold_and_log(
        f, hx, escs, t0_ms, run_name, "baseline_post", center_pwm,
        BASELINE_HOLD_MS, offset_count, "baseline_post"
    )

    f.flush()
    print("Finished:", run_name)


# ============================================================
# Setup functions
# ============================================================

def setup_hx711():
    print("Initializing HX711...")
    print("DAT pin:", HX711_DAT_PIN)
    print("SCK pin:", HX711_SCK_PIN)

    data_pin = Pin(HX711_DAT_PIN, Pin.IN)
    clock_pin = Pin(HX711_SCK_PIN, Pin.OUT)

    # HX711 powers down if SCK is held high. Force low before initializing.
    clock_pin.value(0)
    sleep_ms(750)

    hx = HX711(clock_pin, data_pin, gain=HX711_GAIN)
    print("HX711 initialized.")
    return hx


def setup_escs():
    print("Initializing ESC PWM outputs...")
    print("ESC pins:", ESC_PINS)

    escs = []
    for pin_num in ESC_PINS:
        esc = PWM(Pin(pin_num, Pin.OUT))
        esc.freq(ESC_FREQ_HZ)
        escs.append(esc)

    set_pwm_us(escs, PWM_SAFE_US)
    print("ESC outputs set to safe PWM:", PWM_SAFE_US, "us")
    return escs


def tare_load_cell(hx):
    countdown(
        "Prepare for tare: motors off, no added load on load cell.",
        INITIAL_SAFETY_COUNTDOWN_S,
    )

    print("Taring with", TARE_SAMPLES, "samples...")
    offset_count = read_average(hx, TARE_SAMPLES, delay_ms=TARE_SAMPLE_DELAY_MS)
    print("Offset count:", offset_count)
    return offset_count


def arm_escs(escs):
    countdown(
        "Arming ESCs at safe PWM. Keep clear of motors/propellers.",
        ARM_COUNTDOWN_S,
    )

    set_pwm_us(escs, PWM_SAFE_US)
    sleep_ms(ARM_TIME_MS)
    print("ESC arming complete.")


# ============================================================
# Main
# ============================================================

hx = None
escs = []

try:
    print("Dynamic PRBS thrust data acquisition")
    print("Output file:", LOG_FILE)
    print("SCALE_G_PER_COUNT:", SCALE_G_PER_COUNT)
    print("SCALE_N_PER_COUNT:", SCALE_N_PER_COUNT)
    print("FORCE_SIGN:", FORCE_SIGN)

    hx = setup_hx711()
    escs = setup_escs()

    # Let the system sit safely before tare.
    set_pwm_us(escs, PWM_SAFE_US)
    sleep_ms(1000)

    offset_count = tare_load_cell(hx)
    arm_escs(escs)

    # Re-tare immediately before the dynamic test to reduce offset drift.
    print("Re-taring immediately before PRBS runs...")
    offset_count = read_average(hx, TARE_SAMPLES, delay_ms=TARE_SAMPLE_DELAY_MS)
    print("Dynamic-test offset count:", offset_count)

    maybe_seed_prbs()
    runs = maybe_shuffle_runs(TEST_RUNS)

    print()
    print("Planned runs:")
    for run_name, center, amp, duration, hold in runs:
        print(" -", run_name, "center", center, "amp", amp, "duration_s", duration / 1000.0, "hold_ms", hold)

    with open(LOG_FILE, "w") as f:
        f.write("t_ms,t_s,run_name,segment,pwm_us,raw_count,tared_count,force_N,phase\n")
        t0_ms = ticks_ms()

        for run_name, center, amp, duration, hold in runs:
            run_prbs_sequence(
                f=f,
                hx=hx,
                escs=escs,
                t0_ms=t0_ms,
                run_name=run_name,
                center_pwm=center,
                amp_pwm=amp,
                duration_ms=duration,
                hold_ms=hold,
                offset_count=offset_count,
            )

            print("Returning to safe PWM...")
            set_pwm_us(escs, PWM_SAFE_US)
            sleep_ms(POST_RUN_IDLE_MS)

        set_pwm_us(escs, PWM_SAFE_US)
        sleep_ms(1000)

    print()
    print("Saved:", LOG_FILE)

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    print("Stopping motors.")
    if escs:
        set_pwm_us(escs, PWM_SAFE_US)
        sleep_ms(1000)

    if hx is not None:
        try:
            print("Powering down HX711.")
            hx.power_down()
        except Exception:
            pass

