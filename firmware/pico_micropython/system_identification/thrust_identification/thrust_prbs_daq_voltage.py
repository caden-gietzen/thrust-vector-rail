# thrust_prbs_daq.py
# Dynamic thrust data acquisition for actuator/system identification.
#
# Hardware:
#   - Raspberry Pi Pico / RP2040 running MicroPython
#   - HX711 + load cell
#   - One or more ESC signal pins driven with identical PWM command
#   - Pixhawk TELEM2 MAVLink output wired to Pico UART RX for battery voltage/current
#
# Output CSV columns:
#   t_ms,t_s,set_index,prbs_seed,run_order_seed,run_name,segment,pwm_us,
#   raw_count,tared_count,force_N,
#   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
#   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
#   phase
#
# Notes:
#   - This script is designed for automated multi-set run-and-pull execution.
#   - Each acquisition set writes its own CSV.
#   - The script prints "Saved: <filename>.csv" after each file is complete.
#   - Battery percentage is logged for debugging only. For thrust modeling, prefer voltage.

from machine import Pin, PWM, UART
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711
from lib.mavlink_battery import MavlinkBatteryReader
import urandom


# ============================================================
# User settings
# ============================================================

# Output file base on Pico.
# Each set becomes:
#   thrust_prbs_daq_voltage_set01_seed1001.csv
LOG_FILE_BASE = "thrust_prbs_daq_voltage"

# Multi-set acquisition
NUM_ACQUISITION_SETS = 5

# Seed plan:
#   set 1 uses BASE_PRBS_SEED
#   set 2 uses BASE_PRBS_SEED + 1
#   etc.
BASE_PRBS_SEED = 1001

# Separate seed offset for run-order shuffling.
# This makes the run order deterministic but not identical to the PRBS sequence.
RUN_ORDER_SEED_OFFSET = 50000

# Rest/cooldown between full acquisition sets.
# 120000 ms = 2 min, 180000 ms = 3 min.
COOLDOWN_BETWEEN_SETS_MS = 180000

# If true, retare before each full acquisition set.
# Strongly recommended because the load cell can drift over time.
RETARE_EACH_SET = True

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

# Pixhawk MAVLink battery telemetry
# Wiring, listen-only:
#   Pixhawk TELEM2 TX -> Pico GP1 / UART0 RX
#   Pixhawk GND       -> Pico GND
#
# Pixhawk params:
#   SERIAL2_PROTOCOL = 2
#   SERIAL2_BAUD     = 57
#   SR2_EXT_STAT     = 1 or 2
MAVLINK_UART_ID = 0
MAVLINK_TX_PIN = 0
MAVLINK_RX_PIN = 1
MAVLINK_BAUDRATE = 57600

# Used only for print warnings / analysis flagging.
# The age is still logged even if stale.
BATTERY_STALE_MS = 1000

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

# For serious identification, False is better because it avoids always running
# the same range at the same point in the battery/thermal timeline.
FIXED_RUN_ORDER = False

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


def cooldown_countdown(message, duration_ms):
    """
    Long cooldown countdown with sparse prints.
    """
    if duration_ms <= 0:
        return

    print()
    print(message)
    start_ms = ticks_ms()
    last_print_s = -1

    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        elapsed_ms = ticks_diff(ticks_ms(), start_ms)
        remaining_ms = duration_ms - elapsed_ms
        remaining_s = int((remaining_ms + 999) / 1000)

        # Print every 30 seconds, plus final 10 seconds.
        if remaining_s != last_print_s:
            if remaining_s % 30 == 0 or remaining_s <= 10:
                print("Cooldown remaining:", remaining_s, "s")
                last_print_s = remaining_s

        sleep_ms(200)

    print("Cooldown complete.")


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


def seed_urandom(seed_value, label):
    try:
        urandom.seed(seed_value)
        print(label, "seed:", seed_value)
    except Exception:
        print("NOTE: urandom.seed unavailable for", label, "; sequence may not be exactly repeatable.")


def maybe_shuffle_runs(runs, run_order_seed):
    if FIXED_RUN_ORDER:
        return runs

    seed_urandom(run_order_seed, "Run-order")

    runs = list(runs)
    for i in range(len(runs) - 1, 0, -1):
        j = urandom.getrandbits(16) % (i + 1)
        runs[i], runs[j] = runs[j], runs[i]
    return runs


def prbs_next_pwm(center_pwm, amp_pwm):
    if urandom.getrandbits(1):
        return center_pwm + amp_pwm
    return center_pwm - amp_pwm


def csv_value(value):
    """
    Converts None to blank for cleaner CSV logging.
    """
    if value is None:
        return ""
    return value


def make_log_filename(set_index, prbs_seed):
    return "{}_set{:02d}_seed{}.csv".format(
        LOG_FILE_BASE,
        int(set_index),
        int(prbs_seed),
    )


def print_battery_snapshot(mav_batt, label):
    if mav_batt is None:
        print(label, "battery: no MAVLink reader")
        return

    mav_batt.update()

    if mav_batt.has_battery():
        print(
            label,
            "battery:",
            "V:", mav_batt.voltage_V,
            "A:", mav_batt.current_A,
            "remaining:", mav_batt.battery_remaining_pct,
            "age_ms:", mav_batt.age_ms(),
        )
    else:
        print(label, "battery: no SYS_STATUS decoded yet")


# ============================================================
# Data logging functions
# ============================================================

sample_counter = 0


def write_sample(
    f,
    hx,
    mav_batt,
    t0_ms,
    set_index,
    prbs_seed,
    run_order_seed,
    run_name,
    segment,
    pwm_us,
    offset_count,
    phase,
):
    global sample_counter

    # Update MAVLink before the HX711 read.
    if mav_batt is not None:
        mav_batt.update()

    raw_count = read_average(hx, SAMPLES_RUN)
    tared_count, thrust_N = force_N_from_raw(raw_count, offset_count)

    # Update MAVLink again after HX711 read.
    if mav_batt is not None:
        mav_batt.update()

        (
            battery_voltage_V,
            battery_current_A,
            battery_remaining_pct,
            battery_age_ms,
            mavlink_total_packets,
            mavlink_sys_status_packets,
            mavlink_bad_frames,
        ) = mav_batt.snapshot()

        battery_voltage_V = csv_value(battery_voltage_V)
        battery_current_A = csv_value(battery_current_A)
        battery_remaining_pct = csv_value(battery_remaining_pct)
        battery_age_ms = csv_value(battery_age_ms)

    else:
        battery_voltage_V = ""
        battery_current_A = ""
        battery_remaining_pct = ""
        battery_age_ms = ""
        mavlink_total_packets = ""
        mavlink_sys_status_packets = ""
        mavlink_bad_frames = ""

    t_ms = ticks_diff(ticks_ms(), t0_ms)
    t_s = t_ms / 1000.0

    f.write(
        "{},{:.3f},{},{},{},{},{},{},{:.3f},{:.3f},{:.6f},{},{},{},{},{},{},{},{}\n".format(
            t_ms,
            t_s,
            int(set_index),
            int(prbs_seed),
            int(run_order_seed),
            run_name,
            segment,
            int(pwm_us),
            raw_count,
            tared_count,
            thrust_N,
            battery_voltage_V,
            battery_current_A,
            battery_remaining_pct,
            battery_age_ms,
            mavlink_total_packets,
            mavlink_sys_status_packets,
            mavlink_bad_frames,
            phase,
        )
    )

    sample_counter += 1

    if sample_counter % PRINT_EVERY_N_SAMPLES == 0:
        print(
            "sample", sample_counter,
            "set", set_index,
            "seed", prbs_seed,
            "phase", phase,
            "pwm_us", int(pwm_us),
            "force_N", thrust_N,
            "V", battery_voltage_V,
            "A", battery_current_A,
            "batt_age_ms", battery_age_ms,
        )

        if battery_age_ms != "" and battery_age_ms is not None:
            if battery_age_ms > BATTERY_STALE_MS:
                print("WARNING: battery telemetry stale. age_ms:", battery_age_ms)

    return raw_count, thrust_N


def hold_and_log(
    f,
    hx,
    mav_batt,
    escs,
    t0_ms,
    set_index,
    prbs_seed,
    run_order_seed,
    run_name,
    segment,
    pwm_us,
    duration_ms,
    offset_count,
    phase,
):
    set_pwm_us(escs, pwm_us)
    start_ms = ticks_ms()

    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        write_sample(
            f=f,
            hx=hx,
            mav_batt=mav_batt,
            t0_ms=t0_ms,
            set_index=set_index,
            prbs_seed=prbs_seed,
            run_order_seed=run_order_seed,
            run_name=run_name,
            segment=segment,
            pwm_us=pwm_us,
            offset_count=offset_count,
            phase=phase,
        )


def run_prbs_sequence(
    f,
    hx,
    mav_batt,
    escs,
    t0_ms,
    set_index,
    prbs_seed,
    run_order_seed,
    run_name,
    center_pwm,
    amp_pwm,
    duration_ms,
    hold_ms,
    offset_count,
):
    low = clamp(center_pwm - amp_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)
    high = clamp(center_pwm + amp_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)

    print()
    print("RUN:", run_name)
    print("Set index:", set_index)
    print("PRBS seed:", prbs_seed)
    print("Run-order seed:", run_order_seed)
    print("Center PWM:", center_pwm)
    print("Amplitude:", amp_pwm)
    print("Command bounds:", low, "to", high, "us")
    print("Duration:", duration_ms / 1000.0, "s")
    print("Hold time:", hold_ms, "ms")

    print("Pre-run baseline...")
    hold_and_log(
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        set_index=set_index,
        prbs_seed=prbs_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="baseline_pre",
        pwm_us=center_pwm,
        duration_ms=BASELINE_HOLD_MS,
        offset_count=offset_count,
        phase="baseline_pre",
    )

    print("Settling at center...")
    hold_and_log(
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        set_index=set_index,
        prbs_seed=prbs_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="settle",
        pwm_us=center_pwm,
        duration_ms=PRE_RUN_SETTLE_MS,
        offset_count=offset_count,
        phase="settle",
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
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                t0_ms=t0_ms,
                set_index=set_index,
                prbs_seed=prbs_seed,
                run_order_seed=run_order_seed,
                run_name=run_name,
                segment=segment_idx,
                pwm_us=pwm_cmd,
                offset_count=offset_count,
                phase="prbs",
            )

        segment_idx += 1

    print("Post-run baseline...")
    hold_and_log(
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        set_index=set_index,
        prbs_seed=prbs_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="baseline_post",
        pwm_us=center_pwm,
        duration_ms=BASELINE_HOLD_MS,
        offset_count=offset_count,
        phase="baseline_post",
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

    # HX711.__init__ forces a power cycle and settles the amplifier, so no
    # manual clock-low + settle is needed here.
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


def setup_mavlink_battery():
    print("Initializing MAVLink battery UART...")
    print("UART:", MAVLINK_UART_ID)
    print("TX pin:", MAVLINK_TX_PIN)
    print("RX pin:", MAVLINK_RX_PIN)
    print("Baudrate:", MAVLINK_BAUDRATE)

    uart = UART(
        MAVLINK_UART_ID,
        baudrate=MAVLINK_BAUDRATE,
        tx=Pin(MAVLINK_TX_PIN),
        rx=Pin(MAVLINK_RX_PIN),
    )

    mav_batt = MavlinkBatteryReader(uart)

    warmup_start_ms = ticks_ms()
    while ticks_diff(ticks_ms(), warmup_start_ms) < 1000:
        mav_batt.update()
        sleep_ms(10)

    if mav_batt.has_battery():
        print(
            "MAVLink battery reader initialized.",
            "V:", mav_batt.voltage_V,
            "A:", mav_batt.current_A,
            "remaining:", mav_batt.battery_remaining_pct,
            "age_ms:", mav_batt.age_ms(),
        )
    else:
        print("WARNING: MAVLink battery reader initialized, but no SYS_STATUS battery data yet.")

    return mav_batt


def tare_load_cell(hx, message):
    countdown(
        message,
        INITIAL_SAFETY_COUNTDOWN_S,
    )

    print("Taring with", TARE_SAMPLES, "samples...")
    offset_count = read_average(hx, TARE_SAMPLES, delay_ms=TARE_SAMPLE_DELAY_MS)
    print("Offset count:", offset_count)
    return offset_count


def retare_load_cell_quick(hx):
    print("Re-taring load cell for this acquisition set...")
    offset_count = read_average(hx, TARE_SAMPLES, delay_ms=TARE_SAMPLE_DELAY_MS)
    print("Set offset count:", offset_count)
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
# Acquisition set runner
# ============================================================

def run_acquisition_set(
    set_index,
    hx,
    mav_batt,
    escs,
    offset_count,
):
    global sample_counter

    prbs_seed = BASE_PRBS_SEED + (set_index - 1)
    run_order_seed = prbs_seed + RUN_ORDER_SEED_OFFSET
    log_file = make_log_filename(set_index, prbs_seed)

    print()
    print("============================================================")
    print("ACQUISITION SET", set_index, "of", NUM_ACQUISITION_SETS)
    print("Log file:", log_file)
    print("PRBS seed:", prbs_seed)
    print("Run-order seed:", run_order_seed)
    print("Fixed run order:", FIXED_RUN_ORDER)
    print("============================================================")

    # Seed once for run-order shuffle.
    runs = maybe_shuffle_runs(TEST_RUNS, run_order_seed)

    print()
    print("Planned runs for this set:")
    for run_name, center, amp, duration, hold in runs:
        print(
            " -",
            run_name,
            "center", center,
            "amp", amp,
            "duration_s", duration / 1000.0,
            "hold_ms", hold,
        )

    # Important: seed PRBS after shuffling so the shuffle does not consume
    # bits from the PRBS sequence.
    seed_urandom(prbs_seed, "PRBS")

    print_battery_snapshot(mav_batt, "Before set")

    sample_counter = 0

    with open(log_file, "w") as f:
        f.write(
            "t_ms,t_s,set_index,prbs_seed,run_order_seed,run_name,segment,pwm_us,"
            "raw_count,tared_count,force_N,"
            "battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,"
            "mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,"
            "phase\n"
        )

        t0_ms = ticks_ms()

        for run_name, center, amp, duration, hold in runs:
            run_prbs_sequence(
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                escs=escs,
                t0_ms=t0_ms,
                set_index=set_index,
                prbs_seed=prbs_seed,
                run_order_seed=run_order_seed,
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

    print_battery_snapshot(mav_batt, "After set")
    print()
    print("Saved:", log_file)

    return log_file


# ============================================================
# Main
# ============================================================

hx = None
escs = []
mav_batt = None

try:
    print("Dynamic PRBS thrust data acquisition")
    print("Output base:", LOG_FILE_BASE)
    print("Number of acquisition sets:", NUM_ACQUISITION_SETS)
    print("Base PRBS seed:", BASE_PRBS_SEED)
    print("Cooldown between sets:", COOLDOWN_BETWEEN_SETS_MS / 1000.0, "s")
    print("SCALE_G_PER_COUNT:", SCALE_G_PER_COUNT)
    print("SCALE_N_PER_COUNT:", SCALE_N_PER_COUNT)
    print("FORCE_SIGN:", FORCE_SIGN)

    hx = setup_hx711()
    escs = setup_escs()
    mav_batt = setup_mavlink_battery()

    # Let the system sit safely before first tare.
    set_pwm_us(escs, PWM_SAFE_US)
    sleep_ms(1000)

    offset_count = tare_load_cell(
        hx,
        "Prepare for initial tare: motors off, no added load on load cell.",
    )

    arm_escs(escs)

    saved_files = []

    for set_index in range(1, NUM_ACQUISITION_SETS + 1):
        set_pwm_us(escs, PWM_SAFE_US)
        sleep_ms(1000)

        if RETARE_EACH_SET:
            offset_count = retare_load_cell_quick(hx)

        saved_file = run_acquisition_set(
            set_index=set_index,
            hx=hx,
            mav_batt=mav_batt,
            escs=escs,
            offset_count=offset_count,
        )

        saved_files.append(saved_file)

        # Cooldown between sets, but not after the final set.
        if set_index < NUM_ACQUISITION_SETS:
            print("Returning to safe PWM for cooldown...")
            set_pwm_us(escs, PWM_SAFE_US)

            cooldown_countdown(
                "Cooldown between acquisition sets. Keep area safe.",
                COOLDOWN_BETWEEN_SETS_MS,
            )

    print()
    print("All acquisition sets complete.")
    print("Saved files:")
    for filename in saved_files:
        print(" -", filename)

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