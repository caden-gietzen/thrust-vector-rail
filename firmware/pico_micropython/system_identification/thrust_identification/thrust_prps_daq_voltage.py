# thrust_prps_daq_voltage.py
# Dynamic thrust data acquisition using PRPS:
#   Pseudo Random Periodic Signal = periodic multisine with random phases.
#
# Hardware:
#   - Raspberry Pi Pico / RP2040 running MicroPython
#   - HX711 + load cell
#   - One or more ESC signal pins driven with identical PWM command
#   - Pixhawk TELEM2 MAVLink output wired to Pico UART RX for battery voltage/current
#
# Output CSV columns:
#   t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,pwm_us,
#   raw_count,tared_count,force_N,
#   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
#   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
#   phase,period_index,period_sample_index
#
# Main idea:
#   PRBS:
#       random high/low steps.
#
#   PRPS:
#       periodic input made from selected sinusoids:
#
#       u(t) = center_pwm + amplitude_pwm * normalized_sum_of_sines(t)
#
#   This version is memory-safe for the Pico:
#       - It does NOT store the full waveform.
#       - It computes each PRPS sample on the fly.
#       - It only stores frequency bins and phase values.

from machine import Pin, PWM, UART
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711
from lib.mavlink_battery import MavlinkBatteryReader
import urandom
import math


# ============================================================
# User settings
# ============================================================

LOG_FILE_BASE = "thrust_prps_daq_voltage"

# For first debug run, use 1.
# For full data collection, use 5.
NUM_ACQUISITION_SETS = 1

# Seed plan:
#   set 1 uses BASE_PRPS_SEED
#   set 2 uses BASE_PRPS_SEED + 1
#   etc.
BASE_PRPS_SEED = 2001

# Separate seed offset for run-order shuffling.
RUN_ORDER_SEED_OFFSET = 50000

# Rest/cooldown between full acquisition sets.
# 120000 ms = 2 min, 180000 ms = 3 min.
COOLDOWN_BETWEEN_SETS_MS = 180000

# If true, retare before each full acquisition set.
RETARE_EACH_SET = True

# HX711 pins
HX711_DAT_PIN = 20
HX711_SCK_PIN = 21
HX711_GAIN = 128

# Calibration constants from load_cell_calibration validation.
SCALE_G_PER_COUNT = 0.002409592
SCALE_N_PER_COUNT = SCALE_G_PER_COUNT * 9.80665 / 1000.0

# Force sign convention.
# If positive thrust produces decreasing HX711 counts, use -1.
# If positive thrust produces increasing HX711 counts, use +1.
FORCE_SIGN = 1

# HX711 sampling
TARE_SAMPLES = 100
SAMPLES_RUN = 1
TARE_SAMPLE_DELAY_MS = 20

# ESC pins
ESC_PINS = [13, 14]

# ESC / PWM
ESC_FREQ_HZ = 50
PWM_SAFE_US = 1000
PWM_HARD_MIN_US = 1100
PWM_HARD_MAX_US = 1950

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
BATTERY_STALE_MS = 1000

# Safety/countdown timing
INITIAL_SAFETY_COUNTDOWN_S = 10
ARM_COUNTDOWN_S = 5
ARM_TIME_MS = 5000
PRE_RUN_SETTLE_MS = 3000
POST_RUN_IDLE_MS = 1500
BASELINE_HOLD_MS = 3000


# ============================================================
# PRPS frequency-control toggles
# ============================================================

# Recommended first pass based on:
#   tau ≈ 0.4 s
#   actuator bandwidth ≈ 1/(2*pi*tau) ≈ 0.4 Hz
#
# Goal:
#   Excite below, near, and above the actuator rolloff.
#
# Do not go too high at first. Your load cell, ESC, motor, prop, and sampling
# may make high-frequency estimates look meaningful when they are not.
USE_MANUAL_FREQUENCY_LIST = True

MANUAL_EXCITED_FREQS_HZ = [
    0.05,
    0.075,
    0.10,
    0.15,
    0.20,
    0.30,
    0.40,
    0.60,
    0.80,
    1.00,
    1.25,
    1.50,
    2.00,
]

# Used only if USE_MANUAL_FREQUENCY_LIST = False.
AUTO_FREQ_MIN_HZ = 0.05
AUTO_FREQ_MAX_HZ = 2.00
AUTO_NUM_FREQS = 13
AUTO_FREQ_SPACING = "log"          # "linear" or "log"

# Periodic-signal settings.
#
# PRPS_PERIOD_S controls the frequency resolution:
#   fundamental frequency = 1 / PRPS_PERIOD_S
#
# Example:
#   PRPS_PERIOD_S = 40
#   fundamental = 0.025 Hz
#
# Frequencies are snapped to integer multiples of the fundamental frequency
# so that the signal is exactly periodic.
PRPS_PERIOD_S = 40.0

# For first debug run, 2 is enough.
# For full frequency-response estimation, use 4 or more.
NUM_PERIODS_PER_RUN = 4

# Command update period.
#
# This is the commanded PWM update rate, not necessarily the HX711 sample rate.
#
# Rule of thumb:
#   command_update_rate >= 10 * highest_excited_frequency
#
# For f_max = 2 Hz:
#   20 Hz command update is a decent first pass.
COMMAND_UPDATE_DT_MS = 50          # 50 ms = 20 Hz

# If True, print requested frequency, snapped frequency, and Fourier bin.
PRINT_FREQUENCY_PLAN = True

# If True, use random phase for each excited sinusoid.
# If False, all phases are zero. Usually keep True.
RANDOMIZE_PHASES = True

# If True, each acquisition set gets new random phases.
# If False, all sets use same PRPS waveform when seed is fixed.
NEW_PHASES_EACH_SET = True

# Memory-safe version:
#   Phase search is disabled because repeated waveform generation is RAM-heavy.
#   Later, if needed, do phase/crest-factor optimization offline on desktop.
USE_PHASE_SEARCH = False
NUM_PHASE_SEARCH_TRIALS = 1

# This streaming version does not remove mean by storing the waveform.
# For integer Fourier bins over a full period, the sinusoidal sum should already
# have approximately zero mean over one period.
REMOVE_WAVEFORM_MEAN = False

# Normalize by estimated peak over one period.
NORMALIZE_TO_PEAK = True

# Round PWM command to integer microseconds.
ROUND_PWM_TO_INT = True


# ============================================================
# Test run definitions
# ============================================================

# Each entry:
#   (run_name, center_pwm_us, amplitude_pwm_us)
#
# Duration is determined by:
#   PRPS_PERIOD_S * NUM_PERIODS_PER_RUN
#
# This is deliberate: PRPS should complete whole periods.
TEST_RUNS = [
    ("global_1100_1950", 1525, 425),
    ("local_1400_1650", 1525, 125),
    ("local_1700_1850", 1775, 75),
]

# For serious identification, False avoids always running the same range at
# the same point in the battery/thermal timeline.
FIXED_RUN_ORDER = False

PRINT_EVERY_N_SAMPLES = 50


# ============================================================
# Basic helper functions
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


def random_unit_float():
    # Returns approximately [0, 1).
    return urandom.getrandbits(24) / float(1 << 24)


def maybe_shuffle_runs(runs, run_order_seed):
    if FIXED_RUN_ORDER:
        return runs

    seed_urandom(run_order_seed, "Run-order")

    runs = list(runs)
    for i in range(len(runs) - 1, 0, -1):
        j = urandom.getrandbits(16) % (i + 1)
        runs[i], runs[j] = runs[j], runs[i]
    return runs


def csv_value(value):
    """
    Converts None to blank for cleaner CSV logging.
    """
    if value is None:
        return ""
    return value


def make_log_filename(set_index, prps_seed):
    return "{}_seed{}.csv".format(
        LOG_FILE_BASE,
        int(prps_seed),
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
# PRPS generation - memory-safe streaming version
# ============================================================

def make_auto_frequency_list():
    freqs = []

    if AUTO_NUM_FREQS <= 1:
        return [AUTO_FREQ_MIN_HZ]

    if AUTO_FREQ_SPACING == "linear":
        for i in range(AUTO_NUM_FREQS):
            alpha = i / float(AUTO_NUM_FREQS - 1)
            f = AUTO_FREQ_MIN_HZ + alpha * (AUTO_FREQ_MAX_HZ - AUTO_FREQ_MIN_HZ)
            freqs.append(f)
        return freqs

    if AUTO_FREQ_SPACING == "log":
        log_min = math.log(AUTO_FREQ_MIN_HZ)
        log_max = math.log(AUTO_FREQ_MAX_HZ)

        for i in range(AUTO_NUM_FREQS):
            alpha = i / float(AUTO_NUM_FREQS - 1)
            f = math.exp(log_min + alpha * (log_max - log_min))
            freqs.append(f)
        return freqs

    raise ValueError("AUTO_FREQ_SPACING must be 'linear' or 'log'")


def get_requested_frequencies_hz():
    if USE_MANUAL_FREQUENCY_LIST:
        return list(MANUAL_EXCITED_FREQS_HZ)
    return make_auto_frequency_list()


def snap_frequencies_to_period_bins(requested_freqs_hz, period_s):
    """
    Snap requested frequencies to exact Fourier bins of the PRPS period.

    Fundamental frequency:
        f0 = 1 / period_s

    Allowed frequencies:
        f_k = k * f0
    """
    fundamental_hz = 1.0 / period_s

    bins = []
    snapped_freqs = []
    requested_kept = []
    used_bins = {}

    for f_req in requested_freqs_hz:
        if f_req <= 0:
            continue

        k = int(round(f_req / fundamental_hz))
        if k < 1:
            k = 1

        # Avoid duplicate bins after snapping.
        if k in used_bins:
            continue

        used_bins[k] = True

        bins.append(k)
        snapped_freqs.append(k * fundamental_hz)
        requested_kept.append(f_req)

    return requested_kept, snapped_freqs, bins


def generate_random_phases(num_freqs):
    phases = []

    for _ in range(num_freqs):
        if RANDOMIZE_PHASES:
            phases.append(2.0 * math.pi * random_unit_float())
        else:
            phases.append(0.0)

    return phases


def prps_raw_value(sample_index, samples_per_period, bins, phases):
    """
    Compute one unnormalized PRPS sample.

    The frequency bin k means:
        f_k = k / PRPS_PERIOD_S

    For sample n in one period:
        sin(2*pi*k*n/N + phase)

    where:
        N = samples_per_period

    This function is intentionally streaming:
        - It does not allocate a waveform array.
        - It computes one sample at a time.
    """
    value = 0.0

    for i in range(len(bins)):
        k = bins[i]
        phi = phases[i]
        angle = 2.0 * math.pi * k * sample_index / samples_per_period + phi
        value += math.sin(angle)

    return value


def estimate_peak_and_rms(samples_per_period, bins, phases):
    """
    Estimate normalization constants without storing the waveform.
    """
    peak = 0.0
    sum_sq = 0.0

    for n in range(samples_per_period):
        x = prps_raw_value(n, samples_per_period, bins, phases)

        ax = abs(x)
        if ax > peak:
            peak = ax

        sum_sq += x * x

    if peak <= 0:
        peak = 1.0

    rms = math.sqrt(sum_sq / samples_per_period)

    if rms <= 0:
        crest = 999.0
    else:
        crest = peak / rms

    return peak, rms, crest


def generate_prps_plan(seed_value):
    """
    Generates only the PRPS metadata needed to compute samples on the fly.
    Does NOT allocate a full waveform array.
    """
    requested_freqs = get_requested_frequencies_hz()

    requested_kept, snapped_freqs, bins = snap_frequencies_to_period_bins(
        requested_freqs,
        PRPS_PERIOD_S,
    )

    if len(bins) == 0:
        raise ValueError("No valid PRPS frequencies selected.")

    dt_s = COMMAND_UPDATE_DT_MS / 1000.0
    samples_per_period = int(round(PRPS_PERIOD_S / dt_s))

    if samples_per_period < 4:
        raise ValueError("Too few samples per PRPS period.")

    if PRINT_FREQUENCY_PLAN:
        print()
        print("PRPS frequency plan:")
        print("  Period_s:", PRPS_PERIOD_S)
        print("  Fundamental_Hz:", 1.0 / PRPS_PERIOD_S)
        print("  Command update dt_ms:", COMMAND_UPDATE_DT_MS)
        print("  Command update rate_Hz:", 1000.0 / COMMAND_UPDATE_DT_MS)
        print("  Samples per period:", samples_per_period)
        print("  Excited frequencies:")

        for i in range(len(bins)):
            print(
                "    requested:",
                requested_kept[i],
                "Hz -> snapped:",
                snapped_freqs[i],
                "Hz, bin:",
                bins[i],
            )

    seed_urandom(seed_value, "PRPS phase")
    phases = generate_random_phases(len(bins))

    peak, rms, crest = estimate_peak_and_rms(
        samples_per_period,
        bins,
        phases,
    )

    print()
    print("Selected PRPS streaming signal:")
    print("  Number of excited frequencies:", len(bins))
    print("  Samples per period:", samples_per_period)
    print("  Periods per run:", NUM_PERIODS_PER_RUN)
    print("  Total run duration_s:", PRPS_PERIOD_S * NUM_PERIODS_PER_RUN)
    print("  Peak normalization:", peak)
    print("  RMS before normalization:", rms)
    print("  Crest factor:", crest)

    return requested_kept, snapped_freqs, bins, phases, peak, samples_per_period, crest


def get_normalized_prps_sample(sample_index, samples_per_period, bins, phases, peak):
    x = prps_raw_value(sample_index, samples_per_period, bins, phases)

    if NORMALIZE_TO_PEAK and peak > 0:
        x = x / peak

    return x


def prps_sample_to_pwm(
    center_pwm,
    amplitude_pwm,
    sample_index,
    samples_per_period,
    bins,
    phases,
    peak,
):
    x = get_normalized_prps_sample(
        sample_index,
        samples_per_period,
        bins,
        phases,
        peak,
    )

    pwm = center_pwm + amplitude_pwm * x
    pwm = clamp(pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)

    if ROUND_PWM_TO_INT:
        return int(round(pwm))

    return pwm


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
    prps_seed,
    run_order_seed,
    run_name,
    segment,
    pwm_us,
    offset_count,
    phase,
    period_index,
    period_sample_index,
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
        "{},{:.3f},{},{},{},{},{},{},{:.3f},{:.3f},{:.6f},{},{},{},{},{},{},{},{},{},{}\n".format(
            t_ms,
            t_s,
            int(set_index),
            int(prps_seed),
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
            int(period_index),
            int(period_sample_index),
        )
    )

    sample_counter += 1

    if sample_counter % PRINT_EVERY_N_SAMPLES == 0:
        print(
            "sample", sample_counter,
            "set", set_index,
            "seed", prps_seed,
            "phase", phase,
            "pwm_us", int(pwm_us),
            "force_N", thrust_N,
            "V", battery_voltage_V,
            "A", battery_current_A,
            "batt_age_ms", battery_age_ms,
            "period", period_index,
            "period_sample", period_sample_index,
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
    prps_seed,
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
        loop_start_ms = ticks_ms()

        write_sample(
            f=f,
            hx=hx,
            mav_batt=mav_batt,
            t0_ms=t0_ms,
            set_index=set_index,
            prps_seed=prps_seed,
            run_order_seed=run_order_seed,
            run_name=run_name,
            segment=segment,
            pwm_us=pwm_us,
            offset_count=offset_count,
            phase=phase,
            period_index=-1,
            period_sample_index=-1,
        )

        elapsed_ms = ticks_diff(ticks_ms(), loop_start_ms)
        remaining_ms = COMMAND_UPDATE_DT_MS - elapsed_ms

        if remaining_ms > 0:
            sleep_ms(remaining_ms)


def run_prps_sequence(
    f,
    hx,
    mav_batt,
    escs,
    t0_ms,
    set_index,
    prps_seed,
    run_order_seed,
    run_name,
    center_pwm,
    amplitude_pwm,
    offset_count,
    prps_plan,
):
    requested_freqs, snapped_freqs, bins, phases, peak, samples_per_period, crest_factor = prps_plan

    low = clamp(center_pwm - amplitude_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)
    high = clamp(center_pwm + amplitude_pwm, PWM_HARD_MIN_US, PWM_HARD_MAX_US)

    print()
    print("RUN:", run_name)
    print("Set index:", set_index)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("Center PWM:", center_pwm)
    print("Amplitude:", amplitude_pwm)
    print("Command bounds:", low, "to", high, "us")
    print("PRPS period:", PRPS_PERIOD_S, "s")
    print("Number of periods:", NUM_PERIODS_PER_RUN)
    print("Total duration:", PRPS_PERIOD_S * NUM_PERIODS_PER_RUN, "s")
    print("Command update dt:", COMMAND_UPDATE_DT_MS, "ms")
    print("Samples per period:", samples_per_period)
    print("Crest factor:", crest_factor)

    print("Pre-run baseline...")
    hold_and_log(
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        set_index=set_index,
        prps_seed=prps_seed,
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
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="settle",
        pwm_us=center_pwm,
        duration_ms=PRE_RUN_SETTLE_MS,
        offset_count=offset_count,
        phase="settle",
    )

    print("PRPS running...")

    for period_index in range(NUM_PERIODS_PER_RUN):
        for period_sample_index in range(samples_per_period):

            pwm_cmd = prps_sample_to_pwm(
                center_pwm=center_pwm,
                amplitude_pwm=amplitude_pwm,
                sample_index=period_sample_index,
                samples_per_period=samples_per_period,
                bins=bins,
                phases=phases,
                peak=peak,
            )

            set_pwm_us(escs, pwm_cmd)

            update_start_ms = ticks_ms()

            write_sample(
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                t0_ms=t0_ms,
                set_index=set_index,
                prps_seed=prps_seed,
                run_order_seed=run_order_seed,
                run_name=run_name,
                segment=period_sample_index,
                pwm_us=pwm_cmd,
                offset_count=offset_count,
                phase="prps",
                period_index=period_index,
                period_sample_index=period_sample_index,
            )

            elapsed_ms = ticks_diff(ticks_ms(), update_start_ms)
            remaining_ms = COMMAND_UPDATE_DT_MS - elapsed_ms

            if remaining_ms > 0:
                sleep_ms(remaining_ms)

    print("Post-run baseline...")
    hold_and_log(
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        set_index=set_index,
        prps_seed=prps_seed,
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
# Metadata and acquisition set runner
# ============================================================

def write_prps_metadata_file(log_file, requested_freqs, snapped_freqs, bins, crest_factor):
    metadata_file = log_file.replace(".csv", "_frequency_plan.txt")

    with open(metadata_file, "w") as f:
        f.write("PRPS frequency plan\n")
        f.write("log_file={}\n".format(log_file))
        f.write("PRPS_PERIOD_S={}\n".format(PRPS_PERIOD_S))
        f.write("fundamental_Hz={}\n".format(1.0 / PRPS_PERIOD_S))
        f.write("COMMAND_UPDATE_DT_MS={}\n".format(COMMAND_UPDATE_DT_MS))
        f.write("command_update_rate_Hz={}\n".format(1000.0 / COMMAND_UPDATE_DT_MS))
        f.write("NUM_PERIODS_PER_RUN={}\n".format(NUM_PERIODS_PER_RUN))
        f.write("crest_factor={}\n".format(crest_factor))
        f.write("\n")
        f.write("requested_Hz,snapped_Hz,bin\n")

        for i in range(len(bins)):
            f.write("{},{},{}\n".format(
                requested_freqs[i],
                snapped_freqs[i],
                bins[i],
            ))

    print("Saved metadata:", metadata_file)


def run_acquisition_set(
    set_index,
    hx,
    mav_batt,
    escs,
    offset_count,
):
    global sample_counter

    prps_seed = BASE_PRPS_SEED + (set_index - 1)
    run_order_seed = prps_seed + RUN_ORDER_SEED_OFFSET
    log_file = make_log_filename(set_index, prps_seed)

    print()
    print("============================================================")
    print("ACQUISITION SET", set_index, "of", NUM_ACQUISITION_SETS)
    print("Log file:", log_file)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("Fixed run order:", FIXED_RUN_ORDER)
    print("============================================================")

    runs = maybe_shuffle_runs(TEST_RUNS, run_order_seed)

    print()
    print("Planned runs for this set:")
    for run_name, center, amp in runs:
        print(
            " -",
            run_name,
            "center", center,
            "amp", amp,
            "duration_s", PRPS_PERIOD_S * NUM_PERIODS_PER_RUN,
        )

    # Generate PRPS plan.
    #
    # If NEW_PHASES_EACH_SET is False, all sets use BASE_PRPS_SEED.
    # If True, each set gets its own deterministic random phase waveform.
    if NEW_PHASES_EACH_SET:
        waveform_seed = prps_seed
    else:
        waveform_seed = BASE_PRPS_SEED

    prps_plan = generate_prps_plan(waveform_seed)
    requested_freqs, snapped_freqs, bins, phases, peak, samples_per_period, crest_factor = prps_plan

    write_prps_metadata_file(
        log_file,
        requested_freqs,
        snapped_freqs,
        bins,
        crest_factor,
    )

    print_battery_snapshot(mav_batt, "Before set")

    sample_counter = 0

    with open(log_file, "w") as f:
        f.write(
            "t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,pwm_us,"
            "raw_count,tared_count,force_N,"
            "battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,"
            "mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,"
            "phase,period_index,period_sample_index\n"
        )

        t0_ms = ticks_ms()

        for run_name, center, amp in runs:
            run_prps_sequence(
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                escs=escs,
                t0_ms=t0_ms,
                set_index=set_index,
                prps_seed=prps_seed,
                run_order_seed=run_order_seed,
                run_name=run_name,
                center_pwm=center,
                amplitude_pwm=amp,
                offset_count=offset_count,
                prps_plan=prps_plan,
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
    print("Dynamic PRPS thrust data acquisition")
    print("Output base:", LOG_FILE_BASE)
    print("Number of acquisition sets:", NUM_ACQUISITION_SETS)
    print("Base PRPS seed:", BASE_PRPS_SEED)
    print("Cooldown between sets:", COOLDOWN_BETWEEN_SETS_MS / 1000.0, "s")
    print("SCALE_G_PER_COUNT:", SCALE_G_PER_COUNT)
    print("SCALE_N_PER_COUNT:", SCALE_N_PER_COUNT)
    print("FORCE_SIGN:", FORCE_SIGN)

    print()
    print("PRPS settings:")
    print("  USE_MANUAL_FREQUENCY_LIST:", USE_MANUAL_FREQUENCY_LIST)
    print("  PRPS_PERIOD_S:", PRPS_PERIOD_S)
    print("  NUM_PERIODS_PER_RUN:", NUM_PERIODS_PER_RUN)
    print("  COMMAND_UPDATE_DT_MS:", COMMAND_UPDATE_DT_MS)
    print("  USE_PHASE_SEARCH:", USE_PHASE_SEARCH)
    print("  NUM_PHASE_SEARCH_TRIALS:", NUM_PHASE_SEARCH_TRIALS)
    print("  NEW_PHASES_EACH_SET:", NEW_PHASES_EACH_SET)
    print("  NORMALIZE_TO_PEAK:", NORMALIZE_TO_PEAK)

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