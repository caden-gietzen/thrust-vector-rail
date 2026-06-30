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
#
# Structure note:
#   This file is intentionally organized into functions so VS Code's Outline
#   panel is useful for navigation. The main execution path is:
#
#       main()
#           print_startup_banner()
#           setup_hardware()
#           tare_load_cell()
#           arm_escs()
#           run_all_acquisition_sets()
#           safe_shutdown()

from machine import Pin, PWM, UART
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711
from lib.mavlink_battery import MavlinkBatteryReader
import urandom
import math


# ============================================================
# Configuration
# ============================================================


def make_config():
    """
    Return all user-editable settings in one dictionary.

    Keeping settings inside one function makes the VS Code Outline cleaner and
    prevents a very long wall of global constants at the top of the file.

    If /run_config.json exists on the Pico, values in that JSON file override
    this default configuration. This lets the laptop orchestrator run one
    bounded test segment at a time, pull the CSV, delete it from the Pico,
    and then launch the next segment with a new config.
    """
    cfg = {
        # ----------------------------------------------------
        # File / acquisition settings
        # ----------------------------------------------------
        "LOG_FILE_BASE": "thrust_prps_daq_voltage",
        "NUM_ACQUISITION_SETS": 1,
        "BASE_PRPS_SEED": 2001,
        "RUN_ORDER_SEED_OFFSET": 50000,
        "COOLDOWN_BETWEEN_SETS_MS": 30000,
        "RETARE_EACH_SET": True,

        # ----------------------------------------------------
        # HX711 / load-cell settings
        # ----------------------------------------------------
        "HX711_DAT_PIN": 20,
        "HX711_SCK_PIN": 21,
        "HX711_GAIN": 128,
        "SCALE_G_PER_COUNT": 0.002409592,
        "FORCE_SIGN": 1,
        "TARE_SAMPLES": 100,
        "SAMPLES_RUN": 1,
        "TARE_SAMPLE_DELAY_MS": 20,

        # ----------------------------------------------------
        # ESC / PWM settings
        # ----------------------------------------------------
        "ESC_PINS": [13, 14],
        "ESC_FREQ_HZ": 50,
        "PWM_SAFE_US": 1000,
        "PWM_HARD_MIN_US": 1100,
        "PWM_HARD_MAX_US": 1950,

        # ----------------------------------------------------
        # Pixhawk MAVLink battery telemetry settings
        # ----------------------------------------------------
        "MAVLINK_UART_ID": 0,
        "MAVLINK_TX_PIN": 0,
        "MAVLINK_RX_PIN": 1,
        "MAVLINK_BAUDRATE": 57600,
        "BATTERY_STALE_MS": 1000,

        # ----------------------------------------------------
        # Safety/countdown timing
        # ----------------------------------------------------
        "INITIAL_SAFETY_COUNTDOWN_S": 10,
        "ARM_COUNTDOWN_S": 5,
        "ARM_TIME_MS": 5000,
        "PRE_RUN_SETTLE_MS": 3000,
        "POST_RUN_IDLE_MS": 1500,
        "BASELINE_HOLD_MS": 3000,

        # ----------------------------------------------------
        # PRPS frequency-control toggles
        # ----------------------------------------------------
        "USE_MANUAL_FREQUENCY_LIST": False,
        "MANUAL_EXCITED_FREQS_HZ": [
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
        ],
        "AUTO_FREQ_MIN_HZ": 0.05,
        "AUTO_FREQ_MAX_HZ": 3.00,
        "AUTO_NUM_FREQS": 25,
        "AUTO_FREQ_SPACING": "log",
        "PRPS_PERIOD_S": 40.0,
        "NUM_PERIODS_PER_RUN": 3,
        "COMMAND_UPDATE_DT_MS": 20,
        "PRINT_FREQUENCY_PLAN": False,
        "SAVE_FREQUENCY_PLAN": False,
        "RANDOMIZE_PHASES": True,
        "NEW_PHASES_EACH_SET": True,
        "USE_PHASE_SEARCH": False,
        "NUM_PHASE_SEARCH_TRIALS": 1,
        "REMOVE_WAVEFORM_MEAN": False,
        "NORMALIZE_TO_PEAK": True,
        "ROUND_PWM_TO_INT": True,

        # ----------------------------------------------------
        # Test run definitions
        # ----------------------------------------------------
        "TEST_RUNS": [
            #("global_1100_1950", 1525, 425),
            ("local_1100_1350", 1225, 125),
            #("local_1400_1650", 1525, 125),
            #("local_1700_1850", 1775, 75),
        ],
        "FIXED_RUN_ORDER": False,
        "PRINT_EVERY_N_SAMPLES": 50,
    }

    cfg = apply_external_config_if_present(cfg)
    return cfg



def load_json_file_if_exists(path):
    """
    MicroPython-safe JSON loader.

    Returns:
        dict/list from JSON if the file exists and is valid.
        None if the file does not exist.
    """
    try:
        import ujson as json
    except ImportError:
        import json

    try:
        with open(path, "r") as f:
            return json.load(f)
    except OSError:
        return None


def normalize_external_config(external):
    """
    Accept either of these uploaded formats:

        {"LOG_FILE_BASE": "...", "BASE_PRPS_SEED": 2001}

    or:

        {"name": "segment001", "config": {"LOG_FILE_BASE": "..."}}

    The laptop orchestrator should normally upload only the inner config, but
    accepting both formats makes the Pico side more tolerant during manual tests.
    """
    if external is None:
        return None

    if isinstance(external, dict) and "config" in external:
        maybe_config = external["config"]
        if isinstance(maybe_config, dict):
            return maybe_config

    if isinstance(external, dict):
        return external

    raise ValueError("External run config must be a JSON object/dictionary.")


def coerce_config_types(cfg):
    """
    JSON has arrays, not tuples. This converts fields that the existing code
    treats as tuple-like back into tuple-friendly structures.
    """
    if "TEST_RUNS" in cfg:
        coerced_runs = []
        for run in cfg["TEST_RUNS"]:
            if len(run) != 3:
                raise ValueError("Each TEST_RUNS entry must be [run_name, center_pwm, amplitude_pwm].")
            coerced_runs.append((run[0], run[1], run[2]))
        cfg["TEST_RUNS"] = coerced_runs

    return cfg


def apply_external_config_if_present(cfg):
    """
    Overrides the default config with /run_config.json if present.

    This is intentionally generic: the Pico script does not know about the
    full orchestration plan. It only receives and applies the config for the
    current segment.
    """
    config_path = "/run_config.json"
    external = load_json_file_if_exists(config_path)

    # Some MicroPython builds treat relative root paths more reliably than
    # absolute root paths. This fallback is harmless if the absolute path worked.
    if external is None:
        config_path = "run_config.json"
        external = load_json_file_if_exists(config_path)

    external = normalize_external_config(external)

    if external is None:
        return coerce_config_types(cfg)

    print("Loaded external run config:", config_path)

    for key in external:
        cfg[key] = external[key]

    return coerce_config_types(cfg)



def finalize_config(cfg):
    """
    Add derived values to the configuration dictionary.
    """
    cfg["SCALE_N_PER_COUNT"] = cfg["SCALE_G_PER_COUNT"] * 9.80665 / 1000.0
    return cfg


# ============================================================
# Small utility helpers
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


def csv_value(value):
    """
    Converts None to blank for cleaner CSV logging.
    """
    if value is None:
        return ""
    return value


def seed_urandom(seed_value, label):
    try:
        urandom.seed(seed_value)
        print(label, "seed:", seed_value)
    except Exception:
        print("NOTE: urandom.seed unavailable for", label, "; sequence may not be exactly repeatable.")


def random_unit_float():
    # Returns approximately [0, 1).
    return urandom.getrandbits(24) / float(1 << 24)


def make_log_filename(cfg, set_index, prps_seed):
    # set_index is intentionally not included in the filename here because the
    # seed already differentiates acquisition sets under the current seed plan.
    return "{}_seed{}.csv".format(
        cfg["LOG_FILE_BASE"],
        int(prps_seed),
    )


# ============================================================
# PWM / ESC helpers
# ============================================================


def pwm_us_to_duty_u16(pwm_us):
    # 50 Hz period = 20,000 us
    return int(int(pwm_us) * 65535 / 20000)


def set_pwm_us(cfg, escs, pwm_us):
    pwm_us = clamp(int(pwm_us), cfg["PWM_SAFE_US"], 2000)
    duty = pwm_us_to_duty_u16(pwm_us)
    for esc in escs:
        esc.duty_u16(duty)


def setup_escs(cfg):
    print("Initializing ESC PWM outputs...")
    print("ESC pins:", cfg["ESC_PINS"])

    escs = []
    for pin_num in cfg["ESC_PINS"]:
        esc = PWM(Pin(pin_num, Pin.OUT))
        esc.freq(cfg["ESC_FREQ_HZ"])
        escs.append(esc)

    set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
    print("ESC outputs set to safe PWM:", cfg["PWM_SAFE_US"], "us")
    return escs


def arm_escs(cfg, escs):
    countdown(
        "Arming ESCs at safe PWM. Keep clear of motors/propellers.",
        cfg["ARM_COUNTDOWN_S"],
    )

    set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
    sleep_ms(cfg["ARM_TIME_MS"])
    print("ESC arming complete.")


# ============================================================
# HX711 / force helpers
# ============================================================


def setup_hx711(cfg):
    print("Initializing HX711...")
    print("DAT pin:", cfg["HX711_DAT_PIN"])
    print("SCK pin:", cfg["HX711_SCK_PIN"])

    data_pin = Pin(cfg["HX711_DAT_PIN"], Pin.IN)
    clock_pin = Pin(cfg["HX711_SCK_PIN"], Pin.OUT)

    # HX711.__init__ forces a power cycle and settles the amplifier, so no
    # manual clock-low + settle is needed here.
    hx = HX711(clock_pin, data_pin, gain=cfg["HX711_GAIN"])
    print("HX711 initialized.")
    return hx


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


def force_N_from_raw(cfg, raw_count, offset_count):
    tared_count = raw_count - offset_count
    force_N = cfg["FORCE_SIGN"] * tared_count * cfg["SCALE_N_PER_COUNT"]
    return tared_count, force_N


def tare_load_cell(cfg, hx, message):
    countdown(
        message,
        cfg["INITIAL_SAFETY_COUNTDOWN_S"],
    )

    print("Taring with", cfg["TARE_SAMPLES"], "samples...")
    offset_count = read_average(
        hx,
        cfg["TARE_SAMPLES"],
        delay_ms=cfg["TARE_SAMPLE_DELAY_MS"],
    )
    print("Offset count:", offset_count)
    return offset_count


def retare_load_cell_quick(cfg, hx):
    print("Re-taring load cell for this acquisition set...")
    offset_count = read_average(
        hx,
        cfg["TARE_SAMPLES"],
        delay_ms=cfg["TARE_SAMPLE_DELAY_MS"],
    )
    print("Set offset count:", offset_count)
    return offset_count


# ============================================================
# MAVLink battery telemetry helpers
# ============================================================


def setup_mavlink_battery(cfg):
    print("Initializing MAVLink battery UART...")
    print("UART:", cfg["MAVLINK_UART_ID"])
    print("TX pin:", cfg["MAVLINK_TX_PIN"])
    print("RX pin:", cfg["MAVLINK_RX_PIN"])
    print("Baudrate:", cfg["MAVLINK_BAUDRATE"])

    uart = UART(
        cfg["MAVLINK_UART_ID"],
        baudrate=cfg["MAVLINK_BAUDRATE"],
        tx=Pin(cfg["MAVLINK_TX_PIN"]),
        rx=Pin(cfg["MAVLINK_RX_PIN"]),
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


def get_battery_snapshot(mav_batt):
    if mav_batt is None:
        return "", "", "", "", "", "", ""

    (
        battery_voltage_V,
        battery_current_A,
        battery_remaining_pct,
        battery_age_ms,
        mavlink_total_packets,
        mavlink_sys_status_packets,
        mavlink_bad_frames,
    ) = mav_batt.snapshot()

    return (
        csv_value(battery_voltage_V),
        csv_value(battery_current_A),
        csv_value(battery_remaining_pct),
        csv_value(battery_age_ms),
        mavlink_total_packets,
        mavlink_sys_status_packets,
        mavlink_bad_frames,
    )


# ============================================================
# PRPS frequency planning
# ============================================================


def make_auto_frequency_list(cfg):
    freqs = []

    if cfg["AUTO_NUM_FREQS"] <= 1:
        return [cfg["AUTO_FREQ_MIN_HZ"]]

    if cfg["AUTO_FREQ_SPACING"] == "linear":
        for i in range(cfg["AUTO_NUM_FREQS"]):
            alpha = i / float(cfg["AUTO_NUM_FREQS"] - 1)
            f = cfg["AUTO_FREQ_MIN_HZ"] + alpha * (cfg["AUTO_FREQ_MAX_HZ"] - cfg["AUTO_FREQ_MIN_HZ"])
            freqs.append(f)
        return freqs

    if cfg["AUTO_FREQ_SPACING"] == "log":
        log_min = math.log(cfg["AUTO_FREQ_MIN_HZ"])
        log_max = math.log(cfg["AUTO_FREQ_MAX_HZ"])

        for i in range(cfg["AUTO_NUM_FREQS"]):
            alpha = i / float(cfg["AUTO_NUM_FREQS"] - 1)
            f = math.exp(log_min + alpha * (log_max - log_min))
            freqs.append(f)
        return freqs

    raise ValueError("AUTO_FREQ_SPACING must be 'linear' or 'log'")


def get_requested_frequencies_hz(cfg):
    if cfg["USE_MANUAL_FREQUENCY_LIST"]:
        return list(cfg["MANUAL_EXCITED_FREQS_HZ"])
    return make_auto_frequency_list(cfg)


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


def print_frequency_plan(cfg, requested_kept, snapped_freqs, bins, samples_per_period):
    if not cfg["PRINT_FREQUENCY_PLAN"]:
        return

    print()
    print("PRPS frequency plan:")
    print("  Period_s:", cfg["PRPS_PERIOD_S"])
    print("  Fundamental_Hz:", 1.0 / cfg["PRPS_PERIOD_S"])
    print("  Command update dt_ms:", cfg["COMMAND_UPDATE_DT_MS"])
    print("  Command update rate_Hz:", 1000.0 / cfg["COMMAND_UPDATE_DT_MS"])
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


def generate_random_phases(cfg, num_freqs):
    phases = []

    for _ in range(num_freqs):
        if cfg["RANDOMIZE_PHASES"]:
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


def print_selected_prps_signal(cfg, bins, samples_per_period, peak, rms, crest):
    print()
    print("Selected PRPS streaming signal:")
    print("  Number of excited frequencies:", len(bins))
    print("  Samples per period:", samples_per_period)
    print("  Periods per run:", cfg["NUM_PERIODS_PER_RUN"])
    print("  Total run duration_s:", cfg["PRPS_PERIOD_S"] * cfg["NUM_PERIODS_PER_RUN"])
    print("  Peak normalization:", peak)
    print("  RMS before normalization:", rms)
    print("  Crest factor:", crest)


def generate_prps_plan(cfg, seed_value):
    """
    Generates only the PRPS metadata needed to compute samples on the fly.
    Does NOT allocate a full waveform array.
    """
    requested_freqs = get_requested_frequencies_hz(cfg)

    requested_kept, snapped_freqs, bins = snap_frequencies_to_period_bins(
        requested_freqs,
        cfg["PRPS_PERIOD_S"],
    )

    if len(bins) == 0:
        raise ValueError("No valid PRPS frequencies selected.")

    dt_s = cfg["COMMAND_UPDATE_DT_MS"] / 1000.0
    samples_per_period = int(round(cfg["PRPS_PERIOD_S"] / dt_s))

    if samples_per_period < 4:
        raise ValueError("Too few samples per PRPS period.")

    print_frequency_plan(
        cfg,
        requested_kept,
        snapped_freqs,
        bins,
        samples_per_period,
    )

    seed_urandom(seed_value, "PRPS phase")
    phases = generate_random_phases(cfg, len(bins))

    peak, rms, crest = estimate_peak_and_rms(
        samples_per_period,
        bins,
        phases,
    )

    print_selected_prps_signal(
        cfg,
        bins,
        samples_per_period,
        peak,
        rms,
        crest,
    )

    return requested_kept, snapped_freqs, bins, phases, peak, samples_per_period, crest


def get_normalized_prps_sample(cfg, sample_index, samples_per_period, bins, phases, peak):
    x = prps_raw_value(sample_index, samples_per_period, bins, phases)

    if cfg["NORMALIZE_TO_PEAK"] and peak > 0:
        x = x / peak

    return x


def prps_sample_to_pwm(
    cfg,
    center_pwm,
    amplitude_pwm,
    sample_index,
    samples_per_period,
    bins,
    phases,
    peak,
):
    x = get_normalized_prps_sample(
        cfg,
        sample_index,
        samples_per_period,
        bins,
        phases,
        peak,
    )

    pwm = center_pwm + amplitude_pwm * x
    pwm = clamp(pwm, cfg["PWM_HARD_MIN_US"], cfg["PWM_HARD_MAX_US"])

    if cfg["ROUND_PWM_TO_INT"]:
        return int(round(pwm))

    return pwm


# ============================================================
# Metadata file writing
# ============================================================


def write_prps_metadata_file(cfg, log_file, requested_freqs, snapped_freqs, bins, crest_factor):
    metadata_file = log_file.replace(".csv", "_frequency_plan.txt")

    with open(metadata_file, "w") as f:
        f.write("PRPS frequency plan\n")
        f.write("log_file={}\n".format(log_file))
        f.write("PRPS_PERIOD_S={}\n".format(cfg["PRPS_PERIOD_S"]))
        f.write("fundamental_Hz={}\n".format(1.0 / cfg["PRPS_PERIOD_S"]))
        f.write("COMMAND_UPDATE_DT_MS={}\n".format(cfg["COMMAND_UPDATE_DT_MS"]))
        f.write("command_update_rate_Hz={}\n".format(1000.0 / cfg["COMMAND_UPDATE_DT_MS"]))
        f.write("NUM_PERIODS_PER_RUN={}\n".format(cfg["NUM_PERIODS_PER_RUN"]))
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


# ============================================================
# CSV sample logging
# ============================================================


def write_csv_header(f):
    f.write(
        "t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,pwm_us,"
        "raw_count,tared_count,force_N,"
        "battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,"
        "mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,"
        "phase,period_index,period_sample_index\n"
    )


def print_sample_status(cfg, sample_counter, set_index, prps_seed, phase, pwm_us, thrust_N,
                        battery_voltage_V, battery_current_A, battery_age_ms,
                        period_index, period_sample_index):
    if sample_counter % cfg["PRINT_EVERY_N_SAMPLES"] != 0:
        return

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
        if battery_age_ms > cfg["BATTERY_STALE_MS"]:
            print("WARNING: battery telemetry stale. age_ms:", battery_age_ms)


def write_sample(
    cfg,
    state,
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
    # Update MAVLink before the HX711 read.
    if mav_batt is not None:
        mav_batt.update()

    raw_count = read_average(hx, cfg["SAMPLES_RUN"])
    tared_count, thrust_N = force_N_from_raw(cfg, raw_count, offset_count)

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
    ) = get_battery_snapshot(mav_batt)

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

    state["sample_counter"] += 1

    print_sample_status(
        cfg,
        state["sample_counter"],
        set_index,
        prps_seed,
        phase,
        pwm_us,
        thrust_N,
        battery_voltage_V,
        battery_current_A,
        battery_age_ms,
        period_index,
        period_sample_index,
    )

    return raw_count, thrust_N


# ============================================================
# Hold / baseline routines
# ============================================================


def hold_and_log(
    cfg,
    state,
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
    set_pwm_us(cfg, escs, pwm_us)
    start_ms = ticks_ms()

    while ticks_diff(ticks_ms(), start_ms) < duration_ms:
        loop_start_ms = ticks_ms()

        write_sample(
            cfg=cfg,
            state=state,
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
        remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms

        if remaining_ms > 0:
            sleep_ms(remaining_ms)


def log_pre_run_baseline(cfg, state, f, hx, mav_batt, escs, t0_ms, set_index,
                         prps_seed, run_order_seed, run_name, center_pwm, offset_count):
    print("Pre-run baseline...")
    hold_and_log(
        cfg=cfg,
        state=state,
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
        duration_ms=cfg["BASELINE_HOLD_MS"],
        offset_count=offset_count,
        phase="baseline_pre",
    )


def log_center_settle(cfg, state, f, hx, mav_batt, escs, t0_ms, set_index,
                      prps_seed, run_order_seed, run_name, center_pwm, offset_count):
    print("Settling at center...")
    hold_and_log(
        cfg=cfg,
        state=state,
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
        duration_ms=cfg["PRE_RUN_SETTLE_MS"],
        offset_count=offset_count,
        phase="settle",
    )


def log_post_run_baseline(cfg, state, f, hx, mav_batt, escs, t0_ms, set_index,
                          prps_seed, run_order_seed, run_name, center_pwm, offset_count):
    print("Post-run baseline...")
    hold_and_log(
        cfg=cfg,
        state=state,
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
        duration_ms=cfg["BASELINE_HOLD_MS"],
        offset_count=offset_count,
        phase="baseline_post",
    )


# ============================================================
# PRPS run execution
# ============================================================


def print_run_banner(cfg, set_index, prps_seed, run_order_seed, run_name,
                     center_pwm, amplitude_pwm, samples_per_period, crest_factor):
    low = clamp(center_pwm - amplitude_pwm, cfg["PWM_HARD_MIN_US"], cfg["PWM_HARD_MAX_US"])
    high = clamp(center_pwm + amplitude_pwm, cfg["PWM_HARD_MIN_US"], cfg["PWM_HARD_MAX_US"])

    print()
    print("RUN:", run_name)
    print("Set index:", set_index)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("Center PWM:", center_pwm)
    print("Amplitude:", amplitude_pwm)
    print("Command bounds:", low, "to", high, "us")
    print("PRPS period:", cfg["PRPS_PERIOD_S"], "s")
    print("Number of periods:", cfg["NUM_PERIODS_PER_RUN"])
    print("Total duration:", cfg["PRPS_PERIOD_S"] * cfg["NUM_PERIODS_PER_RUN"], "s")
    print("Command update dt:", cfg["COMMAND_UPDATE_DT_MS"], "ms")
    print("Samples per period:", samples_per_period)
    print("Crest factor:", crest_factor)


def run_prps_periods(cfg, state, f, hx, mav_batt, escs, t0_ms, set_index,
                     prps_seed, run_order_seed, run_name, center_pwm,
                     amplitude_pwm, offset_count, bins, phases, peak,
                     samples_per_period):
    print("PRPS running...")

    for period_index in range(cfg["NUM_PERIODS_PER_RUN"]):
        for period_sample_index in range(samples_per_period):

            pwm_cmd = prps_sample_to_pwm(
                cfg=cfg,
                center_pwm=center_pwm,
                amplitude_pwm=amplitude_pwm,
                sample_index=period_sample_index,
                samples_per_period=samples_per_period,
                bins=bins,
                phases=phases,
                peak=peak,
            )

            set_pwm_us(cfg, escs, pwm_cmd)

            update_start_ms = ticks_ms()

            write_sample(
                cfg=cfg,
                state=state,
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
            remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms

            if remaining_ms > 0:
                sleep_ms(remaining_ms)


def run_prps_sequence(
    cfg,
    state,
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

    print_run_banner(
        cfg,
        set_index,
        prps_seed,
        run_order_seed,
        run_name,
        center_pwm,
        amplitude_pwm,
        samples_per_period,
        crest_factor,
    )

    log_pre_run_baseline(
        cfg,
        state,
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
        offset_count,
    )

    log_center_settle(
        cfg,
        state,
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
        offset_count,
    )

    run_prps_periods(
        cfg,
        state,
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
        bins,
        phases,
        peak,
        samples_per_period,
    )

    log_post_run_baseline(
        cfg,
        state,
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
        offset_count,
    )

    f.flush()
    print("Finished:", run_name)


# ============================================================
# Acquisition set planning
# ============================================================


def maybe_shuffle_runs(cfg, runs, run_order_seed):
    if cfg["FIXED_RUN_ORDER"]:
        return runs

    seed_urandom(run_order_seed, "Run-order")

    runs = list(runs)
    for i in range(len(runs) - 1, 0, -1):
        j = urandom.getrandbits(16) % (i + 1)
        runs[i], runs[j] = runs[j], runs[i]
    return runs


def get_set_seeds(cfg, set_index):
    prps_seed = cfg["BASE_PRPS_SEED"] + (set_index - 1)
    run_order_seed = prps_seed + cfg["RUN_ORDER_SEED_OFFSET"]
    return prps_seed, run_order_seed


def get_waveform_seed(cfg, prps_seed):
    if cfg["NEW_PHASES_EACH_SET"]:
        return prps_seed
    return cfg["BASE_PRPS_SEED"]


def print_acquisition_set_banner(cfg, set_index, log_file, prps_seed, run_order_seed):
    print()
    print("============================================================")
    print("ACQUISITION SET", set_index, "of", cfg["NUM_ACQUISITION_SETS"])
    print("Log file:", log_file)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("Fixed run order:", cfg["FIXED_RUN_ORDER"])
    print("============================================================")


def print_planned_runs(cfg, runs):
    print()
    print("Planned runs for this set:")
    for run_name, center, amp in runs:
        print(
            " -",
            run_name,
            "center", center,
            "amp", amp,
            "duration_s", cfg["PRPS_PERIOD_S"] * cfg["NUM_PERIODS_PER_RUN"],
        )


def prepare_acquisition_set(cfg, set_index):
    prps_seed, run_order_seed = get_set_seeds(cfg, set_index)
    log_file = make_log_filename(cfg, set_index, prps_seed)
    runs = maybe_shuffle_runs(cfg, cfg["TEST_RUNS"], run_order_seed)

    print_acquisition_set_banner(
        cfg,
        set_index,
        log_file,
        prps_seed,
        run_order_seed,
    )

    print_planned_runs(cfg, runs)

    waveform_seed = get_waveform_seed(cfg, prps_seed)
    prps_plan = generate_prps_plan(cfg, waveform_seed)

    requested_freqs, snapped_freqs, bins, phases, peak, samples_per_period, crest_factor = prps_plan

    if cfg["SAVE_FREQUENCY_PLAN"]:
        write_prps_metadata_file(
            cfg,
            log_file,
            requested_freqs,
            snapped_freqs,
            bins,
            crest_factor,
        )

    return prps_seed, run_order_seed, log_file, runs, prps_plan


# ============================================================
# Acquisition set execution
# ============================================================


def run_single_acquisition_file(cfg, state, log_file, runs, prps_plan, set_index,
                                prps_seed, run_order_seed, hx, mav_batt,
                                escs, offset_count):
    with open(log_file, "w") as f:
        write_csv_header(f)

        t0_ms = ticks_ms()

        for run_name, center, amp in runs:
            run_prps_sequence(
                cfg=cfg,
                state=state,
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
            set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
            sleep_ms(cfg["POST_RUN_IDLE_MS"])

        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)


def run_acquisition_set(cfg, set_index, hx, mav_batt, escs, offset_count):
    state = {"sample_counter": 0}

    prps_seed, run_order_seed, log_file, runs, prps_plan = prepare_acquisition_set(
        cfg,
        set_index,
    )

    print_battery_snapshot(mav_batt, "Before set")

    run_single_acquisition_file(
        cfg=cfg,
        state=state,
        log_file=log_file,
        runs=runs,
        prps_plan=prps_plan,
        set_index=set_index,
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        offset_count=offset_count,
    )

    print_battery_snapshot(mav_batt, "After set")
    print()
    print("Saved:", log_file)

    return log_file


def cooldown_after_set_if_needed(cfg, escs, set_index):
    if set_index >= cfg["NUM_ACQUISITION_SETS"]:
        return

    print("Returning to safe PWM for cooldown...")
    set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])

    cooldown_countdown(
        "Cooldown between acquisition sets. Keep area safe.",
        cfg["COOLDOWN_BETWEEN_SETS_MS"],
    )


def run_all_acquisition_sets(cfg, hx, mav_batt, escs, offset_count):
    saved_files = []

    for set_index in range(1, cfg["NUM_ACQUISITION_SETS"] + 1):
        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)

        if cfg["RETARE_EACH_SET"]:
            offset_count = retare_load_cell_quick(cfg, hx)

        saved_file = run_acquisition_set(
            cfg=cfg,
            set_index=set_index,
            hx=hx,
            mav_batt=mav_batt,
            escs=escs,
            offset_count=offset_count,
        )

        saved_files.append(saved_file)

        cooldown_after_set_if_needed(cfg, escs, set_index)

    return saved_files


# ============================================================
# Hardware orchestration
# ============================================================


def setup_hardware(cfg):
    hx = setup_hx711(cfg)
    escs = setup_escs(cfg)
    mav_batt = setup_mavlink_battery(cfg)
    return hx, escs, mav_batt


def safe_shutdown(cfg, hx, escs):
    print("Stopping motors.")

    if escs:
        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)

    if hx is not None:
        try:
            print("Powering down HX711.")
            hx.power_down()
        except Exception:
            pass


# ============================================================
# Startup / shutdown printing
# ============================================================


def print_startup_banner(cfg):
    print("Dynamic PRPS thrust data acquisition")
    print("Output base:", cfg["LOG_FILE_BASE"])
    print("Number of acquisition sets:", cfg["NUM_ACQUISITION_SETS"])
    print("Base PRPS seed:", cfg["BASE_PRPS_SEED"])
    print("Cooldown between sets:", cfg["COOLDOWN_BETWEEN_SETS_MS"] / 1000.0, "s")
    print("SCALE_G_PER_COUNT:", cfg["SCALE_G_PER_COUNT"])
    print("SCALE_N_PER_COUNT:", cfg["SCALE_N_PER_COUNT"])
    print("FORCE_SIGN:", cfg["FORCE_SIGN"])

    print()
    print("PRPS settings:")
    print("  USE_MANUAL_FREQUENCY_LIST:", cfg["USE_MANUAL_FREQUENCY_LIST"])
    print("  PRPS_PERIOD_S:", cfg["PRPS_PERIOD_S"])
    print("  NUM_PERIODS_PER_RUN:", cfg["NUM_PERIODS_PER_RUN"])
    print("  COMMAND_UPDATE_DT_MS:", cfg["COMMAND_UPDATE_DT_MS"])
    print("  USE_PHASE_SEARCH:", cfg["USE_PHASE_SEARCH"])
    print("  NUM_PHASE_SEARCH_TRIALS:", cfg["NUM_PHASE_SEARCH_TRIALS"])
    print("  NEW_PHASES_EACH_SET:", cfg["NEW_PHASES_EACH_SET"])
    print("  NORMALIZE_TO_PEAK:", cfg["NORMALIZE_TO_PEAK"])


def print_saved_files(saved_files):
    print()
    print("All acquisition sets complete.")
    print("Saved files:")
    for filename in saved_files:
        print(" -", filename)


# ============================================================
# Main execution path
# ============================================================


def main():
    cfg = finalize_config(make_config())

    hx = None
    escs = []
    mav_batt = None

    try:
        print_startup_banner(cfg)

        hx, escs, mav_batt = setup_hardware(cfg)

        # Let the system sit safely before first tare.
        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)

        offset_count = tare_load_cell(
            cfg,
            hx,
            "Prepare for initial tare: motors off, no added load on load cell.",
        )

        arm_escs(cfg, escs)

        saved_files = run_all_acquisition_sets(
            cfg=cfg,
            hx=hx,
            mav_batt=mav_batt,
            escs=escs,
            offset_count=offset_count,
        )

        print_saved_files(saved_files)

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        safe_shutdown(cfg, hx, escs)


main()

