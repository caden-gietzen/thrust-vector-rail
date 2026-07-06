# servo_prps_log.py
#
# Purpose:
#   Run PRPS-style servo command tests and log encoder-measured servo angle.
#
# PRPS = Pseudo-Random Periodic Signal
#      = periodic multisine with random phases.
#
# Output CSV columns:
#   t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,
#   servo_us,command_delta_us,command_norm,
#   count,count_zero,count_delta,
#   theta_rad,theta_deg,
#   phase,period_index,period_sample_index
#
# Hardware:
#   Servo PWM: GP15
#   Encoder A: GP18
#   Encoder B: GP19
#
# Structure note:
#   This file is organized into functions so VS Code's Outline panel is useful.
#   The main execution path is:
#
#       main()
#           cfg = finalize_config(make_config())
#           print_startup_banner(cfg)
#           setup_hardware(cfg)
#           run_all_acquisition_sets(cfg, servo)
#           safe_shutdown(cfg, servo)
#
# Orchestration note:
#   If /run_config.json or run_config.json exists on the Pico, values in that
#   JSON file override the defaults from make_config(). This lets the laptop
#   runner execute one bounded segment, pull/delete the CSV, and upload the
#   next segment's config.

import time
from machine import Pin, PWM
import encoder
import urandom
import math


# ------------------------------------------------------------
# Servo truth values (single source of truth).
#
# The zero-angle center command and the static gain live in
# lib/servo_static_map.py (the device-side mirror of
# analysis/utils/servoStaticMap.m). Pull them from there so this logger never
# hardcodes a stale center/gain. If the module isn't flashed to /lib yet -- e.g.
# a standalone run before the orchestrator's "uploads" step has populated it --
# fall back to the last-known literals so the script still runs.
try:
    from servo_static_map import NEUTRAL_US as _TRUTH_CENTER_US
    from servo_static_map import DEG_PER_US as _TRUTH_DEG_PER_US
    _TRUTH_SOURCE = "lib/servo_static_map.py"
except ImportError:
    _TRUTH_CENTER_US = 1431
    _TRUTH_DEG_PER_US = 0.091092
    _TRUTH_SOURCE = "fallback literals (servo_static_map not on /lib)"


# ============================================================
# Configuration
# ============================================================


def make_config():
    """
    Return all user-editable settings in one dictionary.

    The laptop orchestrator can upload /run_config.json to override any of
    these keys for one bounded Pico execution.
    """
    cfg = {
        # ----------------------------------------------------
        # File / acquisition settings
        # ----------------------------------------------------
        "LOG_FILE_BASE": "servo_prps_log",
        "NUM_ACQUISITION_SETS": 1,
        "BASE_PRPS_SEED": 4004,
        "RUN_ORDER_SEED_OFFSET": 50000,
        "NEW_PHASES_EACH_SET": True,
        "FIXED_RUN_ORDER": True,
        "COOLDOWN_BETWEEN_SETS_MS": 0,
        "SAVE_FREQUENCY_PLAN_TXT": False,
        # Log every Nth command tick (>=1). The servo is still COMMANDED every
        # tick; this only decimates which samples are written to CSV, decoupling
        # the on-flash data volume from the excitation playback rate. The shared
        # tick counter spans baseline + settle + prps so logged samples stay
        # uniformly spaced across the whole file. Effective log rate =
        # COMMAND_UPDATE_DT_MS * LOG_EVERY_N_TICKS; keep it >= 2x the top
        # excited frequency.
        "LOG_EVERY_N_TICKS": 1,

        # ----------------------------------------------------
        #  Hardware settings
        # ----------------------------------------------------
        "ENC_A_PIN": 18,
        "ENC_B_PIN": 19,
        "SERVO_PIN": 15,
        "MAX_STEP_RATE": 1000000,
        "DRAIN_HZ": 10000,
        # Upgraded high-speed digital servo: 330 Hz PWM frame rate (~3 ms),
        # matching the 3 ms command tick. Orchestrated segments set this
        # explicitly; this default keeps standalone runs at the same rate.
        "SERVO_FREQ_HZ": 330,

        # ----------------------------------------------------
        # Servo / encoder calibration
        # ----------------------------------------------------
        "COUNTS_PER_REV": 2400.0,
        "SERVO_CENTER_US": _TRUTH_CENTER_US,
        "SERVO_HARD_MIN_US": 400,
        "SERVO_HARD_MAX_US": 2500,

        # ----------------------------------------------------
        # Safety / timing
        # ----------------------------------------------------
        "INITIAL_COUNTDOWN_S": 5,
        "PRE_RUN_SETTLE_MS": 1000,
        "POST_ZERO_SETTLE_MS": 300,
        "BASELINE_HOLD_MS": 2000,
        "POST_RUN_HOLD_MS": 1000,
        "PRINT_EVERY_N_SAMPLES": 200,

        # ----------------------------------------------------
        # PRPS settings
        # ----------------------------------------------------
        "USE_MANUAL_FREQUENCY_LIST": False,
        "MANUAL_EXCITED_FREQS_HZ": [
            0.10,
            0.15,
            0.20,
            0.30,
            0.40,
            0.50,
            0.75,
            1.00,
            1.25,
            1.50,
            2.00,
            2.50,
            3.00,
        ],
        "AUTO_FREQ_MIN_HZ": 0.10,
        "AUTO_FREQ_MAX_HZ": 3.00,
        "AUTO_NUM_FREQS": 13,
        "AUTO_FREQ_SPACING": "log",
        "PRPS_PERIOD_S": 20.0,
        "NUM_PERIODS_PER_RUN": 4,
        # Command tick. Orchestrated 330 Hz campaigns set this to 3 ms (matching
        # SERVO_FREQ_HZ) explicitly; this 5 ms standalone default stays well
        # inside the PWM frame so a manual run is always safe. Logging is on the
        # same tick (LOG_EVERY_N_TICKS=1), so the encoder is sampled at the
        # command rate. Keep the resulting samples_per_period log buffer
        # (11 * period_s / dt_s bytes) under ~24 KB to avoid an RP2040 MemoryError.
        "COMMAND_UPDATE_DT_MS": 5,
        "PRINT_FREQUENCY_PLAN": False,
        "RANDOMIZE_PHASES": True,
        "NORMALIZE_TO_PEAK": True,
        "ROUND_PWM_TO_INT": True,

        # Precomputed (laptop-side) multisine realization. When non-empty these
        # override the on-Pico random phasing / unit amplitudes so the firmware
        # only *plays back* a Schroeder + crest-minimized, taper-shaped design.
        # Each array is one entry per *kept* excited line, aligned to the snapped
        # bin order (see tools/design_servo_prps_excitation.py). Empty arrays
        # preserve the legacy behavior.
        "PRPS_PHASES": [],
        "PRPS_LINE_AMPLITUDES": [],

        # Fully precomputed (laptop-side) one-period command waveform: the
        # NORMALIZED multisine samples (roughly [-1, 1]), packed as little-endian
        # float32 and base64-encoded. When non-empty the firmware plays this back
        # by array index and does NO trig at command time, which is what keeps a
        # fast (e.g. 5 ms) command tick deterministic. Must hold exactly
        # samples_per_period values. Empty preserves the on-Pico live-sum path.
        "PRPS_COMMAND_TABLE_B64": "",

        # Peak realized command-velocity guard (deg/s). The plan's worst-case
        # per-sample command slope is converted to deg/s via the servo static
        # gain and checked against this ceiling before each run; <= 0 disables.
        "PEAK_VELOCITY_LIMIT_DEG_S": 120.0,
        "SERVO_STATIC_DEG_PER_US": _TRUTH_DEG_PER_US,

        # ----------------------------------------------------
        # Test run definitions
        # ----------------------------------------------------
        # Each entry:
        #   (run_name, center_us, amplitude_us)
        #
        # JSON uploads should use arrays:
        #   ["local_center_amp600", 1450, 600]
        "TEST_RUNS": [
            ("local_center_amp600", _TRUTH_CENTER_US, 600),
        ],
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

        {"LOG_FILE_BASE": "...", "BASE_PRPS_SEED": 4001}

    or:

        {"name": "segment001", "config": {"LOG_FILE_BASE": "..."}}
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
                raise ValueError("Each TEST_RUNS entry must be [run_name, center_us, amplitude_us].")
            coerced_runs.append((run[0], run[1], run[2]))
        cfg["TEST_RUNS"] = coerced_runs

    return cfg


def apply_external_config_if_present(cfg):
    """
    Overrides the default config with /run_config.json if present.

    The Pico script does not know about the full orchestration plan. It only
    receives and applies the config for the current segment.
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
    cfg["RAD_PER_COUNT"] = 2.0 * math.pi / cfg["COUNTS_PER_REV"]
    cfg["DEG_PER_COUNT"] = 360.0 / cfg["COUNTS_PER_REV"]
    validate_timing_and_frequency_config(cfg)
    return cfg


def validate_timing_and_frequency_config(cfg):
    """
    Catch timing/frequency mismatches before the rail moves.

    The command table and FRF analysis both assume the PRPS period contains an
    integer number of command ticks. Logging may be decimated, but the logged
    Nyquist rate must still cover every excited line.
    """
    dt_ms = float(cfg["COMMAND_UPDATE_DT_MS"])
    if dt_ms <= 0:
        raise ValueError("COMMAND_UPDATE_DT_MS must be > 0.")

    period_s = float(cfg["PRPS_PERIOD_S"])
    if period_s <= 0:
        raise ValueError("PRPS_PERIOD_S must be > 0.")

    samples_per_period = int(round(period_s / (dt_ms / 1000.0)))
    reconstructed_period_s = samples_per_period * dt_ms / 1000.0
    if abs(reconstructed_period_s - period_s) > 1e-6:
        raise ValueError("PRPS_PERIOD_S must be an integer multiple of COMMAND_UPDATE_DT_MS.")

    log_n = int(cfg.get("LOG_EVERY_N_TICKS", 1))
    if log_n < 1:
        raise ValueError("LOG_EVERY_N_TICKS must be >= 1.")

    requested_freqs = get_requested_frequencies_hz(cfg)
    requested_kept, snapped_freqs, bins = snap_frequencies_to_period_bins(
        requested_freqs,
        period_s,
    )
    if len(bins) == 0:
        raise ValueError("No valid PRPS frequencies selected.")

    command_rate_hz = 1000.0 / dt_ms
    log_rate_hz = command_rate_hz / log_n
    max_freq_hz = max(snapped_freqs)

    if max_freq_hz >= 0.5 * command_rate_hz:
        raise ValueError("Excited frequency exceeds command Nyquist rate.")

    if max_freq_hz >= 0.5 * log_rate_hz:
        raise ValueError("Excited frequency exceeds logged-sample Nyquist rate.")


# ============================================================
# Small utility helpers
# ============================================================


def countdown(message, seconds):
    print()
    print(message)
    for remaining in range(seconds, 0, -1):
        print(remaining)
        time.sleep(1)
    print("Starting now.")


def cooldown_countdown(message, duration_ms):
    if duration_ms <= 0:
        return

    print()
    print(message)

    start_ms = time.ticks_ms()
    last_print_s = -1

    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
        remaining_ms = duration_ms - elapsed_ms
        remaining_s = int((remaining_ms + 999) / 1000)

        if remaining_s != last_print_s:
            if remaining_s % 10 == 0 or remaining_s <= 5:
                print("Cooldown remaining:", remaining_s, "s")
                last_print_s = remaining_s

        time.sleep_ms(200)

    print("Cooldown complete.")


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def seed_urandom(seed_value, label):
    try:
        urandom.seed(seed_value)
        print(label, "seed:", seed_value)
    except Exception:
        print("NOTE: urandom.seed unavailable for", label)


def random_unit_float():
    return urandom.getrandbits(24) / float(1 << 24)


def format_freq_for_label(freq_hz):
    if abs(freq_hz - int(freq_hz)) < 1e-9:
        return str(int(freq_hz))

    text = "{:.6g}".format(freq_hz)
    return text.replace(".", "p")


def make_frequency_range_label(freqs_hz):
    if not freqs_hz:
        return "no_freqs"

    return "{}_to_{}Hz".format(
        format_freq_for_label(min(freqs_hz)),
        format_freq_for_label(max(freqs_hz)),
    )


# ============================================================
# Servo / PWM helpers
# ============================================================


def write_pwm_us(cfg, pwm, pulse_us):
    pulse_us = int(clamp(
        pulse_us,
        cfg["SERVO_HARD_MIN_US"],
        cfg["SERVO_HARD_MAX_US"],
    ))
    pwm.duty_ns(int(pulse_us * 1000))


def setup_servo(cfg):
    print("Initializing servo PWM.")
    print("Servo pin:", cfg["SERVO_PIN"])

    servo = PWM(Pin(cfg["SERVO_PIN"]))
    servo.freq(cfg["SERVO_FREQ_HZ"])

    write_pwm_us(cfg, servo, cfg["SERVO_CENTER_US"])
    time.sleep_ms(1000)

    return servo


# ============================================================
# Encoder / angle helpers
# ============================================================


def setup_encoder(cfg):
    print("Initializing encoder.")
    print("Encoder pins:", cfg["ENC_A_PIN"], cfg["ENC_B_PIN"])
    print("Max step rate:", cfg["MAX_STEP_RATE"])
    print("Drain Hz:", cfg["DRAIN_HZ"])

    encoder.init_configured(
        cfg["ENC_A_PIN"],
        cfg["ENC_B_PIN"],
        cfg["MAX_STEP_RATE"],
        cfg["DRAIN_HZ"],
    )

    time.sleep_ms(100)


def center_and_zero_encoder(cfg, servo, center_us):
    print("Centering servo.")
    write_pwm_us(cfg, servo, center_us)
    time.sleep_ms(cfg["PRE_RUN_SETTLE_MS"])

    print("Zeroing encoder.")
    encoder.zero()
    time.sleep_ms(cfg["POST_ZERO_SETTLE_MS"])

    count_zero = encoder.get_count()
    print("Local zero count:", count_zero)

    return count_zero


def count_to_theta_rad(cfg, count_delta):
    return count_delta * cfg["RAD_PER_COUNT"]


def count_to_theta_deg(cfg, count_delta):
    return count_delta * cfg["DEG_PER_COUNT"]


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
            freqs.append(cfg["AUTO_FREQ_MIN_HZ"] + alpha * (cfg["AUTO_FREQ_MAX_HZ"] - cfg["AUTO_FREQ_MIN_HZ"]))
        return freqs

    if cfg["AUTO_FREQ_SPACING"] == "log":
        log_min = math.log(cfg["AUTO_FREQ_MIN_HZ"])
        log_max = math.log(cfg["AUTO_FREQ_MAX_HZ"])

        for i in range(cfg["AUTO_NUM_FREQS"]):
            alpha = i / float(cfg["AUTO_NUM_FREQS"] - 1)
            freqs.append(math.exp(log_min + alpha * (log_max - log_min)))
        return freqs

    raise ValueError("AUTO_FREQ_SPACING must be 'linear' or 'log'")


def get_requested_frequencies_hz(cfg):
    if cfg["USE_MANUAL_FREQUENCY_LIST"]:
        return list(cfg["MANUAL_EXCITED_FREQS_HZ"])
    return make_auto_frequency_list(cfg)


def snap_frequencies_to_period_bins(requested_freqs_hz, period_s):
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
    print("  Number of periods:", cfg["NUM_PERIODS_PER_RUN"])
    print("  Total duration_s:", cfg["PRPS_PERIOD_S"] * cfg["NUM_PERIODS_PER_RUN"])
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
    # Precomputed phases (Schroeder + crest-min, laptop-side) take precedence.
    provided = cfg.get("PRPS_PHASES")
    if provided:
        if len(provided) != num_freqs:
            raise ValueError(
                "PRPS_PHASES length {} != number of excited lines {}".format(
                    len(provided), num_freqs
                )
            )
        return [float(p) for p in provided]

    phases = []

    for _ in range(num_freqs):
        if cfg["RANDOMIZE_PHASES"]:
            phases.append(2.0 * math.pi * random_unit_float())
        else:
            phases.append(0.0)

    return phases


def generate_line_amplitudes(cfg, num_freqs):
    # Precomputed per-line amplitude envelope (A_flat below f_c, 1/f above).
    # Empty -> unit amplitude on every line (legacy behavior).
    provided = cfg.get("PRPS_LINE_AMPLITUDES")
    if provided:
        if len(provided) != num_freqs:
            raise ValueError(
                "PRPS_LINE_AMPLITUDES length {} != number of excited lines {}".format(
                    len(provided), num_freqs
                )
            )
        return [float(a) for a in provided]
    return [1.0 for _ in range(num_freqs)]


def prps_raw_value(sample_index, samples_per_period, bins, phases, amplitudes):
    value = 0.0

    for i in range(len(bins)):
        k = bins[i]
        phi = phases[i]
        a = amplitudes[i]
        angle = 2.0 * math.pi * k * sample_index / samples_per_period + phi
        value += a * math.sin(angle)

    return value


def estimate_peak_and_rms(samples_per_period, bins, phases, amplitudes):
    peak = 0.0
    sum_sq = 0.0
    peak_diff = 0.0

    # Seed the running difference with the wrap-around transition (the signal is
    # periodic, so the last->first step is a real command slope too).
    prev = prps_raw_value(samples_per_period - 1, samples_per_period, bins, phases, amplitudes)

    for n in range(samples_per_period):
        x = prps_raw_value(n, samples_per_period, bins, phases, amplitudes)

        ax = abs(x)
        if ax > peak:
            peak = ax

        sum_sq += x * x

        d = abs(x - prev)
        if d > peak_diff:
            peak_diff = d
        prev = x

    if peak <= 0:
        peak = 1.0

    rms = math.sqrt(sum_sq / samples_per_period)

    if rms <= 0:
        crest = 999.0
    else:
        crest = peak / rms

    return peak, rms, crest, peak_diff


def print_selected_prps_signal(bins, peak, rms, crest):
    print()
    print("Selected PRPS signal:")
    print("  Number of excited frequencies:", len(bins))
    print("  Peak normalization:", peak)
    print("  RMS before normalization:", rms)
    print("  Crest factor:", crest)


def decode_command_table(cfg, samples_per_period):
    """
    Decode a laptop-precomputed, base64'd float32 command table into an
    array('f'). The table is one period of the NORMALIZED multisine command
    (values ~[-1, 1]); run_prps_periods plays it back by index so the Pico does
    no trig at command time. Returns None when no table was provided (legacy
    on-Pico live-sum path).
    """
    b64 = cfg.get("PRPS_COMMAND_TABLE_B64", "")
    if not b64:
        return None

    import gc
    import ubinascii
    from array import array

    gc.collect()
    raw = ubinascii.a2b_base64(b64)

    # Free the ~40 KB base64 string immediately; we only need the bytes now.
    cfg["PRPS_COMMAND_TABLE_B64"] = ""
    b64 = None
    gc.collect()

    n = len(raw) // 4
    if n != samples_per_period:
        raise ValueError(
            "PRPS_COMMAND_TABLE_B64 holds {} samples but samples_per_period is {}".format(
                n, samples_per_period
            )
        )

    # array('f', <bytes>) reinterprets the buffer as float32 in ONE contiguous
    # allocation. Building it by append/generator instead triggers
    # geometric-growth reallocations that fragment the heap and MemoryError.
    table = array('f', raw)

    raw = None
    gc.collect()

    return table


def estimate_table_stats(table):
    """
    Peak, RMS, crest factor, and worst-case sample-to-sample |diff| (wrap-around
    included) of a precomputed command table. Table-playback analog of
    estimate_peak_and_rms(); feeds the printout and the velocity guard.
    """
    n = len(table)
    peak = 0.0
    sum_sq = 0.0
    peak_diff = 0.0
    prev = table[n - 1]

    for i in range(n):
        x = table[i]
        ax = x if x >= 0 else -x
        if ax > peak:
            peak = ax
        sum_sq += x * x
        d = x - prev
        if d < 0:
            d = -d
        if d > peak_diff:
            peak_diff = d
        prev = x

    if peak <= 0:
        peak = 1.0

    rms = math.sqrt(sum_sq / n)
    crest = 999.0 if rms <= 0 else peak / rms
    return peak, rms, crest, peak_diff


def generate_prps_plan(cfg, seed_value):
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

    print_frequency_plan(cfg, requested_kept, snapped_freqs, bins, samples_per_period)

    # Fast path: a fully precomputed table means no on-Pico trig at command time.
    command_table = decode_command_table(cfg, samples_per_period)
    if command_table is not None:
        peak, rms, crest, peak_raw_diff = estimate_table_stats(command_table)
        print("Using precomputed command table:", samples_per_period, "samples")
        print_selected_prps_signal(bins, peak, rms, crest)
        return {
            "requested_freqs": requested_kept,
            "snapped_freqs": snapped_freqs,
            "bins": bins,
            "phases": [],
            "amplitudes": [],
            "peak": peak,
            "samples_per_period": samples_per_period,
            "crest": crest,
            "peak_raw_diff": peak_raw_diff,
            "command_table": command_table,
        }

    seed_urandom(seed_value, "PRPS phase")
    phases = generate_random_phases(cfg, len(bins))
    amplitudes = generate_line_amplitudes(cfg, len(bins))

    peak, rms, crest, peak_raw_diff = estimate_peak_and_rms(
        samples_per_period,
        bins,
        phases,
        amplitudes,
    )

    print_selected_prps_signal(bins, peak, rms, crest)

    return {
        "requested_freqs": requested_kept,
        "snapped_freqs": snapped_freqs,
        "bins": bins,
        "phases": phases,
        "amplitudes": amplitudes,
        "peak": peak,
        "samples_per_period": samples_per_period,
        "crest": crest,
        "peak_raw_diff": peak_raw_diff,
        "command_table": None,
    }


def get_normalized_prps_sample(cfg, sample_index, samples_per_period, bins, phases, amplitudes, peak):
    x = prps_raw_value(sample_index, samples_per_period, bins, phases, amplitudes)

    if cfg["NORMALIZE_TO_PEAK"] and peak > 0:
        x = x / peak

    return x


def prps_sample_to_command(
    cfg,
    center_us,
    amplitude_us,
    sample_index,
    samples_per_period,
    bins,
    phases,
    amplitudes,
    peak,
):
    command_norm = get_normalized_prps_sample(
        cfg,
        sample_index,
        samples_per_period,
        bins,
        phases,
        amplitudes,
        peak,
    )

    servo_us = center_us + amplitude_us * command_norm
    servo_us = clamp(servo_us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"])

    if cfg["ROUND_PWM_TO_INT"]:
        servo_us = int(round(servo_us))

    command_delta_us = servo_us - center_us

    return servo_us, command_delta_us, command_norm


# ============================================================
# Metadata file writing
# ============================================================


def write_prps_metadata_file(cfg, log_file, requested_freqs, snapped_freqs, bins, crest_factor):
    metadata_file = log_file.replace(".csv", "_frequency_plan.txt")

    with open(metadata_file, "w") as f:
        f.write("Servo PRPS frequency plan\n")
        f.write("log_file={}\n".format(log_file))
        f.write("PRPS_PERIOD_S={}\n".format(cfg["PRPS_PERIOD_S"]))
        f.write("fundamental_Hz={}\n".format(1.0 / cfg["PRPS_PERIOD_S"]))
        f.write("COMMAND_UPDATE_DT_MS={}\n".format(cfg["COMMAND_UPDATE_DT_MS"]))
        f.write("command_update_rate_Hz={}\n".format(1000.0 / cfg["COMMAND_UPDATE_DT_MS"]))
        f.write("NUM_PERIODS_PER_RUN={}\n".format(cfg["NUM_PERIODS_PER_RUN"]))
        f.write("COUNTS_PER_REV={}\n".format(cfg["COUNTS_PER_REV"]))
        f.write("RAD_PER_COUNT={}\n".format(cfg["RAD_PER_COUNT"]))
        f.write("DEG_PER_COUNT={}\n".format(cfg["DEG_PER_COUNT"]))
        f.write("SERVO_CENTER_US={}\n".format(cfg["SERVO_CENTER_US"]))
        f.write("crest_factor={}\n".format(crest_factor))
        f.write("\n")
        f.write("run_idx,run_name,center_us,amplitude_us\n")

        for i in range(len(cfg["TEST_RUNS"])):
            run_name, center_us, amplitude_us = cfg["TEST_RUNS"][i]
            f.write("{},{},{},{}\n".format(
                i,
                run_name,
                center_us,
                amplitude_us,
            ))

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
        "t_ms,t_s,set_index,prps_seed,run_order_seed,run_name,segment,"
        "servo_us,command_delta_us,command_norm,"
        "count,count_zero,count_delta,"
        "theta_rad,theta_deg,"
        "phase,period_index,period_sample_index\n"
    )


def print_sample_status(cfg, sample_counter, set_index, prps_seed, run_name,
                        phase, servo_us, command_delta_us, count_delta,
                        theta_deg, period_index, period_sample_index):
    if sample_counter % cfg["PRINT_EVERY_N_SAMPLES"] != 0:
        return

    print(
        "sample", sample_counter,
        "set", set_index,
        "seed", prps_seed,
        "run", run_name,
        "phase", phase,
        "servo_us", int(servo_us),
        "cmd_delta_us", int(command_delta_us),
        "count_delta", int(count_delta),
        "theta_deg", "{:.3f}".format(theta_deg),
        "period", period_index,
        "period_sample", period_sample_index,
    )


def write_sample(
    cfg,
    state,
    f,
    t0_ms,
    set_index,
    prps_seed,
    run_order_seed,
    run_name,
    segment,
    servo_us,
    command_delta_us,
    command_norm,
    count_zero,
    phase,
    period_index,
    period_sample_index,
):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, t0_ms)
    t_s = t_ms / 1000.0

    count = encoder.get_count()
    count_delta = count - count_zero

    theta_rad = count_to_theta_rad(cfg, count_delta)
    theta_deg = count_to_theta_deg(cfg, count_delta)

    f.write(
        "{},{:.3f},{},{},{},{},{},{},{},{:.6f},{},{},{},{:.8f},{:.5f},{},{},{}\n".format(
            int(t_ms),
            t_s,
            int(set_index),
            int(prps_seed),
            int(run_order_seed),
            run_name,
            segment,
            int(servo_us),
            int(command_delta_us),
            command_norm,
            int(count),
            int(count_zero),
            int(count_delta),
            theta_rad,
            theta_deg,
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
        run_name,
        phase,
        servo_us,
        command_delta_us,
        count_delta,
        theta_deg,
        period_index,
        period_sample_index,
    )


# ============================================================
# Log-rate decimation
# ============================================================


def should_log_this_tick(cfg, state):
    """
    Advance the shared command-tick counter and report whether this tick should
    be logged. Decouples CSV row rate from the servo command rate so a long,
    fast-tick PRPS run fits in Pico flash. The counter lives in `state` so it is
    continuous across baseline/settle/prps within one acquisition file, keeping
    logged samples uniformly spaced.
    """
    n = cfg.get("LOG_EVERY_N_TICKS", 1)
    if n < 1:
        n = 1

    do_log = (state["tick_index"] % n == 0)
    state["tick_index"] += 1
    return do_log


# ============================================================
# Hold / baseline routines
# ============================================================


def hold_and_log(
    cfg,
    state,
    f,
    servo,
    t0_ms,
    set_index,
    prps_seed,
    run_order_seed,
    run_name,
    segment,
    center_us,
    servo_us,
    duration_ms,
    count_zero,
    phase,
):
    servo_us = int(clamp(servo_us, cfg["SERVO_HARD_MIN_US"], cfg["SERVO_HARD_MAX_US"]))
    command_delta_us = servo_us - center_us
    command_norm = 0.0

    write_pwm_us(cfg, servo, servo_us)

    start_ms = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start_ms) < duration_ms:
        loop_start_ms = time.ticks_ms()

        if should_log_this_tick(cfg, state):
            write_sample(
                cfg=cfg,
                state=state,
                f=f,
                t0_ms=t0_ms,
                set_index=set_index,
                prps_seed=prps_seed,
                run_order_seed=run_order_seed,
                run_name=run_name,
                segment=segment,
                servo_us=servo_us,
                command_delta_us=command_delta_us,
                command_norm=command_norm,
                count_zero=count_zero,
                phase=phase,
                period_index=-1,
                period_sample_index=-1,
            )

        elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
        remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms

        if remaining_ms > 0:
            time.sleep_ms(int(remaining_ms))


def log_pre_run_baseline(cfg, state, f, servo, t0_ms, set_index,
                         prps_seed, run_order_seed, run_name,
                         center_us, count_zero):
    print("Pre-run baseline.")
    hold_and_log(
        cfg=cfg,
        state=state,
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        set_index=set_index,
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="baseline_pre",
        center_us=center_us,
        servo_us=center_us,
        duration_ms=cfg["BASELINE_HOLD_MS"],
        count_zero=count_zero,
        phase="baseline_pre",
    )


def log_center_settle(cfg, state, f, servo, t0_ms, set_index,
                      prps_seed, run_order_seed, run_name,
                      center_us, count_zero):
    print("Settling at run center.")
    hold_and_log(
        cfg=cfg,
        state=state,
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        set_index=set_index,
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="settle",
        center_us=center_us,
        servo_us=center_us,
        duration_ms=cfg["PRE_RUN_SETTLE_MS"],
        count_zero=count_zero,
        phase="settle",
    )


def log_post_run_baseline(cfg, state, f, servo, t0_ms, set_index,
                          prps_seed, run_order_seed, run_name,
                          center_us, count_zero):
    print("Post-run baseline.")
    hold_and_log(
        cfg=cfg,
        state=state,
        f=f,
        servo=servo,
        t0_ms=t0_ms,
        set_index=set_index,
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        run_name=run_name,
        segment="baseline_post",
        center_us=center_us,
        servo_us=center_us,
        duration_ms=cfg["BASELINE_HOLD_MS"],
        count_zero=count_zero,
        phase="baseline_post",
    )


# ============================================================
# PRPS run execution
# ============================================================


def estimate_peak_command_velocity_deg_s(cfg, amplitude_us, peak, peak_raw_diff):
    """
    Worst-case realized command velocity (deg/s) for this run's amplitude.

    The plan stores peak_raw_diff = max |x[n+1]-x[n]| of the *raw* multisine.
    Normalizing (same as the playback path), scaling by amplitude_us, dividing
    by the sample period, and applying the servo static gain gives the peak
    commanded angular rate.
    """
    dt_s = cfg["COMMAND_UPDATE_DT_MS"] / 1000.0

    if cfg["NORMALIZE_TO_PEAK"] and peak > 0:
        peak_norm_diff = peak_raw_diff / peak
    else:
        peak_norm_diff = peak_raw_diff

    peak_us_per_sample = amplitude_us * peak_norm_diff
    return peak_us_per_sample / dt_s * abs(cfg["SERVO_STATIC_DEG_PER_US"])


def check_peak_velocity(cfg, amplitude_us, peak, peak_raw_diff):
    limit = cfg["PEAK_VELOCITY_LIMIT_DEG_S"]

    vel = estimate_peak_command_velocity_deg_s(cfg, amplitude_us, peak, peak_raw_diff)
    print("Est. peak command velocity deg/s:", vel)

    if limit <= 0:
        return

    print("Peak velocity limit deg/s:", limit)
    if vel > limit:
        raise ValueError(
            "Estimated peak command velocity {:.1f} deg/s exceeds limit {:.1f} deg/s".format(
                vel, limit
            )
        )


def print_run_banner(cfg, set_index, prps_seed, run_order_seed, run_idx,
                     run_name, center_us, amplitude_us, samples_per_period,
                     crest_factor, peak, peak_raw_diff):
    low = center_us - amplitude_us
    high = center_us + amplitude_us

    print()
    print("============================================================")
    print("RUN:", run_name)
    print("Run idx:", run_idx)
    print("Set index:", set_index)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("============================================================")
    print("Center us:", center_us)
    print("Amplitude us:", amplitude_us)
    print("Command bounds:", low, "to", high)
    print("PRPS period s:", cfg["PRPS_PERIOD_S"])
    print("Periods per run:", cfg["NUM_PERIODS_PER_RUN"])
    print("Total duration s:", cfg["PRPS_PERIOD_S"] * cfg["NUM_PERIODS_PER_RUN"])
    print("Command dt ms:", cfg["COMMAND_UPDATE_DT_MS"])
    print("Samples per period:", samples_per_period)
    print("Crest factor:", crest_factor)

    if low < cfg["SERVO_HARD_MIN_US"] or high > cfg["SERVO_HARD_MAX_US"]:
        raise ValueError("Requested PRPS command exceeds hard servo safety bounds.")

    check_peak_velocity(cfg, amplitude_us, peak, peak_raw_diff)


def write_prps_buffer(cfg, state, f, set_index, prps_seed, run_order_seed, run_name,
                      center_us, amplitude_us, count_zero, prps_plan, buf, n_logged):
    """
    Write the buffered PRPS samples to CSV after the timed run has finished.
    Only t_ms and count are measured (stored during the loop as '<iiBH' records);
    every other column is reconstructed here (untimed), so the hot loop never
    touches string formatting or flash.
    """
    import struct

    command_table = prps_plan.get("command_table")
    samples_per_period = prps_plan["samples_per_period"]
    bins = prps_plan["bins"]
    phases = prps_plan["phases"]
    amplitudes = prps_plan["amplitudes"]
    peak = prps_plan["peak"]

    rad_per_count = cfg["RAD_PER_COUNT"]
    deg_per_count = cfg["DEG_PER_COUNT"]
    lo = cfg["SERVO_HARD_MIN_US"]
    hi = cfg["SERVO_HARD_MAX_US"]
    round_pwm = cfg["ROUND_PWM_TO_INT"]

    for b in range(n_logged):
        t_ms, count, period_index, psi = struct.unpack_from('<iiBH', buf, b * 11)

        if command_table is not None:
            command_norm = command_table[psi]
        else:
            command_norm = get_normalized_prps_sample(
                cfg, psi, samples_per_period, bins, phases, amplitudes, peak)

        servo_us = clamp(center_us + amplitude_us * command_norm, lo, hi)
        if round_pwm:
            servo_us = int(round(servo_us))
        command_delta_us = servo_us - center_us

        count_delta = count - count_zero
        theta_rad = count_delta * rad_per_count
        theta_deg = count_delta * deg_per_count

        f.write(
            "{},{:.3f},{},{},{},{},{},{},{},{:.6f},{},{},{},{:.8f},{:.5f},{},{},{}\n".format(
                int(t_ms),
                t_ms / 1000.0,
                int(set_index),
                int(prps_seed),
                int(run_order_seed),
                run_name,
                "prps",
                int(servo_us),
                int(command_delta_us),
                command_norm,
                int(count),
                int(count_zero),
                int(count_delta),
                theta_rad,
                theta_deg,
                "prps",
                int(period_index),
                int(psi),
            )
        )
        state["sample_counter"] += 1


def run_prps_periods(cfg, state, f, servo, t0_ms, set_index,
                     prps_seed, run_order_seed, run_name, center_us,
                     amplitude_us, count_zero, prps_plan):
    """
    Timed command/acquisition loop. The servo is commanded every tick; logged
    samples are buffered to preallocated RAM arrays (no string formatting, no
    flash I/O), which is what keeps the tick deterministic. Rows are flushed to
    CSV by write_prps_buffer() after all periods complete.
    """
    print("Running PRPS.")

    import gc
    import struct

    command_table = prps_plan["command_table"]
    samples_per_period = prps_plan["samples_per_period"]
    bins = prps_plan["bins"]
    phases = prps_plan["phases"]
    amplitudes = prps_plan["amplitudes"]
    peak = prps_plan["peak"]

    lo = cfg["SERVO_HARD_MIN_US"]
    hi = cfg["SERVO_HARD_MAX_US"]
    round_pwm = cfg["ROUND_PWM_TO_INT"]
    dt_ms = cfg["COMMAND_UPDATE_DT_MS"]
    num_periods = cfg["NUM_PERIODS_PER_RUN"]

    log_n = cfg.get("LOG_EVERY_N_TICKS", 1)
    if log_n < 1:
        log_n = 1

    # ONE PERIOD's worth of fixed-size records, written in place with
    # struct.pack_into (no per-tick allocation, no heap fragmentation). Each
    # record is '<iiBH' = t_ms(i32), count(i32), period(u8), period_sample(u16)
    # = 11 bytes. We buffer a single period (~22 KB) rather than the whole run
    # (~88 KB): the larger block can't be allocated contiguously once the heap
    # is fragmented by the command table and config. Each period's rows are
    # flushed to CSV in the gap between periods, while the servo holds center.
    cap = samples_per_period // log_n + 4
    gc.collect()
    buf = bytearray(11 * cap)

    for period_index in range(num_periods):
        n_logged = 0

        for period_sample_index in range(samples_per_period):
            loop_start_ms = time.ticks_ms()

            if command_table is not None:
                # Table playback: a single array lookup, no trig.
                command_norm = command_table[period_sample_index]
                servo_us = clamp(center_us + amplitude_us * command_norm, lo, hi)
                if round_pwm:
                    servo_us = int(round(servo_us))
            else:
                servo_us, command_delta_us, command_norm = prps_sample_to_command(
                    cfg=cfg,
                    center_us=center_us,
                    amplitude_us=amplitude_us,
                    sample_index=period_sample_index,
                    samples_per_period=samples_per_period,
                    bins=bins,
                    phases=phases,
                    amplitudes=amplitudes,
                    peak=peak,
                )

            write_pwm_us(cfg, servo, servo_us)

            # Buffer only — no formatting, no file write inside the timed loop.
            if should_log_this_tick(cfg, state) and n_logged < cap:
                struct.pack_into(
                    '<iiBH', buf, n_logged * 11,
                    time.ticks_diff(time.ticks_ms(), t0_ms),
                    encoder.get_count(),
                    period_index,
                    period_sample_index,
                )
                n_logged += 1

            elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_start_ms)
            remaining_ms = dt_ms - elapsed_ms

            if remaining_ms > 0:
                time.sleep_ms(int(remaining_ms))

        # Period complete: hold center and flush this period's rows (untimed).
        write_pwm_us(cfg, servo, center_us)
        write_prps_buffer(
            cfg, state, f, set_index, prps_seed, run_order_seed, run_name,
            center_us, amplitude_us, count_zero, prps_plan, buf, n_logged,
        )
        print("Period", period_index, "written:", n_logged, "samples")


def run_prps_sequence(
    cfg,
    state,
    f,
    servo,
    t0_ms,
    set_index,
    prps_seed,
    run_order_seed,
    run_idx,
    run_name,
    center_us,
    amplitude_us,
    count_zero,
    prps_plan,
):
    peak = prps_plan["peak"]
    samples_per_period = prps_plan["samples_per_period"]
    crest = prps_plan["crest"]
    peak_raw_diff = prps_plan["peak_raw_diff"]

    print_run_banner(
        cfg,
        set_index,
        prps_seed,
        run_order_seed,
        run_idx,
        run_name,
        center_us,
        amplitude_us,
        samples_per_period,
        crest,
        peak,
        peak_raw_diff,
    )

    log_pre_run_baseline(
        cfg,
        state,
        f,
        servo,
        t0_ms,
        set_index,
        prps_seed,
        run_order_seed,
        run_name,
        center_us,
        count_zero,
    )

    log_center_settle(
        cfg,
        state,
        f,
        servo,
        t0_ms,
        set_index,
        prps_seed,
        run_order_seed,
        run_name,
        center_us,
        count_zero,
    )

    run_prps_periods(
        cfg,
        state,
        f,
        servo,
        t0_ms,
        set_index,
        prps_seed,
        run_order_seed,
        run_name,
        center_us,
        amplitude_us,
        count_zero,
        prps_plan,
    )

    log_post_run_baseline(
        cfg,
        state,
        f,
        servo,
        t0_ms,
        set_index,
        prps_seed,
        run_order_seed,
        run_name,
        center_us,
        count_zero,
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


def make_log_filename(cfg, set_index, prps_seed):
    if len(cfg["TEST_RUNS"]) == 1:
        run_name, center_us, amplitude_us = cfg["TEST_RUNS"][0]

        return "{}_amp{}_seed{}.csv".format(
            cfg["LOG_FILE_BASE"],
            int(amplitude_us),
            int(prps_seed),
        )

    return "{}_seed{}_set{:02d}.csv".format(
        cfg["LOG_FILE_BASE"],
        int(prps_seed),
        int(set_index),
    )


def print_acquisition_set_banner(cfg, set_index, log_file, prps_seed, run_order_seed):
    print()
    print("============================================================")
    print("ACQUISITION SET", set_index, "of", cfg["NUM_ACQUISITION_SETS"])
    print("Log file:", log_file)
    print("PRPS seed:", prps_seed)
    print("Run-order seed:", run_order_seed)
    print("Fixed run order:", cfg["FIXED_RUN_ORDER"])
    print("New phases each set:", cfg["NEW_PHASES_EACH_SET"])
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

    print_acquisition_set_banner(cfg, set_index, log_file, prps_seed, run_order_seed)
    print_planned_runs(cfg, runs)

    waveform_seed = get_waveform_seed(cfg, prps_seed)
    prps_plan = generate_prps_plan(cfg, waveform_seed)

    if cfg["SAVE_FREQUENCY_PLAN_TXT"]:
        write_prps_metadata_file(
            cfg,
            log_file,
            prps_plan["requested_freqs"],
            prps_plan["snapped_freqs"],
            prps_plan["bins"],
            prps_plan["crest"],
        )

    return prps_seed, run_order_seed, log_file, runs, prps_plan


# ============================================================
# Acquisition set execution
# ============================================================


def run_single_acquisition_file(cfg, state, log_file, runs, prps_plan, set_index,
                                prps_seed, run_order_seed, servo, count_zero):
    with open(log_file, "w") as f:
        write_csv_header(f)

        t0_ms = time.ticks_ms()

        for run_idx in range(len(runs)):
            run_name, center_us, amplitude_us = runs[run_idx]

            run_prps_sequence(
                cfg=cfg,
                state=state,
                f=f,
                servo=servo,
                t0_ms=t0_ms,
                set_index=set_index,
                prps_seed=prps_seed,
                run_order_seed=run_order_seed,
                run_idx=run_idx,
                run_name=run_name,
                center_us=center_us,
                amplitude_us=amplitude_us,
                count_zero=count_zero,
                prps_plan=prps_plan,
            )

            print("Returning to center.")
            write_pwm_us(cfg, servo, center_us)
            time.sleep_ms(cfg["POST_RUN_HOLD_MS"])


def run_acquisition_set(cfg, set_index, servo):
    state = {"sample_counter": 0, "tick_index": 0}

    prps_seed, run_order_seed, log_file, runs, prps_plan = prepare_acquisition_set(
        cfg,
        set_index,
    )

    # Use the first run center as the acquisition-set zero reference.
    first_run_name, first_center_us, first_amp_us = runs[0]
    count_zero = center_and_zero_encoder(cfg, servo, first_center_us)

    run_single_acquisition_file(
        cfg=cfg,
        state=state,
        log_file=log_file,
        runs=runs,
        prps_plan=prps_plan,
        set_index=set_index,
        prps_seed=prps_seed,
        run_order_seed=run_order_seed,
        servo=servo,
        count_zero=count_zero,
    )

    print()
    print("Saved:", log_file)

    return log_file


def cooldown_after_set_if_needed(cfg, servo, set_index):
    if set_index >= cfg["NUM_ACQUISITION_SETS"]:
        return

    if cfg["COOLDOWN_BETWEEN_SETS_MS"] <= 0:
        return

    print("Holding center during cooldown.")
    write_pwm_us(cfg, servo, cfg["SERVO_CENTER_US"])

    cooldown_countdown(
        "Cooldown between acquisition sets.",
        cfg["COOLDOWN_BETWEEN_SETS_MS"],
    )


def run_all_acquisition_sets(cfg, servo):
    saved_files = []

    for set_index in range(1, cfg["NUM_ACQUISITION_SETS"] + 1):
        print()
        print("Preparing acquisition set", set_index, "of", cfg["NUM_ACQUISITION_SETS"])

        saved_file = run_acquisition_set(
            cfg=cfg,
            set_index=set_index,
            servo=servo,
        )

        saved_files.append(saved_file)
        cooldown_after_set_if_needed(cfg, servo, set_index)

    return saved_files


# ============================================================
# Hardware orchestration
# ============================================================


def setup_hardware(cfg):
    setup_encoder(cfg)
    servo = setup_servo(cfg)
    return servo


def safe_shutdown(cfg, servo):
    print()
    print("Centering servo and shutting down.")

    if servo is not None:
        write_pwm_us(cfg, servo, cfg["SERVO_CENTER_US"])
        time.sleep_ms(500)
        servo.deinit()

    print("Done.")


# ============================================================
# Startup / shutdown printing
# ============================================================


def print_startup_banner(cfg):
    print()
    print("Servo PRPS logger")
    print("Servo pin:", cfg["SERVO_PIN"])
    print("Encoder pins:", cfg["ENC_A_PIN"], cfg["ENC_B_PIN"])
    print("Counts per rev:", cfg["COUNTS_PER_REV"])
    print("Rad per count:", cfg["RAD_PER_COUNT"])
    print("Deg per count:", cfg["DEG_PER_COUNT"])

    print()
    print("Acquisition settings:")
    print("  Output base:", cfg["LOG_FILE_BASE"])
    print("  NUM_ACQUISITION_SETS:", cfg["NUM_ACQUISITION_SETS"])
    print("  BASE_PRPS_SEED:", cfg["BASE_PRPS_SEED"])
    print("  RUN_ORDER_SEED_OFFSET:", cfg["RUN_ORDER_SEED_OFFSET"])
    print("  NEW_PHASES_EACH_SET:", cfg["NEW_PHASES_EACH_SET"])
    print("  FIXED_RUN_ORDER:", cfg["FIXED_RUN_ORDER"])
    print("  COOLDOWN_BETWEEN_SETS_MS:", cfg["COOLDOWN_BETWEEN_SETS_MS"])
    print("  SAVE_FREQUENCY_PLAN_TXT:", cfg["SAVE_FREQUENCY_PLAN_TXT"])
    print("  LOG_EVERY_N_TICKS:", cfg["LOG_EVERY_N_TICKS"])

    print()
    print("Servo safety:")
    print("  Truth source:", _TRUTH_SOURCE)
    print("  SERVO_CENTER_US:", cfg["SERVO_CENTER_US"])
    print("  SERVO_STATIC_DEG_PER_US:", cfg["SERVO_STATIC_DEG_PER_US"])
    print("  SERVO_HARD_MIN_US:", cfg["SERVO_HARD_MIN_US"])
    print("  SERVO_HARD_MAX_US:", cfg["SERVO_HARD_MAX_US"])

    print()
    print("PRPS settings:")
    print("  USE_MANUAL_FREQUENCY_LIST:", cfg["USE_MANUAL_FREQUENCY_LIST"])
    print("  AUTO_FREQ_MIN_HZ:", cfg["AUTO_FREQ_MIN_HZ"])
    print("  AUTO_FREQ_MAX_HZ:", cfg["AUTO_FREQ_MAX_HZ"])
    print("  AUTO_NUM_FREQS:", cfg["AUTO_NUM_FREQS"])
    print("  AUTO_FREQ_SPACING:", cfg["AUTO_FREQ_SPACING"])
    print("  PRPS_PERIOD_S:", cfg["PRPS_PERIOD_S"])
    print("  NUM_PERIODS_PER_RUN:", cfg["NUM_PERIODS_PER_RUN"])
    print("  COMMAND_UPDATE_DT_MS:", cfg["COMMAND_UPDATE_DT_MS"])
    print("  Command update rate Hz:", 1000.0 / cfg["COMMAND_UPDATE_DT_MS"])
    print("  Logged sample rate Hz:", 1000.0 / (cfg["COMMAND_UPDATE_DT_MS"] * cfg["LOG_EVERY_N_TICKS"]))
    print("  NORMALIZE_TO_PEAK:", cfg["NORMALIZE_TO_PEAK"])
    print("  RANDOMIZE_PHASES:", cfg["RANDOMIZE_PHASES"])
    print("  ROUND_PWM_TO_INT:", cfg["ROUND_PWM_TO_INT"])

    print()
    print("Test runs:")
    for i in range(len(cfg["TEST_RUNS"])):
        run_name, center_us, amplitude_us = cfg["TEST_RUNS"][i]
        print(
            "  run_idx", i,
            "name", run_name,
            "center_us", center_us,
            "amplitude_us", amplitude_us,
            "range", center_us - amplitude_us, "to", center_us + amplitude_us,
        )


def print_saved_files(saved_files):
    print()
    print("All servo PRPS acquisition sets complete.")
    print("Saved files:")
    for filename in saved_files:
        print(" -", filename)


# ============================================================
# Main execution path
# ============================================================


def main():
    cfg = finalize_config(make_config())

    servo = None

    try:
        print_startup_banner(cfg)

        servo = setup_hardware(cfg)

        countdown(
            "Keep mechanism clear. PRPS test begins after countdown.",
            cfg["INITIAL_COUNTDOWN_S"],
        )

        saved_files = run_all_acquisition_sets(cfg, servo)

        print_saved_files(saved_files)

    finally:
        safe_shutdown(cfg, servo)


main()

