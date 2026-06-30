# thrust_sweep_log.py
# Static thrust characterization via stepped PWM sweep.
#
# Hardware:
#   - Raspberry Pi Pico / RP2040 running MicroPython
#   - HX711 + load cell
#   - One ESC signal pin on GP14
#   - Pixhawk TELEM2 MAVLink output wired to Pico UART RX for battery voltage/current
#
# Output CSV columns:
#   t_ms,t_s,sweep_index,run_name,segment,pwm_us,throttle_pct,
#   raw_count,tared_count,force_N,
#   battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,
#   mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,
#   phase,step_index,direction
#
# Sweep behavior:
#   Each step holds the PWM command for DWELL_MS milliseconds, sampling
#   continuously, then advances to the next setpoint.
#
#   Hardware limits:
#       - PWM_SAFE_US     = 1000 (minimum/idle command)
#       - PWM_HARD_MIN_US = 1100 (saturation below this: motor stalls / no thrust)
#       - PWM_HARD_MAX_US = 1950 (saturation above this: ESC max, no further thrust gain)
#
#   Default sweep range is configured by throttle percentage. Set
#   DEFAULT_MAX_THROTTLE_PCT below; the PWM sweep stop, run label, and output
#   filename are derived from that one value unless explicitly overridden.
#
# Structure note:
#   This file is intentionally organized into functions so VS Code's Outline
#   panel is useful for navigation. The main execution path is:
#
#       main()
#           print_startup_banner()
#           setup_hardware()  # ESC PWM is initialized before HX711/MAVLink
#           tare_load_cell()
#           run_all_sweeps()
#           safe_shutdown()

from machine import Pin, PWM, UART
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711
try:
    from lib.mavlink_battery import MavlinkBatteryReader
except ImportError:
    from mavlink_battery import MavlinkBatteryReader
try:
    from lib.load_cell_calibration import run_weight_calibration
except ImportError:
    from load_cell_calibration import run_weight_calibration


# ============================================================
# Configuration
# ============================================================


DEFAULT_MAX_THROTTLE_PCT = 50
DEFAULT_ESC_PIN = 14


def format_throttle_pct_label(throttle_pct):
    throttle_pct = float(throttle_pct)
    if throttle_pct == int(throttle_pct):
        return "{}pct".format(int(throttle_pct))
    return "{:.1f}pct".format(throttle_pct).replace(".", "p")


def make_config():
    """
    Return all user-editable settings in one dictionary.

    If /run_config.json exists on the Pico, values in that JSON file override
    this default configuration.
    """
    cfg = {
        # ----------------------------------------------------
        # File / acquisition settings
        # ----------------------------------------------------
        # Leave as None to derive from MAX_THROTTLE_PCT and ESC_PINS after
        # any orchestration overrides are loaded.
        "LOG_FILE_BASE": None,
        "NUM_SWEEPS": 1,
        "COOLDOWN_BETWEEN_SWEEPS_MS": 30000,
        "RETARE_EACH_SWEEP": True,

        # ----------------------------------------------------
        # HX711 / load-cell settings
        # ----------------------------------------------------
        "HX711_DAT_PIN": 20,
        "HX711_SCK_PIN": 21,
        "HX711_GAIN": 128,
        "SCALE_G_PER_COUNT": 0.002409592,
        "FORCE_SIGN": 1,
        "TARE_SAMPLES": 100,
        "SAMPLES_RUN": 2,
        "TARE_SAMPLE_DELAY_MS": 20,

        # Optional timed in-run calibration using known weights. Disabled: force
        # uses SCALE_G_PER_COUNT above, which must hold the scale derived from
        # the load_cell_characterization run.
        "CALIBRATE_LOAD_CELL_WITH_WEIGHTS": False,
        "CALIBRATION_MASSES_G": [50, 200],
        "CALIBRATION_TARE_SAMPLES": 50,
        "CALIBRATION_READ_SAMPLES_PER_MASS": 25,
        "CALIBRATION_SAMPLE_DELAY_MS": 100,
        "CALIBRATION_SETTLE_DELAY_MS": 1500,
        "CALIBRATION_PLACEMENT_COUNTDOWN_S": 10,

        # ----------------------------------------------------
        # ESC / PWM settings
        # ----------------------------------------------------
        "ESC_PINS": [DEFAULT_ESC_PIN],
        "ESC_FREQ_HZ": 50,
        "PWM_SAFE_US": 1000,
        "PWM_MAX_US": 2000,
        "PWM_HARD_MIN_US": 1100,
        "PWM_HARD_MAX_US": 1950,

        # ----------------------------------------------------
        # Throttle-percent sweep settings
        # ----------------------------------------------------
        # Percent is mapped over PWM_SAFE_US..PWM_MAX_US, so 50% is 1500 us
        # with the default 1000-2000 us ESC command span.
        "USE_THROTTLE_PERCENT_SWEEP": True,
        "MIN_THROTTLE_PCT": 0,
        "MAX_THROTTLE_PCT": DEFAULT_MAX_THROTTLE_PCT,
        "THROTTLE_STEP_PCT": 5,
        "MIN_THROTTLE_SETTLE_MS": 1000,
        "BATTERY_CONNECT_WAIT_MS": 12000,

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
        "BASELINE_HOLD_MS": 3000,
        "POST_SWEEP_IDLE_MS": 2000,

        # ----------------------------------------------------
        # Sweep parameters
        # ----------------------------------------------------
        # Used only when USE_THROTTLE_PERCENT_SWEEP is False.
        "SWEEP_START_US": 1000,
        "SWEEP_STOP_US": 1300,
        "SWEEP_STEP_US": 50,
        # Time to hold each PWM step while sampling.
        "DWELL_MS": 2000,
        # If True, also sweep back down to START after reaching STOP.
        "BIDIRECTIONAL": True,
        # Settle time after applying a new PWM command before sampling begins.
        "SETTLE_MS": 200,
        # Sample interval inside each dwell window.
        "COMMAND_UPDATE_DT_MS": 20,

        # ----------------------------------------------------
        # Sweep run definitions
        # Each entry is (run_name,) — currently one global sweep.
        # Multiple entries allow different configurations in one file.
        # ----------------------------------------------------
        "SWEEP_RUNS": None,

        "PRINT_EVERY_N_SAMPLES": 50,
    }

    cfg = apply_external_config_if_present(cfg)
    return cfg


# ============================================================
# External config loading (same pattern as thrust_prps_daq_voltage.py)
# ============================================================


def load_json_file_if_exists(path):
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
    if external is None:
        return None

    if isinstance(external, dict) and "config" in external:
        maybe_config = external["config"]
        if isinstance(maybe_config, dict):
            return maybe_config

    if isinstance(external, dict):
        return external

    raise ValueError("External run config must be a JSON object/dictionary.")


def apply_external_config_if_present(cfg):
    config_path = "/run_config.json"
    external = load_json_file_if_exists(config_path)

    if external is None:
        config_path = "run_config.json"
        external = load_json_file_if_exists(config_path)

    external = normalize_external_config(external)

    if external is None:
        return cfg

    print("Loaded external run config:", config_path)

    for key in external:
        cfg[key] = external[key]

    return cfg


def finalize_config(cfg):
    if cfg.get("USE_THROTTLE_PERCENT_SWEEP", False):
        cfg["SWEEP_START_US"] = throttle_pct_to_pwm_us(cfg, cfg["MIN_THROTTLE_PCT"])
        cfg["SWEEP_STOP_US"] = throttle_pct_to_pwm_us(cfg, cfg["MAX_THROTTLE_PCT"])

        step_us = throttle_pct_to_pwm_span_us(cfg, cfg["THROTTLE_STEP_PCT"])
        if step_us <= 0:
            raise ValueError("THROTTLE_STEP_PCT must produce a positive PWM step.")
        cfg["SWEEP_STEP_US"] = step_us

    apply_derived_run_names(cfg)

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


def throttle_pct_to_pwm_span_us(cfg, throttle_pct):
    span_us = cfg["PWM_MAX_US"] - cfg["PWM_SAFE_US"]
    return int(span_us * float(throttle_pct) / 100.0)


def throttle_pct_to_pwm_us(cfg, throttle_pct):
    throttle_pct = clamp(float(throttle_pct), 0.0, 100.0)
    return cfg["PWM_SAFE_US"] + throttle_pct_to_pwm_span_us(cfg, throttle_pct)


def pwm_us_to_throttle_pct(cfg, pwm_us):
    span_us = cfg["PWM_MAX_US"] - cfg["PWM_SAFE_US"]
    if span_us <= 0:
        return 0.0
    return 100.0 * (float(pwm_us) - cfg["PWM_SAFE_US"]) / float(span_us)


def csv_value(value):
    if value is None:
        return ""
    return value


def get_primary_esc_pin(cfg):
    esc_pins = cfg.get("ESC_PINS", [DEFAULT_ESC_PIN])
    if len(esc_pins) == 0:
        return DEFAULT_ESC_PIN
    return int(esc_pins[0])


def apply_derived_run_names(cfg):
    max_throttle_label = format_throttle_pct_label(cfg["MAX_THROTTLE_PCT"])
    min_throttle_label = format_throttle_pct_label(cfg["MIN_THROTTLE_PCT"])
    esc_pin = get_primary_esc_pin(cfg)

    if cfg.get("LOG_FILE_BASE") is None:
        cfg["LOG_FILE_BASE"] = "thrust_sweep_log_{}_gp{}".format(
            max_throttle_label,
            esc_pin,
        )

    if cfg.get("SWEEP_RUNS") is None:
        cfg["SWEEP_RUNS"] = [
            "gp{}_{}_{}".format(
                esc_pin,
                min_throttle_label,
                max_throttle_label,
            )
        ]


# ============================================================
# PWM / ESC helpers
# ============================================================


def pwm_us_to_duty_u16(pwm_us):
    return int(int(pwm_us) * 65535 / 20000)


def set_pwm_us(cfg, escs, pwm_us):
    pwm_us = clamp(int(pwm_us), cfg["PWM_SAFE_US"], cfg["PWM_MAX_US"])
    duty = pwm_us_to_duty_u16(pwm_us)
    for esc in escs:
        esc.duty_u16(duty)
    return pwm_us


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


def wait_for_battery_after_min_throttle(cfg, escs):
    min_pwm_us = set_pwm_us(cfg, escs, cfg["SWEEP_START_US"])

    print()
    print("Minimum throttle command is active before battery connection.")
    print("  PWM:", min_pwm_us, "us")
    print("  throttle_pct: {:.1f}".format(pwm_us_to_throttle_pct(cfg, min_pwm_us)))
    sleep_ms(cfg["MIN_THROTTLE_SETTLE_MS"])

    cooldown_countdown(
        "Plug in battery now. Holding minimum throttle before acquisition.",
        cfg["BATTERY_CONNECT_WAIT_MS"],
    )


# ============================================================
# HX711 / force helpers
# ============================================================


def setup_hx711(cfg):
    print("Initializing HX711...")
    print("DAT pin:", cfg["HX711_DAT_PIN"])
    print("SCK pin:", cfg["HX711_SCK_PIN"])

    data_pin = Pin(cfg["HX711_DAT_PIN"], Pin.IN)
    clock_pin = Pin(cfg["HX711_SCK_PIN"], Pin.OUT)

    # HX711.__init__ now forces a power cycle and settles the amplifier, so no
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
    print("Re-taring load cell for this sweep...")
    offset_count = read_average(
        hx,
        cfg["TARE_SAMPLES"],
        delay_ms=cfg["TARE_SAMPLE_DELAY_MS"],
    )
    print("Set offset count:", offset_count)
    return offset_count


def maybe_calibrate_load_cell(cfg, hx):
    if not cfg["CALIBRATE_LOAD_CELL_WITH_WEIGHTS"]:
        return None

    print()
    print("Running timed load-cell weight calibration.")
    print("Calibration masses (g):", cfg["CALIBRATION_MASSES_G"])

    scale_g_per_count, offset_count = run_weight_calibration(hx, cfg)
    cfg["SCALE_G_PER_COUNT"] = scale_g_per_count
    cfg["SCALE_N_PER_COUNT"] = cfg["SCALE_G_PER_COUNT"] * 9.80665 / 1000.0

    print("Updated SCALE_G_PER_COUNT:", cfg["SCALE_G_PER_COUNT"])
    print("Updated SCALE_N_PER_COUNT:", cfg["SCALE_N_PER_COUNT"])
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
# CSV sample logging
# ============================================================


def make_log_filename(cfg, sweep_index):
    return "{}_sweep{:02d}.csv".format(
        cfg["LOG_FILE_BASE"],
        int(sweep_index),
    )


def write_csv_header(f):
    f.write(
        "t_ms,t_s,sweep_index,run_name,segment,pwm_us,throttle_pct,"
        "raw_count,tared_count,force_N,"
        "battery_voltage_V,battery_current_A,battery_remaining_pct,battery_age_ms,"
        "mavlink_total_packets,mavlink_sys_status_packets,mavlink_bad_frames,"
        "phase,step_index,direction\n"
    )


def print_sample_status(cfg, sample_counter, sweep_index, phase, pwm_us, thrust_N,
                        battery_voltage_V, battery_current_A, battery_age_ms,
                        step_index, direction):
    if sample_counter % cfg["PRINT_EVERY_N_SAMPLES"] != 0:
        return

    print(
        "sample", sample_counter,
        "sweep", sweep_index,
        "phase", phase,
        "pwm_us", int(pwm_us),
        "throttle_pct", "{:.1f}".format(pwm_us_to_throttle_pct(cfg, pwm_us)),
        "force_N", thrust_N,
        "V", battery_voltage_V,
        "A", battery_current_A,
        "batt_age_ms", battery_age_ms,
        "step", step_index,
        "dir", direction,
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
    sweep_index,
    run_name,
    segment,
    pwm_us,
    offset_count,
    phase,
    step_index,
    direction,
):
    if mav_batt is not None:
        mav_batt.update()

    raw_count = read_average(hx, cfg["SAMPLES_RUN"])
    tared_count, thrust_N = force_N_from_raw(cfg, raw_count, offset_count)

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
        "{},{:.3f},{},{},{},{},{:.3f},{:.3f},{:.3f},{:.6f},{},{},{},{},{},{},{},{},{},{}\n".format(
            t_ms,
            t_s,
            int(sweep_index),
            run_name,
            segment,
            int(pwm_us),
            pwm_us_to_throttle_pct(cfg, pwm_us),
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
            int(step_index),
            direction,
        )
    )

    state["sample_counter"] += 1

    print_sample_status(
        cfg,
        state["sample_counter"],
        sweep_index,
        phase,
        pwm_us,
        thrust_N,
        battery_voltage_V,
        battery_current_A,
        battery_age_ms,
        step_index,
        direction,
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
    sweep_index,
    run_name,
    segment,
    pwm_us,
    duration_ms,
    offset_count,
    phase,
    step_index,
    direction,
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
            sweep_index=sweep_index,
            run_name=run_name,
            segment=segment,
            pwm_us=pwm_us,
            offset_count=offset_count,
            phase=phase,
            step_index=step_index,
            direction=direction,
        )

        elapsed_ms = ticks_diff(ticks_ms(), loop_start_ms)
        remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms

        if remaining_ms > 0:
            sleep_ms(remaining_ms)


def log_baseline(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count, label):
    print("Baseline ({})...".format(label))
    hold_and_log(
        cfg=cfg,
        state=state,
        f=f,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        t0_ms=t0_ms,
        sweep_index=sweep_index,
        run_name=run_name,
        segment=label,
        pwm_us=cfg["PWM_SAFE_US"],
        duration_ms=cfg["BASELINE_HOLD_MS"],
        offset_count=offset_count,
        phase=label,
        step_index=-1,
        direction="none",
    )


# ============================================================
# Sweep execution
# ============================================================


def build_sweep_setpoints(cfg):
    """
    Build the list of PWM setpoints for the sweep.

    Returns a list of (pwm_us, step_index, direction) tuples.
    The range is SWEEP_START_US to SWEEP_STOP_US inclusive, stepped by
    SWEEP_STEP_US. If BIDIRECTIONAL is True, the down-sweep from STOP back
    to START is appended.
    """
    start = cfg["SWEEP_START_US"]
    stop = cfg["SWEEP_STOP_US"]
    step = cfg["SWEEP_STEP_US"]

    up_setpoints = []
    pwm = start
    step_idx = 0

    while pwm <= stop:
        up_setpoints.append((pwm, step_idx, "up"))
        pwm += step
        step_idx += 1

    if cfg["BIDIRECTIONAL"]:
        down_setpoints = []
        for i, (pwm_val, _, _) in enumerate(reversed(up_setpoints)):
            down_setpoints.append((pwm_val, i, "down"))

        return up_setpoints + down_setpoints

    return up_setpoints


def run_sweep_steps(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count, setpoints):
    print("Sweep running...")

    for pwm_us, step_index, direction in setpoints:
        # Apply the new command and wait for settle.
        set_pwm_us(cfg, escs, pwm_us)
        sleep_ms(cfg["SETTLE_MS"])

        dwell_start_ms = ticks_ms()

        while ticks_diff(ticks_ms(), dwell_start_ms) < cfg["DWELL_MS"]:
            loop_start_ms = ticks_ms()

            write_sample(
                cfg=cfg,
                state=state,
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                t0_ms=t0_ms,
                sweep_index=sweep_index,
                run_name=run_name,
                segment=step_index,
                pwm_us=pwm_us,
                offset_count=offset_count,
                phase="sweep",
                step_index=step_index,
                direction=direction,
            )

            elapsed_ms = ticks_diff(ticks_ms(), loop_start_ms)
            remaining_ms = cfg["COMMAND_UPDATE_DT_MS"] - elapsed_ms

            if remaining_ms > 0:
                sleep_ms(remaining_ms)


def run_sweep_sequence(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count):
    setpoints = build_sweep_setpoints(cfg)

    total_steps = len(setpoints)
    total_duration_s = total_steps * (cfg["SETTLE_MS"] + cfg["DWELL_MS"]) / 1000.0

    print()
    print("RUN:", run_name)
    print("Sweep index:", sweep_index)
    print("Start:", cfg["SWEEP_START_US"], "us")
    print("Stop:", cfg["SWEEP_STOP_US"], "us")
    print("Step:", cfg["SWEEP_STEP_US"], "us")
    print("Dwell per step:", cfg["DWELL_MS"], "ms")
    print("Settle per step:", cfg["SETTLE_MS"], "ms")
    print("Bidirectional:", cfg["BIDIRECTIONAL"])
    print("Total steps:", total_steps)
    print("Approx total duration:", total_duration_s, "s")
    print("Saturation limits (expected): {} to {} us".format(
        cfg["PWM_HARD_MIN_US"], cfg["PWM_HARD_MAX_US"]))

    log_baseline(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count, "baseline_pre")

    run_sweep_steps(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count, setpoints)

    log_baseline(cfg, state, f, hx, mav_batt, escs, t0_ms, sweep_index, run_name, offset_count, "baseline_post")

    f.flush()
    print("Finished:", run_name)


# ============================================================
# Sweep set execution
# ============================================================


def run_single_sweep_file(cfg, state, log_file, sweep_index, hx, mav_batt, escs, offset_count):
    with open(log_file, "w") as f:
        write_csv_header(f)

        t0_ms = ticks_ms()

        for run_name in cfg["SWEEP_RUNS"]:
            run_sweep_sequence(
                cfg=cfg,
                state=state,
                f=f,
                hx=hx,
                mav_batt=mav_batt,
                escs=escs,
                t0_ms=t0_ms,
                sweep_index=sweep_index,
                run_name=run_name,
                offset_count=offset_count,
            )

            print("Returning to safe PWM...")
            set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
            sleep_ms(cfg["POST_SWEEP_IDLE_MS"])

        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)


def run_one_sweep(cfg, sweep_index, hx, mav_batt, escs, offset_count):
    state = {"sample_counter": 0}

    log_file = make_log_filename(cfg, sweep_index)

    print()
    print("============================================================")
    print("SWEEP", sweep_index, "of", cfg["NUM_SWEEPS"])
    print("Log file:", log_file)
    print("============================================================")

    print_battery_snapshot(mav_batt, "Before sweep")

    run_single_sweep_file(
        cfg=cfg,
        state=state,
        log_file=log_file,
        sweep_index=sweep_index,
        hx=hx,
        mav_batt=mav_batt,
        escs=escs,
        offset_count=offset_count,
    )

    print_battery_snapshot(mav_batt, "After sweep")
    print()
    print("Saved:", log_file)

    return log_file


def run_all_sweeps(cfg, hx, mav_batt, escs, offset_count):
    saved_files = []

    for sweep_index in range(1, cfg["NUM_SWEEPS"] + 1):
        set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])
        sleep_ms(1000)

        if cfg["RETARE_EACH_SWEEP"]:
            offset_count = retare_load_cell_quick(cfg, hx)

        saved_file = run_one_sweep(
            cfg=cfg,
            sweep_index=sweep_index,
            hx=hx,
            mav_batt=mav_batt,
            escs=escs,
            offset_count=offset_count,
        )

        saved_files.append(saved_file)

        if sweep_index < cfg["NUM_SWEEPS"]:
            print("Returning to safe PWM for cooldown...")
            set_pwm_us(cfg, escs, cfg["PWM_SAFE_US"])

            cooldown_countdown(
                "Cooldown between sweeps. Keep area safe.",
                cfg["COOLDOWN_BETWEEN_SWEEPS_MS"],
            )

    return saved_files


# ============================================================
# Hardware orchestration
# ============================================================


def setup_hardware(cfg):
    escs = setup_escs(cfg)
    wait_for_battery_after_min_throttle(cfg, escs)
    hx = setup_hx711(cfg)
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
    print("Static thrust sweep data acquisition")
    print("Output base:", cfg["LOG_FILE_BASE"])
    print("Number of sweeps:", cfg["NUM_SWEEPS"])
    print("Cooldown between sweeps:", cfg["COOLDOWN_BETWEEN_SWEEPS_MS"] / 1000.0, "s")
    print("SCALE_G_PER_COUNT:", cfg["SCALE_G_PER_COUNT"])
    print("SCALE_N_PER_COUNT:", cfg["SCALE_N_PER_COUNT"])
    print("FORCE_SIGN:", cfg["FORCE_SIGN"])
    print("CALIBRATE_LOAD_CELL_WITH_WEIGHTS:", cfg["CALIBRATE_LOAD_CELL_WITH_WEIGHTS"])
    print("CALIBRATION_MASSES_G:", cfg["CALIBRATION_MASSES_G"])
    print("ESC pins:", cfg["ESC_PINS"])
    print("PWM_SAFE_US:", cfg["PWM_SAFE_US"])
    print("PWM_MAX_US:", cfg["PWM_MAX_US"])

    print()
    print("Sweep settings:")
    print("  USE_THROTTLE_PERCENT_SWEEP:", cfg["USE_THROTTLE_PERCENT_SWEEP"])
    print("  MIN_THROTTLE_PCT:", cfg["MIN_THROTTLE_PCT"])
    print("  MAX_THROTTLE_PCT:", cfg["MAX_THROTTLE_PCT"])
    print("  THROTTLE_STEP_PCT:", cfg["THROTTLE_STEP_PCT"])
    print("  BATTERY_CONNECT_WAIT_MS:", cfg["BATTERY_CONNECT_WAIT_MS"])
    print("  SWEEP_START_US:", cfg["SWEEP_START_US"])
    print("  SWEEP_STOP_US:", cfg["SWEEP_STOP_US"])
    print("  SWEEP_STEP_US:", cfg["SWEEP_STEP_US"])
    print("  DWELL_MS:", cfg["DWELL_MS"])
    print("  SETTLE_MS:", cfg["SETTLE_MS"])
    print("  BIDIRECTIONAL:", cfg["BIDIRECTIONAL"])
    print("  PWM_HARD_MIN_US (saturation lower):", cfg["PWM_HARD_MIN_US"])
    print("  PWM_HARD_MAX_US (saturation upper):", cfg["PWM_HARD_MAX_US"])


def print_saved_files(saved_files):
    print()
    print("All sweeps complete.")
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

        set_pwm_us(cfg, escs, cfg["SWEEP_START_US"])
        sleep_ms(1000)

        offset_count = maybe_calibrate_load_cell(cfg, hx)

        if offset_count is None:
            offset_count = tare_load_cell(
                cfg,
                hx,
                "Prepare for initial tare: motors off, no added load on load cell.",
            )

        saved_files = run_all_sweeps(
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
