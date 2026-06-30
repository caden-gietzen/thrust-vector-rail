# load_cell_characterization.py
# HX711 load-cell uncertainty characterization logger (motor OFF, dead weights).
#
# Purpose:
#   Quantify how well the bench instrument knows force, to feed Monte Carlo
#   perturbation ranges on the constant thrust T (and, if wanted, the EKF prior /
#   process noise on the unmeasured T state). This is NOT an operational sensor
#   model -- thrust is not measured at runtime.
#
# Campaign (no motor, dead weights only). To keep the on-Pico CSV small, each
# run logs ONE up/down pass; collect 3 separate datasets and compose them in
# analysis (run-to-run spread across the 3 files gives repeatability):
#   1. Warm-up log from power-up (settling), no load.
#   2. Tare (zero) hold.
#   3. One up-and-down weight ladder, holding each level a fixed dwell. The log
#      rate is capped (SAMPLE_DELAY_MS) so flash doesn't fill.
#
# Weights should bracket the thrust range (~0.2-4.2 N ~ 20-425 gf), e.g.
# 0 / 50 / 200 / 500 g (500 g caps the ~425 g max thrust).
#
# Architecture:
#   HX711 power cycle + settle is handled in HX711.__init__ (lib/hx711_gpio.py
#   -> power_cycle()), so there is no manual clock-low/settle here.
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21
#
# Output CSV columns:
#   t_ms,t_s,phase,repeat,hold_index,direction,known_mass_g,sample_idx,raw_count,tared_count
#
# Requires lib/hx711_gpio.py on the Pico (auto-synced by run_pico_and_pull.py).

import json
from machine import Pin
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711


# ============================================================
# Configuration
# ============================================================

def make_config():
    return {
        "HX711_DAT_PIN": 20,
        "HX711_SCK_PIN": 21,
        "HX711_GAIN": 128,

        "LOG_FILE_BASE": "load_cell_characterization",

        # Ascending weight levels (g). The descending leg is the reverse minus
        # the peak, giving an up-then-down ladder so hysteresis is observable.
        "LADDER_MASSES_G": [0, 50, 200, 500],
        # One up/down pass per run (keeps the on-Pico CSV small). Collect 3
        # separate datasets and compose them in analysis.
        "NUM_LADDER_REPEATS": 1,

        # Timing.
        "WARMUP_S": 20,
        "DWELL_S": 45,
        "SETTLE_DELAY_MS": 1500,
        "PLACEMENT_COUNTDOWN_S": 10,

        # Cap the log rate so the on-Pico CSV stays small regardless of the
        # HX711's native sample rate (~80 SPS fills flash fast). ~90 ms between
        # logged samples gives ~10 SPS -> a few hundred samples per 45 s hold,
        # plenty for the uncertainty budget.
        "SAMPLE_DELAY_MS": 90,

        # Tare
        "TARE_SAMPLES": 50,
        "TARE_SAMPLE_DELAY_MS": 20,
        "TARE_COUNTDOWN_S": 10,
    }


def load_json_file_if_exists(path):
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def apply_external_config_if_present(cfg):
    external = load_json_file_if_exists("/run_config.json")
    if external is None:
        external = load_json_file_if_exists("run_config.json")
    if external is None:
        return cfg
    print("Loaded external run config.")
    for key in external:
        cfg[key] = external[key]
    return cfg


def finalize_config(cfg):
    masses = list(cfg["LADDER_MASSES_G"])
    # Up leg: levels as given. Down leg: reverse minus the peak (already held on
    # the way up). e.g. [0,50,200,500] -> down [200,50,0].
    holds = [(m, "up") for m in masses]
    holds += [(m, "down") for m in reversed(masses[:-1])]
    cfg["_LADDER_HOLDS"] = holds
    return cfg


# ============================================================
# Helpers
# ============================================================

def countdown(message, seconds):
    print()
    print(message)
    for remaining in range(seconds, 0, -1):
        print(remaining)
        sleep(1)
    print("Starting now.")


def setup_hx711(cfg):
    print("Initializing HX711...")
    print("DAT pin:", cfg["HX711_DAT_PIN"])
    print("SCK pin:", cfg["HX711_SCK_PIN"])

    data_pin = Pin(cfg["HX711_DAT_PIN"], Pin.IN)
    clock_pin = Pin(cfg["HX711_SCK_PIN"], Pin.OUT)

    # HX711.__init__ forces a power cycle and settles the amplifier.
    hx = HX711(clock_pin, data_pin, gain=cfg["HX711_GAIN"])

    print("HX711 internal GAIN code:", hx.GAIN, "(expect 1 for gain=128)")
    if hx.GAIN != 1:
        print("WARNING: expected internal GAIN == 1 for gain=128, got", hx.GAIN)

    print("HX711 initialized.")
    return hx


def safe_read(hx):
    """Return a raw count, or None if the sensor did not respond."""
    try:
        return hx.read()
    except OSError as e:
        print("READ ERROR (no response):", e)
        return None


def tare(cfg, hx):
    print("Taring with", cfg["TARE_SAMPLES"], "samples...")
    total = 0
    good = 0
    for _ in range(cfg["TARE_SAMPLES"]):
        raw = safe_read(hx)
        if raw is not None:
            total += raw
            good += 1
        sleep_ms(cfg["TARE_SAMPLE_DELAY_MS"])

    offset = total / good if good else 0
    print("Tare offset:", offset, "(", good, "valid samples )")
    return offset


def log_window(f, hx, start_ms, phase, repeat, hold_index, direction, mass_g,
               offset, window_ms, sample_delay_ms):
    """Stream raw samples to the CSV for window_ms, pacing at sample_delay_ms to
    bound the file size. Returns the sample count."""
    win_start = ticks_ms()
    sample_idx = 0

    while ticks_diff(ticks_ms(), win_start) < window_ms:
        raw = safe_read(hx)
        if raw is None:
            continue

        tared = raw - offset
        t_ms = ticks_diff(ticks_ms(), start_ms)
        sample_idx += 1

        f.write("{},{:.3f},{},{},{},{},{},{},{},{:.1f}\n".format(
            t_ms, t_ms / 1000.0, phase, repeat, hold_index,
            direction, mass_g, sample_idx, raw, tared))

        if sample_idx % 25 == 0:
            f.flush()

        if sample_delay_ms > 0:
            sleep_ms(sample_delay_ms)

    f.flush()
    return sample_idx


# ============================================================
# Main
# ============================================================

def main():
    cfg = finalize_config(apply_external_config_if_present(make_config()))

    print("Load-cell uncertainty characterization")
    print("Ladder masses (g):", cfg["LADDER_MASSES_G"])
    print("Ladder repeats:", cfg["NUM_LADDER_REPEATS"])
    print("Dwell per hold (s):", cfg["DWELL_S"])

    hx = None
    f = None

    try:
        hx = setup_hx711(cfg)

        filename = cfg["LOG_FILE_BASE"] + ".csv"
        f = open(filename, "w")
        f.write("t_ms,t_s,phase,repeat,hold_index,direction,"
                "known_mass_g,sample_idx,raw_count,tared_count\n")

        start_ms = ticks_ms()
        offset = 0
        hold_index = 0

        # --- Warm-up (no load) ---
        hold_index += 1
        print()
        print("Warm-up logging (no load) for", cfg["WARMUP_S"], "s...")
        log_window(f, hx, start_ms, "warmup", 0, hold_index, "none", 0,
                   offset, cfg["WARMUP_S"] * 1000, cfg["SAMPLE_DELAY_MS"])

        # --- Tare / zero ---
        countdown("Remove ALL load from the load cell for tare (zero).",
                  cfg["TARE_COUNTDOWN_S"])
        sleep_ms(cfg["SETTLE_DELAY_MS"])
        offset = tare(cfg, hx)

        # --- Up/down ladder, repeated ---
        for repeat in range(1, cfg["NUM_LADDER_REPEATS"] + 1):
            print()
            print("============================================================")
            print("Ladder pass", repeat, "of", cfg["NUM_LADDER_REPEATS"])
            print("============================================================")

            for (mass_g, direction) in cfg["_LADDER_HOLDS"]:
                hold_index += 1

                if mass_g == 0:
                    msg = "Pass {}: REMOVE all load (0 g, {} leg).".format(repeat, direction)
                else:
                    msg = "Pass {}: set load to {} g ({} leg).".format(repeat, mass_g, direction)
                countdown(msg, cfg["PLACEMENT_COUNTDOWN_S"])

                print("Settling...")
                sleep_ms(cfg["SETTLE_DELAY_MS"])

                print("Logging", mass_g, "g", direction, "for", cfg["DWELL_S"], "s...")
                n = log_window(f, hx, start_ms, "ladder", repeat, hold_index,
                               direction, mass_g, offset, cfg["DWELL_S"] * 1000,
                               cfg["SAMPLE_DELAY_MS"])
                print("  logged", n, "samples")

        f.flush()
        print()
        print("Characterization logging complete.")
        print("Saved:", filename)

    finally:
        if f is not None:
            f.close()
        if hx is not None:
            print("Powering down HX711.")
            hx.power_down()


main()
