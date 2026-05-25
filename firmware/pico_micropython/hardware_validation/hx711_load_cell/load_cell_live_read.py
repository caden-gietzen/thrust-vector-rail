# load_cell_live_read.py
# Live HX711 load cell readout using the known calibration scale factor.
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21
#
# Workflow:
#   1. Run script.
#   2. Script gives countdown to remove all load.
#   3. Script tares the sensor.
#   4. Script prints live mass_g and force_N at each sample.
#   Press Ctrl+C to stop.
#
# Requires hx711_gpio.py to be present on the Pico filesystem.

from machine import Pin
from utime import sleep, sleep_ms, ticks_ms, ticks_diff
from hx711_gpio import HX711


# ============================================================
# Configuration
# ============================================================


def make_config():
    return {
        # HX711 hardware
        "HX711_DAT_PIN": 20,
        "HX711_SCK_PIN": 21,
        "HX711_GAIN": 128,

        # Calibration — from thrust_sweep_log / load_cell_calibration results.
        # Positive force_sign means increasing counts = increasing load.
        "SCALE_G_PER_COUNT": 0.002409592,
        "FORCE_SIGN": 1,

        # Tare
        "TARE_SAMPLES": 50,
        "TARE_SAMPLE_DELAY_MS": 20,
        "TARE_COUNTDOWN_S": 5,

        # Continuous readout
        "READ_INTERVAL_MS": 100,
        "PRINT_EVERY_N_SAMPLES": 1,
    }


def finalize_config(cfg):
    cfg["SCALE_N_PER_COUNT"] = cfg["SCALE_G_PER_COUNT"] * 9.80665 / 1000.0
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

    clock_pin.value(0)
    sleep_ms(750)

    hx = HX711(clock_pin, data_pin, gain=cfg["HX711_GAIN"])
    print("HX711 initialized.")
    return hx


def tare(cfg, hx):
    countdown(
        "Remove all load from load cell for tare.",
        cfg["TARE_COUNTDOWN_S"],
    )

    print("Taring with", cfg["TARE_SAMPLES"], "samples...")
    total = 0
    for _ in range(cfg["TARE_SAMPLES"]):
        total += hx.read()
        sleep_ms(cfg["TARE_SAMPLE_DELAY_MS"])

    offset_count = total / cfg["TARE_SAMPLES"]
    print("Tare complete. Offset count:", offset_count)
    return offset_count


def read_and_convert(cfg, hx, offset_count):
    raw_count = hx.read()
    tared_count = raw_count - offset_count
    mass_g = cfg["FORCE_SIGN"] * tared_count * cfg["SCALE_G_PER_COUNT"]
    force_N = cfg["FORCE_SIGN"] * tared_count * cfg["SCALE_N_PER_COUNT"]
    return raw_count, tared_count, mass_g, force_N


# ============================================================
# Main
# ============================================================


def main():
    cfg = finalize_config(make_config())

    print("Load cell live readout")
    print("SCALE_G_PER_COUNT:", cfg["SCALE_G_PER_COUNT"])
    print("SCALE_N_PER_COUNT:", cfg["SCALE_N_PER_COUNT"])
    print("FORCE_SIGN:", cfg["FORCE_SIGN"])

    hx = None

    try:
        hx = setup_hx711(cfg)
        offset_count = tare(cfg, hx)

        print()
        print("Reading. Press Ctrl+C to stop.")
        print("{:<8} {:<14} {:<14} {:<14} {:<14}".format(
            "sample", "raw_count", "tared_count", "mass_g", "force_N"))

        sample = 0

        while True:
            loop_start_ms = ticks_ms()

            raw_count, tared_count, mass_g, force_N = read_and_convert(cfg, hx, offset_count)
            sample += 1

            if sample % cfg["PRINT_EVERY_N_SAMPLES"] == 0:
                print("{:<8} {:<14.0f} {:<14.0f} {:<14.3f} {:<14.6f}".format(
                    sample, raw_count, tared_count, mass_g, force_N))

            elapsed_ms = ticks_diff(ticks_ms(), loop_start_ms)
            remaining_ms = cfg["READ_INTERVAL_MS"] - elapsed_ms
            if remaining_ms > 0:
                sleep_ms(remaining_ms)

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        if hx is not None:
            print("Powering down HX711.")
            hx.power_down()


main()
