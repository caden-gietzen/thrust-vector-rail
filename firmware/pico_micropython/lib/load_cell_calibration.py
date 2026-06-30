# load_cell_calibration.py
# Small reusable HX711 weight-calibration helper for MicroPython firmware.
#
# The routine is intentionally non-interactive: it uses timed countdowns instead
# of input(), so scripts can run under tools/run_pico_and_pull.py.

from utime import sleep, sleep_ms


def countdown(message, seconds):
    print()
    print(message)

    for remaining in range(int(seconds), 0, -1):
        print(remaining)
        sleep(1)

    print("Starting now.")


def read_average(hx, n=1, delay_ms=0):
    total = 0
    good = 0

    for _ in range(int(n)):
        try:
            total += hx.read()
            good += 1
        except OSError as e:
            print("HX711 read error:", e)

        if delay_ms > 0:
            sleep_ms(int(delay_ms))

    if good == 0:
        raise OSError("No valid HX711 samples")

    return total / good


def fit_scale_g_per_count(masses_g, tared_counts):
    numerator = 0.0
    denominator = 0.0

    for i in range(len(masses_g)):
        mass_g = float(masses_g[i])
        count = float(tared_counts[i])
        numerator += mass_g * count
        denominator += mass_g * mass_g

    if denominator <= 0:
        raise ValueError("Calibration masses must include at least one positive mass.")

    counts_per_g = numerator / denominator

    if counts_per_g == 0:
        raise ValueError("Calibration fit produced zero counts_per_g.")

    return abs(1.0 / counts_per_g), counts_per_g


def run_weight_calibration(hx, cfg):
    masses_g = cfg["CALIBRATION_MASSES_G"]

    if not masses_g:
        raise ValueError("CALIBRATION_MASSES_G must contain at least one mass.")

    countdown(
        "Remove all load from the load cell for calibration tare.",
        cfg["CALIBRATION_PLACEMENT_COUNTDOWN_S"],
    )

    print("Settling before calibration tare...")
    sleep_ms(cfg["CALIBRATION_SETTLE_DELAY_MS"])

    print("Calibration tare samples:", cfg["CALIBRATION_TARE_SAMPLES"])
    tare_offset = read_average(
        hx,
        cfg["CALIBRATION_TARE_SAMPLES"],
        delay_ms=cfg["CALIBRATION_SAMPLE_DELAY_MS"],
    )
    print("Calibration tare offset:", tare_offset)

    tared_counts = []

    for mass_g in masses_g:
        countdown(
            "Place known calibration mass: {} g.".format(mass_g),
            cfg["CALIBRATION_PLACEMENT_COUNTDOWN_S"],
        )

        print("Settling with", mass_g, "g...")
        sleep_ms(cfg["CALIBRATION_SETTLE_DELAY_MS"])

        avg_count = read_average(
            hx,
            cfg["CALIBRATION_READ_SAMPLES_PER_MASS"],
            delay_ms=cfg["CALIBRATION_SAMPLE_DELAY_MS"],
        )
        tared_count = avg_count - tare_offset

        if tared_count == 0:
            raise ValueError("Calibration mass produced zero tared count.")

        tared_counts.append(tared_count)

        print("Calibration mass:", mass_g, "g")
        print("  average count:", avg_count)
        print("  tared count:", tared_count)
        print("  point grams_per_count:", abs(float(mass_g) / float(tared_count)))

    scale_g_per_count, counts_per_g = fit_scale_g_per_count(masses_g, tared_counts)

    print()
    print("Load-cell calibration fit:")
    print("  counts_per_g:", counts_per_g)
    print("  SCALE_G_PER_COUNT:", scale_g_per_count)

    countdown(
        "Remove calibration weights for final sweep tare.",
        cfg["CALIBRATION_PLACEMENT_COUNTDOWN_S"],
    )

    print("Settling before final sweep tare...")
    sleep_ms(cfg["CALIBRATION_SETTLE_DELAY_MS"])

    final_offset = read_average(
        hx,
        cfg["TARE_SAMPLES"],
        delay_ms=cfg["TARE_SAMPLE_DELAY_MS"],
    )
    print("Final sweep tare offset:", final_offset)

    return scale_g_per_count, final_offset
