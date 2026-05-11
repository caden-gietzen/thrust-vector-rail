# load_cell_calibration.py
# HX711 load cell calibration script for Raspberry Pi Pico.
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21
#
# Expected output CSV columns:
#   t_ms,t_s,phase,known_mass_g,raw_count,avg_count,tared_count,counts_per_g_estimate
#
# Workflow:
#   1. Run script.
#   2. Script gives countdown to remove all load.
#   3. Script tares/zeros the sensor.
#   4. Script gives countdowns before each known calibration mass.
#   5. Script logs raw and averaged HX711 counts.
#
# Requires hx711_gpio.py to be present on the Pico filesystem:
#   /hx711_gpio.py
# or:
#   /lib/hx711_gpio.py

import time
from machine import Pin
from hx711_gpio import HX711


# ----------------------------
# Pin configuration
# ----------------------------

HX711_DAT_PIN = 20
HX711_SCK_PIN = 21

HX711_GAIN = 128


# ----------------------------
# Calibration settings
# ----------------------------

FILENAME = "load_cell_calibration.csv"

KNOWN_MASSES_G = [
    0,
    50,
    100,
    200,
    500,
]

TARE_SAMPLES = 25
READ_SAMPLES_PER_MASS = 20
SAMPLE_DELAY_MS = 100

SETTLE_DELAY_MS = 1500

# Time given to place/remove weights before each phase.
PLACEMENT_COUNTDOWN_S = 10


# ----------------------------
# Helper functions
# ----------------------------

def countdown(message, seconds):
    print()
    print(message)

    for remaining in range(seconds, 0, -1):
        print(remaining)
        time.sleep(1)

    print("Starting now.")


def get_time_fields(start_ms):
    now_ms = time.ticks_ms()
    t_ms = time.ticks_diff(now_ms, start_ms)
    t_s = t_ms / 1000.0
    return t_ms, t_s


def read_average_with_logging(hx, f, start_ms, phase, known_mass_g, offset):
    """
    Reads multiple HX711 samples, logs each raw sample,
    and returns the average count for this mass.
    """
    raw_values = []

    for sample_idx in range(READ_SAMPLES_PER_MASS):
        raw_count = hx.read()
        raw_values.append(raw_count)

        avg_count_so_far = sum(raw_values) / len(raw_values)
        tared_count = raw_count - offset

        if known_mass_g != 0:
            counts_per_g_estimate = (avg_count_so_far - offset) / known_mass_g
        else:
            counts_per_g_estimate = 0

        t_ms, t_s = get_time_fields(start_ms)

        f.write("{},{:.3f},{},{},{},{:.3f},{:.3f},{:.9f}\n".format(
            t_ms,
            t_s,
            phase,
            known_mass_g,
            raw_count,
            avg_count_so_far,
            tared_count,
            counts_per_g_estimate
        ))

        f.flush()

        print(
            "mass_g:",
            known_mass_g,
            "sample:",
            sample_idx + 1,
            "raw:",
            raw_count,
            "tared:",
            tared_count
        )

        time.sleep_ms(SAMPLE_DELAY_MS)

    return sum(raw_values) / len(raw_values)


# ----------------------------
# Main script
# ----------------------------

print("Initializing HX711...")
print("DAT pin:", HX711_DAT_PIN)
print("SCK pin:", HX711_SCK_PIN)

data_pin = Pin(HX711_DAT_PIN, Pin.IN)
clock_pin = Pin(HX711_SCK_PIN, Pin.OUT)

# HX711 powers down if SCK is held high.
# Force SCK low and give the amplifier time to become ready.
clock_pin.value(0)
print("Waiting for HX711 to become ready...")
time.sleep_ms(750)

hx = HX711(clock_pin, data_pin, gain=HX711_GAIN)

try:
    print("HX711 initialized.")
    print("Output file:", FILENAME)

    countdown(
        "Remove all load from the load cell for tare.",
        PLACEMENT_COUNTDOWN_S
    )

    print("Settling before tare...")
    time.sleep_ms(SETTLE_DELAY_MS)

    print("Taring with", TARE_SAMPLES, "samples...")
    offset = hx.read_average(TARE_SAMPLES)
    hx.set_offset(offset)

    print("Tare complete.")
    print("Offset:", offset)

    start_ms = time.ticks_ms()

    with open(FILENAME, "w") as f:
        f.write("t_ms,t_s,phase,known_mass_g,raw_count,avg_count,tared_count,counts_per_g_estimate\n")

        print()
        print("Logging tare validation point...")
        read_average_with_logging(
            hx=hx,
            f=f,
            start_ms=start_ms,
            phase="tare_validation",
            known_mass_g=0,
            offset=offset
        )

        for mass_g in KNOWN_MASSES_G:
            if mass_g == 0:
                continue

            countdown(
                "Place known mass: {} g on the load cell.".format(mass_g),
                PLACEMENT_COUNTDOWN_S
            )

            print("Settling...")
            time.sleep_ms(SETTLE_DELAY_MS)

            print("Reading mass:", mass_g, "g")
            avg_count = read_average_with_logging(
                hx=hx,
                f=f,
                start_ms=start_ms,
                phase="calibration",
                known_mass_g=mass_g,
                offset=offset
            )

            tared_avg = avg_count - offset
            counts_per_g = tared_avg / mass_g

            print()
            print("Mass:", mass_g, "g")
            print("Average raw count:", avg_count)
            print("Average tared count:", tared_avg)
            print("Counts per gram estimate:", counts_per_g)

    print()
    print("Calibration logging complete.")
    print("Saved:", FILENAME)

finally:
    print("Powering down HX711.")
    hx.power_down()