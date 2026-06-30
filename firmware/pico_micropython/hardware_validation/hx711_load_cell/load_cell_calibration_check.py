# load_cell_calibration_check.py
# HX711 calibration + live-readout sanity check for Raspberry Pi Pico.
#
# This runs the SAME quick calibration that load-cell tests use before a run
# (lib/load_cell_calibration.run_weight_calibration: tare -> 50 g -> 200 g ->
# fit scale -> final re-tare), then drops into a continuous live grams readout
# so you can place arbitrary weights and confirm the readings make sense before
# trusting the sensor for logging/analysis.
#
# The HX711 power-cycle reset is handled inside the driver (HX711.__init__ ->
# power_cycle), so it always happens on construction here and everywhere else.
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21
#
# Requires on the Pico filesystem (both belong in /lib):
#   hx711_gpio.py
#   load_cell_calibration.py

import time
from machine import Pin
from hx711_gpio import HX711
try:
    from lib.load_cell_calibration import run_weight_calibration
except ImportError:
    from load_cell_calibration import run_weight_calibration


# ----------------------------
# Pin configuration
# ----------------------------

HX711_DAT_PIN = 20
HX711_SCK_PIN = 21
HX711_GAIN = 128


# ----------------------------
# Live readout settings
# ----------------------------

# LIVE_READOUT_SECONDS = 0 runs until you stop the script (Ctrl+C); a positive
# value bounds the readout duration.
LIVE_READOUT_SECONDS = 0
LIVE_READOUT_SAMPLE_AVG = 5
LIVE_READOUT_DELAY_MS = 150


# ----------------------------
# Calibration config
# ----------------------------
# Keys consumed by run_weight_calibration(). Kept consistent with the thrust
# test config so the calibration here matches what those tests perform.

def make_calibration_config():
    return {
        "CALIBRATION_MASSES_G": [50, 200],
        "CALIBRATION_TARE_SAMPLES": 25,
        "CALIBRATION_READ_SAMPLES_PER_MASS": 20,
        "CALIBRATION_SAMPLE_DELAY_MS": 100,
        "CALIBRATION_SETTLE_DELAY_MS": 1500,
        "CALIBRATION_PLACEMENT_COUNTDOWN_S": 10,
        # Used by the final sweep re-tare inside run_weight_calibration.
        "TARE_SAMPLES": 25,
        "TARE_SAMPLE_DELAY_MS": 100,
    }


# ----------------------------
# Helpers
# ----------------------------

def read_avg_simple(hx, n_samples):
    total = 0
    for _ in range(n_samples):
        total += hx.read()
    return total / n_samples


# ----------------------------
# Main script
# ----------------------------

print("Initializing HX711...")
print("DAT pin:", HX711_DAT_PIN)
print("SCK pin:", HX711_SCK_PIN)

data_pin = Pin(HX711_DAT_PIN, Pin.IN)
clock_pin = Pin(HX711_SCK_PIN, Pin.OUT)

# Construction power-cycles the chip and settles it (HX711.__init__).
hx = HX711(clock_pin, data_pin, gain=HX711_GAIN)

# After init, channel A @ gain 128 maps to the driver's internal GAIN code 1
# (one extra clock pulse). Print it as a sanity check.
print("HX711 internal GAIN code:", hx.GAIN, "(expect 1 for gain=128)")
if hx.GAIN != 1:
    print("WARNING: expected internal GAIN == 1 for gain=128, got", hx.GAIN)

try:
    print("HX711 initialized.")

    cfg = make_calibration_config()
    scale_g_per_count, offset_count = run_weight_calibration(hx, cfg)

    print()
    print("Calibration result:")
    print("  scale (g/count):", scale_g_per_count)
    print("  tare offset (count):", offset_count)

    # ----------------------------
    # Live grams readout
    # ----------------------------
    # grams = (raw - offset) * scale_g_per_count. The magnitude is what the
    # sanity check cares about; sign depends on load-cell wiring polarity.
    print()
    print("Live readout in grams. Place/remove weights to sanity-check.")
    if LIVE_READOUT_SECONDS > 0:
        print("Running for", LIVE_READOUT_SECONDS, "s...")
    else:
        print("Running until stopped (Ctrl+C).")

    live_start = time.ticks_ms()
    while True:
        avg = read_avg_simple(hx, LIVE_READOUT_SAMPLE_AVG)
        tared = avg - offset_count
        grams = tared * scale_g_per_count
        print("grams: {:.1f}   (tared_count: {:.0f})".format(grams, tared))

        if (LIVE_READOUT_SECONDS > 0 and
                time.ticks_diff(time.ticks_ms(), live_start) >= LIVE_READOUT_SECONDS * 1000):
            break

        time.sleep_ms(LIVE_READOUT_DELAY_MS)

    print("Live readout complete.")

finally:
    print("Powering down HX711.")
    hx.power_down()
