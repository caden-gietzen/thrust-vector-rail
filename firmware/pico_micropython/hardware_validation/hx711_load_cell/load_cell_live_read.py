# load_cell_live_read.py
# Live HX711 RAW-count readout for sanity / wiggle testing (NO calibration).
#
# Purpose:
#   Tare once, then continuously stream the raw 24-bit HX711 count, a tared
#   count, the peak-to-peak spread over a short rolling window, and a railed
#   flag. This is the tool for a connection "wiggle test".
#
# Wiggle test procedure:
#   1. Run this with no load and let it settle to a steady baseline.
#   2. Gently wiggle ONE wire / connector at a time, watching the output:
#        - the 4 load-cell bridge wires first (E+, E-, A+, A-) -- these set the
#          analog value and are the prime suspect for railing / sign flips,
#        - then DOUT, SCK, VCC, GND.
#   3. A solid rig holds the count within a few hundred counts (small pp_window).
#      If wiggling a specific wire spikes the count, rails it (flag RAILED),
#      flips its sign, or triggers "READ ERROR (no response)", that wire/joint
#      is the fault.
#
# Architecture:
#   The HX711 power cycle + settle is handled inside HX711.__init__
#   (lib/hx711_gpio.py -> power_cycle()), so there is no manual clock-low/settle
#   here. No calibration scale is used -- this reports raw counts only.
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21
#
# Requires lib/hx711_gpio.py on the Pico (auto-synced by run_pico_and_pull.py).
# Press Ctrl+C to stop.

from machine import Pin
from utime import sleep, sleep_ms
from hx711_gpio import HX711


# ============================================================
# Configuration
# ============================================================

HX711_DAT_PIN = 20
HX711_SCK_PIN = 21
HX711_GAIN = 128

# Tare
DO_TARE = True
TARE_SAMPLES = 50
TARE_SAMPLE_DELAY_MS = 20
TARE_COUNTDOWN_S = 5

# Continuous readout
READ_INTERVAL_MS = 100
WINDOW_SAMPLES = 20            # rolling window for peak-to-peak

# HX711 is a signed 24-bit ADC: full scale is +/-2^23.
HX711_FULL_SCALE = 0x7FFFFF    # 8388607
RAIL_THRESHOLD = 8000000       # |count| above this ~ railed (stuck-at-large symptom)


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


def setup_hx711():
    print("Initializing HX711...")
    print("DAT pin:", HX711_DAT_PIN)
    print("SCK pin:", HX711_SCK_PIN)

    data_pin = Pin(HX711_DAT_PIN, Pin.IN)
    clock_pin = Pin(HX711_SCK_PIN, Pin.OUT)

    # HX711.__init__ forces a power cycle and settles the amplifier.
    hx = HX711(clock_pin, data_pin, gain=HX711_GAIN)

    print("HX711 internal GAIN code:", hx.GAIN, "(expect 1 for gain=128)")
    if hx.GAIN != 1:
        print("WARNING: expected internal GAIN == 1 for gain=128, got", hx.GAIN)

    print("HX711 initialized.")
    return hx


def safe_read(hx):
    """Return the raw count, or None if the sensor did not respond. A None
    during a wiggle test usually points at DOUT/SCK/power."""
    try:
        return hx.read()
    except OSError as e:
        print("READ ERROR (no response):", e)
        return None


def tare(hx):
    countdown("Remove all load from the load cell for tare.", TARE_COUNTDOWN_S)
    print("Taring with", TARE_SAMPLES, "samples...")

    total = 0
    good = 0
    for _ in range(TARE_SAMPLES):
        raw = safe_read(hx)
        if raw is not None:
            total += raw
            good += 1
        sleep_ms(TARE_SAMPLE_DELAY_MS)

    if good == 0:
        print("Tare failed: no valid samples. Offset set to 0.")
        return 0

    offset = total / good
    print("Tare complete. Offset count:", offset, "(", good, "valid samples )")
    return offset


# ============================================================
# Main
# ============================================================

def main():
    print("Load cell RAW live readout (no calibration) - wiggle test tool")

    hx = None
    try:
        hx = setup_hx711()
        offset = tare(hx) if DO_TARE else 0

        print()
        print("Reading raw counts. Wiggle ONE wire at a time. Ctrl+C to stop.")
        print("{:<8} {:<14} {:<14} {:<12} {:<8}".format(
            "sample", "raw_count", "tared_count", "pp_window", "flag"))

        window = []
        sample = 0

        while True:
            raw = safe_read(hx)
            sample += 1

            if raw is None:
                # safe_read already printed the error; keep cadence.
                sleep_ms(READ_INTERVAL_MS)
                continue

            tared = raw - offset

            window.append(raw)
            if len(window) > WINDOW_SAMPLES:
                window.pop(0)
            pp = max(window) - min(window)

            flag = "RAILED" if abs(raw) >= RAIL_THRESHOLD else "ok"

            print("{:<8} {:<14.0f} {:<14.0f} {:<12.0f} {:<8}".format(
                sample, raw, tared, pp, flag))

            sleep_ms(READ_INTERVAL_MS)

    except KeyboardInterrupt:
        print("Stopped.")

    finally:
        if hx is not None:
            print("Powering down HX711.")
            hx.power_down()


main()
