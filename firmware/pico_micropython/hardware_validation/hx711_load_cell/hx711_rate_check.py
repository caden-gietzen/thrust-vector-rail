from machine import Pin
import time

DOUT = Pin(20, Pin.IN)
SCK = Pin(21, Pin.OUT)
SCK.value(0)

def hx711_read_raw():
    # Wait until HX711 data is ready
    while DOUT.value() == 1:
        pass

    value = 0

    # Read 24 bits
    for _ in range(24):
        SCK.value(1)
        value = (value << 1) | DOUT.value()
        SCK.value(0)

    # 25th pulse: gain = 128, channel A
    SCK.value(1)
    SCK.value(0)

    # Convert signed 24-bit value
    if value & 0x800000:
        value -= 0x1000000

    return value

N = 200
times_us = []

print("Measuring HX711 sample rate...")

for i in range(N):
    hx711_read_raw()
    times_us.append(time.ticks_us())

# Compute time differences between consecutive completed readings
dts = []
for i in range(1, len(times_us)):
    dt = time.ticks_diff(times_us[i], times_us[i - 1])
    dts.append(dt)

avg_dt_us = sum(dts) / len(dts)
fs_hz = 1_000_000 / avg_dt_us

print("Average dt:", avg_dt_us, "us")
print("Estimated sample rate:", fs_hz, "Hz")
print("Nyquist frequency:", fs_hz / 2, "Hz")