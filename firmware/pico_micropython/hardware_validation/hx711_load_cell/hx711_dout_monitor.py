# hx711_dout_monitor.py
# Simple HX711 DOUT readiness monitor.
#
# Purpose:
#   Verify that the HX711 DOUT/DAT pin goes low when data is ready.
#
# Expected behavior:
#   DOUT = 1  -> data not ready
#   DOUT = 0  -> data ready
#
# Wiring:
#   HX711 DAT / DOUT -> GP20
#   HX711 SCK / CLK  -> GP21

import time
from machine import Pin

HX711_DAT_PIN = 20
HX711_SCK_PIN = 21

data_pin = Pin(HX711_DAT_PIN, Pin.IN)
clock_pin = Pin(HX711_SCK_PIN, Pin.OUT)

# Keep SCK low so HX711 is awake.
clock_pin.value(0)

print("HX711 DOUT monitor")
print("DAT pin:", HX711_DAT_PIN)
print("SCK pin:", HX711_SCK_PIN)
print("Expected: DOUT should periodically read 0 when data is ready.")

while True:
    print("DOUT =", data_pin.value())
    time.sleep_ms(250)