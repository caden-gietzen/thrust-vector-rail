# mavlink_uart_sniff.py
# Verify that Pixhawk MAVLink bytes are reaching the Pico over UART.
#
# Wiring:
#   Pixhawk TELEM2 TX -> Pico GP1 / UART0 RX
#   Pixhawk TELEM2 RX -> Pico GP0 / UART0 TX
#   Pixhawk GND       -> Pico GND
#
# Mission Planner:
#   SERIAL2_PROTOCOL = 2
#   SERIAL2_BAUD     = 57

from machine import UART, Pin
import time

UART_ID = 0
UART_TX_PIN = 0
UART_RX_PIN = 1
BAUDRATE = 57600

uart = UART(
    UART_ID,
    baudrate=BAUDRATE,
    tx=Pin(UART_TX_PIN),
    rx=Pin(UART_RX_PIN),
)

print("MAVLink UART sniff test")
print("UART:", UART_ID)
print("TX pin:", UART_TX_PIN)
print("RX pin:", UART_RX_PIN)
print("Baudrate:", BAUDRATE)
print("Waiting for bytes...")

total_bytes = 0
last_report_ms = time.ticks_ms()

while True:
    n = uart.any()

    if n:
        data = uart.read(n)
        total_bytes += len(data)

        print("Received", len(data), "bytes:", data[:32])

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_report_ms) > 1000:
        print("Total bytes received:", total_bytes)
        last_report_ms = now_ms

    time.sleep_ms(20)