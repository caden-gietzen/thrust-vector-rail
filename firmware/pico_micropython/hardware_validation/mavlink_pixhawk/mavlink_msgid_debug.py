# mavlink_msgid_debug.py
# Prints MAVLink v1/v2 message IDs received on Pico UART.
#
# Wiring:
#   Pixhawk TELEM2 TX -> Pico GP1 / UART0 RX
#   Pixhawk TELEM2 RX -> Pico GP0 / UART0 TX  optional for listen-only
#   Pixhawk GND       -> Pico GND
#
# Mission Planner:
#   SERIAL2_PROTOCOL = 2
#   SERIAL2_BAUD     = 57

from machine import UART, Pin
import time


# ----------------------------
# UART settings
# ----------------------------

UART_ID = 0
UART_TX_PIN = 0
UART_RX_PIN = 1
BAUDRATE = 57600


# ----------------------------
# MAVLink constants
# ----------------------------

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD


# ----------------------------
# UART init
# ----------------------------

uart = UART(
    UART_ID,
    baudrate=BAUDRATE,
    tx=Pin(UART_TX_PIN),
    rx=Pin(UART_RX_PIN),
)


# ----------------------------
# Parser state
# ----------------------------

buf = bytearray()

total_bytes = 0
total_frames = 0
bad_bytes = 0
msg_counts = {}

last_print = time.ticks_ms()


def add_msg(msg_id):
    global total_frames

    total_frames += 1
    msg_counts[msg_id] = msg_counts.get(msg_id, 0) + 1


def drop_bytes(n):
    """
    MicroPython-safe buffer trimming.

    Do NOT use:
        del buf[:n]

    because some Pico MicroPython builds do not support
    item/slice deletion on bytearray.
    """
    global buf

    if n <= 0:
        return

    buf = buf[n:]


def clear_buffer():
    global buf
    buf = bytearray()


def parse_buffer():
    global buf, bad_bytes

    while True:
        # Need at least start byte + payload length byte.
        if len(buf) < 2:
            return

        # Find next MAVLink start byte.
        start = -1

        for i in range(len(buf)):
            b = buf[i]

            if b == MAVLINK_V1_STX or b == MAVLINK_V2_STX:
                start = i
                break

        # No MAVLink start byte found.
        if start < 0:
            bad_bytes += len(buf)
            clear_buffer()
            return

        # Drop garbage before the start byte.
        if start > 0:
            bad_bytes += start
            drop_bytes(start)

        # Need to re-check after dropping.
        if len(buf) < 2:
            return

        stx = buf[0]

        # ----------------------------
        # MAVLink 1 frame
        # ----------------------------
        if stx == MAVLINK_V1_STX:
            # MAVLink 1 header is 6 bytes:
            # magic, len, seq, sysid, compid, msgid
            if len(buf) < 6:
                return

            payload_len = buf[1]
            frame_len = 6 + payload_len + 2  # header + payload + checksum

            if len(buf) < frame_len:
                return

            msg_id = buf[5]
            add_msg(msg_id)

            drop_bytes(frame_len)

        # ----------------------------
        # MAVLink 2 frame
        # ----------------------------
        elif stx == MAVLINK_V2_STX:
            # MAVLink 2 header is 10 bytes:
            # magic, len, incompat_flags, compat_flags, seq,
            # sysid, compid, msgid_low, msgid_mid, msgid_high
            if len(buf) < 10:
                return

            payload_len = buf[1]
            incompat_flags = buf[2]

            signature_len = 13 if (incompat_flags & 0x01) else 0
            frame_len = 10 + payload_len + 2 + signature_len

            if len(buf) < frame_len:
                return

            msg_id = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            add_msg(msg_id)

            drop_bytes(frame_len)

        else:
            bad_bytes += 1
            drop_bytes(1)


# ----------------------------
# Main loop
# ----------------------------

print("MAVLink message ID debugger")
print("UART:", UART_ID)
print("TX pin:", UART_TX_PIN)
print("RX pin:", UART_RX_PIN)
print("Baudrate:", BAUDRATE)
print("Waiting for MAVLink frames...")

while True:
    n = uart.any()

    if n:
        data = uart.read(n)

        if data:
            total_bytes += len(data)
            buf.extend(data)

    parse_buffer()

    now = time.ticks_ms()

    if time.ticks_diff(now, last_print) >= 1000:
        print()
        print("total_bytes:", total_bytes)
        print("total_frames:", total_frames)
        print("bad_bytes:", bad_bytes)
        print("buffer_len:", len(buf))
        print("message IDs seen:")

        for msg_id in sorted(msg_counts):
            print("  msg_id:", msg_id, "count:", msg_counts[msg_id])

        last_print = now

    time.sleep_ms(5)