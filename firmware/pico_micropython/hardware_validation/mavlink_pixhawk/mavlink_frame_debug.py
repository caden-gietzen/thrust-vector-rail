# mavlink_frame_debug.py
# Debug MAVLink UART stream on Raspberry Pi Pico.
#
# Goal:
#   Confirm whether the incoming UART bytes contain valid-looking MAVLink frames.
#
# Wiring:
#   Pixhawk TELEM TX -> Pico GP1 / UART0 RX
#   Pixhawk TELEM RX -> Pico GP0 / UART0 TX
#   Pixhawk GND      -> Pico GND

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
# Debug state
# ----------------------------

buffer = bytearray()

total_bytes = 0
v1_start_count = 0
v2_start_count = 0
v1_frames = 0
v2_frames = 0
dropped_bytes = 0

last_print_ms = time.ticks_ms()


# ----------------------------
# Helpers
# ----------------------------

def drop_bytes(n):
    global buffer
    buffer = buffer[n:]


def clear_buffer():
    global buffer
    buffer = bytearray()


def hex_bytes(data, max_len=80):
    shown = data[:max_len]
    return " ".join("{:02X}".format(b) for b in shown)


def message_name(msg_id):
    if msg_id == 0:
        return "HEARTBEAT"
    elif msg_id == 1:
        return "SYS_STATUS"
    elif msg_id == 24:
        return "GPS_RAW_INT"
    elif msg_id == 30:
        return "ATTITUDE"
    elif msg_id == 33:
        return "GLOBAL_POSITION_INT"
    elif msg_id == 74:
        return "VFR_HUD"
    elif msg_id == 147:
        return "BATTERY_STATUS"
    else:
        return "OTHER"


# ----------------------------
# Startup prints
# ----------------------------

print("MAVLink frame debug")
print("UART:", UART_ID)
print("TX pin:", UART_TX_PIN)
print("RX pin:", UART_RX_PIN)
print("Baudrate:", BAUDRATE)
print("Waiting for bytes...")


# ----------------------------
# Main loop
# ----------------------------

while True:
    n = uart.any()

    if n:
        data = uart.read(n)

        if data:
            total_bytes += len(data)
            buffer.extend(data)

    # Parse as much as possible.
    while True:
        if len(buffer) < 1:
            break

        # Search for MAVLink start byte.
        start_index = -1

        for i, b in enumerate(buffer):
            if b == MAVLINK_V1_STX or b == MAVLINK_V2_STX:
                start_index = i
                break

        # No start byte found.
        if start_index < 0:
            dropped_bytes += len(buffer)
            clear_buffer()
            break

        # Drop garbage before start byte.
        if start_index > 0:
            dropped_bytes += start_index
            drop_bytes(start_index)

        if len(buffer) < 2:
            break

        stx = buffer[0]

        # ----------------------------
        # MAVLink v1 frame
        # ----------------------------

        if stx == MAVLINK_V1_STX:
            v1_start_count += 1

            if len(buffer) < 6:
                break

            payload_len = buffer[1]
            frame_len = 6 + payload_len + 2

            if len(buffer) < frame_len:
                break

            frame = bytes(buffer[:frame_len])
            drop_bytes(frame_len)

            seq = frame[2]
            sys_id = frame[3]
            comp_id = frame[4]
            msg_id = frame[5]

            v1_frames += 1

            print()
            print("MAVLink v1 frame found")
            print("  payload_len:", payload_len)
            print("  frame_len:", frame_len)
            print("  seq:", seq)
            print("  sys_id:", sys_id)
            print("  comp_id:", comp_id)
            print("  msg_id:", msg_id)
            print("  message:", message_name(msg_id))
            print("  raw:", hex_bytes(frame))

        # ----------------------------
        # MAVLink v2 frame
        # ----------------------------

        elif stx == MAVLINK_V2_STX:
            v2_start_count += 1

            if len(buffer) < 10:
                break

            payload_len = buffer[1]
            incompat_flags = buffer[2]

            signature_len = 13 if (incompat_flags & 0x01) else 0
            frame_len = 10 + payload_len + 2 + signature_len

            if len(buffer) < frame_len:
                break

            frame = bytes(buffer[:frame_len])
            drop_bytes(frame_len)

            seq = frame[4]
            sys_id = frame[5]
            comp_id = frame[6]
            msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)

            v2_frames += 1

            print()
            print("MAVLink v2 frame found")
            print("  payload_len:", payload_len)
            print("  frame_len:", frame_len)
            print("  seq:", seq)
            print("  sys_id:", sys_id)
            print("  comp_id:", comp_id)
            print("  msg_id:", msg_id)
            print("  message:", message_name(msg_id))
            print("  raw:", hex_bytes(frame))

        # ----------------------------
        # Should not happen, but safe fallback
        # ----------------------------

        else:
            dropped_bytes += 1
            drop_bytes(1)

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_print_ms) >= 1000:
        print()
        print("Status")
        print("  total_bytes:", total_bytes)
        print("  v1_start_count:", v1_start_count)
        print("  v2_start_count:", v2_start_count)
        print("  v1_frames:", v1_frames)
        print("  v2_frames:", v2_frames)
        print("  dropped_bytes:", dropped_bytes)
        print("  buffer_len:", len(buffer))

        if len(buffer) > 0:
            print("  buffer:", hex_bytes(buffer))

        last_print_ms = now_ms

    time.sleep_ms(5)