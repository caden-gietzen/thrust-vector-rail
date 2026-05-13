# mavlink_battery_monitor_fixed.py
# Minimal MAVLink battery monitor for Raspberry Pi Pico.
#
# Goal:
#   Receive MAVLink from Pixhawk over UART and print battery voltage/current.
#
# Wiring, listen-only for now:
#   Pixhawk TELEM2 TX -> Pico GP1 / UART0 RX
#   Pixhawk GND       -> Pico GND
#
# Mission Planner:
#   SERIAL2_PROTOCOL = 2
#   SERIAL2_BAUD     = 57
#   SR2_EXT_STAT     = 1 or 2

from machine import UART, Pin
import time
import struct


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

MSG_ID_SYS_STATUS = 1

SYS_STATUS_MIN_LEN = 31
SYS_STATUS_VOLTAGE_OFFSET = 14      # uint16, millivolts
SYS_STATUS_CURRENT_OFFSET = 16      # int16, centi-amps
SYS_STATUS_REMAINING_OFFSET = 30    # int8, percent


class MinimalMavlinkBatteryReader:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = b""

        self.voltage_V = None
        self.current_A = None
        self.battery_remaining_pct = None
        self.last_update_ms = None

        self.total_packets = 0
        self.sys_status_packets = 0
        self.bad_frames = 0
        self.total_bytes = 0

    def update(self):
        n = self.uart.any()

        if n:
            data = self.uart.read(n)
            if data:
                self.total_bytes += len(data)
                self.buffer += data

        self._parse_buffer()

    def has_battery(self):
        return self.last_update_ms is not None

    def age_ms(self):
        if self.last_update_ms is None:
            return None

        return time.ticks_diff(time.ticks_ms(), self.last_update_ms)

    def _drop_bytes(self, n):
        self.buffer = self.buffer[n:]

    def _parse_buffer(self):
        while True:
            # Need at least start byte + payload length.
            if len(self.buffer) < 2:
                return

            # Find next MAVLink start byte.
            start_index = -1

            for i in range(len(self.buffer)):
                b = self.buffer[i]

                if b == MAVLINK_V1_STX or b == MAVLINK_V2_STX:
                    start_index = i
                    break

            # No start byte found. Drop all garbage.
            if start_index < 0:
                self.bad_frames += 1
                self.buffer = b""
                return

            # Drop garbage before start byte.
            if start_index > 0:
                self._drop_bytes(start_index)

            if len(self.buffer) < 2:
                return

            stx = self.buffer[0]

            if stx == MAVLINK_V1_STX:
                consumed = self._parse_v1_frame()

            elif stx == MAVLINK_V2_STX:
                consumed = self._parse_v2_frame()

            else:
                self.bad_frames += 1
                self._drop_bytes(1)
                consumed = True

            # Critical fix:
            # If parser needs more bytes, stop the while loop.
            if not consumed:
                return

    def _parse_v1_frame(self):
        if len(self.buffer) < 6:
            return False

        payload_len = self.buffer[1]
        frame_len = 6 + payload_len + 2  # header + payload + checksum

        if len(self.buffer) < frame_len:
            return False

        frame = self.buffer[:frame_len]
        self._drop_bytes(frame_len)

        msg_id = frame[5]
        payload = frame[6:6 + payload_len]

        self.total_packets += 1
        self._handle_message(msg_id, payload)

        return True

    def _parse_v2_frame(self):
        if len(self.buffer) < 10:
            return False

        payload_len = self.buffer[1]
        incompat_flags = self.buffer[2]

        signature_len = 13 if (incompat_flags & 0x01) else 0
        frame_len = 10 + payload_len + 2 + signature_len

        if len(self.buffer) < frame_len:
            return False

        frame = self.buffer[:frame_len]
        self._drop_bytes(frame_len)

        msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
        payload = frame[10:10 + payload_len]

        self.total_packets += 1
        self._handle_message(msg_id, payload)

        return True

    def _handle_message(self, msg_id, payload):
        if msg_id != MSG_ID_SYS_STATUS:
            return

        print("Got SYS_STATUS. payload_len:", len(payload))

        if len(payload) < SYS_STATUS_MIN_LEN:
            print("SYS_STATUS too short:", len(payload))
            return

        voltage_mV = struct.unpack_from(
            "<H",
            payload,
            SYS_STATUS_VOLTAGE_OFFSET
        )[0]

        current_cA = struct.unpack_from(
            "<h",
            payload,
            SYS_STATUS_CURRENT_OFFSET
        )[0]

        battery_remaining = struct.unpack_from(
            "<b",
            payload,
            SYS_STATUS_REMAINING_OFFSET
        )[0]

        print(
            "RAW SYS_STATUS:",
            "voltage_mV:", voltage_mV,
            "current_cA:", current_cA,
            "remaining:", battery_remaining
        )

        if voltage_mV == 0xFFFF:
            self.voltage_V = None
        else:
            self.voltage_V = voltage_mV / 1000.0

        if current_cA == -1:
            self.current_A = None
        else:
            self.current_A = current_cA / 100.0

        self.battery_remaining_pct = battery_remaining
        self.last_update_ms = time.ticks_ms()
        self.sys_status_packets += 1


# ----------------------------
# Main test script
# ----------------------------

uart = UART(
    UART_ID,
    baudrate=BAUDRATE,
    tx=Pin(UART_TX_PIN),
    rx=Pin(UART_RX_PIN),
)

mav = MinimalMavlinkBatteryReader(uart)

print("Minimal MAVLink battery monitor FIXED")
print("UART:", UART_ID)
print("TX pin:", UART_TX_PIN)
print("RX pin:", UART_RX_PIN)
print("Baudrate:", BAUDRATE)
print("Waiting for SYS_STATUS battery data...")

last_print_ms = time.ticks_ms()

while True:
    mav.update()

    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_print_ms) >= 500:
        if mav.has_battery():
            print(
                "V:",
                mav.voltage_V,
                "A:",
                mav.current_A,
                "remaining:",
                mav.battery_remaining_pct,
                "age_ms:",
                mav.age_ms(),
                "sys_status_packets:",
                mav.sys_status_packets,
                "total_packets:",
                mav.total_packets,
                "total_bytes:",
                mav.total_bytes,
                "buffer_len:",
                len(mav.buffer),
                "bad_frames:",
                mav.bad_frames,
            )
        else:
            print(
                "No decoded battery yet.",
                "sys_status_packets:",
                mav.sys_status_packets,
                "total_packets:",
                mav.total_packets,
                "total_bytes:",
                mav.total_bytes,
                "buffer_len:",
                len(mav.buffer),
                "bad_frames:",
                mav.bad_frames,
            )

        last_print_ms = now_ms

    time.sleep_ms(5)