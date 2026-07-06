# mavlink_battery.py
# Minimal non-blocking MAVLink SYS_STATUS battery reader for MicroPython / Pico.
#
# Decodes:
#   MAVLink msg_id 1 = SYS_STATUS
#
# Battery fields:
#   voltage_battery   uint16, millivolts
#   current_battery   int16, centi-amps
#   battery_remaining int8, percent
#
# On construction the reader also *transmits* a MAV_CMD_SET_MESSAGE_INTERVAL
# request so the flight controller streams SYS_STATUS at a chosen rate (default
# 10 Hz) instead of the ArduPilot EXT_STAT default (~1-2 Hz). This uses the same
# UART the caller passes in, whose TX is already wired to the autopilot's
# telemetry RX (e.g. GP0 in the DAQ scripts). Harmless if TX is unconnected.

import time
import struct

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD

MSG_ID_SYS_STATUS = 1

SYS_STATUS_MIN_LEN = 31
SYS_STATUS_VOLTAGE_OFFSET = 14
SYS_STATUS_CURRENT_OFFSET = 16
SYS_STATUS_REMAINING_OFFSET = 30

# --- Outbound stream-rate request (COMMAND_LONG / SET_MESSAGE_INTERVAL) ---
MSG_ID_COMMAND_LONG = 76
CRC_EXTRA_COMMAND_LONG = 152          # MAVLink per-message CRC seed for COMMAND_LONG
MAV_CMD_SET_MESSAGE_INTERVAL = 511

DEFAULT_STREAM_RATE_HZ = 10


def _crc_accumulate(b, crc):
    """MAVLink CRC-16/MCRF4XX single-byte accumulate (init 0xFFFF)."""
    tmp = b ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


class MavlinkBatteryReader:
    def __init__(
        self,
        uart,
        request_stream_rate_hz=DEFAULT_STREAM_RATE_HZ,
        auto_request=True,
        sysid=255,
        compid=190,
        target_system=1,
        target_component=1,
    ):
        self.uart = uart
        self.buffer = b""

        self.voltage_V = None
        self.current_A = None
        self.battery_remaining_pct = None
        self.last_update_ms = None

        self.total_bytes = 0
        self.total_packets = 0
        self.sys_status_packets = 0
        self.bad_frames = 0

        # Outbound (stream-rate request) identity + state.
        self.sysid = sysid
        self.compid = compid
        self.target_system = target_system
        self.target_component = target_component
        self._tx_seq = 0

        # Ask the FC to stream SYS_STATUS at the requested rate on startup.
        if auto_request and request_stream_rate_hz:
            self.request_message_interval(MSG_ID_SYS_STATUS, request_stream_rate_hz)

    def _build_v1_frame(self, msg_id, payload, crc_extra):
        seq = self._tx_seq
        self._tx_seq = (seq + 1) & 0xFF

        header = bytes((len(payload), seq, self.sysid, self.compid, msg_id))

        crc = 0xFFFF
        for b in header:
            crc = _crc_accumulate(b, crc)
        for b in payload:
            crc = _crc_accumulate(b, crc)
        crc = _crc_accumulate(crc_extra, crc)

        return bytes((MAVLINK_V1_STX,)) + header + payload + struct.pack("<H", crc)

    def request_message_interval(self, msg_id, rate_hz):
        """
        Ask the flight controller to emit `msg_id` at `rate_hz` via
        MAV_CMD_SET_MESSAGE_INTERVAL (wrapped in COMMAND_LONG, MAVLink v1).

        Requires the Pico UART TX to be wired to the autopilot telemetry RX;
        it is a no-op on the wire otherwise. `rate_hz <= 0` restores the FC's
        default stream rate for that message.
        """
        if rate_hz and rate_hz > 0:
            interval_us = int(1000000 / rate_hz)
        else:
            interval_us = 0  # 0 = restore stream default

        # COMMAND_LONG payload, wire order (fields sorted largest-type-first):
        #   param1..param7 (float) then command (uint16) then
        #   target_system, target_component, confirmation (uint8).
        payload = struct.pack(
            "<7f",
            float(msg_id),        # param1: target message id
            float(interval_us),   # param2: interval, microseconds
            0.0, 0.0, 0.0, 0.0, 0.0,
        )
        payload += struct.pack("<H", MAV_CMD_SET_MESSAGE_INTERVAL)
        payload += bytes((self.target_system, self.target_component, 0))

        frame = self._build_v1_frame(
            MSG_ID_COMMAND_LONG, payload, CRC_EXTRA_COMMAND_LONG
        )
        try:
            self.uart.write(frame)
        except Exception:
            # A wiring/UART-write failure must never take down the DAQ loop.
            pass

    def update(self):
        """
        Non-blocking read/parse.

        Call this often from the main DAQ loop.
        """
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
        """
        Age of the most recent decoded SYS_STATUS battery value.
        Returns None if no battery packet has been decoded yet.
        """
        if self.last_update_ms is None:
            return None

        return time.ticks_diff(time.ticks_ms(), self.last_update_ms)

    def snapshot(self):
        """
        Returns a tuple that is safe to log.

        If no value has been received yet, numerical fields are blank strings.
        """
        if not self.has_battery():
            return ("", "", "", "", self.total_packets, self.sys_status_packets, self.bad_frames)

        return (
            self.voltage_V,
            self.current_A,
            self.battery_remaining_pct,
            self.age_ms(),
            self.total_packets,
            self.sys_status_packets,
            self.bad_frames,
        )

    def _drop_bytes(self, n):
        self.buffer = self.buffer[n:]

    def _parse_buffer(self):
        while True:
            if len(self.buffer) < 2:
                return

            start_index = -1

            for i in range(len(self.buffer)):
                b = self.buffer[i]
                if b == MAVLINK_V1_STX or b == MAVLINK_V2_STX:
                    start_index = i
                    break

            if start_index < 0:
                self.bad_frames += 1
                self.buffer = b""
                return

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

            if not consumed:
                return

    def _parse_v1_frame(self):
        if len(self.buffer) < 6:
            return False

        payload_len = self.buffer[1]
        frame_len = 6 + payload_len + 2

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

        if len(payload) < SYS_STATUS_MIN_LEN:
            return

        voltage_mV = struct.unpack_from("<H", payload, SYS_STATUS_VOLTAGE_OFFSET)[0]
        current_cA = struct.unpack_from("<h", payload, SYS_STATUS_CURRENT_OFFSET)[0]
        battery_remaining = struct.unpack_from("<b", payload, SYS_STATUS_REMAINING_OFFSET)[0]

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