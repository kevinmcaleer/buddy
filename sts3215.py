"""MicroPython driver for Feetech STS3215 serial bus servos.

Protocol is Dynamixel-derived: packets are
    0xFF 0xFF [ID] [LEN] [INSTRUCTION] [PARAMS...] [CHECKSUM]
where LEN = len(PARAMS) + 2 and CHECKSUM = (~(ID+LEN+INSTRUCTION+sum(PARAMS))) & 0xFF.
Multi-byte register values are little-endian.

Register addresses sourced from:
  - Feetech STS3215 datasheet / memory table (Feetech / WaveShare wiki)
  - Cross-checked against the lerobot project (huggingface/lerobot) and
    the Waveshare STS Serial Bus Servo wiki
    (https://www.waveshare.com/wiki/ST3215_Servo).
"""

from machine import UART, Pin
import time


# Instructions
_INST_PING = 0x01
_INST_READ = 0x02
_INST_WRITE = 0x03
_INST_SYNC_WRITE = 0x83

# Register map (STS3215). EEPROM area is locked by default; writing to EEPROM
# regs (e.g. ID at 0x05) requires unlocking via the Lock register (0x37).
REG_ID = 0x05               # 1 byte, EEPROM
REG_TORQUE_ENABLE = 0x28    # 1 byte
REG_GOAL_ACC = 0x29         # 1 byte
REG_GOAL_POSITION = 0x2A    # 2 bytes LE
REG_GOAL_SPEED = 0x2E       # 2 bytes LE
REG_LOCK = 0x37             # 1 byte; 0 = unlock EEPROM, 1 = lock
REG_PRESENT_POSITION = 0x38  # 2 bytes LE
REG_PRESENT_LOAD = 0x3C     # 2 bytes LE; bit 10 is sign (1 = CCW load)
REG_PRESENT_TEMPERATURE = 0x3F  # 1 byte, degrees Celsius

# Position scaling: 0..4095 spans the full 360° range.
POSITION_MAX = 4095
DEGREES_MAX = 360.0


class ServoError(Exception):
    """Raised on any protocol failure: timeout, bad checksum, wrong-id reply,
    malformed packet, or non-zero servo error byte."""


def _checksum(data):
    """Feetech checksum: bitwise NOT of the byte-sum of (ID, LEN, INST, PARAMS)."""
    return (~sum(data)) & 0xFF


def degrees_to_position(deg):
    """Map 0..360 degrees to 0..4095 servo ticks. Clamps out-of-range input."""
    if deg < 0:
        deg = 0
    elif deg > DEGREES_MAX:
        deg = DEGREES_MAX
    return int(round(deg * POSITION_MAX / DEGREES_MAX))


def position_to_degrees(pos):
    """Map 0..4095 servo ticks to 0..360 degrees. Clamps out-of-range input."""
    if pos < 0:
        pos = 0
    elif pos > POSITION_MAX:
        pos = POSITION_MAX
    return pos * DEGREES_MAX / POSITION_MAX


class STS3215:
    """Driver for one or more STS3215 servos on a single half-duplex UART bus.

    direction_pin (optional): Pin object used to switch a tri-state buffer
    (e.g. SN74LVC2G241) between TX and RX. Convention used here:
        HIGH  = transmit enabled (drive bus)
        LOW   = receive enabled  (release bus to listen)
    This matches Waveshare's STS bus servo driver board and most community
    schematics. If your hardware uses inverted polarity, wrap the pin.
    """

    # Per-byte transmission time at 1 Mbaud (the STS3215 default) is ~10 µs.
    # We pad slightly so the final stop bit clears before flipping the buffer.
    _TX_FLUSH_US = 100

    def __init__(self, uart, direction_pin=None, timeout_ms=50):
        self._uart = uart
        self._dir = direction_pin
        self._timeout_ms = timeout_ms
        if self._dir is not None:
            self._dir.value(0)  # start in RX

    # ---- low-level packet helpers ----

    def _build_packet(self, servo_id, instruction, params=()):
        params = bytes(params)
        length = len(params) + 2
        body = bytes((servo_id, length, instruction)) + params
        return b"\xff\xff" + body + bytes((_checksum(body),))

    def _send(self, packet):
        if self._dir is not None:
            self._dir.value(1)
        self._uart.write(packet)
        if self._dir is not None:
            # Wait for the UART FIFO to clear before releasing the bus.
            time.sleep_us(self._TX_FLUSH_US * len(packet))
            self._dir.value(0)

    def _read_exact(self, n):
        """Read exactly n bytes from the UART, polling until timeout_ms elapses."""
        buf = b""
        deadline = time.ticks_add(time.ticks_ms(), self._timeout_ms)
        while len(buf) < n:
            chunk = self._uart.read(n - len(buf))
            if chunk:
                buf += chunk
            else:
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    raise ServoError("timeout waiting for response")
                time.sleep_ms(1)
        return buf

    def _recv(self, expected_id, expected_param_len):
        """Read one status packet and return its parameter bytes.

        Validates the 0xFF 0xFF header, ID, length, checksum, and error byte.
        """
        header = self._read_exact(4)  # 0xFF 0xFF, ID, LEN
        if header[0] != 0xFF or header[1] != 0xFF:
            raise ServoError("bad header: {!r}".format(header[:2]))
        servo_id = header[2]
        length = header[3]
        if servo_id != expected_id:
            raise ServoError("wrong id in reply: got {} expected {}".format(
                servo_id, expected_id))
        if length != expected_param_len + 2:
            raise ServoError("unexpected length {} (want {})".format(
                length, expected_param_len + 2))
        # length covers ERR + PARAMS + CHECKSUM, so remaining = length
        rest = self._read_exact(length)
        err = rest[0]
        params = rest[1:-1]
        chk = rest[-1]
        body = bytes((servo_id, length, err)) + params
        if _checksum(body) != chk:
            raise ServoError("bad checksum")
        if err:
            raise ServoError("servo error byte 0x{:02x}".format(err))
        return params

    def _write_register(self, servo_id, address, data):
        """WRITE instruction to one register address. data is bytes/bytearray/tuple."""
        params = bytes((address,)) + bytes(data)
        self._send(self._build_packet(servo_id, _INST_WRITE, params))
        if servo_id != 0xFE:  # broadcast doesn't reply
            self._recv(servo_id, 0)

    def _read_register(self, servo_id, address, length):
        params = bytes((address, length))
        self._send(self._build_packet(servo_id, _INST_READ, params))
        return self._recv(servo_id, length)

    # ---- public API ----

    def ping(self, servo_id):
        """Return True if the servo with this id replies, False on timeout/error."""
        self._send(self._build_packet(servo_id, _INST_PING))
        try:
            self._recv(servo_id, 0)
        except ServoError:
            return False
        return True

    def read_position(self, servo_id):
        data = self._read_register(servo_id, REG_PRESENT_POSITION, 2)
        return data[0] | (data[1] << 8)

    def write_position(self, servo_id, position, speed=0, accel=0):
        """Move servo to position (0..4095) at given speed (0..4095) and accel (0..255).

        Writes acc, position and speed in one packet so the servo applies them
        atomically (reg layout is contiguous: ACC=0x29, POS=0x2A LE, _, _, SPEED=0x2E LE).
        """
        position &= 0xFFFF
        speed &= 0xFFFF
        accel &= 0xFF
        data = bytes((
            accel,
            position & 0xFF, (position >> 8) & 0xFF,
            0, 0,  # goal time placeholder (REG 0x2C-0x2D, leave 0)
            speed & 0xFF, (speed >> 8) & 0xFF,
        ))
        self._write_register(servo_id, REG_GOAL_ACC, data)

    def enable_torque(self, servo_id):
        self._write_register(servo_id, REG_TORQUE_ENABLE, b"\x01")

    def disable_torque(self, servo_id):
        self._write_register(servo_id, REG_TORQUE_ENABLE, b"\x00")

    def set_id(self, servo_id, new_id):
        """Change a servo's id. EEPROM lock is toggled around the write because
        the ID register lives in locked EEPROM on STS3215."""
        if not 0 <= new_id <= 253:
            raise ValueError("new_id must be 0..253")
        self._write_register(servo_id, REG_LOCK, b"\x00")  # unlock EEPROM
        try:
            self._write_register(servo_id, REG_ID, bytes((new_id,)))
        finally:
            # Re-lock against the *new* id (the servo answers as new_id immediately
            # after the ID write completes).
            self._write_register(new_id, REG_LOCK, b"\x01")

    def read_temperature(self, servo_id):
        data = self._read_register(servo_id, REG_PRESENT_TEMPERATURE, 1)
        return data[0]

    def read_load(self, servo_id):
        """Return signed load. Feetech encodes magnitude in bits 0..9 and
        direction in bit 10 (1 -> negative / CCW)."""
        data = self._read_register(servo_id, REG_PRESENT_LOAD, 2)
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x03FF
        if raw & 0x0400:
            return -magnitude
        return magnitude
