"""Tests for the Feetech STS3215 MicroPython driver.

Packet bytes used in assertions are hand-computed against the protocol spec.
"""
import pytest

from sts3215 import (
    STS3215,
    ServoError,
    _checksum,
    degrees_to_position,
    position_to_degrees,
    REG_GOAL_ACC,
    REG_GOAL_POSITION,
    REG_PRESENT_POSITION,
    REG_PRESENT_LOAD,
    REG_PRESENT_TEMPERATURE,
    REG_TORQUE_ENABLE,
    REG_ID,
    REG_LOCK,
)


# ---- helpers ----------------------------------------------------------------

def _status(servo_id, params=b"", err=0):
    """Build a well-formed status packet for the fake servo to reply with."""
    length = len(params) + 2
    body = bytes((servo_id, length, err)) + bytes(params)
    return b"\xff\xff" + body + bytes((_checksum(body),))


# ---- checksum --------------------------------------------------------------

def test_checksum_zero():
    # ~0 & 0xFF = 0xFF
    assert _checksum(b"\x00") == 0xFF


def test_checksum_known_values():
    # ping ID=1: ID+LEN+INST = 1+2+1 = 4 -> ~4 & 0xFF = 0xFB
    assert _checksum(bytes((1, 2, 1))) == 0xFB
    # read pos ID=1: 1+4+2+0x38+2 = 0x41 -> ~0x41 & 0xFF = 0xBE
    assert _checksum(bytes((1, 4, 2, 0x38, 2))) == 0xBE
    # status ping reply ID=1, err=0: 1+2+0 = 3 -> 0xFC
    assert _checksum(bytes((1, 2, 0))) == 0xFC


# ---- degrees <-> position --------------------------------------------------

def test_degrees_position_boundaries():
    assert degrees_to_position(0) == 0
    assert degrees_to_position(360) == 4095
    assert position_to_degrees(0) == 0.0
    assert position_to_degrees(4095) == pytest.approx(360.0)


def test_degrees_position_clamp():
    assert degrees_to_position(-10) == 0
    assert degrees_to_position(400) == 4095
    assert position_to_degrees(-1) == 0.0
    assert position_to_degrees(9999) == pytest.approx(360.0)


def test_round_trip_midrange():
    for deg in (0, 45, 90, 180, 270, 360):
        p = degrees_to_position(deg)
        assert abs(position_to_degrees(p) - deg) < 0.2


# ---- packet building -------------------------------------------------------

def test_ping_packet_bytes(driver, uart):
    uart.feed(_status(1))
    assert driver.ping(1) is True
    assert bytes(uart.written) == b"\xff\xff\x01\x02\x01\xfb"


def test_ping_returns_false_on_timeout(driver, uart):
    # No reply queued -> _recv times out -> ping returns False.
    assert driver.ping(2) is False


def test_read_position_packet_and_parse(driver, uart):
    # 2048 = 0x0800 -> low=0x00 high=0x08
    uart.feed(_status(1, params=b"\x00\x08"))
    pos = driver.read_position(1)
    assert pos == 2048
    assert bytes(uart.written) == b"\xff\xff\x01\x04\x02\x38\x02\xbe"


def test_write_position_packet(driver, uart):
    # write_position triggers a 1-byte status reply
    uart.feed(_status(1))
    driver.write_position(1, 2048, speed=1000, accel=50)
    expected = bytes([
        0xFF, 0xFF, 0x01, 0x0A, 0x03,
        REG_GOAL_ACC,
        50,            # accel
        0x00, 0x08,    # position 2048 LE
        0x00, 0x00,    # goal time placeholder
        0xE8, 0x03,    # speed 1000 LE
        0xA3,          # checksum
    ])
    assert bytes(uart.written) == expected


def test_write_position_addresses_goal_acc_register(driver, uart):
    """Sanity: the multi-byte write begins at the acceleration register so the
    contiguous block lays out as ACC | POS_L POS_H | TIME_L TIME_H | SPD_L SPD_H,
    which matches REG_GOAL_POSITION = REG_GOAL_ACC + 1."""
    assert REG_GOAL_POSITION == REG_GOAL_ACC + 1


def test_enable_torque_packet(driver, uart):
    uart.feed(_status(1))
    driver.enable_torque(1)
    expected = bytes([
        0xFF, 0xFF, 0x01, 0x04, 0x03, REG_TORQUE_ENABLE, 0x01,
    ])
    expected += bytes((_checksum(expected[2:]),))
    assert bytes(uart.written) == expected


def test_disable_torque_packet(driver, uart):
    uart.feed(_status(1))
    driver.disable_torque(1)
    last = uart.written
    # check the data byte position (after addr) is 0x00
    assert last[6] == 0x00
    assert last[5] == REG_TORQUE_ENABLE


# ---- response parsing / error paths ----------------------------------------

def test_bad_checksum_raises(driver, uart):
    pkt = bytearray(_status(1, params=b"\x00\x08"))
    pkt[-1] ^= 0xFF  # corrupt checksum
    uart.feed(bytes(pkt))
    with pytest.raises(ServoError, match="checksum"):
        driver.read_position(1)


def test_wrong_id_raises(driver, uart):
    uart.feed(_status(servo_id=2, params=b"\x00\x08"))  # asked for id 1
    with pytest.raises(ServoError, match="wrong id"):
        driver.read_position(1)


def test_timeout_raises(driver, uart):
    # no bytes queued
    with pytest.raises(ServoError, match="timeout"):
        driver.read_position(1)


def test_servo_error_byte_raises(driver, uart):
    uart.feed(_status(1, params=b"", err=0x20))
    with pytest.raises(ServoError, match="error byte"):
        driver.enable_torque(1)


def test_bad_header_raises(driver, uart):
    uart.feed(b"\x00\x00\x01\x02\x00\xfc")  # bogus header
    with pytest.raises(ServoError, match="header"):
        driver._read_register(1, 0x38, 2)


def test_unexpected_length_raises(driver, uart):
    # claim a 3-byte payload when the caller asked for 2
    bogus = _status(1, params=b"\x00\x08\x00")
    uart.feed(bogus)
    with pytest.raises(ServoError, match="length"):
        driver.read_position(1)


# ---- direction-pin polarity ------------------------------------------------

def test_direction_pin_high_before_write_low_after(driver_with_pin, uart, pin):
    uart.feed(_status(1))
    pin.history.clear()
    driver_with_pin.ping(1)
    # First transition during the call must be high (TX), then low (RX).
    assert pin.history[0] == 1
    assert pin.history[-1] == 0


def test_direction_pin_low_during_recv(driver_with_pin, uart, pin):
    """After _send returns, the pin must be LOW so the bus can drive RX into us."""
    uart.feed(_status(1, params=b"\x00\x08"))
    pin.history.clear()
    driver_with_pin.read_position(1)
    # Sequence: starts high (TX) then drops to low (RX) before reading.
    # We expect at least one 1->0 transition.
    transitions = list(zip(pin.history, pin.history[1:]))
    assert (1, 0) in transitions


def test_direction_pin_initialised_low():
    """Constructor should leave the bus in RX (low) so a stray servo doesn't
    contend with us before we decide to talk."""
    from conftest import FakePin, FakeUART
    p = FakePin()
    u = FakeUART()
    STS3215(u, direction_pin=p)
    assert p.value() == 0


# ---- read_temperature / read_load -----------------------------------------

def test_read_temperature(driver, uart):
    uart.feed(_status(1, params=b"\x2A"))  # 42 °C
    assert driver.read_temperature(1) == 42


def test_read_load_positive(driver, uart):
    # 0x0064 = 100, sign bit 10 clear -> +100
    uart.feed(_status(1, params=b"\x64\x00"))
    assert driver.read_load(1) == 100


def test_read_load_negative(driver, uart):
    # bit 10 set + magnitude 100 -> -100
    raw = 0x0400 | 100
    uart.feed(_status(1, params=bytes((raw & 0xFF, (raw >> 8) & 0xFF))))
    assert driver.read_load(1) == -100


def test_read_load_zero(driver, uart):
    uart.feed(_status(1, params=b"\x00\x00"))
    assert driver.read_load(1) == 0


def test_read_load_max_positive(driver, uart):
    # 0x03FF = 1023 (max magnitude with sign bit clear)
    uart.feed(_status(1, params=b"\xFF\x03"))
    assert driver.read_load(1) == 1023


# ---- set_id (EEPROM unlock dance) ------------------------------------------

def test_set_id_unlocks_writes_id_relocks(driver, uart):
    # three writes happen in sequence: lock=0, id=new, lock=1
    uart.feed(_status(1))   # ack for unlock to id=1
    uart.feed(_status(1))   # ack for id write to id=1
    uart.feed(_status(7))   # ack for re-lock to new id=7

    driver.set_id(1, 7)

    written = bytes(uart.written)
    # 3 packets, header+id+len+inst+addr+val+chk = 8 bytes each
    assert len(written) == 24
    # first packet: WRITE LOCK 0 to id=1
    assert written[2] == 1 and written[5] == REG_LOCK and written[6] == 0
    # second packet: WRITE ID new=7 to id=1
    assert written[10] == 1 and written[13] == REG_ID and written[14] == 7
    # third packet: WRITE LOCK 1 to id=7  (note new id!)
    assert written[18] == 7 and written[21] == REG_LOCK and written[22] == 1


def test_set_id_rejects_invalid_id(driver, uart):
    with pytest.raises(ValueError):
        driver.set_id(1, 254)
    with pytest.raises(ValueError):
        driver.set_id(1, -1)
