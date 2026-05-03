"""Tests for the high-level :mod:`buddy.arm` abstraction.

Packet bytes used in assertions are hand-computed from the Feetech sync-write
spec (instruction 0x83) and cross-checked against
:func:`sts3215._checksum`.
"""
import pytest

from sts3215 import _checksum, REG_GOAL_ACC, POSITION_MAX, DEGREES_MAX

from buddy.arm import (
    Arm,
    JointConfig,
    DEFAULT_JOINT_CONFIGS,
    _angle_to_position,
    _position_to_angle,
    _BROADCAST_ID,
    _SYNC_WRITE_DATA_LEN,
    _SYNC_WRITE_START,
)


# ---- helpers ---------------------------------------------------------------

def _status(servo_id, params=b"", err=0):
    """Build a well-formed status packet for the fake servo to reply with."""
    length = len(params) + 2
    body = bytes((servo_id, length, err)) + bytes(params)
    return b"\xff\xff" + body + bytes((_checksum(body),))


def _two_joint_arm(driver):
    """Two-joint arm useful for hand-calculated sync-write expectations."""
    cfgs = {
        "j1": {"servo_id": 1, "min_angle": 0.0, "max_angle": 360.0, "home": 0.0},
        "j2": {"servo_id": 2, "min_angle": 0.0, "max_angle": 360.0, "home": 0.0},
    }
    return Arm(driver, joint_configs=cfgs)


# ---- JointConfig validation -----------------------------------------------

def test_joint_config_rejects_invalid_sign():
    with pytest.raises(ValueError):
        JointConfig(servo_id=1, sign=0)


def test_joint_config_rejects_invalid_servo_id():
    with pytest.raises(ValueError):
        JointConfig(servo_id=300)


def test_joint_config_rejects_inverted_limits():
    with pytest.raises(ValueError):
        JointConfig(servo_id=1, min_angle=200, max_angle=100)


def test_joint_config_from_dict_round_trip():
    d = {"servo_id": 3, "min_angle": 10.0, "max_angle": 350.0, "home": 90.0,
         "sign": -1, "offset": 5.0}
    jc = JointConfig.from_dict(d)
    assert jc.servo_id == 3
    assert jc.sign == -1
    assert jc.offset == 5.0
    assert jc.min_angle == 10.0
    assert jc.max_angle == 350.0


def test_joint_config_default_open_close_falls_back_to_limits():
    jc = JointConfig(servo_id=6, min_angle=10.0, max_angle=170.0, is_gripper=True)
    assert jc.open_angle == 170.0
    assert jc.close_angle == 10.0


# ---- angle <-> position transforms -----------------------------------------

def test_angle_to_position_identity_no_sign_no_offset():
    jc = JointConfig(servo_id=1, min_angle=0, max_angle=360)
    # 180 degrees -> midpoint of 0..4095 -> 2048 (rounded)
    assert _angle_to_position(jc, 180.0) == int(round(180.0 * POSITION_MAX / DEGREES_MAX))


def test_angle_to_position_clamps_to_joint_limits():
    jc = JointConfig(servo_id=1, min_angle=10.0, max_angle=170.0, home=90.0)
    # below min clamps up to min
    pos_lo = _angle_to_position(jc, -50.0)
    assert pos_lo == int(round(10.0 * POSITION_MAX / DEGREES_MAX))
    # above max clamps down to max
    pos_hi = _angle_to_position(jc, 999.0)
    assert pos_hi == int(round(170.0 * POSITION_MAX / DEGREES_MAX))


def test_angle_to_position_applies_sign_and_offset():
    # sign=-1, offset=180 means user 0 maps to servo 180; user 90 maps to 90.
    jc = JointConfig(servo_id=1, min_angle=0, max_angle=360, sign=-1, offset=180.0)
    assert _angle_to_position(jc, 0.0) == int(round(180.0 * POSITION_MAX / DEGREES_MAX))
    assert _angle_to_position(jc, 90.0) == int(round(90.0 * POSITION_MAX / DEGREES_MAX))


def test_angle_to_position_wraps_modulo_360():
    # offset that pushes us past 360 should wrap, not blow up.
    jc = JointConfig(servo_id=1, min_angle=0, max_angle=360, offset=400.0)
    # user 0 + offset 400 -> 40 servo degrees after wrap
    assert _angle_to_position(jc, 0.0) == int(round(40.0 * POSITION_MAX / DEGREES_MAX))


def test_position_to_angle_inverse_of_angle_to_position():
    jc = JointConfig(servo_id=1, min_angle=0, max_angle=360, sign=-1, offset=180.0)
    for user_deg in (0.0, 45.0, 90.0, 180.0, 270.0):
        pos = _angle_to_position(jc, user_deg)
        recovered = _position_to_angle(jc, pos)
        # tolerance of ~0.2 deg (one tick at 4096 steps over 360 deg = 0.088)
        assert abs(((recovered - user_deg) + 180) % 360 - 180) < 0.2


# ---- Arm construction -----------------------------------------------------

def test_arm_loads_default_configs(driver):
    arm = Arm(driver)
    assert len(arm) == 6
    names = arm.joint_names
    assert "base" in names and "gripper" in names
    # Order is preserved from the dict
    assert names[0] == "base"
    assert names[-1] == "gripper"


def test_arm_load_from_plain_dict(driver):
    cfgs = {
        "a": {"servo_id": 1},
        "b": {"servo_id": 2, "sign": -1, "offset": 90.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    assert len(arm) == 2
    assert arm.joint("b").sign == -1


def test_arm_accepts_jointconfig_instances(driver):
    cfgs = {"a": JointConfig(servo_id=1), "b": JointConfig(servo_id=2)}
    arm = Arm(driver, joint_configs=cfgs)
    assert arm.joint(0).servo_id == 1
    assert arm.joint("b").servo_id == 2


# ---- single-joint move ----------------------------------------------------

def test_move_joint_writes_expected_packet(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(1))
    arm.move_joint(0, 180.0, speed=1000, accel=50)
    # 180 deg -> 2048 ticks (rounded)
    expected_pos = int(round(180.0 * POSITION_MAX / DEGREES_MAX))
    pkt = bytes(uart.written)
    # WRITE instruction = 0x03 at offset 4
    assert pkt[4] == 0x03
    # Address = REG_GOAL_ACC at offset 5
    assert pkt[5] == REG_GOAL_ACC
    # accel = 50 at offset 6
    assert pkt[6] == 50
    # position LE at offsets 7,8
    pos = pkt[7] | (pkt[8] << 8)
    assert pos == expected_pos
    # speed LE at offsets 11,12 (after 2-byte time placeholder)
    spd = pkt[11] | (pkt[12] << 8)
    assert spd == 1000


def test_move_joint_clamps_silently(driver, uart):
    cfgs = {"j1": {"servo_id": 1, "min_angle": 30.0, "max_angle": 200.0, "home": 100.0}}
    arm = Arm(driver, joint_configs=cfgs)
    uart.feed(_status(1))
    arm.move_joint(0, 999.0)  # way over the max
    pkt = bytes(uart.written)
    pos = pkt[7] | (pkt[8] << 8)
    expected_pos = int(round(200.0 * POSITION_MAX / DEGREES_MAX))
    assert pos == expected_pos


def test_move_joint_uses_default_speed_and_accel(driver, uart):
    cfgs = {"j1": {"servo_id": 1}}
    arm = Arm(driver, joint_configs=cfgs, default_speed=500, default_accel=20)
    uart.feed(_status(1))
    arm.move_joint(0, 90.0)  # no speed/accel kwargs
    pkt = bytes(uart.written)
    assert pkt[6] == 20
    spd = pkt[11] | (pkt[12] << 8)
    assert spd == 500


def test_move_joint_by_name(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(2))
    arm.move_joint("j2", 90.0)
    pkt = bytes(uart.written)
    # ID byte is at offset 2
    assert pkt[2] == 2


# ---- sync-write -----------------------------------------------------------

def test_move_all_emits_single_sync_write_packet(driver, uart):
    """Sync-write must produce exactly ONE packet on the bus regardless of
    how many joints are being moved."""
    arm = _two_joint_arm(driver)
    arm.move_all([180.0, 180.0], speed=1000, accel=50)

    pkt = bytes(uart.written)
    # Exactly one packet: starts with 0xFF 0xFF, the only 0xFF 0xFF in the
    # buffer should be at offset 0.
    assert pkt[:2] == b"\xff\xff"
    assert pkt.count(b"\xff\xff") == 1


def test_move_all_packet_bytes_match_spec(driver, uart):
    """Hand-computed bytes for a 2-joint sync-write at pos=2048, speed=1000, accel=50."""
    arm = _two_joint_arm(driver)
    arm.move_all([180.0, 180.0], speed=1000, accel=50)

    n = 2
    data_len = _SYNC_WRITE_DATA_LEN  # 7
    total_len = (data_len + 1) * n + 4  # 20 = 0x14
    expected_body = bytes([
        _BROADCAST_ID,         # 0xFE
        total_len,             # 0x14
        0x83,                  # SYNC_WRITE
        _SYNC_WRITE_START,     # REG_GOAL_ACC = 0x29
        data_len,              # 0x07
        # joint 1
        0x01, 50, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
        # joint 2
        0x02, 50, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03,
    ])
    expected = b"\xff\xff" + expected_body + bytes((_checksum(expected_body),))
    assert bytes(uart.written) == expected


def test_move_all_dict_only_writes_named_joints(driver, uart):
    arm = _two_joint_arm(driver)
    arm.move_all({"j2": 90.0}, speed=500, accel=10)

    pkt = bytes(uart.written)
    # Packet body is at offset 2 onwards.
    # LEN at offset 3 should equal (7+1)*1 + 4 = 12.
    assert pkt[3] == (7 + 1) * 1 + 4
    # The block after the 7-byte header (offsets 9..16) must have ID 0x02.
    # offset 0:0xFF 1:0xFF 2:0xFE 3:LEN 4:0x83 5:addr 6:dlen 7:ID 8:acc ...
    assert pkt[7] == 0x02


def test_move_all_dict_clamps_too(driver, uart):
    cfgs = {
        "j1": {"servo_id": 1, "min_angle": 0.0, "max_angle": 100.0},
        "j2": {"servo_id": 2, "min_angle": 0.0, "max_angle": 100.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    arm.move_all({"j1": 999.0}, speed=1000, accel=50)
    pkt = bytes(uart.written)
    pos = pkt[9] | (pkt[10] << 8)  # j1's position low/high after id+accel
    expected_pos = int(round(100.0 * POSITION_MAX / DEGREES_MAX))
    assert pos == expected_pos


def test_move_all_list_wrong_length_raises(driver):
    arm = _two_joint_arm(driver)
    with pytest.raises(ValueError, match="expected"):
        arm.move_all([0.0])  # only one angle for a 2-joint arm


def test_move_all_empty_dict_raises(driver):
    arm = _two_joint_arm(driver)
    with pytest.raises(ValueError, match="at least one"):
        arm.move_all({})


def test_move_all_uses_default_speed_when_omitted(driver, uart):
    arm = Arm(
        driver,
        joint_configs={"j1": {"servo_id": 1}},
        default_speed=750, default_accel=15,
    )
    arm.move_all([0.0])  # no speed/accel kwargs
    pkt = bytes(uart.written)
    # Layout: 0xFF 0xFF 0xFE LEN 0x83 START DLEN  ID ACC POS_L POS_H 0 0 SPD_L SPD_H CHK
    assert pkt[8] == 15
    assert (pkt[13] | (pkt[14] << 8)) == 750


def test_sync_write_applies_per_joint_sign_offset(driver, uart):
    cfgs = {
        # j2's sign is flipped — user 0 should map to servo 180, user 90 to servo 90.
        "j1": {"servo_id": 1},
        "j2": {"servo_id": 2, "sign": -1, "offset": 180.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    arm.move_all([0.0, 0.0], speed=100, accel=5)
    pkt = bytes(uart.written)
    # Two-joint sync packet, j1 block starts at offset 7, j2 block at 7+8=15
    # Each block: ID ACC POS_L POS_H TIME_L TIME_H SPD_L SPD_H
    j1_pos = pkt[9] | (pkt[10] << 8)
    j2_pos = pkt[17] | (pkt[18] << 8)
    assert j1_pos == 0
    expected_j2 = int(round(180.0 * POSITION_MAX / DEGREES_MAX))
    assert j2_pos == expected_j2


# ---- read_all -------------------------------------------------------------

def test_read_all_returns_user_facing_angles(driver, uart):
    arm = _two_joint_arm(driver)
    # Queue two read replies: 2048 -> 180 deg, 1024 -> 90 deg
    uart.feed(_status(1, params=b"\x00\x08"))  # 0x0800 = 2048
    uart.feed(_status(2, params=b"\x00\x04"))  # 0x0400 = 1024
    angles = arm.read_all()
    assert len(angles) == 2
    assert abs(angles[0] - 180.0) < 0.2
    assert abs(angles[1] - 90.0) < 0.2


def test_read_all_inverts_sign_and_offset(driver, uart):
    cfgs = {"j1": {"servo_id": 1, "sign": -1, "offset": 180.0}}
    arm = Arm(driver, joint_configs=cfgs)
    # Servo at 180 deg ticks -> user-facing 0 deg under sign=-1, offset=180.
    uart.feed(_status(1, params=b"\x00\x08"))  # 2048 ticks ~ 180 deg
    angles = arm.read_all()
    assert abs(((angles[0] + 180) % 360) - 180) < 0.5  # ~0


# ---- home -----------------------------------------------------------------

def test_home_sends_sync_write_with_home_angles(driver, uart):
    cfgs = {
        "j1": {"servo_id": 1, "min_angle": 0, "max_angle": 360, "home": 180.0},
        "j2": {"servo_id": 2, "min_angle": 0, "max_angle": 360, "home": 90.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    arm.home(speed=500, accel=10)
    pkt = bytes(uart.written)
    # Single sync packet
    assert pkt.count(b"\xff\xff") == 1
    # j1 block @ offset 7, j2 block @ offset 15
    j1_pos = pkt[9] | (pkt[10] << 8)
    j2_pos = pkt[17] | (pkt[18] << 8)
    assert j1_pos == int(round(180.0 * POSITION_MAX / DEGREES_MAX))
    assert j2_pos == int(round(90.0 * POSITION_MAX / DEGREES_MAX))


# ---- torque ---------------------------------------------------------------

def test_enable_torque_all_writes_one_packet_per_joint(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(1))
    uart.feed(_status(2))
    arm.enable_torque("all")
    # Two write packets -> two 0xFF 0xFF headers
    assert bytes(uart.written).count(b"\xff\xff") == 2


def test_disable_torque_all(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(1))
    uart.feed(_status(2))
    arm.disable_torque()  # default = "all"
    assert bytes(uart.written).count(b"\xff\xff") == 2


def test_enable_torque_single_joint_by_index(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(2))
    arm.enable_torque(1)  # second joint by index
    pkt = bytes(uart.written)
    assert pkt[2] == 2  # ID = 2


def test_enable_torque_single_joint_by_name(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(1))
    arm.enable_torque("j1")
    pkt = bytes(uart.written)
    assert pkt[2] == 1


def test_disable_torque_single_joint(driver, uart):
    arm = _two_joint_arm(driver)
    uart.feed(_status(2))
    arm.disable_torque(1)
    assert bytes(uart.written).count(b"\xff\xff") == 1


# ---- gripper helpers ------------------------------------------------------

def test_gripper_open_moves_to_open_angle(driver, uart):
    cfgs = {
        "arm": {"servo_id": 1},
        "grip": {"servo_id": 2, "min_angle": 0.0, "max_angle": 180.0,
                 "is_gripper": True, "open_angle": 170.0, "close_angle": 10.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    uart.feed(_status(2))
    arm.gripper_open(speed=200, accel=5)
    pkt = bytes(uart.written)
    # ID at offset 2 must be the gripper's id
    assert pkt[2] == 2
    pos = pkt[7] | (pkt[8] << 8)
    expected = int(round(170.0 * POSITION_MAX / DEGREES_MAX))
    assert pos == expected


def test_gripper_close_moves_to_close_angle(driver, uart):
    cfgs = {
        "arm": {"servo_id": 1},
        "grip": {"servo_id": 2, "min_angle": 0.0, "max_angle": 180.0,
                 "is_gripper": True, "open_angle": 170.0, "close_angle": 10.0},
    }
    arm = Arm(driver, joint_configs=cfgs)
    uart.feed(_status(2))
    arm.gripper_close()
    pkt = bytes(uart.written)
    assert pkt[2] == 2
    pos = pkt[7] | (pkt[8] << 8)
    expected = int(round(10.0 * POSITION_MAX / DEGREES_MAX))
    assert pos == expected


def test_gripper_methods_raise_when_no_gripper_configured(driver):
    cfgs = {"j1": {"servo_id": 1}}
    arm = Arm(driver, joint_configs=cfgs)
    with pytest.raises(ValueError, match="is_gripper"):
        arm.gripper_open()
    with pytest.raises(ValueError, match="is_gripper"):
        arm.gripper_close()


def test_default_configs_include_gripper(driver, uart):
    arm = Arm(driver)  # uses DEFAULT_JOINT_CONFIGS
    uart.feed(_status(6))
    arm.gripper_open()  # should not raise
    pkt = bytes(uart.written)
    assert pkt[2] == 6  # default gripper ID


# ---- direction-pin sanity for sync-write ----------------------------------

def test_sync_write_uses_direction_pin(driver_with_pin, uart, pin):
    """Sync-write goes through driver._send so the direction pin must toggle."""
    cfgs = {"j1": {"servo_id": 1}, "j2": {"servo_id": 2}}
    arm = Arm(driver_with_pin, joint_configs=cfgs)
    pin.history.clear()
    arm.move_all([0.0, 0.0])
    # TX raises pin high then drops it back low.
    assert 1 in pin.history
    assert pin.history[-1] == 0
