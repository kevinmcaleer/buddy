"""Tests for :mod:`buddy.cli` -- the text command parser.

Every command is exercised via :func:`buddy.cli.dispatch` against a mock
:class:`Arm` (and, where needed, a mock IK callable).  The test suite
targets >= 80 % line coverage on ``buddy/cli.py``.
"""
from collections import OrderedDict

import pytest

from buddy.cli import dispatch, _HELP_TEXT


# ---------------------------------------------------------------------------
# Fake Arm (mirrors the FakeArm from test_web_server.py)
# ---------------------------------------------------------------------------

class FakeJointConfig:
    def __init__(self, servo_id, min_angle=0.0, max_angle=360.0, is_gripper=False):
        self.servo_id = servo_id
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.is_gripper = is_gripper
        self.home = 180.0


class FakeDriver:
    def __init__(self):
        self.temps = {}

    def read_temperature(self, servo_id):
        return self.temps.get(servo_id, 25)


class FakeArm:
    """Records every method call for later assertion."""

    def __init__(self, names=("base", "shoulder", "elbow", "wrist", "wrist_rot", "gripper")):
        self._names = list(names)
        self._joints = OrderedDict()
        for i, n in enumerate(names):
            self._joints[n] = FakeJointConfig(
                servo_id=i + 1,
                is_gripper=(n == "gripper"),
            )
        self.next_read = [10.0 * (i + 1) for i in range(len(names))]
        self.calls = []
        self._driver = FakeDriver()

    @property
    def joint_names(self):
        return list(self._names)

    def joint(self, key):
        if isinstance(key, str):
            return self._joints[key]
        return list(self._joints.values())[key]

    def __len__(self):
        return len(self._joints)

    def read_all(self):
        self.calls.append(("read_all",))
        return list(self.next_read)

    def move_joint(self, key, angle_deg, speed=None, accel=None):
        self.calls.append(("move_joint", key, angle_deg, speed, accel))

    def move_all(self, angles, speed=None, accel=None):
        self.calls.append(("move_all", angles, speed, accel))

    def enable_torque(self, key="all"):
        self.calls.append(("enable_torque", key))

    def disable_torque(self, key="all"):
        self.calls.append(("disable_torque", key))

    def gripper_open(self, speed=None, accel=None):
        self.calls.append(("gripper_open",))

    def gripper_close(self, speed=None, accel=None):
        self.calls.append(("gripper_close",))

    def home(self, speed=None, accel=None):
        self.calls.append(("home",))


@pytest.fixture
def arm():
    return FakeArm()


# ---------------------------------------------------------------------------
# help
# ---------------------------------------------------------------------------

def test_help_lists_all_commands(arm):
    result = dispatch("help", arm)
    assert "move" in result
    assert "pose" in result
    assert "home" in result
    assert "torque" in result
    assert "grip" in result
    assert "read" in result
    assert "help" in result


def test_help_returns_full_help_text(arm):
    assert dispatch("help", arm) == _HELP_TEXT


# ---------------------------------------------------------------------------
# move
# ---------------------------------------------------------------------------

def test_move_valid_joint(arm):
    result = dispatch("move J1 90", arm)
    assert "OK" in result
    assert "J1" in result
    assert "90.0" in result
    assert ("move_joint", 0, 90.0, None, None) in arm.calls


def test_move_joint_case_insensitive(arm):
    result = dispatch("move j3 45.5", arm)
    assert "OK" in result
    assert ("move_joint", 2, 45.5, None, None) in arm.calls


def test_move_last_joint(arm):
    result = dispatch("move J6 100", arm)
    assert "OK" in result
    assert ("move_joint", 5, 100.0, None, None) in arm.calls


def test_move_missing_args(arm):
    result = dispatch("move J1", arm)
    assert "Error" in result
    assert "usage" in result


def test_move_too_many_args(arm):
    result = dispatch("move J1 90 extra", arm)
    assert "Error" in result


def test_move_invalid_joint_format(arm):
    result = dispatch("move X1 90", arm)
    assert "Error" in result
    assert "J<n>" in result


def test_move_invalid_joint_number(arm):
    result = dispatch("move Jabc 90", arm)
    assert "Error" in result
    assert "invalid joint number" in result


def test_move_joint_out_of_range_zero(arm):
    result = dispatch("move J0 90", arm)
    assert "Error" in result
    assert "between 1 and" in result


def test_move_joint_out_of_range_high(arm):
    result = dispatch("move J99 90", arm)
    assert "Error" in result
    assert "between 1 and" in result


def test_move_invalid_angle(arm):
    result = dispatch("move J1 abc", arm)
    assert "Error" in result
    assert "invalid angle" in result


def test_move_arm_raises(arm):
    def boom(*a, **kw):
        raise RuntimeError("bus error")
    arm.move_joint = boom
    result = dispatch("move J1 90", arm)
    assert "Error" in result
    assert "bus error" in result


# ---------------------------------------------------------------------------
# pose
# ---------------------------------------------------------------------------

def test_pose_xyz_only(arm):
    def fake_ik(x, y, z, pitch, roll):
        return [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]

    result = dispatch("pose 100 200 300", arm, kinematics=fake_ik)
    assert "OK" in result
    assert "100.0" in result
    assert ("move_all", [10.0, 20.0, 30.0, 40.0, 50.0, 60.0], None, None) in arm.calls


def test_pose_with_pitch(arm):
    def fake_ik(x, y, z, pitch, roll):
        assert pitch == 45.0
        assert roll == 0.0
        return [0.0] * 6

    result = dispatch("pose 10 20 30 45", arm, kinematics=fake_ik)
    assert "OK" in result


def test_pose_with_pitch_and_roll(arm):
    def fake_ik(x, y, z, pitch, roll):
        assert pitch == 45.0
        assert roll == 90.0
        return [0.0] * 6

    result = dispatch("pose 10 20 30 45 90", arm, kinematics=fake_ik)
    assert "OK" in result
    assert "45.0" in result
    assert "90.0" in result


def test_pose_no_kinematics(arm):
    result = dispatch("pose 10 20 30", arm, kinematics=None)
    assert "Error" in result
    assert "kinematics not available" in result


def test_pose_too_few_args(arm):
    result = dispatch("pose 10 20", arm, kinematics=lambda *a: [0] * 6)
    assert "Error" in result
    assert "usage" in result


def test_pose_too_many_args(arm):
    result = dispatch("pose 10 20 30 40 50 60", arm, kinematics=lambda *a: [0] * 6)
    assert "Error" in result
    assert "usage" in result


def test_pose_non_numeric_args(arm):
    result = dispatch("pose abc 20 30", arm, kinematics=lambda *a: [0] * 6)
    assert "Error" in result
    assert "numbers" in result


def test_pose_ik_raises(arm):
    def bad_ik(x, y, z, pitch, roll):
        raise ValueError("unreachable target")

    result = dispatch("pose 10 20 30", arm, kinematics=bad_ik)
    assert "Error" in result
    assert "unreachable target" in result


def test_pose_move_all_raises(arm):
    def ok_ik(x, y, z, pitch, roll):
        return [0.0] * 6

    def bad_move(*a, **kw):
        raise RuntimeError("bus error")

    arm.move_all = bad_move
    result = dispatch("pose 10 20 30", arm, kinematics=ok_ik)
    assert "Error" in result
    assert "bus error" in result


# ---------------------------------------------------------------------------
# home
# ---------------------------------------------------------------------------

def test_home(arm):
    result = dispatch("home", arm)
    assert "OK" in result
    assert "home" in result
    assert ("home",) in arm.calls


def test_home_raises(arm):
    arm.home = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("stuck"))
    result = dispatch("home", arm)
    assert "Error" in result
    assert "stuck" in result


# ---------------------------------------------------------------------------
# torque
# ---------------------------------------------------------------------------

def test_torque_on_all(arm):
    result = dispatch("torque on", arm)
    assert "OK" in result
    assert "torque on" in result
    assert "all joints" in result
    assert ("enable_torque", "all") in arm.calls


def test_torque_off_all(arm):
    result = dispatch("torque off", arm)
    assert "OK" in result
    assert "torque off" in result
    assert ("disable_torque", "all") in arm.calls


def test_torque_on_specific_joint(arm):
    result = dispatch("torque on J2", arm)
    assert "OK" in result
    assert "J2" in result
    assert ("enable_torque", 1) in arm.calls


def test_torque_off_specific_joint(arm):
    result = dispatch("torque off J3", arm)
    assert "OK" in result
    assert ("disable_torque", 2) in arm.calls


def test_torque_case_insensitive(arm):
    result = dispatch("torque ON j1", arm)
    assert "OK" in result
    assert ("enable_torque", 0) in arm.calls


def test_torque_missing_action(arm):
    result = dispatch("torque", arm)
    assert "Error" in result
    assert "usage" in result


def test_torque_invalid_action(arm):
    result = dispatch("torque maybe", arm)
    assert "Error" in result
    assert "'on' or 'off'" in result


def test_torque_invalid_joint_format(arm):
    result = dispatch("torque on X1", arm)
    assert "Error" in result
    assert "J<n>" in result


def test_torque_invalid_joint_number(arm):
    result = dispatch("torque on Jabc", arm)
    assert "Error" in result
    assert "invalid joint number" in result


def test_torque_joint_out_of_range(arm):
    result = dispatch("torque on J99", arm)
    assert "Error" in result
    assert "between 1 and" in result


def test_torque_raises(arm):
    arm.enable_torque = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("nope"))
    result = dispatch("torque on", arm)
    assert "Error" in result
    assert "nope" in result


# ---------------------------------------------------------------------------
# grip
# ---------------------------------------------------------------------------

def test_grip_open(arm):
    result = dispatch("grip open", arm)
    assert "OK" in result
    assert "opened" in result
    assert ("gripper_open",) in arm.calls


def test_grip_close(arm):
    result = dispatch("grip close", arm)
    assert "OK" in result
    assert "closed" in result
    assert ("gripper_close",) in arm.calls


def test_grip_invalid_action(arm):
    result = dispatch("grip wiggle", arm)
    assert "Error" in result
    assert "'open' or 'close'" in result


def test_grip_missing_action(arm):
    result = dispatch("grip", arm)
    assert "Error" in result
    assert "usage" in result


def test_grip_too_many_args(arm):
    result = dispatch("grip open close", arm)
    assert "Error" in result


def test_grip_open_raises(arm):
    arm.gripper_open = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("jam"))
    result = dispatch("grip open", arm)
    assert "Error" in result
    assert "jam" in result


def test_grip_close_raises(arm):
    arm.gripper_close = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("jam"))
    result = dispatch("grip close", arm)
    assert "Error" in result
    assert "jam" in result


# ---------------------------------------------------------------------------
# read
# ---------------------------------------------------------------------------

def test_read_shows_all_joints(arm):
    result = dispatch("read", arm)
    assert "Joint states:" in result
    # One line per joint (6 joints)
    for i, name in enumerate(arm.joint_names):
        assert "J{}".format(i + 1) in result
        assert name in result
    assert "deg" in result
    assert "temp" in result


def test_read_shows_temperatures(arm):
    arm._driver.temps = {1: 30, 2: 35}
    result = dispatch("read", arm)
    assert "30 C" in result
    assert "25 C" in result  # default for others


def test_read_without_driver(arm):
    arm._driver = None
    result = dispatch("read", arm)
    assert "N/A" in result  # temp is N/A
    assert "Joint states:" in result


def test_read_without_read_temperature(arm):
    arm._driver = object()  # no read_temperature
    result = dispatch("read", arm)
    assert "N/A" in result


def test_read_raises(arm):
    def boom():
        raise RuntimeError("bus timeout")
    arm.read_all = boom
    result = dispatch("read", arm)
    assert "Error" in result
    assert "bus timeout" in result


# ---------------------------------------------------------------------------
# Unknown / empty commands
# ---------------------------------------------------------------------------

def test_unknown_command(arm):
    result = dispatch("dance", arm)
    assert "Unknown command" in result
    assert "dance" in result
    assert "help" in result


def test_empty_string(arm):
    result = dispatch("", arm)
    assert result == ""


def test_whitespace_only(arm):
    result = dispatch("   ", arm)
    assert result == ""


def test_command_with_extra_whitespace(arm):
    result = dispatch("  home  ", arm)
    assert "OK" in result


# ---------------------------------------------------------------------------
# dispatch preserves original case in unknown-command messages
# ---------------------------------------------------------------------------

def test_unknown_preserves_original_token(arm):
    result = dispatch("Dance", arm)
    assert "Dance" in result
