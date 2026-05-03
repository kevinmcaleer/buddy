"""Tests for buddy.calibration -- save/load round-trip, missing file,
corrupt file, and apply_calibration."""

import json
import os
import pytest

from buddy.arm import Arm, JointConfig
from buddy.calibration import (
    CalibrationError,
    apply_calibration,
    calibrate,
    capture,
    load,
    save,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _FakeDriver:
    """Minimal driver stub that records writes and returns canned positions."""

    def __init__(self, positions=None):
        # positions: dict of servo_id -> raw position (0..4095)
        self._positions = positions or {}

    def read_position(self, servo_id):
        return self._positions.get(servo_id, 2048)

    def write_position(self, servo_id, position, speed=0, accel=0):
        pass

    def enable_torque(self, servo_id):
        pass

    def disable_torque(self, servo_id):
        pass

    def _send(self, packet):
        pass


def _make_arm(positions=None):
    driver = _FakeDriver(positions=positions)
    configs = {
        "base":     {"servo_id": 1, "min_angle":   0.0, "max_angle": 360.0, "home": 180.0},
        "shoulder": {"servo_id": 2, "min_angle":  30.0, "max_angle": 330.0, "home": 180.0},
    }
    return Arm(driver, joint_configs=configs)


# ---------------------------------------------------------------------------
# capture
# ---------------------------------------------------------------------------

class TestCapture:
    def test_returns_dict_with_joint_data(self):
        arm = _make_arm(positions={1: 2048, 2: 1024})
        data = capture(arm)
        assert "joints" in data
        assert "base" in data["joints"]
        assert "shoulder" in data["joints"]
        # Each joint entry has home, min_angle, max_angle.
        base = data["joints"]["base"]
        assert "home" in base
        assert "min_angle" in base
        assert "max_angle" in base

    def test_captures_current_angles_as_home(self):
        arm = _make_arm(positions={1: 2048, 2: 1024})
        data = capture(arm)
        # Position 2048 is ~180 deg, 1024 is ~90 deg.
        assert abs(data["joints"]["base"]["home"] - 180.0) < 1.0
        assert abs(data["joints"]["shoulder"]["home"] - 90.0) < 1.0


# ---------------------------------------------------------------------------
# save / load round-trip
# ---------------------------------------------------------------------------

class TestSaveLoad:
    def test_round_trip(self, tmp_path):
        path = str(tmp_path / "cal.json")
        arm = _make_arm(positions={1: 2048, 2: 1024})
        original = capture(arm)
        save(original, path=path)
        loaded = load(path=path)
        assert loaded is not None
        assert loaded["joints"]["base"]["home"] == original["joints"]["base"]["home"]
        assert loaded["joints"]["shoulder"]["home"] == original["joints"]["shoulder"]["home"]

    def test_calibrate_convenience(self, tmp_path):
        path = str(tmp_path / "cal.json")
        arm = _make_arm(positions={1: 2048, 2: 1024})
        data = calibrate(arm, path=path)
        assert "joints" in data
        loaded = load(path=path)
        assert loaded == data

    def test_save_creates_file(self, tmp_path):
        path = str(tmp_path / "cal.json")
        save({"joints": {}}, path=path)
        assert os.path.exists(path)


# ---------------------------------------------------------------------------
# load edge cases
# ---------------------------------------------------------------------------

class TestLoadEdgeCases:
    def test_missing_file_returns_none(self, tmp_path):
        path = str(tmp_path / "nonexistent.json")
        assert load(path=path) is None

    def test_corrupt_json_raises(self, tmp_path):
        path = str(tmp_path / "bad.json")
        with open(path, "w") as f:
            f.write("{not valid json!!!")
        with pytest.raises(CalibrationError, match="invalid JSON"):
            load(path=path)

    def test_non_object_raises(self, tmp_path):
        path = str(tmp_path / "array.json")
        with open(path, "w") as f:
            f.write("[1, 2, 3]")
        with pytest.raises(CalibrationError, match="JSON object"):
            load(path=path)

    def test_missing_joints_key_raises(self, tmp_path):
        path = str(tmp_path / "no_joints.json")
        with open(path, "w") as f:
            f.write('{"foo": "bar"}')
        with pytest.raises(CalibrationError, match="joints"):
            load(path=path)

    def test_joints_not_dict_raises(self, tmp_path):
        path = str(tmp_path / "bad_joints.json")
        with open(path, "w") as f:
            f.write('{"joints": [1, 2]}')
        with pytest.raises(CalibrationError, match="joints"):
            load(path=path)


# ---------------------------------------------------------------------------
# apply_calibration
# ---------------------------------------------------------------------------

class TestApplyCalibration:
    def test_updates_home(self):
        arm = _make_arm()
        cal = {"joints": {"base": {"home": 42.0}}}
        apply_calibration(arm, cal)
        assert arm.joint("base").home == 42.0

    def test_updates_min_max(self):
        arm = _make_arm()
        cal = {"joints": {"shoulder": {"min_angle": 10.0, "max_angle": 300.0}}}
        apply_calibration(arm, cal)
        assert arm.joint("shoulder").min_angle == 10.0
        assert arm.joint("shoulder").max_angle == 300.0

    def test_unknown_joint_ignored(self):
        arm = _make_arm()
        cal = {"joints": {"nonexistent": {"home": 99.0}}}
        # Should not raise.
        apply_calibration(arm, cal)

    def test_partial_update_preserves_others(self):
        arm = _make_arm()
        original_min = arm.joint("base").min_angle
        cal = {"joints": {"base": {"home": 55.0}}}
        apply_calibration(arm, cal)
        assert arm.joint("base").home == 55.0
        assert arm.joint("base").min_angle == original_min

    def test_empty_joints_dict_is_noop(self):
        arm = _make_arm()
        original_home = arm.joint("base").home
        apply_calibration(arm, {"joints": {}})
        assert arm.joint("base").home == original_home


# ---------------------------------------------------------------------------
# save error
# ---------------------------------------------------------------------------

class TestSaveError:
    def test_save_to_invalid_path_raises(self):
        with pytest.raises(CalibrationError, match="failed to write"):
            save({"joints": {}}, path="/nonexistent_dir_abc123/cal.json")
