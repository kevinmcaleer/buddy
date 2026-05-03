"""Tests for buddy.safety -- soft-stop on over-temp, over-load,
configurable thresholds, and warning callbacks."""

import pytest

from buddy.arm import Arm
from buddy.safety import (
    DEFAULT_LOAD_LIMIT,
    DEFAULT_TEMP_LIMIT,
    DEFAULT_WARN_FRACTION,
    SafetyMonitor,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _FakeDriver:
    """Driver stub with configurable per-servo temperature and load."""

    def __init__(self, temps=None, loads=None):
        self.temps = temps or {}   # servo_id -> temp (int)
        self.loads = loads or {}   # servo_id -> load (int, signed)
        self.torque_disabled = []  # servo_ids that got disable_torque

    def read_temperature(self, servo_id):
        if servo_id in self.temps:
            return self.temps[servo_id]
        return 25  # ambient default

    def read_load(self, servo_id):
        if servo_id in self.loads:
            return self.loads[servo_id]
        return 0

    def write_position(self, servo_id, position, speed=0, accel=0):
        pass

    def enable_torque(self, servo_id):
        pass

    def disable_torque(self, servo_id):
        self.torque_disabled.append(servo_id)

    def _send(self, packet):
        pass


def _make_arm_and_driver(temps=None, loads=None):
    driver = _FakeDriver(temps=temps, loads=loads)
    configs = {
        "j1": {"servo_id": 1, "min_angle": 0, "max_angle": 360, "home": 180},
        "j2": {"servo_id": 2, "min_angle": 0, "max_angle": 360, "home": 180},
    }
    arm = Arm(driver, joint_configs=configs)
    return arm, driver


# ---------------------------------------------------------------------------
# Normal operation (no trip)
# ---------------------------------------------------------------------------

class TestNormalOperation:
    def test_check_returns_true_when_safe(self):
        arm, driver = _make_arm_and_driver(temps={1: 30, 2: 35})
        sm = SafetyMonitor(arm, driver)
        assert sm.check() is True
        assert not sm.tripped

    def test_check_returns_true_multiple_times(self):
        arm, driver = _make_arm_and_driver()
        sm = SafetyMonitor(arm, driver)
        for _ in range(10):
            assert sm.check() is True


# ---------------------------------------------------------------------------
# Temperature soft-stop
# ---------------------------------------------------------------------------

class TestTemperatureTrip:
    def test_trips_on_over_temp(self):
        arm, driver = _make_arm_and_driver(temps={1: 25, 2: 75})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        assert sm.check() is False
        assert sm.tripped
        assert "temperature" in sm.trip_reason
        assert "2" in sm.trip_reason

    def test_disables_torque_on_trip(self):
        arm, driver = _make_arm_and_driver(temps={1: 80})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        sm.check()
        # Both joints should have torque disabled.
        assert 1 in driver.torque_disabled
        assert 2 in driver.torque_disabled

    def test_exact_limit_trips(self):
        arm, driver = _make_arm_and_driver(temps={1: 70})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        assert sm.check() is False

    def test_below_limit_safe(self):
        arm, driver = _make_arm_and_driver(temps={1: 69})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        assert sm.check() is True


# ---------------------------------------------------------------------------
# Load soft-stop
# ---------------------------------------------------------------------------

class TestLoadTrip:
    def test_trips_on_over_load(self):
        arm, driver = _make_arm_and_driver(loads={2: 950})
        sm = SafetyMonitor(arm, driver, load_limit=900)
        assert sm.check() is False
        assert sm.tripped
        assert "load" in sm.trip_reason

    def test_trips_on_negative_load(self):
        arm, driver = _make_arm_and_driver(loads={1: -950})
        sm = SafetyMonitor(arm, driver, load_limit=900)
        assert sm.check() is False

    def test_below_load_limit_safe(self):
        arm, driver = _make_arm_and_driver(loads={1: 800})
        sm = SafetyMonitor(arm, driver, load_limit=900)
        assert sm.check() is True


# ---------------------------------------------------------------------------
# Warning callback
# ---------------------------------------------------------------------------

class TestWarning:
    def test_temp_warning_fires(self):
        warnings = []
        arm, driver = _make_arm_and_driver(temps={1: 58})
        # 70 * 0.8 = 56, so 58 should warn.
        sm = SafetyMonitor(arm, driver, temp_limit=70, warn_fraction=0.8,
                           on_warn=warnings.append)
        assert sm.check() is True  # still safe
        assert len(warnings) >= 1
        assert "temperature" in warnings[0]

    def test_load_warning_fires(self):
        warnings = []
        arm, driver = _make_arm_and_driver(loads={2: 750})
        # 900 * 0.8 = 720, so 750 should warn.
        sm = SafetyMonitor(arm, driver, load_limit=900, warn_fraction=0.8,
                           on_warn=warnings.append)
        assert sm.check() is True
        assert any("load" in w for w in warnings)

    def test_no_warning_below_threshold(self):
        warnings = []
        arm, driver = _make_arm_and_driver(temps={1: 30}, loads={1: 100})
        sm = SafetyMonitor(arm, driver, temp_limit=70, load_limit=900,
                           warn_fraction=0.8, on_warn=warnings.append)
        sm.check()
        assert len(warnings) == 0

    def test_no_callback_set(self):
        arm, driver = _make_arm_and_driver(temps={1: 58})
        sm = SafetyMonitor(arm, driver, temp_limit=70, warn_fraction=0.8)
        # Should not raise even though warning threshold is exceeded.
        assert sm.check() is True


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_trip(self):
        arm, driver = _make_arm_and_driver(temps={1: 80})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        sm.check()
        assert sm.tripped
        sm.reset()
        assert not sm.tripped
        assert sm.trip_reason == ""

    def test_check_returns_false_while_tripped(self):
        arm, driver = _make_arm_and_driver(temps={1: 80})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        sm.check()
        # Subsequent checks should still return False.
        assert sm.check() is False

    def test_check_works_after_reset_with_safe_readings(self):
        arm, driver = _make_arm_and_driver(temps={1: 80})
        sm = SafetyMonitor(arm, driver, temp_limit=70)
        sm.check()
        sm.reset()
        # Now put temperature back to safe.
        driver.temps[1] = 30
        assert sm.check() is True


# ---------------------------------------------------------------------------
# Configurable thresholds
# ---------------------------------------------------------------------------

class TestConfigurableThresholds:
    def test_custom_temp_limit(self):
        arm, driver = _make_arm_and_driver(temps={1: 55})
        sm = SafetyMonitor(arm, driver, temp_limit=50)
        assert sm.check() is False
        assert sm.tripped

    def test_custom_load_limit(self):
        arm, driver = _make_arm_and_driver(loads={1: 200})
        sm = SafetyMonitor(arm, driver, load_limit=150)
        assert sm.check() is False
        assert sm.tripped

    def test_threshold_properties(self):
        arm, driver = _make_arm_and_driver()
        sm = SafetyMonitor(arm, driver)
        assert sm.temp_limit == DEFAULT_TEMP_LIMIT
        assert sm.load_limit == DEFAULT_LOAD_LIMIT
        assert sm.warn_fraction == DEFAULT_WARN_FRACTION

    def test_threshold_setters(self):
        arm, driver = _make_arm_and_driver()
        sm = SafetyMonitor(arm, driver)
        sm.temp_limit = 80
        sm.load_limit = 500
        sm.warn_fraction = 0.9
        assert sm.temp_limit == 80
        assert sm.load_limit == 500
        assert sm.warn_fraction == 0.9


# ---------------------------------------------------------------------------
# Exception handling in read_temperature / read_load
# ---------------------------------------------------------------------------

class TestReadExceptions:
    def test_temp_read_exception_treated_as_none(self):
        arm, driver = _make_arm_and_driver()

        def _fail(sid):
            raise RuntimeError("bus error")

        driver.read_temperature = _fail
        sm = SafetyMonitor(arm, driver)
        # Should not trip -- read failure is treated as unavailable data.
        assert sm.check() is True

    def test_load_read_exception_treated_as_none(self):
        arm, driver = _make_arm_and_driver()

        def _fail(sid):
            raise RuntimeError("bus error")

        driver.read_load = _fail
        sm = SafetyMonitor(arm, driver)
        assert sm.check() is True
