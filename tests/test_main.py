"""Tests for main.py -- boot flow smoke test with all dependencies mocked.

We verify the correct init sequence: Wi-Fi -> Arm -> Calibration -> Home ->
Web server or CLI fallback.  Every hardware dependency is mocked.
"""

import importlib
import sys
import types
from unittest.mock import MagicMock, patch
import pytest


# ---------------------------------------------------------------------------
# Helpers to build the mocked environment
# ---------------------------------------------------------------------------

def _mock_wifi_module():
    """Return a fake ``buddy.web.wifi`` with a connect() that succeeds."""
    mod = types.ModuleType("buddy.web.wifi")
    mod.connect = MagicMock(return_value={
        "mode": "sta",
        "ssid": "test-net",
        "ip": "192.168.1.42",
        "wlan": MagicMock(),
    })
    return mod


def _mock_arm_class():
    """Return a mock Arm class that produces a mock arm instance."""
    arm = MagicMock()
    arm.joint_names = ["base", "shoulder"]
    arm.read_all.return_value = [180.0, 180.0]
    arm.__len__ = lambda self: 2
    arm.joints = [MagicMock(servo_id=1), MagicMock(servo_id=2)]
    return arm


def _mock_calibration_module(cal_data=None):
    mod = types.ModuleType("buddy.calibration")
    mod.load = MagicMock(return_value=cal_data)
    mod.apply_calibration = MagicMock()
    mod.CalibrationError = type("CalibrationError", (Exception,), {})
    return mod


def _fresh_import_main():
    """Force a fresh import of main.py (drop any cached version)."""
    if "main" in sys.modules:
        del sys.modules["main"]
    import main
    return main


# ---------------------------------------------------------------------------
# Test: boot flow with Wi-Fi and web server
# ---------------------------------------------------------------------------

class TestBootFlow:
    def test_wifi_connect_called_first(self):
        """Verify that Wi-Fi connect is attempted."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        cal_mod = _mock_calibration_module()

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        mock_service = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, mock_service))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()

        # app.run raises to exit the blocking loop, so we fall to CLI
        mock_app.run.side_effect = RuntimeError("exit web loop")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                wifi_mod.connect.assert_called()

    def test_calibration_applied_when_present(self):
        """Verify calibration is loaded and applied when a file exists."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        cal_data = {"joints": {"base": {"home": 90.0}}}
        cal_mod = _mock_calibration_module(cal_data=cal_data)

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, MagicMock()))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()
        mock_app.run.side_effect = RuntimeError("exit")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                cal_mod.load.assert_called()
                cal_mod.apply_calibration.assert_called()

    def test_no_calibration_file_continues(self):
        """Boot proceeds normally if calibration.json doesn't exist."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        cal_mod = _mock_calibration_module(cal_data=None)

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, MagicMock()))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()
        mock_app.run.side_effect = RuntimeError("exit")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                cal_mod.apply_calibration.assert_not_called()

    def test_wifi_failure_falls_back_to_cli(self):
        """If Wi-Fi fails, skip web server, run CLI."""
        wifi_mod = types.ModuleType("buddy.web.wifi")
        wifi_mod.connect = MagicMock(side_effect=RuntimeError("no wifi"))

        arm = _mock_arm_class()
        cal_mod = _mock_calibration_module()

        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                # CLI repl should be called since web is skipped.
                cli_mod.repl.assert_called()

    def test_home_called_after_calibration(self):
        """Verify the arm is homed after calibration is applied."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        cal_mod = _mock_calibration_module()

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, MagicMock()))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()
        mock_app.run.side_effect = RuntimeError("exit")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                arm.enable_torque.assert_called()
                arm.home.assert_called()

    def test_calibration_load_error_continues(self):
        """If calibration load raises, boot continues with defaults."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        cal_mod = types.ModuleType("buddy.calibration")
        cal_mod.load = MagicMock(side_effect=RuntimeError("corrupt"))
        cal_mod.apply_calibration = MagicMock()

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, MagicMock()))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()
        mock_app.run.side_effect = RuntimeError("exit")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                # Should still home despite calibration failure.
                arm.home.assert_called()
                cal_mod.apply_calibration.assert_not_called()

    def test_homing_failure_continues(self):
        """If homing raises, boot continues to web/cli."""
        wifi_mod = _mock_wifi_module()
        arm = _mock_arm_class()
        arm.home.side_effect = RuntimeError("servo timeout")
        cal_mod = _mock_calibration_module()

        server_mod = types.ModuleType("buddy.web.server")
        mock_app = MagicMock()
        server_mod.create_app = MagicMock(return_value=(mock_app, MagicMock()))
        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock()
        mock_app.run.side_effect = RuntimeError("exit")

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.web.server": server_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                mod.main()
                # CLI should still be called.
                cli_mod.repl.assert_called()

    def test_cli_failure_exits(self):
        """If CLI repl raises, boot exits with code 1."""
        wifi_mod = types.ModuleType("buddy.web.wifi")
        wifi_mod.connect = MagicMock(side_effect=RuntimeError("no wifi"))
        arm = _mock_arm_class()
        cal_mod = _mock_calibration_module()

        cli_mod = types.ModuleType("buddy.cli")
        cli_mod.dispatch = MagicMock()
        cli_mod.repl = MagicMock(side_effect=RuntimeError("repl died"))

        with patch.dict(sys.modules, {
            "buddy.web.wifi": wifi_mod,
            "buddy.web": types.ModuleType("buddy.web"),
            "buddy.calibration": cal_mod,
            "buddy.cli": cli_mod,
        }):
            with patch("buddy.arm.Arm", return_value=arm):
                mod = _fresh_import_main()
                with pytest.raises(SystemExit) as exc_info:
                    mod.main()
                assert exc_info.value.code == 1
