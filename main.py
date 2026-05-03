"""Boot entry point for the Buddy arm on MicroPython.

Execution order
---------------
1. Connect Wi-Fi (STA mode with AP fallback).
2. Initialise the STS3215 driver and Arm.
3. Load calibration from flash (if present) and apply it.
4. Home the arm.
5. Start the web server (with CLI dispatch and IK wired in).
6. Fall back to the CLI REPL if the web server fails.

All steps are wrapped in try/except with user-friendly messages so the
operator sees a clear diagnosis if something goes wrong during boot.
"""

import sys


def main():
    """Top-level boot sequence -- called at module load (bottom of file)."""

    # ---- 1. Wi-Fi -------------------------------------------------------
    print("[buddy] Connecting to Wi-Fi ...")
    try:
        from buddy.web.wifi import connect
        wifi_info = connect()
        print("[buddy] Wi-Fi: mode={}, ssid={}, ip={}".format(
            wifi_info["mode"], wifi_info["ssid"], wifi_info.get("ip", "?")))
    except Exception as exc:
        print("[buddy] Wi-Fi failed: {}".format(exc))
        print("[buddy] Continuing without network -- CLI only.")
        wifi_info = None

    # ---- 2. Driver + Arm ------------------------------------------------
    print("[buddy] Initialising arm ...")
    try:
        from machine import UART, Pin
        from sts3215 import STS3215
        from buddy.arm import Arm

        uart = UART(0, baudrate=1_000_000, tx=Pin(0), rx=Pin(1))
        direction_pin = Pin(2, Pin.OUT)
        driver = STS3215(uart, direction_pin=direction_pin)
        arm = Arm(driver)
    except Exception as exc:
        print("[buddy] Arm init failed: {}".format(exc))
        print("[buddy] Cannot continue without an arm. Dropping to REPL.")
        sys.exit(1)

    # ---- 3. Calibration -------------------------------------------------
    print("[buddy] Loading calibration ...")
    try:
        from buddy.calibration import load as load_cal, apply_calibration
        cal_data = load_cal()
        if cal_data is not None:
            apply_calibration(arm, cal_data)
            print("[buddy] Calibration applied.")
        else:
            print("[buddy] No calibration file found -- using defaults.")
    except Exception as exc:
        print("[buddy] Calibration load failed: {} -- using defaults.".format(exc))

    # ---- 4. Home --------------------------------------------------------
    print("[buddy] Homing arm ...")
    try:
        arm.enable_torque()
        arm.home()
        print("[buddy] Arm homed.")
    except Exception as exc:
        print("[buddy] Homing failed: {}".format(exc))

    # ---- 5. IK helper ---------------------------------------------------
    def _ik_adapter(x, y, z, pitch=0.0, roll=0.0):
        from buddy.kinematics import inverse
        return inverse((x, y, z, pitch, roll))

    # ---- 6. Web server ---------------------------------------------------
    if wifi_info is not None:
        print("[buddy] Starting web server ...")
        try:
            from buddy.cli import dispatch as cli_dispatch
            from buddy.web.server import create_app
            app, _service = create_app(
                arm,
                pose_to_joints=_ik_adapter,
                cli_dispatch=cli_dispatch,
            )
            print("[buddy] Web server starting on {}:80".format(
                wifi_info.get("ip", "0.0.0.0")))
            app.run(host="0.0.0.0", port=80)
        except Exception as exc:
            print("[buddy] Web server failed: {}".format(exc))
            print("[buddy] Falling back to CLI REPL.")

    # ---- 7. CLI REPL fallback -------------------------------------------
    print("[buddy] Starting CLI REPL ...")
    try:
        from buddy.cli import repl
        repl(arm, kinematics=_ik_adapter)
    except Exception as exc:
        print("[buddy] CLI failed: {}".format(exc))
        sys.exit(1)


if __name__ == "__main__":  # pragma: no cover - auto-executed on MicroPython
    main()
