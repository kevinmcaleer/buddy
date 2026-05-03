"""Calibration persistence for the Buddy arm.

Captures the current joint angles as new home positions and joint limits,
then saves them to a JSON file on flash.  On boot, the calibration file
is loaded (if present) and applied to the :class:`~buddy.arm.Arm` joint
configs so the arm picks up where the user left off.

File format (``calibration.json``)
----------------------------------
::

    {
      "joints": {
        "base": {"home": 180.0, "min_angle": 0.0, "max_angle": 360.0},
        "shoulder": {"home": 180.0, "min_angle": 30.0, "max_angle": 330.0},
        ...
      }
    }

Only the fields present in the file are overwritten; missing fields keep
their compiled-in defaults.
"""

try:
    import ujson as json  # MicroPython
except ImportError:       # pragma: no cover - CPython
    import json


DEFAULT_CALIBRATION_PATH = "calibration.json"


class CalibrationError(Exception):
    """Raised on save/load failures."""


def capture(arm):
    """Read current joint angles and return a calibration dict.

    The dict maps each joint name to ``{"home": <current_angle>,
    "min_angle": <cfg.min_angle>, "max_angle": <cfg.max_angle>}``.
    """
    names = arm.joint_names
    angles = arm.read_all()
    data = {}
    for i, name in enumerate(names):
        cfg = arm.joint(i)
        data[name] = {
            "home": angles[i],
            "min_angle": cfg.min_angle,
            "max_angle": cfg.max_angle,
        }
    return {"joints": data}


def save(calibration_data, path=DEFAULT_CALIBRATION_PATH):
    """Persist *calibration_data* to a JSON file.

    Parameters
    ----------
    calibration_data : dict
        As returned by :func:`capture`.
    path : str
        File path on flash.
    """
    try:
        with open(path, "w") as f:
            f.write(json.dumps(calibration_data))
    except OSError as exc:
        raise CalibrationError(
            "failed to write calibration to {!r}: {}".format(path, exc)
        )


def load(path=DEFAULT_CALIBRATION_PATH):
    """Load calibration data from a JSON file.

    Returns
    -------
    dict or None
        The calibration dict, or ``None`` if the file does not exist.

    Raises
    ------
    CalibrationError
        If the file exists but contains invalid JSON or an unexpected
        structure.
    """
    try:
        with open(path, "r") as f:
            raw = f.read()
    except OSError:
        # File doesn't exist -- that's fine on first boot.
        return None
    try:
        data = json.loads(raw)
    except (ValueError, TypeError) as exc:
        raise CalibrationError(
            "calibration file {!r} contains invalid JSON: {}".format(path, exc)
        )
    if not isinstance(data, dict):
        raise CalibrationError(
            "calibration file must contain a JSON object"
        )
    if "joints" not in data or not isinstance(data["joints"], dict):
        raise CalibrationError(
            "calibration file must contain a 'joints' mapping"
        )
    return data


def apply_calibration(arm, calibration_data):
    """Apply loaded calibration to an :class:`~buddy.arm.Arm` instance.

    Updates each joint's ``home``, ``min_angle`` and ``max_angle``
    attributes in-place from the calibration dict.  Only the fields
    present in the file are changed.
    """
    joints_data = calibration_data.get("joints", {})
    for name, overrides in joints_data.items():
        try:
            cfg = arm.joint(name)
        except (KeyError, IndexError):
            # Joint name from file not in current arm config -- skip.
            continue
        if "home" in overrides:
            cfg.home = float(overrides["home"])
        if "min_angle" in overrides:
            cfg.min_angle = float(overrides["min_angle"])
        if "max_angle" in overrides:
            cfg.max_angle = float(overrides["max_angle"])


def calibrate(arm, path=DEFAULT_CALIBRATION_PATH):
    """One-shot convenience: capture current pose and save to flash.

    Returns the captured calibration dict.
    """
    data = capture(arm)
    save(data, path=path)
    return data
