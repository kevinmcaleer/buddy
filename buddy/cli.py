"""Text command interface for the Buddy arm.

Provides a small command parser usable from:

* the MicroPython REPL over USB (via :func:`repl`), and
* the web console (via the ``POST /cli`` endpoint in
  :mod:`buddy.web.server`).

Both paths share a single :func:`dispatch` entry point so that every
command is parsed identically regardless of transport.

Commands
--------
=================================  ==========================================
``move J<n> <deg>``                Move joint *n* (1-based) to *deg* degrees.
``pose <x> <y> <z> [pitch roll]`` IK move; pitch/roll default to 0.
``home``                           Drive all joints to their home position.
``torque on|off [J<n>]``           Enable/disable torque (all if no joint).
``grip open|close``                Open or close the gripper.
``read``                           Print current joint angles, positions,
                                   temperatures.
``help``                           List all commands with brief descriptions.
=================================  ==========================================

MicroPython constraints
-----------------------
* No ``argparse`` -- commands are parsed manually with ``str.split()``.
* ``sys.stdin`` for REPL input (available in MicroPython).
* Error messages are user-friendly strings, not tracebacks.
"""

try:
    import sys
except ImportError:  # pragma: no cover
    import sys


# ---------------------------------------------------------------------------
# Help text
# ---------------------------------------------------------------------------

_HELP_TEXT = """\
Buddy CLI commands:
  move J<n> <deg>               Move joint n (1-based) to <deg> degrees
  pose <x> <y> <z> [pitch roll] IK move (pitch/roll default to 0)
  home                          Move all joints to home position
  torque on|off [J<n>]          Enable/disable torque (all if no joint)
  grip open|close               Open or close the gripper
  read                          Print current joint angles and temperatures
  help                          Show this help message"""


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def _cmd_move(parts, arm, _kin):
    """``move J<n> <deg>``"""
    if len(parts) != 3:
        return "Error: usage: move J<n> <degrees>"
    joint_tok = parts[1]
    if not joint_tok.upper().startswith("J"):
        return "Error: joint must be specified as J<n> (e.g. J1)"
    try:
        joint_num = int(joint_tok[1:])
    except ValueError:
        return "Error: invalid joint number: {}".format(joint_tok)
    if joint_num < 1 or joint_num > len(arm):
        return "Error: joint number must be between 1 and {}".format(len(arm))
    try:
        deg = float(parts[2])
    except ValueError:
        return "Error: invalid angle: {}".format(parts[2])
    try:
        arm.move_joint(joint_num - 1, deg)
    except Exception as exc:
        return "Error: {}".format(exc)
    return "OK: J{} moved to {:.1f} deg".format(joint_num, deg)


def _cmd_pose(parts, arm, kin):
    """``pose <x> <y> <z> [pitch roll]``"""
    if kin is None:
        return "Error: kinematics not available"
    if len(parts) < 4:
        return "Error: usage: pose <x> <y> <z> [pitch roll]"
    if len(parts) > 6:
        return "Error: usage: pose <x> <y> <z> [pitch roll]"
    try:
        coords = [float(v) for v in parts[1:]]
    except ValueError:
        return "Error: all pose values must be numbers"
    # Pad pitch/roll with 0 if not provided.
    while len(coords) < 5:
        coords.append(0.0)
    x, y, z, pitch, roll = coords
    try:
        angles = kin(x, y, z, pitch, roll)
    except Exception as exc:
        return "Error: {}".format(exc)
    try:
        arm.move_all(angles)
    except Exception as exc:
        return "Error: {}".format(exc)
    return "OK: moved to pose ({:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f})".format(
        x, y, z, pitch, roll
    )


def _cmd_home(_parts, arm, _kin):
    """``home``"""
    try:
        arm.home()
    except Exception as exc:
        return "Error: {}".format(exc)
    return "OK: moved to home position"


def _cmd_torque(parts, arm, _kin):
    """``torque on|off [J<n>]``"""
    if len(parts) < 2:
        return "Error: usage: torque on|off [J<n>]"
    action = parts[1].lower()
    if action not in ("on", "off"):
        return "Error: torque action must be 'on' or 'off'"
    key = "all"
    if len(parts) >= 3:
        joint_tok = parts[2]
        if not joint_tok.upper().startswith("J"):
            return "Error: joint must be specified as J<n> (e.g. J1)"
        try:
            joint_num = int(joint_tok[1:])
        except ValueError:
            return "Error: invalid joint number: {}".format(joint_tok)
        if joint_num < 1 or joint_num > len(arm):
            return "Error: joint number must be between 1 and {}".format(len(arm))
        key = joint_num - 1
    try:
        if action == "on":
            arm.enable_torque(key)
        else:
            arm.disable_torque(key)
    except Exception as exc:
        return "Error: {}".format(exc)
    target = "all joints" if key == "all" else "J{}".format(key + 1)
    return "OK: torque {} for {}".format(action, target)


def _cmd_grip(parts, arm, _kin):
    """``grip open|close``"""
    if len(parts) != 2:
        return "Error: usage: grip open|close"
    action = parts[1].lower()
    if action == "open":
        try:
            arm.gripper_open()
        except Exception as exc:
            return "Error: {}".format(exc)
        return "OK: gripper opened"
    elif action == "close":
        try:
            arm.gripper_close()
        except Exception as exc:
            return "Error: {}".format(exc)
        return "OK: gripper closed"
    else:
        return "Error: grip action must be 'open' or 'close'"


def _cmd_read(_parts, arm, _kin):
    """``read``"""
    names = arm.joint_names
    try:
        angles = arm.read_all()
    except Exception as exc:
        return "Error: could not read joints: {}".format(exc)
    lines = []
    for i, name in enumerate(names):
        angle = angles[i] if i < len(angles) else None
        # Try to read temperature via the driver if available.
        temp = None
        driver = getattr(arm, "_driver", None)
        if driver is not None and hasattr(driver, "read_temperature"):
            joint_cfg = arm.joint(i)
            try:
                temp = driver.read_temperature(joint_cfg.servo_id)
            except Exception:
                pass
        angle_str = "{:.1f} deg".format(angle) if angle is not None else "N/A"
        temp_str = "{} C".format(temp) if temp is not None else "N/A"
        lines.append("  J{} ({}): angle={}, temp={}".format(
            i + 1, name, angle_str, temp_str
        ))
    return "Joint states:\n" + "\n".join(lines)


def _cmd_help(_parts, _arm, _kin):
    """``help``"""
    return _HELP_TEXT


# ---------------------------------------------------------------------------
# Dispatch table
# ---------------------------------------------------------------------------

_COMMANDS = {
    "move": _cmd_move,
    "pose": _cmd_pose,
    "home": _cmd_home,
    "torque": _cmd_torque,
    "grip": _cmd_grip,
    "read": _cmd_read,
    "help": _cmd_help,
}


def dispatch(line, arm, kinematics=None):
    """Parse and execute a single CLI command.

    Parameters
    ----------
    line : str
        Raw command text (e.g. ``"move J1 90"``).
    arm : buddy.arm.Arm
        The arm instance to operate on.
    kinematics : callable, optional
        IK solver; signature ``(x, y, z, pitch, roll) -> list[float]``.
        Required only for ``pose`` commands.

    Returns
    -------
    str
        Human-readable result or error message.
    """
    line = line.strip()
    if not line:
        return ""
    parts = line.split()
    cmd = parts[0].lower()
    handler = _COMMANDS.get(cmd)
    if handler is None:
        return "Unknown command: {}. Type 'help' for available commands.".format(
            parts[0]
        )
    return handler(parts, arm, kinematics)


def repl(arm, kinematics=None):  # pragma: no cover - requires interactive stdin
    """Interactive REPL loop for use from the MicroPython console.

    Reads lines from ``sys.stdin`` via ``input()``, dispatches each one
    through :func:`dispatch`, and prints the result.  Exits on EOF
    (``Ctrl-D``) or ``KeyboardInterrupt`` (``Ctrl-C``).
    """
    print("Buddy CLI ready. Type 'help' for commands.")
    while True:
        try:
            line = input("buddy> ")
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break
        result = dispatch(line, arm, kinematics=kinematics)
        if result:
            print(result)
