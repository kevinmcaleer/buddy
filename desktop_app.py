"""Desktop demo app for the Buddy arm web UI.

Runs the full web server on localhost with a simulated arm so developers
can test and demo the web frontend without real hardware.

Usage::

    python3 desktop_app.py [--port PORT]

Then open http://localhost:8080 in a browser.  The 3D viewer, sliders,
IK panel, and CLI console all work against the simulated arm.
"""

import argparse
import random
import sys
import os

# Ensure the repo root is on the path so ``buddy`` and ``sts3215`` imports
# work regardless of working directory.
_REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

# ---- Fake ``machine`` module shim ------------------------------------------
# The arm module imports from sts3215, which imports ``machine``.  We provide a
# minimal stub so the import chain succeeds under CPython.

import types
import time as _time

if "machine" not in sys.modules:
    _machine = types.ModuleType("machine")

    class _FakeUART:
        def __init__(self, *args, **kwargs):
            pass
        def write(self, data):
            return len(data)
        def read(self, n=None):
            return None

    class _FakePin:
        OUT = 1
        IN = 0
        def __init__(self, *args, **kwargs):
            self._value = 0
        def value(self, v=None):
            if v is None:
                return self._value
            self._value = int(bool(v))

    _machine.UART = _FakeUART
    _machine.Pin = _FakePin
    sys.modules["machine"] = _machine

# Install time shims for MicroPython-specific APIs.
if not hasattr(_time, "sleep_ms"):
    _time.sleep_ms = lambda ms: _time.sleep(ms / 1000.0)
if not hasattr(_time, "sleep_us"):
    _time.sleep_us = lambda us: _time.sleep(us / 1_000_000.0)
if not hasattr(_time, "ticks_ms"):
    _time.ticks_ms = lambda: int(_time.monotonic() * 1000)
if not hasattr(_time, "ticks_add"):
    _time.ticks_add = lambda t, d: t + d
if not hasattr(_time, "ticks_diff"):
    _time.ticks_diff = lambda a, b: a - b


# ---- SimulatedArm -----------------------------------------------------------

from buddy.arm import JointConfig, DEFAULT_JOINT_CONFIGS


class SimulatedArm:
    """In-memory arm simulation that implements the same interface as Arm.

    Tracks joint angles internally and responds to all public Arm methods
    without requiring real hardware.  Adds optional random noise to reads
    to make the 3D viewer more interesting during demos.
    """

    def __init__(self, joint_configs=None, noise_deg=0.1):
        """Initialise with default joint configs.

        Parameters
        ----------
        joint_configs : dict, optional
            Joint name -> config dict mapping (same format as Arm).
        noise_deg : float
            Maximum random noise (+-) added to read_all results in degrees.
            Set to 0 for deterministic behaviour.
        """
        if joint_configs is None:
            joint_configs = DEFAULT_JOINT_CONFIGS

        self._joints = []
        self._names = []
        self._by_name = {}
        for name, cfg in joint_configs.items():
            if isinstance(cfg, JointConfig):
                jc = cfg
            else:
                jc = JointConfig.from_dict(cfg)
            self._joints.append(jc)
            self._names.append(name)
            self._by_name[name] = jc

        self._noise_deg = noise_deg
        # Internal state: current angle for each joint (start at home).
        self._angles = [j.home for j in self._joints]
        self._torque_enabled = [True] * len(self._joints)

    # ---- Introspection (matches Arm API) ------------------------------------

    def __len__(self):
        return len(self._joints)

    @property
    def joint_names(self):
        return list(self._names)

    @property
    def joints(self):
        return list(self._joints)

    def joint(self, key):
        """Resolve a joint by index or name."""
        if isinstance(key, str):
            return self._by_name[key]
        return self._joints[key]

    # ---- Motion -------------------------------------------------------------

    def _clamp(self, joint, angle_deg):
        if angle_deg < joint.min_angle:
            return joint.min_angle
        if angle_deg > joint.max_angle:
            return joint.max_angle
        return angle_deg

    def move_joint(self, key, angle_deg, speed=None, accel=None):
        """Move one joint to the specified angle (clamped to limits)."""
        joint = self.joint(key)
        idx = key if isinstance(key, int) else self._names.index(
            key if isinstance(key, str) else None
        )
        if isinstance(key, str):
            idx = self._names.index(key)
        elif isinstance(key, int):
            idx = key
        angle_deg = self._clamp(joint, angle_deg)
        self._angles[idx] = angle_deg

    def move_all(self, angles, speed=None, accel=None):
        """Move all joints simultaneously.

        ``angles`` may be a list (one per joint) or a dict (name -> angle).
        """
        if isinstance(angles, dict):
            for name, angle in angles.items():
                joint = self._by_name[name]
                idx = self._names.index(name)
                self._angles[idx] = self._clamp(joint, angle)
        else:
            if len(angles) != len(self._joints):
                raise ValueError(
                    "expected {} angles, got {}".format(
                        len(self._joints), len(angles)
                    )
                )
            for i, (joint, angle) in enumerate(zip(self._joints, angles)):
                self._angles[i] = self._clamp(joint, angle)

    def read_all(self):
        """Return current joint angles with optional noise."""
        if self._noise_deg == 0:
            return list(self._angles)
        return [
            a + random.uniform(-self._noise_deg, self._noise_deg)
            for a in self._angles
        ]

    # ---- Canned poses -------------------------------------------------------

    def home(self, speed=None, accel=None):
        """Move all joints to their configured home angles."""
        self._angles = [j.home for j in self._joints]

    # ---- Torque enable/disable -----------------------------------------------

    def _resolve_keys(self, key):
        if key is None or key == "all":
            return list(range(len(self._joints)))
        if isinstance(key, str):
            return [self._names.index(key)]
        return [key]

    def enable_torque(self, key="all"):
        """Enable torque on one or all joints."""
        for idx in self._resolve_keys(key):
            self._torque_enabled[idx] = True

    def disable_torque(self, key="all"):
        """Disable torque on one or all joints."""
        for idx in self._resolve_keys(key):
            self._torque_enabled[idx] = False

    # ---- Gripper helpers -----------------------------------------------------

    def _gripper_joint(self):
        for joint in self._joints:
            if joint.is_gripper:
                return joint
        raise ValueError("no joint flagged as is_gripper=True")

    def gripper_open(self, speed=None, accel=None):
        """Drive the gripper to its open angle."""
        joint = self._gripper_joint()
        idx = self._joints.index(joint)
        self.move_joint(idx, joint.open_angle, speed=speed, accel=accel)

    def gripper_close(self, speed=None, accel=None):
        """Drive the gripper to its close angle."""
        joint = self._gripper_joint()
        idx = self._joints.index(joint)
        self.move_joint(idx, joint.close_angle, speed=speed, accel=accel)


# ---- IK adapter -------------------------------------------------------------

def _pose_to_joints(pose_dict):
    """Adapter that converts a pose dict from the web UI to joint angles.

    The web UI sends ``{"x": ..., "y": ..., "z": ..., "pitch": ..., "roll": ...}``
    and the IK solver expects a 5-tuple ``(x, y, z, pitch, roll)``.

    Returns a dict of joint names -> angles so that move_all only updates
    the 5 kinematic joints and leaves the gripper unchanged.
    """
    from buddy.kinematics import inverse

    x = pose_dict.get("x", 0.0)
    y = pose_dict.get("y", 0.0)
    z = pose_dict.get("z", 0.0)
    pitch = pose_dict.get("pitch", 0.0)
    roll = pose_dict.get("roll", 0.0)
    angles = inverse((x, y, z, pitch, roll))
    # Map the 5 IK angles to the first 5 joint names (the 6th is the gripper,
    # which IK does not control).
    joint_names = list(DEFAULT_JOINT_CONFIGS.keys())
    return {name: angle for name, angle in zip(joint_names[:5], angles)}


# ---- CLI dispatch adapter ---------------------------------------------------

def _cli_dispatch(command, arm, kinematics=None):
    """Wrap buddy.cli.dispatch with an IK adapter for the pose command."""
    from buddy.cli import dispatch

    # The CLI's pose command calls kinematics(x, y, z, pitch, roll) with
    # positional arguments, while our IK adapter expects a tuple.
    def _ik_callable(x, y, z, pitch=0.0, roll=0.0):
        from buddy.kinematics import inverse
        return inverse((x, y, z, pitch, roll))

    ik = _ik_callable if kinematics is None else kinematics
    return dispatch(command, arm, kinematics=ik)


# ---- Main entry point -------------------------------------------------------

def run_server(port=8080):
    """Create and start the web server with a simulated arm."""
    from buddy.web.server import create_app

    arm = SimulatedArm(noise_deg=0.05)
    app, service = create_app(
        arm,
        pose_to_joints=_pose_to_joints,
        cli_dispatch=_cli_dispatch,
        stream_hz=20,
    )

    print()
    print("=" * 60)
    print("  Buddy Arm - Desktop Demo")
    print("=" * 60)
    print()
    print("  Web UI:  http://localhost:{}".format(port))
    print("  API:     http://localhost:{}/state".format(port))
    print()
    print("  Features available:")
    print("    - Joint sliders (move simulated servos)")
    print("    - 3D viewer (real-time joint state via WebSocket)")
    print("    - IK panel (Cartesian pose -> joint angles)")
    print("    - CLI console (text commands: help, move, pose, etc.)")
    print()
    print("  Press Ctrl+C to stop.")
    print("=" * 60)
    print()

    app.run(host="0.0.0.0", port=port, debug=True)


def main():
    parser = argparse.ArgumentParser(
        description="Desktop demo for the Buddy arm web UI"
    )
    parser.add_argument(
        "--port", "-p",
        type=int,
        default=8080,
        help="Port to serve on (default: 8080)",
    )
    args = parser.parse_args()
    run_server(port=args.port)


if __name__ == "__main__":
    main()
