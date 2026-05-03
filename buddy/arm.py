"""High-level arm abstraction for a 6-servo Feetech STS3215 arm.

The :class:`Arm` wraps a single :class:`sts3215.STS3215` driver instance (one
shared half-duplex UART bus) and addresses each joint by its servo ID.  The
arm exposes joint-space operations in **degrees** and applies a per-joint
sign/offset transform so that user-facing angles can stay sensible regardless
of how the servos are physically mounted (mirrored gears, offset zero
positions, etc.).

Coordinated motion is implemented via the Feetech *sync-write* instruction
(``0x83``), so every joint receives its goal position, speed and acceleration
in a single bus frame.  This produces noticeably smoother motion than 6
sequential WRITE packets because the servos all latch their setpoints in the
same UART transaction.

Bounds policy
-------------
Out-of-range joint angles are **silently clamped** to the per-joint
``min_angle`` / ``max_angle`` window.  This matches the clip-don't-raise
convention already used by :func:`sts3215.degrees_to_position` and keeps the
high-level API forgiving for jog/teleop loops that may briefly request
angles outside the soft limits.  If a caller needs hard validation, they
should compare against the configured limits before calling :meth:`move_joint`
/ :meth:`move_all`.

Joint configuration
-------------------
All joint metadata is loaded from a single dict (or JSON-decoded mapping)
keyed by joint name.  See :data:`DEFAULT_JOINT_CONFIGS` for an example
6-joint layout (5 revolute joints + 1 parallel-jaw gripper).
"""

from sts3215 import (
    _INST_SYNC_WRITE,
    _checksum,
    REG_GOAL_ACC,
    POSITION_MAX,
    DEGREES_MAX,
)


# Broadcast ID used as the packet ID for sync-write frames.
_BROADCAST_ID = 0xFE

# Layout of the contiguous register block written by sync-write.  We mirror the
# 7-byte block already used by STS3215.write_position so the servo applies
# acceleration, position, goal-time placeholder and speed atomically.
_SYNC_WRITE_START = REG_GOAL_ACC
_SYNC_WRITE_DATA_LEN = 7  # ACC | POS_L POS_H | TIME_L TIME_H | SPD_L SPD_H


class JointConfig:
    """Static metadata for one servo joint.

    Parameters
    ----------
    servo_id : int
        Bus ID (1..253) of the physical servo.
    min_angle, max_angle : float
        Soft limits in **user-facing degrees**.  Requests outside this range
        are clamped to it.
    home : float
        Angle (in user-facing degrees) commanded by :meth:`Arm.home`.
    sign : int
        ``+1`` or ``-1``.  Multiplies the user-facing angle before applying
        ``offset``; use ``-1`` for joints whose mechanical zero advances the
        opposite way to the convention you want to expose.
    offset : float
        Degrees added **after** the sign flip.  ``servo_deg = sign * user_deg
        + offset``.  Use this to align the servo's electrical zero with the
        joint's logical zero.
    is_gripper : bool
        Marks the joint as a gripper.  :meth:`Arm.gripper_open` /
        :meth:`Arm.gripper_close` operate on the first joint with this flag.
    open_angle, close_angle : float, optional
        User-facing angles representing the fully open / fully closed gripper
        positions.  Required when ``is_gripper=True``.
    """

    __slots__ = (
        "servo_id", "min_angle", "max_angle", "home",
        "sign", "offset", "is_gripper", "open_angle", "close_angle",
    )

    def __init__(
        self,
        servo_id,
        min_angle=0.0,
        max_angle=360.0,
        home=180.0,
        sign=1,
        offset=0.0,
        is_gripper=False,
        open_angle=None,
        close_angle=None,
    ):
        if sign not in (1, -1):
            raise ValueError("sign must be +1 or -1")
        if not 0 <= servo_id <= 253:
            raise ValueError("servo_id must be 0..253")
        if min_angle > max_angle:
            raise ValueError("min_angle must be <= max_angle")
        self.servo_id = servo_id
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.home = float(home)
        self.sign = sign
        self.offset = float(offset)
        self.is_gripper = bool(is_gripper)
        self.open_angle = (
            float(open_angle) if open_angle is not None else float(max_angle)
        )
        self.close_angle = (
            float(close_angle) if close_angle is not None else float(min_angle)
        )

    @classmethod
    def from_dict(cls, d):
        """Build a :class:`JointConfig` from a plain dict (e.g. JSON-decoded)."""
        return cls(**d)


# Reasonable defaults for a generic 6-DoF arm with a parallel-jaw gripper at
# the wrist.  Users are expected to override these in their application code
# to match their mechanical design.
DEFAULT_JOINT_CONFIGS = {
    "base":     {"servo_id": 1, "min_angle":   0.0, "max_angle": 360.0, "home": 180.0},
    "shoulder": {"servo_id": 2, "min_angle":  30.0, "max_angle": 330.0, "home": 180.0},
    "elbow":    {"servo_id": 3, "min_angle":  30.0, "max_angle": 330.0, "home": 180.0},
    "wrist":    {"servo_id": 4, "min_angle":   0.0, "max_angle": 360.0, "home": 180.0},
    "wrist_rot":{"servo_id": 5, "min_angle":   0.0, "max_angle": 360.0, "home": 180.0},
    "gripper":  {"servo_id": 6, "min_angle":   0.0, "max_angle": 180.0, "home":  90.0,
                 "is_gripper": True, "open_angle": 180.0, "close_angle": 0.0},
}


def _angle_to_position(joint, angle_deg):
    """Apply per-joint clamp + sign/offset, then convert to a 0..4095 tick."""
    if angle_deg < joint.min_angle:
        angle_deg = joint.min_angle
    elif angle_deg > joint.max_angle:
        angle_deg = joint.max_angle
    servo_deg = joint.sign * angle_deg + joint.offset
    # Wrap into 0..360 so a negative offset (e.g. sign=-1) doesn't underflow.
    servo_deg = servo_deg % DEGREES_MAX
    return int(round(servo_deg * POSITION_MAX / DEGREES_MAX)) & 0xFFFF


def _position_to_angle(joint, position):
    """Inverse of :func:`_angle_to_position` — does **not** clamp."""
    servo_deg = (position & 0xFFFF) * DEGREES_MAX / POSITION_MAX
    user_deg = (servo_deg - joint.offset) * joint.sign
    return user_deg % DEGREES_MAX


class Arm:
    """A coordinated 6-servo arm sharing one half-duplex UART bus.

    Parameters
    ----------
    driver : sts3215.STS3215
        Pre-constructed driver bound to the UART that all joints sit on.
    joint_configs : dict
        Mapping of joint name -> :class:`JointConfig` *or* plain dict suitable
        for :meth:`JointConfig.from_dict`.  Iteration order of the dict is
        preserved as the canonical joint order (Python 3.7+ / MicroPython
        retain insertion order).
    default_speed, default_accel : int
        Used when the per-call ``speed`` / ``accel`` arguments are omitted.
    """

    def __init__(self, driver, joint_configs=None, default_speed=1000, default_accel=50):
        if joint_configs is None:
            joint_configs = DEFAULT_JOINT_CONFIGS
        self._driver = driver
        self._default_speed = default_speed
        self._default_accel = default_accel
        # Normalise to JointConfig instances and remember insertion order.
        self._joints = []      # list[JointConfig]
        self._names = []       # parallel list[str]
        self._by_name = {}
        for name, cfg in joint_configs.items():
            if isinstance(cfg, JointConfig):
                jc = cfg
            else:
                jc = JointConfig.from_dict(cfg)
            self._joints.append(jc)
            self._names.append(name)
            self._by_name[name] = jc

    # ---- introspection ----------------------------------------------------

    def __len__(self):
        return len(self._joints)

    @property
    def joint_names(self):
        return list(self._names)

    @property
    def joints(self):
        return list(self._joints)

    def joint(self, key):
        """Resolve a joint by index *or* name."""
        if isinstance(key, str):
            return self._by_name[key]
        return self._joints[key]

    # ---- bounds clamping --------------------------------------------------

    def _clamp(self, joint, angle_deg):
        if angle_deg < joint.min_angle:
            return joint.min_angle
        if angle_deg > joint.max_angle:
            return joint.max_angle
        return angle_deg

    # ---- single-joint motion ---------------------------------------------

    def move_joint(self, key, angle_deg, speed=None, accel=None):
        """Move one joint to *angle_deg* (in user-facing degrees).

        Out-of-range angles are clamped silently; see module docstring.
        """
        joint = self.joint(key)
        angle_deg = self._clamp(joint, angle_deg)
        position = _angle_to_position(joint, angle_deg)
        if speed is None:
            speed = self._default_speed
        if accel is None:
            accel = self._default_accel
        self._driver.write_position(joint.servo_id, position, speed=speed, accel=accel)

    # ---- multi-joint sync-write ------------------------------------------

    def _build_sync_write_packet(self, entries):
        """Build the raw sync-write frame for the given (joint, position, speed, accel) entries.

        Packet layout (Dynamixel-derived)::

            0xFF 0xFF 0xFE LEN 0x83 START_ADDR DATA_LEN
                [ID1 ACC POS_L POS_H 0 0 SPD_L SPD_H]
                [ID2 ACC POS_L POS_H 0 0 SPD_L SPD_H]
                ...
                CHECKSUM

        ``LEN = (DATA_LEN + 1) * N + 4`` and the checksum is the bitwise NOT
        of the sum of every byte after the 0xFF 0xFF header.
        """
        n = len(entries)
        if n == 0:
            raise ValueError("sync-write requires at least one joint")
        data_len = _SYNC_WRITE_DATA_LEN
        total_len = (data_len + 1) * n + 4
        body = bytearray()
        body.append(_BROADCAST_ID)
        body.append(total_len)
        body.append(_INST_SYNC_WRITE)
        body.append(_SYNC_WRITE_START)
        body.append(data_len)
        for joint, position, speed, accel in entries:
            position &= 0xFFFF
            speed &= 0xFFFF
            accel &= 0xFF
            body.append(joint.servo_id)
            body.append(accel)
            body.append(position & 0xFF)
            body.append((position >> 8) & 0xFF)
            body.append(0x00)  # goal time low
            body.append(0x00)  # goal time high
            body.append(speed & 0xFF)
            body.append((speed >> 8) & 0xFF)
        chk = _checksum(bytes(body))
        return b"\xff\xff" + bytes(body) + bytes((chk,))

    def _sync_write(self, entries):
        """Push the sync-write packet onto the bus via the driver's UART."""
        packet = self._build_sync_write_packet(entries)
        # Re-use the driver's own send path so direction-pin handling stays
        # consistent.  Sync-write is a broadcast and never expects a reply.
        self._driver._send(packet)

    def move_all(self, angles, speed=None, accel=None):
        """Move every joint simultaneously via a single sync-write frame.

        ``angles`` may be:
          * a list/tuple of N floats in joint-name order, or
          * a dict ``{name: angle}`` (missing joints are left untouched -
            those joints simply aren't included in the sync-write packet).
        """
        if speed is None:
            speed = self._default_speed
        if accel is None:
            accel = self._default_accel

        entries = []
        if isinstance(angles, dict):
            for name, angle in angles.items():
                joint = self._by_name[name]
                clamped = self._clamp(joint, angle)
                entries.append(
                    (joint, _angle_to_position(joint, clamped), speed, accel)
                )
        else:
            if len(angles) != len(self._joints):
                raise ValueError(
                    "expected {} angles, got {}".format(len(self._joints), len(angles))
                )
            for joint, angle in zip(self._joints, angles):
                clamped = self._clamp(joint, angle)
                entries.append(
                    (joint, _angle_to_position(joint, clamped), speed, accel)
                )

        self._sync_write(entries)

    # ---- reading ----------------------------------------------------------

    def read_all(self):
        """Return a list of present angles (user-facing degrees) for each joint.

        Reads happen sequentially because Feetech servos can only reply one at
        a time; the bus is half-duplex.
        """
        out = []
        for joint in self._joints:
            pos = self._driver.read_position(joint.servo_id)
            out.append(_position_to_angle(joint, pos))
        return out

    # ---- canned poses -----------------------------------------------------

    def home(self, speed=None, accel=None):
        """Move every joint to its configured home angle in one sync-write frame."""
        angles = [j.home for j in self._joints]
        self.move_all(angles, speed=speed, accel=accel)

    # ---- torque enable/disable -------------------------------------------

    def _resolve_keys(self, key):
        """Helper: ``"all"``/``None`` -> every joint; otherwise a single joint."""
        if key is None or key == "all":
            return list(self._joints)
        return [self.joint(key)]

    def enable_torque(self, key="all"):
        """Enable torque on one joint or all joints.

        ``key`` may be ``"all"`` (default), an integer index, or a joint name.
        """
        for joint in self._resolve_keys(key):
            self._driver.enable_torque(joint.servo_id)

    def disable_torque(self, key="all"):
        """Disable torque on one joint or all joints (lets the arm go limp)."""
        for joint in self._resolve_keys(key):
            self._driver.disable_torque(joint.servo_id)

    # ---- gripper helpers -------------------------------------------------

    def _gripper_joint(self):
        for joint in self._joints:
            if joint.is_gripper:
                return joint
        raise ValueError("no joint flagged as is_gripper=True")

    def gripper_open(self, speed=None, accel=None):
        """Drive the gripper joint to its configured ``open_angle``."""
        joint = self._gripper_joint()
        idx = self._joints.index(joint)
        self.move_joint(idx, joint.open_angle, speed=speed, accel=accel)

    def gripper_close(self, speed=None, accel=None):
        """Drive the gripper joint to its configured ``close_angle``."""
        joint = self._gripper_joint()
        idx = self._joints.index(joint)
        self.move_joint(idx, joint.close_angle, speed=speed, accel=accel)
