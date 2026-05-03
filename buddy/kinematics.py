"""Forward and inverse kinematics for the Buddy 6-servo arm.

Approach
========
The default Buddy arm has six servos but only five are used to position
the tool tip — the sixth is a parallel-jaw gripper.  The first five joints
form the standard "5-DoF hobby arm" topology::

    base (yaw)  ->  shoulder (pitch)  ->  elbow (pitch)
                ->  wrist (pitch)     ->  wrist_rot (roll)

With this geometry the analytical solution is straightforward and *much*
faster than a numeric (Jacobian / damped-least-squares) iteration on a
microcontroller.  We therefore use a closed-form geometric IK:

    1. The base yaw is recovered directly from ``atan2(y, x)``.
    2. The shoulder/elbow form a planar 2-link manipulator in the
       vertical plane that rotates with the base; it is solved with the
       law of cosines.
    3. The wrist pitch is set from the desired *tool pitch* (angle of the
       gripper approach vector relative to horizontal).
    4. The wrist roll is the requested ``tool_roll`` passed through.

A 5-DoF arm cannot independently command all three orientation degrees
of freedom.  The pose interface is therefore a 5-tuple
``(x, y, z, tool_pitch_deg, tool_roll_deg)`` and ``forward()`` returns
the same shape so that ``forward → inverse`` round-trips are exact
(modulo float precision).

Geometry config
---------------
:data:`DEFAULT_LINKS` holds the five physical link parameters used for
both FK and IK.  This is intentionally simpler than a full DH table —
the arm topology is fixed (revolute joints with the orientations
described above), so we only need lengths and offsets:

* ``base_height``   – vertical offset from the base mount to the
  shoulder joint.
* ``upper_arm``     – length from the shoulder to the elbow.
* ``forearm``       – length from the elbow to the wrist pitch axis.
* ``wrist_offset``  – axial offset between the wrist pitch axis and the
  wrist roll axis (often 0 for clean spherical-wrist designs).
* ``tool_length``   – distance from the wrist roll axis to the tool tip
  (gripper finger contact point).

A DH parameter view (:func:`dh_table`) is provided for compatibility
with downstream tools that expect classical Denavit-Hartenberg input.

All distances are millimetres.  Default values are *placeholders*; real
hardware requires measurement (see README).

Sentinel / error handling
-------------------------
* Out-of-reach Cartesian targets raise :class:`KinematicsError` with a
  message naming the failing constraint ("target too far",
  "target too close", ...).
* Joint-limit violations raise :class:`KinematicsError`; the caller can
  catch and decide whether to clamp or refuse.
* Singular targets on the vertical base axis are detected and a
  documented sentinel of ``base = 0`` is returned so the arm does not
  spin unpredictably.

Units
-----
* All Cartesian coordinates: millimetres.
* All joint and pose angles: degrees, matching :class:`buddy.arm.JointConfig`.
"""

import math


# ---------------------------------------------------------------------------
# Geometry config
# ---------------------------------------------------------------------------
#
# Default link lengths (mm).  These are placeholder values for a generic
# 5-DoF hobby arm; real hardware requires measurement.

DEFAULT_LINKS = {
    "base_height":  100.0,   # base mount to shoulder pitch axis
    "upper_arm":    120.0,   # shoulder to elbow
    "forearm":      120.0,   # elbow to wrist pitch axis
    "wrist_offset":   0.0,   # wrist pitch to wrist roll axis (axial)
    "tool_length":   60.0,   # wrist roll to tool tip
}


def dh_table(links=DEFAULT_LINKS):
    """Return a classical Denavit-Hartenberg parameter table for the chain.

    Each row is ``(a, alpha, d, theta_offset)`` in standard DH convention.
    The chain is::

        joint 1 (base yaw)   : a=0,                alpha=+pi/2, d=base_height
        joint 2 (shoulder)   : a=upper_arm,        alpha=0,     d=0
        joint 3 (elbow)      : a=forearm,          alpha=0,     d=0
        joint 4 (wrist pitch): a=0,                alpha=+pi/2, d=wrist_offset
        joint 5 (wrist roll) : a=0,                alpha=0,     d=tool_length

    Returned as a tuple of tuples for easy iteration.
    """
    return (
        (0.0,             math.pi / 2, links["base_height"], 0.0),
        (links["upper_arm"], 0.0,      0.0,                  0.0),
        (links["forearm"],   0.0,      0.0,                  0.0),
        (0.0,             math.pi / 2, links["wrist_offset"], 0.0),
        (0.0,             0.0,         links["tool_length"], 0.0),
    )


# Backwards-compatible alias.  Issue #4 asks for a "DH parameter table"
# config; we expose both names so callers can pick whichever they prefer.
DEFAULT_DH_PARAMS = dh_table()


# Joint indices.
_BASE, _SHOULDER, _ELBOW, _WRIST, _WRIST_ROT = 0, 1, 2, 3, 4

# The kinematic chain has five revolute joints; the Buddy arm's sixth servo
# is the gripper and is ignored here.
NUM_JOINTS = 5

# Tolerances
_POS_EPS = 1e-6     # mm; below this we treat the target as on-axis
_REACH_EPS = 1e-3   # mm; numerical slack on reach checks
_LIMIT_EPS = 1e-6   # deg; slack on joint-limit comparisons


class KinematicsError(Exception):
    """Raised when a pose cannot be reached or a joint limit would be violated."""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _deg(rad):
    return rad * 180.0 / math.pi


def _rad(deg):
    return deg * math.pi / 180.0


def _wrap_deg(angle):
    """Wrap an angle into the (-180, 180] degree range."""
    a = ((angle + 180.0) % 360.0) - 180.0
    if a == -180.0:
        return 180.0
    return a


def _normalise_into_limits(joint_cfg, angle_deg):
    """Return an equivalent angle that lies inside ``joint_cfg`` limits.

    Tries the raw angle and ±360° shifts.  If none fit, returns ``None``.
    """
    if joint_cfg is None:
        return angle_deg
    for a in (angle_deg, angle_deg + 360.0, angle_deg - 360.0):
        if (joint_cfg.min_angle - _LIMIT_EPS
                <= a <= joint_cfg.max_angle + _LIMIT_EPS):
            return a
    return None


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward(joint_angles, links=DEFAULT_LINKS):
    """Return the Cartesian pose of the tool tip.

    Parameters
    ----------
    joint_angles : sequence of float
        Five joint angles in **degrees**, in chain order
        ``(base, shoulder, elbow, wrist, wrist_rot)``.

        * ``base``      – rotation about the vertical world axis.
          ``0`` deg points the arm along +x.
        * ``shoulder``  – pitch up from horizontal.  ``0`` deg = arm
          extending horizontally; ``+90`` deg = arm pointing straight up.
        * ``elbow``     – pitch relative to the upper arm.  ``0`` deg =
          forearm continues in the same direction as the upper arm.
        * ``wrist``     – pitch relative to the forearm.  ``0`` deg = the
          tool axis continues in the same direction.
        * ``wrist_rot`` – roll about the tool approach axis.

    links : dict, optional
        Link length dictionary (see :data:`DEFAULT_LINKS`).

    Returns
    -------
    (x, y, z, tool_pitch_deg, tool_roll_deg)
        Position of the tool tip in millimetres in the base frame.
        ``tool_pitch_deg`` is the angle of the tool approach axis above
        the horizontal plane (positive = pointing up), and
        ``tool_roll_deg`` is the rotation of the gripper about that axis.
    """
    if len(joint_angles) != NUM_JOINTS:
        raise KinematicsError(
            "expected {} joint angles, got {}".format(NUM_JOINTS, len(joint_angles))
        )

    base, shoulder, elbow, wrist, wrist_rot = joint_angles
    base_r = _rad(base)
    shoulder_r = _rad(shoulder)
    elbow_r = _rad(elbow)
    wrist_r_rad = _rad(wrist)

    a2 = links["upper_arm"]
    a3 = links["forearm"]
    d5 = links["tool_length"]
    h0 = links["base_height"]
    # wrist_offset shifts the wrist roll axis along the forearm; for the
    # default zero value this collapses to the simple geometry below.
    d4 = links["wrist_offset"]

    # Cumulative pitch in the vertical plane.
    p2 = shoulder_r
    p3 = shoulder_r + elbow_r
    p4 = shoulder_r + elbow_r + wrist_r_rad

    # Position of each joint in the (r, z) plane that rotates with the base.
    r_elbow = a2 * math.cos(p2)
    z_elbow = h0 + a2 * math.sin(p2)
    r_wrist = r_elbow + a3 * math.cos(p3)
    z_wrist = z_elbow + a3 * math.sin(p3)
    # The wrist_offset (d4) advances the wrist roll axis along the *tool*
    # approach axis.  For d4=0 this term vanishes.
    r_roll = r_wrist + d4 * math.cos(p4)
    z_roll = z_wrist + d4 * math.sin(p4)
    r_tip = r_roll + d5 * math.cos(p4)
    z_tip = z_roll + d5 * math.sin(p4)

    x = r_tip * math.cos(base_r)
    y = r_tip * math.sin(base_r)
    z = z_tip

    tool_pitch_deg = _wrap_deg(_deg(p4))
    tool_roll_deg = _wrap_deg(wrist_rot)
    return (x, y, z, tool_pitch_deg, tool_roll_deg)


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def inverse(target_pose,
            links=DEFAULT_LINKS,
            joint_configs=None,
            elbow_up=True):
    """Solve for joint angles that achieve ``target_pose``.

    Parameters
    ----------
    target_pose : (x, y, z, tool_pitch_deg, tool_roll_deg)
        Desired tool-tip position in millimetres and orientation in degrees.
        ``tool_pitch_deg`` is measured from the horizontal plane,
        ``tool_roll_deg`` is the gripper roll about the tool approach axis.
    links : dict, optional
        Link length dictionary; defaults to :data:`DEFAULT_LINKS`.
    joint_configs : sequence of :class:`~buddy.arm.JointConfig` or None
        Per-joint soft limits.  If supplied, any solution that violates a
        joint's ``min_angle`` / ``max_angle`` raises
        :class:`KinematicsError`.  The sequence must contain at least
        :data:`NUM_JOINTS` entries; extras (e.g. the gripper) are ignored.
    elbow_up : bool, optional
        Pick the elbow-up branch of the planar 2R solution.  When
        ``False`` the elbow-down branch is returned.

    Returns
    -------
    list of float
        Five joint angles in degrees in chain order
        ``(base, shoulder, elbow, wrist, wrist_rot)``.

    Raises
    ------
    KinematicsError
        If the pose is unreachable, the math goes singular, or a joint
        limit would be violated.
    """
    if len(target_pose) != 5:
        raise KinematicsError(
            "expected 5-tuple pose (x, y, z, pitch, roll), got {}".format(
                len(target_pose)
            )
        )
    x, y, z, tool_pitch_deg, tool_roll_deg = target_pose

    a2 = links["upper_arm"]
    a3 = links["forearm"]
    d5 = links["tool_length"]
    h0 = links["base_height"]
    d4 = links["wrist_offset"]

    # ---- 1. Base rotation -------------------------------------------------
    horiz_r = math.sqrt(x * x + y * y)
    if horiz_r < _POS_EPS:
        # Singular: target on the vertical axis means base yaw is undefined.
        # Return a documented sentinel of base=0 so behaviour is predictable.
        base_deg = 0.0
    else:
        base_deg = _deg(math.atan2(y, x))

    tool_pitch_rad = _rad(tool_pitch_deg)
    cos_p = math.cos(tool_pitch_rad)
    sin_p = math.sin(tool_pitch_rad)

    # ---- 2. Wrist position (subtract tool-length contribution) -----------
    # The wrist pitch axis sits one (tool_length + wrist_offset) back along
    # the approach axis from the tool tip, in the (r, z) plane.
    back = d5 + d4
    wrist_r_pos = horiz_r - back * cos_p
    wrist_z_pos = z - h0 - back * sin_p

    # ---- 3. Planar 2R for shoulder + elbow -------------------------------
    dist_sq = wrist_r_pos * wrist_r_pos + wrist_z_pos * wrist_z_pos
    dist = math.sqrt(dist_sq)
    max_reach = a2 + a3
    min_reach = abs(a2 - a3)
    if dist > max_reach + _REACH_EPS:
        raise KinematicsError(
            "target too far: distance {:.3f} > max reach {:.3f}".format(
                dist, max_reach
            )
        )
    if dist < min_reach - _REACH_EPS:
        raise KinematicsError(
            "target too close: distance {:.3f} < min reach {:.3f}".format(
                dist, min_reach
            )
        )

    # Law of cosines for the elbow interior angle.
    cos_elbow = (dist_sq - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
    # Numerical guard.
    if cos_elbow > 1.0:
        cos_elbow = 1.0
    elif cos_elbow < -1.0:
        cos_elbow = -1.0
    elbow_rad = math.acos(cos_elbow)
    if not elbow_up:
        elbow_rad = -elbow_rad

    # Shoulder angle = polar angle to wrist in (r, z) - inner triangle angle.
    phi = math.atan2(wrist_z_pos, wrist_r_pos)
    psi = math.atan2(a3 * math.sin(elbow_rad), a2 + a3 * math.cos(elbow_rad))
    shoulder_rad = phi - psi

    shoulder_deg = _deg(shoulder_rad)
    elbow_deg = _deg(elbow_rad)

    # ---- 4. Wrist pitch from desired tool pitch --------------------------
    wrist_deg = tool_pitch_deg - shoulder_deg - elbow_deg

    # ---- 5. Wrist roll passes through ------------------------------------
    wrist_rot_deg = tool_roll_deg

    angles = [
        _wrap_deg(base_deg),
        _wrap_deg(shoulder_deg),
        _wrap_deg(elbow_deg),
        _wrap_deg(wrist_deg),
        _wrap_deg(wrist_rot_deg),
    ]

    # ---- 6. Joint-limit enforcement --------------------------------------
    if joint_configs is not None:
        if len(joint_configs) < NUM_JOINTS:
            raise KinematicsError(
                "joint_configs must contain at least {} entries".format(NUM_JOINTS)
            )
        for i in range(NUM_JOINTS):
            cfg = joint_configs[i]
            normalised = _normalise_into_limits(cfg, angles[i])
            if normalised is None:
                raise KinematicsError(
                    "joint {} angle {:.2f} deg out of limits "
                    "[{}, {}]".format(
                        i, angles[i], cfg.min_angle, cfg.max_angle
                    )
                )
            angles[i] = normalised

    return angles


__all__ = [
    "DEFAULT_LINKS",
    "DEFAULT_DH_PARAMS",
    "NUM_JOINTS",
    "KinematicsError",
    "dh_table",
    "forward",
    "inverse",
]
