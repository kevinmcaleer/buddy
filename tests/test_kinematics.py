"""Tests for :mod:`buddy.kinematics`.

These tests are pure math — no fake servos required.  They cover:

* Forward kinematics at known poses (all-zeros, vertical, base-rotated).
* Forward → inverse round-trip across a battery of reachable targets.
* Singularity handling (target on the vertical base axis).
* Out-of-reach poses raising :class:`KinematicsError`.
* Joint-limit enforcement against a real :class:`buddy.arm.JointConfig`.
* Elbow-up vs. elbow-down branch selection.
* The DH parameter table is exposed as a tuple of 5 rows.
"""
import math

import pytest

from buddy.arm import JointConfig
from buddy.kinematics import (
    DEFAULT_DH_PARAMS,
    DEFAULT_LINKS,
    KinematicsError,
    NUM_JOINTS,
    dh_table,
    forward,
    inverse,
)


# ---- helpers ---------------------------------------------------------------

def _pose_close(a, b, pos_tol=1e-3, ang_tol=1e-3):
    return (
        abs(a[0] - b[0]) < pos_tol
        and abs(a[1] - b[1]) < pos_tol
        and abs(a[2] - b[2]) < pos_tol
        and abs(a[3] - b[3]) < ang_tol
        and abs(a[4] - b[4]) < ang_tol
    )


def _angles_close(a, b, tol=1e-3):
    if len(a) != len(b):
        return False
    for x, y in zip(a, b):
        # Account for ±360 equivalence.
        diff = ((x - y + 540) % 360) - 180
        if abs(diff) > tol:
            return False
    return True


# ---- DH table ---------------------------------------------------------------

def test_dh_table_has_five_rows():
    table = dh_table()
    assert len(table) == NUM_JOINTS
    for row in table:
        assert len(row) == 4  # (a, alpha, d, theta_offset)


def test_default_dh_params_alias_matches_dh_table():
    assert DEFAULT_DH_PARAMS == dh_table()


def test_dh_table_uses_supplied_links():
    custom = dict(DEFAULT_LINKS)
    custom["upper_arm"] = 250.0
    table = dh_table(custom)
    # joint 2 (index 1) row's a-parameter is upper_arm
    assert table[1][0] == 250.0


# ---- Forward kinematics: known reference poses -----------------------------

def test_forward_all_zeros_extends_along_x():
    pose = forward([0.0, 0.0, 0.0, 0.0, 0.0])
    expected_x = (
        DEFAULT_LINKS["upper_arm"]
        + DEFAULT_LINKS["forearm"]
        + DEFAULT_LINKS["wrist_offset"]
        + DEFAULT_LINKS["tool_length"]
    )
    assert abs(pose[0] - expected_x) < 1e-6
    assert abs(pose[1]) < 1e-6
    assert abs(pose[2] - DEFAULT_LINKS["base_height"]) < 1e-6
    assert abs(pose[3]) < 1e-6  # tool pitch
    assert abs(pose[4]) < 1e-6  # tool roll


def test_forward_shoulder_90_points_up():
    pose = forward([0.0, 90.0, 0.0, 0.0, 0.0])
    expected_z = (
        DEFAULT_LINKS["base_height"]
        + DEFAULT_LINKS["upper_arm"]
        + DEFAULT_LINKS["forearm"]
        + DEFAULT_LINKS["wrist_offset"]
        + DEFAULT_LINKS["tool_length"]
    )
    assert abs(pose[0]) < 1e-6
    assert abs(pose[1]) < 1e-6
    assert abs(pose[2] - expected_z) < 1e-6
    assert abs(pose[3] - 90.0) < 1e-6


def test_forward_base_rotation_swings_to_y_axis():
    pose = forward([90.0, 0.0, 0.0, 0.0, 0.0])
    # x=0 (within float tolerance), y=full reach
    expected_r = (
        DEFAULT_LINKS["upper_arm"]
        + DEFAULT_LINKS["forearm"]
        + DEFAULT_LINKS["wrist_offset"]
        + DEFAULT_LINKS["tool_length"]
    )
    assert abs(pose[0]) < 1e-6
    assert abs(pose[1] - expected_r) < 1e-6


def test_forward_returns_wrist_roll_unchanged():
    pose = forward([0.0, 0.0, 0.0, 0.0, 45.0])
    assert abs(pose[4] - 45.0) < 1e-6


def test_forward_rejects_wrong_arity():
    with pytest.raises(KinematicsError, match="expected 5"):
        forward([0.0, 0.0, 0.0, 0.0])  # only 4 angles


# ---- Forward → inverse round-trip ------------------------------------------

@pytest.mark.parametrize("target", [
    (200.0,    0.0, 100.0,   0.0,   0.0),
    (150.0,  100.0, 200.0,  30.0,   0.0),
    (100.0, -100.0, 250.0, -45.0,  90.0),
    (200.0,   50.0, 150.0,   0.0,  30.0),
    (250.0,    0.0, 150.0,  10.0, -45.0),
    (-150.0, 100.0, 200.0, -10.0,   0.0),
    (-200.0,   0.0, 100.0,   0.0,   0.0),  # behind the base
    (50.0,   200.0, 200.0,  20.0,  60.0),
])
def test_forward_inverse_round_trip(target):
    angles = inverse(target)
    back = forward(angles)
    assert _pose_close(target, back, pos_tol=1e-3, ang_tol=1e-3)


def test_inverse_then_forward_returns_consistent_joints():
    """Two solutions of inverse for the same target should yield the same FK."""
    target = (180.0, 90.0, 180.0, 15.0, 0.0)
    up = inverse(target, elbow_up=True)
    down = inverse(target, elbow_up=False)
    assert not _angles_close(up, down)  # different solutions
    assert _pose_close(forward(up), forward(down))


# ---- Singularity / sentinel handling ---------------------------------------

def test_inverse_on_vertical_axis_returns_base_zero_sentinel():
    # x = y = 0, target straight up.  The base yaw is undefined; the
    # documented sentinel is base = 0.
    target = (0.0, 0.0, 400.0, 90.0, 0.0)
    angles = inverse(target)
    assert angles[0] == 0.0  # exact sentinel value
    # FK should still recover the requested pose modulo the base sentinel.
    back = forward(angles)
    assert abs(back[0]) < 1e-6
    assert abs(back[1]) < 1e-6
    assert abs(back[2] - 400.0) < 1e-6


# ---- Out-of-reach handling -------------------------------------------------

def test_inverse_too_far_raises():
    # 10 metres is well past the 300 mm + small wrist offset reach.
    with pytest.raises(KinematicsError, match="too far"):
        inverse((10000.0, 0.0, 100.0, 0.0, 0.0))


def test_inverse_too_close_raises_when_links_unequal():
    # Make the links unequal so there is a non-zero minimum reach.
    links = dict(DEFAULT_LINKS)
    links["upper_arm"] = 200.0
    links["forearm"] = 50.0
    # Place the wrist at the shoulder origin (after subtracting tool length).
    target = (
        links["tool_length"] + links["wrist_offset"],
        0.0,
        links["base_height"],
        0.0,
        0.0,
    )
    with pytest.raises(KinematicsError, match="too close"):
        inverse(target, links=links)


def test_inverse_rejects_wrong_arity_pose():
    with pytest.raises(KinematicsError, match="expected 5"):
        inverse((100.0, 0.0, 100.0, 0.0))  # only 4 elements


# ---- Joint-limit enforcement -----------------------------------------------

def _full_range_configs():
    """Five joints all unconstrained (-180..180)."""
    return [JointConfig(servo_id=i + 1, min_angle=-180.0, max_angle=180.0,
                        home=0.0)
            for i in range(NUM_JOINTS)]


def test_inverse_passes_with_full_range_limits():
    cfgs = _full_range_configs()
    target = (200.0, 0.0, 150.0, 10.0, 0.0)
    # Should not raise.
    angles = inverse(target, joint_configs=cfgs)
    assert len(angles) == NUM_JOINTS


def test_inverse_violates_joint_limit_raises():
    # Force a tight limit on the shoulder so the obvious solution is
    # outside the window.
    cfgs = _full_range_configs()
    cfgs[1] = JointConfig(servo_id=2, min_angle=80.0, max_angle=100.0,
                          home=90.0)
    # A target straight ahead makes the shoulder ~ -54 deg, which is
    # well outside [80, 100].
    target = (200.0, 0.0, 100.0, 0.0, 0.0)
    with pytest.raises(KinematicsError, match="out of limits"):
        inverse(target, joint_configs=cfgs)


def test_inverse_joint_configs_too_short_raises():
    cfgs = _full_range_configs()[:2]  # only two configs supplied
    target = (200.0, 0.0, 100.0, 0.0, 0.0)
    with pytest.raises(KinematicsError, match="at least"):
        inverse(target, joint_configs=cfgs)


def test_inverse_normalises_into_360_window():
    """A solution of -45 deg should map into a [0, 360]-style joint window."""
    # Build configs whose windows are 0..360 (as the default arm uses).
    cfgs = [JointConfig(servo_id=i + 1, min_angle=0.0, max_angle=360.0,
                        home=180.0)
            for i in range(NUM_JOINTS)]
    # Target that yields a negative base angle (-90 deg).
    target = (0.0, -200.0, 100.0, 0.0, 0.0)
    angles = inverse(target, joint_configs=cfgs)
    # The base angle should have been shifted by +360 to land in 0..360.
    assert 0.0 <= angles[0] <= 360.0
    # The other angles must still be inside the window or have been
    # normalised; sanity-check by re-computing FK.
    # FK takes the same angle modulo 360 for the base; pose should match.
    back = forward(angles)
    assert abs(back[0]) < 1e-3
    assert abs(back[1] - (-200.0)) < 1e-3


def test_inverse_accepts_none_joint_configs():
    # Explicit None should behave the same as omitting the argument.
    angles_default = inverse((200.0, 0.0, 100.0, 0.0, 0.0))
    angles_none = inverse((200.0, 0.0, 100.0, 0.0, 0.0), joint_configs=None)
    assert _angles_close(angles_default, angles_none)


def test_inverse_per_joint_none_skips_limit_check():
    """Within the joint_configs sequence, individual ``None`` entries should
    skip the per-joint limit check for that joint only."""
    cfgs = _full_range_configs()
    cfgs[0] = None  # base unconstrained
    target = (200.0, 0.0, 100.0, 0.0, 0.0)
    angles = inverse(target, joint_configs=cfgs)
    assert len(angles) == NUM_JOINTS


def test_inverse_at_max_reach_succeeds():
    """A target exactly at the maximum reach exercises the cos_elbow=+1
    numerical guard and must still return a valid arm-extended solution."""
    full_reach = (
        DEFAULT_LINKS["upper_arm"]
        + DEFAULT_LINKS["forearm"]
    )
    # Place the wrist exactly at full extension; the tool extends in the
    # same direction so the tip lands at full_reach + back along +x.
    back = DEFAULT_LINKS["tool_length"] + DEFAULT_LINKS["wrist_offset"]
    target = (full_reach + back, 0.0, DEFAULT_LINKS["base_height"], 0.0, 0.0)
    angles = inverse(target)
    # Elbow should be ~0 (arm fully extended).
    assert abs(angles[2]) < 1e-3


def test_inverse_at_min_reach_succeeds():
    """A target exactly at the minimum reach (folded arm) must succeed and
    exercises the cos_elbow=-1 numerical guard."""
    links = dict(DEFAULT_LINKS)
    links["upper_arm"] = 200.0
    links["forearm"] = 50.0
    # Wrist on top of the shoulder -> distance = 0, but min_reach is
    # |200-50| = 150, so we want distance == 150 (fully folded).
    back = links["tool_length"] + links["wrist_offset"]
    # Place tool tip horizontally so the wrist is 150 mm from the
    # shoulder along the same axis (tool offset added back in).
    target = (links["upper_arm"] - links["forearm"] + back,
              0.0,
              links["base_height"],
              0.0, 0.0)
    angles = inverse(target, links=links)
    # Elbow should be ~180 deg (fully folded).
    assert abs(abs(angles[2]) - 180.0) < 1e-3


# ---- Elbow-up / elbow-down branches ----------------------------------------

def test_elbow_up_and_down_yield_same_tip_position():
    target = (200.0, 0.0, 150.0, 0.0, 0.0)
    up = inverse(target, elbow_up=True)
    down = inverse(target, elbow_up=False)
    pose_up = forward(up)
    pose_down = forward(down)
    # Position should be identical; orientation likewise (we constrain it).
    assert abs(pose_up[0] - pose_down[0]) < 1e-3
    assert abs(pose_up[1] - pose_down[1]) < 1e-3
    assert abs(pose_up[2] - pose_down[2]) < 1e-3
    # And the elbow joint values themselves should differ in sign.
    assert math.copysign(1.0, up[2]) != math.copysign(1.0, down[2])


# ---- Custom link parameters -------------------------------------------------

def test_custom_links_round_trip():
    custom = {
        "base_height":  150.0,
        "upper_arm":    180.0,
        "forearm":      150.0,
        "wrist_offset":  10.0,
        "tool_length":   75.0,
    }
    target = (300.0, 50.0, 200.0, 5.0, 0.0)
    angles = inverse(target, links=custom)
    back = forward(angles, links=custom)
    assert _pose_close(target, back, pos_tol=1e-3, ang_tol=1e-3)
