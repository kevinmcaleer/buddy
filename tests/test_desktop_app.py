"""Tests for the desktop demo app's SimulatedArm and integration with create_app.

Covers:
- SimulatedArm construction and defaults
- Joint introspection (names, configs, indexing)
- Single-joint and multi-joint motion (list and dict)
- Angle clamping to joint limits
- read_all (with and without noise)
- Home pose
- Torque enable/disable
- Gripper open/close
- Error handling (wrong angle count, no gripper)
- Integration with create_app and IK adapter
"""
import asyncio

import pytest

from microdot.test_client import TestClient

# Import after conftest installs the fake machine module.
from desktop_app import SimulatedArm, _pose_to_joints, _cli_dispatch
from buddy.arm import DEFAULT_JOINT_CONFIGS, JointConfig
from buddy.web.server import create_app, ArmService, state_payload


# ---- async helper -----------------------------------------------------------

def _run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class SyncClient:
    def __init__(self, app):
        self.app = app
        self._client = TestClient(app)

    def get(self, path, **kwargs):
        return _run(self._client.get(path, **kwargs))

    def post(self, path, body=None, **kwargs):
        return _run(self._client.post(path, body=body, **kwargs))


# ---- SimulatedArm construction ----------------------------------------------


class TestSimulatedArmConstruction:
    def test_default_config(self):
        arm = SimulatedArm()
        assert len(arm) == 6
        assert arm.joint_names == list(DEFAULT_JOINT_CONFIGS.keys())

    def test_custom_config(self):
        cfg = {
            "j1": {"servo_id": 1, "min_angle": 0, "max_angle": 180, "home": 90},
            "j2": {"servo_id": 2, "min_angle": 0, "max_angle": 360, "home": 180},
        }
        arm = SimulatedArm(joint_configs=cfg)
        assert len(arm) == 2
        assert arm.joint_names == ["j1", "j2"]

    def test_starts_at_home(self):
        arm = SimulatedArm(noise_deg=0)
        angles = arm.read_all()
        for angle, jc in zip(angles, arm.joints):
            assert angle == jc.home

    def test_noise_disabled(self):
        arm = SimulatedArm(noise_deg=0)
        a1 = arm.read_all()
        a2 = arm.read_all()
        assert a1 == a2

    def test_noise_enabled(self):
        arm = SimulatedArm(noise_deg=1.0)
        # With noise, repeated reads should occasionally differ.
        results = set()
        for _ in range(50):
            results.add(tuple(round(a, 6) for a in arm.read_all()))
        # Very unlikely all 50 reads are identical with +-1 deg noise.
        assert len(results) > 1


# ---- Joint introspection ----------------------------------------------------


class TestJointIntrospection:
    def test_joint_by_index(self):
        arm = SimulatedArm()
        jc = arm.joint(0)
        assert isinstance(jc, JointConfig)
        assert jc.servo_id == 1  # base

    def test_joint_by_name(self):
        arm = SimulatedArm()
        jc = arm.joint("shoulder")
        assert jc.servo_id == 2

    def test_joint_names_list(self):
        arm = SimulatedArm()
        names = arm.joint_names
        assert "base" in names
        assert "gripper" in names

    def test_joints_property(self):
        arm = SimulatedArm()
        joints = arm.joints
        assert len(joints) == 6
        assert all(isinstance(j, JointConfig) for j in joints)


# ---- Motion -----------------------------------------------------------------


class TestMotion:
    def test_move_joint_by_index(self):
        arm = SimulatedArm(noise_deg=0)
        arm.move_joint(0, 45.0)
        assert arm.read_all()[0] == 45.0

    def test_move_joint_by_name(self):
        arm = SimulatedArm(noise_deg=0)
        arm.move_joint("base", 90.0)
        assert arm.read_all()[0] == 90.0

    def test_move_joint_clamped_high(self):
        arm = SimulatedArm(noise_deg=0)
        # Shoulder max is 330
        arm.move_joint("shoulder", 999.0)
        assert arm.read_all()[1] == 330.0

    def test_move_joint_clamped_low(self):
        arm = SimulatedArm(noise_deg=0)
        # Shoulder min is 30
        arm.move_joint("shoulder", -10.0)
        assert arm.read_all()[1] == 30.0

    def test_move_all_list(self):
        arm = SimulatedArm(noise_deg=0)
        target = [10.0, 50.0, 50.0, 90.0, 90.0, 45.0]
        arm.move_all(target)
        assert arm.read_all() == target

    def test_move_all_dict(self):
        arm = SimulatedArm(noise_deg=0)
        arm.move_all({"base": 100.0, "shoulder": 200.0})
        angles = arm.read_all()
        assert angles[0] == 100.0
        assert angles[1] == 200.0

    def test_move_all_wrong_count_raises(self):
        arm = SimulatedArm()
        with pytest.raises(ValueError, match="expected 6"):
            arm.move_all([1, 2, 3])

    def test_move_all_clamps(self):
        arm = SimulatedArm(noise_deg=0)
        # Shoulder limits: 30..330; send 999
        arm.move_all([180, 999, 180, 180, 180, 90])
        assert arm.read_all()[1] == 330.0

    def test_move_all_speed_accel_accepted(self):
        arm = SimulatedArm(noise_deg=0)
        target = [10.0, 50.0, 50.0, 90.0, 90.0, 45.0]
        arm.move_all(target, speed=500, accel=10)
        assert arm.read_all() == target


# ---- Home -------------------------------------------------------------------


class TestHome:
    def test_home_resets_all_joints(self):
        arm = SimulatedArm(noise_deg=0)
        arm.move_all([10, 50, 50, 90, 90, 45])
        arm.home()
        angles = arm.read_all()
        for angle, jc in zip(angles, arm.joints):
            assert angle == jc.home

    def test_home_accepts_speed_accel(self):
        arm = SimulatedArm(noise_deg=0)
        arm.home(speed=100, accel=5)
        angles = arm.read_all()
        for angle, jc in zip(angles, arm.joints):
            assert angle == jc.home


# ---- Torque ------------------------------------------------------------------


class TestTorque:
    def test_enable_all(self):
        arm = SimulatedArm()
        arm.disable_torque()
        arm.enable_torque()
        assert all(arm._torque_enabled)

    def test_disable_all(self):
        arm = SimulatedArm()
        arm.disable_torque()
        assert not any(arm._torque_enabled)

    def test_enable_by_name(self):
        arm = SimulatedArm()
        arm.disable_torque()
        arm.enable_torque("base")
        assert arm._torque_enabled[0] is True
        assert arm._torque_enabled[1] is False

    def test_disable_by_index(self):
        arm = SimulatedArm()
        arm.disable_torque(0)
        assert arm._torque_enabled[0] is False
        assert arm._torque_enabled[1] is True


# ---- Gripper -----------------------------------------------------------------


class TestGripper:
    def test_gripper_open(self):
        arm = SimulatedArm(noise_deg=0)
        arm.gripper_open()
        gripper_cfg = arm.joint("gripper")
        idx = arm.joint_names.index("gripper")
        assert arm.read_all()[idx] == gripper_cfg.open_angle

    def test_gripper_close(self):
        arm = SimulatedArm(noise_deg=0)
        arm.gripper_close()
        gripper_cfg = arm.joint("gripper")
        idx = arm.joint_names.index("gripper")
        assert arm.read_all()[idx] == gripper_cfg.close_angle

    def test_gripper_speed_accel(self):
        arm = SimulatedArm(noise_deg=0)
        arm.gripper_open(speed=100, accel=5)
        gripper_cfg = arm.joint("gripper")
        idx = arm.joint_names.index("gripper")
        assert arm.read_all()[idx] == gripper_cfg.open_angle

    def test_no_gripper_raises(self):
        cfg = {"j1": {"servo_id": 1, "min_angle": 0, "max_angle": 360, "home": 180}}
        arm = SimulatedArm(joint_configs=cfg)
        with pytest.raises(ValueError, match="is_gripper"):
            arm.gripper_open()


# ---- IK adapter --------------------------------------------------------------


class TestIKAdapter:
    def test_pose_to_joints_returns_dict_of_5(self):
        result = _pose_to_joints({"x": 200, "y": 0, "z": 100, "pitch": 0, "roll": 0})
        assert isinstance(result, dict)
        assert len(result) == 5  # 5 kinematic joints (gripper excluded)
        # Keys are joint names.
        assert "base" in result
        assert "gripper" not in result

    def test_pose_to_joints_defaults(self):
        # Should not raise with minimal input.
        result = _pose_to_joints({"x": 200, "y": 0, "z": 100})
        assert isinstance(result, dict)
        assert len(result) == 5

    def test_unreachable_pose_raises(self):
        from buddy.kinematics import KinematicsError
        with pytest.raises(KinematicsError):
            _pose_to_joints({"x": 9999, "y": 9999, "z": 9999})


# ---- CLI dispatch adapter ----------------------------------------------------


class TestCLIDispatch:
    def test_help_command(self):
        arm = SimulatedArm()
        result = _cli_dispatch("help", arm)
        assert "Buddy CLI" in result

    def test_read_command(self):
        arm = SimulatedArm()
        result = _cli_dispatch("read", arm)
        assert "base" in result

    def test_home_command(self):
        arm = SimulatedArm(noise_deg=0)
        arm.move_all([10, 50, 50, 90, 90, 45])
        result = _cli_dispatch("home", arm)
        assert "OK" in result
        angles = arm.read_all()
        for angle, jc in zip(angles, arm.joints):
            assert angle == jc.home

    def test_move_command(self):
        arm = SimulatedArm(noise_deg=0)
        result = _cli_dispatch("move J1 45", arm)
        assert "OK" in result
        assert arm.read_all()[0] == 45.0

    def test_unknown_command(self):
        arm = SimulatedArm()
        result = _cli_dispatch("foobar", arm)
        assert "Unknown command" in result

    def test_dispatch_with_custom_kinematics(self):
        """When a kinematics callable is provided, it is passed through."""
        arm = SimulatedArm()
        called = {}

        def my_kin(x, y, z, pitch=0.0, roll=0.0):
            called["args"] = (x, y, z, pitch, roll)
            return [180.0, 180.0, 180.0, 180.0, 180.0, 90.0]

        result = _cli_dispatch("pose 100 0 200", arm, kinematics=my_kin)
        assert "OK" in result
        assert called["args"] == (100.0, 0.0, 200.0, 0.0, 0.0)


# ---- Integration with create_app ---------------------------------------------


class TestWebIntegration:
    def test_create_app_with_simulated_arm(self):
        arm = SimulatedArm(noise_deg=0)
        app, service = create_app(arm, pose_to_joints=_pose_to_joints)
        assert isinstance(service, ArmService)

    def test_state_endpoint(self):
        arm = SimulatedArm(noise_deg=0)
        app, _ = create_app(arm, pose_to_joints=_pose_to_joints)
        client = SyncClient(app)
        resp = client.get("/state")
        assert resp.status_code == 200
        body = resp.json
        assert "joints" in body
        assert len(body["joints"]) == 6
        assert body["joints"][0]["name"] == "base"
        assert body["error"] is None

    def test_move_endpoint_joint_mode(self):
        arm = SimulatedArm(noise_deg=0)
        app, _ = create_app(arm, pose_to_joints=_pose_to_joints)
        client = SyncClient(app)
        angles = [45.0, 90.0, 90.0, 180.0, 180.0, 90.0]
        resp = client.post("/move", body={"angles": angles})
        assert resp.status_code == 200
        assert resp.json["ok"] is True
        assert arm.read_all() == angles

    def test_move_endpoint_pose_mode(self):
        arm = SimulatedArm(noise_deg=0)
        app, _ = create_app(arm, pose_to_joints=_pose_to_joints)
        client = SyncClient(app)
        resp = client.post("/move", body={
            "pose": {"x": 200, "y": 0, "z": 100, "pitch": 0, "roll": 0}
        })
        assert resp.status_code == 200
        assert resp.json["ok"] is True
        assert resp.json["mode"] == "pose"

    def test_home_endpoint(self):
        arm = SimulatedArm(noise_deg=0)
        app, _ = create_app(arm)
        client = SyncClient(app)
        # Move away from home first.
        arm.move_all([10, 50, 50, 90, 90, 45])
        resp = client.post("/home", body={})
        assert resp.status_code == 200
        assert resp.json["ok"] is True
        for angle, jc in zip(arm.read_all(), arm.joints):
            assert angle == jc.home

    def test_torque_endpoint(self):
        arm = SimulatedArm()
        app, _ = create_app(arm)
        client = SyncClient(app)
        resp = client.post("/torque", body={"enabled": False})
        assert resp.status_code == 200
        assert not any(arm._torque_enabled)

    def test_gripper_endpoint(self):
        arm = SimulatedArm(noise_deg=0)
        app, _ = create_app(arm)
        client = SyncClient(app)
        resp = client.post("/gripper", body={"action": "open"})
        assert resp.status_code == 200
        gripper_idx = arm.joint_names.index("gripper")
        assert arm.read_all()[gripper_idx] == arm.joint("gripper").open_angle

    def test_cli_endpoint(self):
        arm = SimulatedArm()
        app, _ = create_app(arm, cli_dispatch=_cli_dispatch)
        client = SyncClient(app)
        resp = client.post("/cli", body={"command": "help"})
        assert resp.status_code == 200
        assert resp.json["ok"] is True
        assert "Buddy CLI" in resp.json["result"]

    def test_state_payload_schema(self):
        arm = SimulatedArm(noise_deg=0)
        service = ArmService(arm)
        payload = state_payload(service)
        assert set(payload) == {"joints", "gripper", "error"}
        expected_keys = {
            "name", "servo_id", "min_angle", "max_angle",
            "is_gripper", "angle", "temperature", "torque",
        }
        for j in payload["joints"]:
            assert set(j) == expected_keys

    def test_ik_end_to_end(self):
        """Pose -> IK adapter -> move_all -> state update round-trip.

        The IK may produce angles outside the arm's joint limits (the
        default config uses a different zero convention).  The key assertion
        is that the pipeline completes without error and the arm state
        is updated -- clamping is expected and correct.
        """
        arm = SimulatedArm(noise_deg=0)
        pose = {"x": 200, "y": 0, "z": 100, "pitch": 0, "roll": 0}
        angle_dict = _pose_to_joints(pose)
        assert isinstance(angle_dict, dict)
        assert len(angle_dict) == 5  # 5 kinematic joints
        # move_all with a dict should succeed (values clamped internally).
        arm.move_all(angle_dict)
        result = arm.read_all()
        # Verify joints were updated (not still at home for base at least).
        # Base angle is 0.0 from IK, and home is 180.0.
        assert result[0] == 0.0  # base: 0 is within 0..360
        # Gripper should remain at home since IK doesn't touch it.
        gripper_idx = arm.joint_names.index("gripper")
        assert result[gripper_idx] == arm.joint("gripper").home
