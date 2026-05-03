"""Integration tests for :mod:`buddy.web.server`.

Endpoints are exercised via microdot's :class:`TestClient` against a fake
:class:`Arm`.  The fake records every call so we can assert that each route
delegates correctly without touching real servos.

The WebSocket loop is blocking (it runs ``while True``) so we test its
payload schema directly through :func:`buddy.web.server.state_payload`
rather than spinning up an ASGI test harness.
"""
import asyncio
from collections import OrderedDict

import pytest

from microdot.test_client import TestClient

from buddy.web.server import (
    ArmService,
    create_app,
    state_payload,
    DEFAULT_STREAM_HZ,
)


# Microdot's TestClient methods are coroutines; wrap them so the rest of the
# tests stay synchronous and easy to read.

def _run(coro):
    """Run *coro* on a fresh event loop and return its result.

    ``asyncio.run`` would create + close a loop per call; on Python 3.12+
    ``asyncio.get_event_loop`` no longer auto-creates one outside of
    ``run``, so we make our own loop and reuse it for the lifetime of the
    test process.
    """
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class SyncClient:
    """Tiny sync facade around microdot.test_client.TestClient."""

    def __init__(self, app):
        self.app = app
        self._client = TestClient(app)

    def get(self, path, **kwargs):
        return _run(self._client.get(path, **kwargs))

    def post(self, path, body=None, **kwargs):
        return _run(self._client.post(path, body=body, **kwargs))


# ---- Fake arm --------------------------------------------------------------


class FakeJointConfig:
    def __init__(self, servo_id, min_angle=0.0, max_angle=360.0, is_gripper=False):
        self.servo_id = servo_id
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.is_gripper = is_gripper
        self.home = 180.0


class FakeDriver:
    """Stand-in for sts3215.STS3215; only ``read_temperature`` is exercised."""

    def __init__(self):
        self.temps = {}

    def read_temperature(self, servo_id):
        return self.temps.get(servo_id, 25)


class FakeArm:
    """Records every Arm method call so tests can introspect what happened."""

    def __init__(self, names=("base", "shoulder", "gripper"), read_raises=False):
        self._names = list(names)
        self._joints = OrderedDict()
        for i, n in enumerate(names):
            self._joints[n] = FakeJointConfig(
                servo_id=i + 1,
                is_gripper=(n == "gripper"),
            )
        # Public stubs so the test can set the next read result.
        self.next_read = [10.0 * (i + 1) for i in range(len(names))]
        self.read_raises = read_raises
        self.calls = []
        self._driver = FakeDriver()

    @property
    def joint_names(self):
        return list(self._names)

    def joint(self, key):
        if isinstance(key, str):
            return self._joints[key]
        return list(self._joints.values())[key]

    def __len__(self):
        return len(self._joints)

    def read_all(self):
        self.calls.append(("read_all",))
        if self.read_raises:
            raise RuntimeError("bus timeout")
        return list(self.next_read)

    def move_all(self, angles, speed=None, accel=None):
        self.calls.append(("move_all", angles, speed, accel))

    def enable_torque(self, key="all"):
        self.calls.append(("enable_torque", key))

    def disable_torque(self, key="all"):
        self.calls.append(("disable_torque", key))

    def gripper_open(self, speed=None, accel=None):
        self.calls.append(("gripper_open", speed, accel))

    def gripper_close(self, speed=None, accel=None):
        self.calls.append(("gripper_close", speed, accel))

    def home(self, speed=None, accel=None):
        self.calls.append(("home", speed, accel))


@pytest.fixture
def fake_arm():
    return FakeArm()


@pytest.fixture
def app_and_client(fake_arm):
    app, _service = create_app(fake_arm)
    return app, SyncClient(app)


# ---- helpers ---------------------------------------------------------------

def _json(resp):
    """Microdot TestClient gives us a Response with a `.json` property."""
    return resp.json


# ---- /state ----------------------------------------------------------------


def test_state_returns_full_snapshot(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.get("/state")
    assert resp.status_code == 200
    body = _json(resp)
    assert "joints" in body
    assert len(body["joints"]) == 3
    j0 = body["joints"][0]
    assert j0["name"] == "base"
    assert j0["servo_id"] == 1
    # Angle came from FakeArm.next_read[0] = 10.0
    assert j0["angle"] == 10.0
    assert j0["temperature"] == 25
    # Gripper summary should point at the "gripper" entry.
    assert body["gripper"]["name"] == "gripper"
    assert body["error"] is None


def test_state_payload_helper_matches_endpoint(fake_arm):
    """The /ws stream and /state must emit the same schema — both go
    through :func:`state_payload`.  Pin the keys so we don't accidentally
    drift the schema."""
    service = ArmService(fake_arm)
    payload = state_payload(service)
    assert set(payload) == {"joints", "gripper", "error"}
    expected_joint_keys = {
        "name", "servo_id", "min_angle", "max_angle",
        "is_gripper", "angle", "temperature", "torque",
    }
    for j in payload["joints"]:
        assert set(j) == expected_joint_keys


def test_state_handles_read_failure_gracefully(fake_arm):
    fake_arm.read_raises = True
    app, _ = create_app(fake_arm)
    client = SyncClient(app)
    resp = client.get("/state")
    assert resp.status_code == 200  # not 500 — we degrade not blow up
    body = _json(resp)
    assert body["error"] is not None
    # Each joint's angle is None when the bulk read failed.
    assert all(j["angle"] is None for j in body["joints"])


# ---- /move (joint mode) ----------------------------------------------------


def test_move_with_angles_list(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post(
        "/move",
        body={"angles": [10.0, 20.0, 30.0], "speed": 500, "accel": 25},
    )
    assert resp.status_code == 200
    assert _json(resp) == {"ok": True, "mode": "joint"}
    assert ("move_all", [10.0, 20.0, 30.0], 500, 25) in fake_arm.calls


def test_move_with_angles_dict(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/move", body={"angles": {"shoulder": 90.0}})
    assert resp.status_code == 200
    # speed / accel default to None (server forwards defaults to Arm).
    assert ("move_all", {"shoulder": 90.0}, None, None) in fake_arm.calls


def test_move_invalid_angles_type_returns_400(app_and_client):
    _, client = app_and_client
    resp = client.post("/move", body={"angles": "not a list"})
    assert resp.status_code == 400
    assert _json(resp)["ok"] is False


def test_move_with_no_body_keys_returns_400(app_and_client):
    _, client = app_and_client
    resp = client.post("/move", body={})
    assert resp.status_code == 400


def test_move_arm_failure_surfaces_as_500(fake_arm):
    fake_arm.move_all = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("bus timeout"))
    app, _ = create_app(fake_arm)
    client = SyncClient(app)
    resp = client.post("/move", body={"angles": [0, 0, 0]})
    assert resp.status_code == 500
    assert "bus timeout" in _json(resp)["error"]


# ---- /move (Cartesian / IK adapter) ----------------------------------------


def test_move_pose_without_ik_returns_501(app_and_client):
    _, client = app_and_client
    resp = client.post("/move", body={"pose": {"x": 0.1, "y": 0.0, "z": 0.2}})
    assert resp.status_code == 501
    assert "no IK adapter" in _json(resp)["error"]


def test_move_pose_with_ik_adapter_routes_through(fake_arm):
    captured = {}

    def fake_ik(pose):
        captured["pose"] = pose
        return [11.0, 22.0, 33.0]

    app, _ = create_app(fake_arm, pose_to_joints=fake_ik)
    client = SyncClient(app)
    resp = client.post("/move", body={"pose": {"x": 0.1, "y": 0.2, "z": 0.3}})
    assert resp.status_code == 200
    assert _json(resp) == {"ok": True, "mode": "pose"}
    assert captured["pose"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert ("move_all", [11.0, 22.0, 33.0], None, None) in fake_arm.calls


def test_move_pose_ik_exception_returns_500(fake_arm):
    def boom(pose):
        raise ValueError("unreachable")

    app, _ = create_app(fake_arm, pose_to_joints=boom)
    client = SyncClient(app)
    resp = client.post("/move", body={"pose": {"x": 99}})
    assert resp.status_code == 500
    assert "IK error" in _json(resp)["error"]


def test_move_pose_must_be_object(app_and_client):
    _, client = app_and_client
    resp = client.post("/move", body={"pose": [1, 2, 3]})
    assert resp.status_code == 400


# ---- /torque ---------------------------------------------------------------


def test_torque_enable_all(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/torque", body={"enabled": True})
    assert resp.status_code == 200
    body = _json(resp)
    assert body == {"ok": True, "enabled": True, "joint": "all"}
    assert ("enable_torque", "all") in fake_arm.calls


def test_torque_disable_named_joint(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/torque", body={"enabled": False, "joint": "shoulder"})
    assert resp.status_code == 200
    assert ("disable_torque", "shoulder") in fake_arm.calls


def test_torque_disable_indexed_joint(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/torque", body={"enabled": False, "joint": 1})
    assert resp.status_code == 200
    assert ("disable_torque", 1) in fake_arm.calls


def test_torque_missing_enabled_returns_400(app_and_client):
    _, client = app_and_client
    resp = client.post("/torque", body={})
    assert resp.status_code == 400


def test_torque_invalid_joint_key_returns_400(app_and_client):
    _, client = app_and_client
    resp = client.post("/torque", body={"enabled": True, "joint": True})
    assert resp.status_code == 400


def test_torque_failure_returns_500(fake_arm):
    def boom(*a, **kw):
        raise RuntimeError("packet error")

    fake_arm.enable_torque = boom
    app, _ = create_app(fake_arm)
    client = SyncClient(app)
    resp = client.post("/torque", body={"enabled": True})
    assert resp.status_code == 500


# ---- /gripper --------------------------------------------------------------


def test_gripper_open(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/gripper", body={"action": "open"})
    assert resp.status_code == 200
    assert _json(resp) == {"ok": True, "action": "open"}
    assert any(c[0] == "gripper_open" for c in fake_arm.calls)


def test_gripper_close(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/gripper", body={"action": "close"})
    assert resp.status_code == 200
    assert any(c[0] == "gripper_close" for c in fake_arm.calls)


def test_gripper_unknown_action_returns_400(app_and_client):
    _, client = app_and_client
    resp = client.post("/gripper", body={"action": "wiggle"})
    assert resp.status_code == 400


def test_gripper_arm_failure_returns_500(fake_arm):
    fake_arm.gripper_open = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("nope"))
    app, _ = create_app(fake_arm)
    client = SyncClient(app)
    resp = client.post("/gripper", body={"action": "open"})
    assert resp.status_code == 500


# ---- /home ----------------------------------------------------------------


def test_home_with_no_body(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/home", body={})
    assert resp.status_code == 200
    assert _json(resp)["ok"] is True
    assert any(c[0] == "home" for c in fake_arm.calls)


def test_home_with_speed_accel(app_and_client, fake_arm):
    _, client = app_and_client
    resp = client.post("/home", body={"speed": 200, "accel": 5})
    assert resp.status_code == 200
    assert ("home", 200, 5) in fake_arm.calls


def test_home_failure_returns_500(fake_arm):
    fake_arm.home = lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("stuck"))
    app, _ = create_app(fake_arm)
    client = SyncClient(app)
    resp = client.post("/home", body={})
    assert resp.status_code == 500


# ---- ArmService directly --------------------------------------------------


def test_service_state_handles_missing_driver_temperature():
    """An Arm whose driver lacks read_temperature must still serialise OK."""
    arm = FakeArm()
    arm._driver = object()  # no read_temperature attribute
    svc = ArmService(arm)
    payload = svc.state()
    for j in payload["joints"]:
        assert j["temperature"] is None


def test_service_state_with_no_driver_attribute():
    """Some Arm-like duck types might omit ``_driver`` entirely."""
    class NoDriverArm(FakeArm):
        pass

    arm = NoDriverArm()
    delattr(arm, "_driver")
    svc = ArmService(arm)
    payload = svc.state()
    assert payload["joints"][0]["temperature"] is None


def test_service_gripper_unknown_action():
    svc = ArmService(FakeArm())
    _, err = svc.gripper("twist")
    assert err is not None
    assert "unknown gripper action" in err


# ---- create_app return contract ------------------------------------------


def test_create_app_returns_service_for_introspection(fake_arm):
    app, service = create_app(fake_arm)
    assert isinstance(service, ArmService)
    # The service is also stashed on the app so middleware/handlers can
    # reach it without a closure.
    assert app.service is service
    assert app.stream_hz == DEFAULT_STREAM_HZ


def test_create_app_custom_stream_hz(fake_arm):
    app, _ = create_app(fake_arm, stream_hz=10)
    assert app.stream_hz == 10


# ---- WS payload schema ---------------------------------------------------


def test_ws_payload_shape_matches_state_endpoint(app_and_client, fake_arm):
    """The WS broadcast loop calls ``state_payload(service)`` — verify the
    schema directly so we don't have to spin up an ASGI test harness."""
    _, client = app_and_client
    rest = _json(client.get("/state"))
    ws_payload = state_payload(client.app.service)
    assert set(rest) == set(ws_payload)
    assert len(rest["joints"]) == len(ws_payload["joints"])
    for r, w in zip(rest["joints"], ws_payload["joints"]):
        assert set(r) == set(w)
