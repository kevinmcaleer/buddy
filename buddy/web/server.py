"""HTTP + WebSocket backend for the Buddy arm.

Framework decision
------------------
We use **microdot** (https://github.com/miguelgrinberg/microdot) rather than
hand-rolled sockets:

* Microdot ships a MicroPython-compatible build (``microdot`` on PyPI installs
  cleanly under both CPython and MicroPython — same API on both).
* It gives us routing, request parsing, JSON helpers, the ``test_client``
  harness and a tiny WebSocket implementation in ~20 KB.
* Hand-rolling the same surface area on top of ``socket``/``select`` would be
  several hundred lines of code (HTTP request line + headers parser, chunked
  encoding for SSE/WS, etc.) for no functional gain — and it would make this
  module much harder to unit-test from CPython.

The cost is one extra file on the device (``microdot.py`` from
:command:`mip install microdot`) and ~40 KB of RAM at runtime, which is well
inside the budget on a Pico W / ESP32.

Endpoint summary
----------------
====================  ===========================================================
``GET  /state``       JSON snapshot: per-joint angles, gripper, torque,
                      temperature.
``POST /move``        Body either ``{"angles": {...}|[...]}`` (joint-space) or
                      ``{"pose": {...}}`` (Cartesian — delegated to a
                      caller-supplied IK callable; Phase 4 plugs in here).
``POST /torque``      Body ``{"enabled": bool, "joint": "all"|name|index}``.
``POST /gripper``     Body ``{"action": "open"|"close"}``.
``POST /home``        No body. Drives the arm to its configured home pose.
``POST /cli``         Body ``{"command": "..."}``; dispatches via the CLI
                      parser and returns ``{"result": "..."}``.
``GET  /ws``          WebSocket stream — joint state JSON at ``stream_hz`` Hz
                      (default 20 Hz) for the 3D viewer.
====================  ===========================================================

Cartesian / IK integration point
--------------------------------
:func:`create_app` accepts a ``pose_to_joints`` callable.  When the
``/move`` endpoint receives ``{"pose": {...}}``, it forwards the pose dict
to that callable and expects either a list of joint angles in arm order or a
``{joint_name: angle}`` mapping.  Phase 4's IK module is expected to plug in
here without any further changes to the server.
"""

try:
    from microdot import Microdot
    from microdot.websocket import with_websocket
except ImportError:  # pragma: no cover - smoke import diagnostic
    raise ImportError(
        "microdot is required for buddy.web.server. "
        "On MicroPython: `mip install microdot`. "
        "On CPython: `pip install microdot`."
    )

try:
    import ujson as json   # MicroPython
except ImportError:        # pragma: no cover - CPython
    import json

try:
    import utime as time   # MicroPython
except ImportError:        # pragma: no cover - CPython
    import time


# Default streaming rate for /ws.  20 Hz keeps the 3D viewer smooth without
# saturating the half-duplex servo bus with read traffic.
DEFAULT_STREAM_HZ = 20


def _sleep_ms(ms):
    """Cross-runtime millisecond sleep (works on both CPython and MicroPython)."""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(ms))
    else:  # pragma: no cover - CPython only
        time.sleep(ms / 1000.0)


def _safe_call(fn, *args, **kwargs):
    """Invoke ``fn(*args, **kwargs)`` returning ``(value, error_str_or_None)``.

    Used by the request handlers so a single misbehaving servo (timeout, bad
    checksum, etc.) returns a structured 500 instead of a stack trace.
    """
    try:
        return fn(*args, **kwargs), None
    except Exception as exc:
        return None, "{}: {}".format(type(exc).__name__, exc)


class ArmService:
    """Façade around an :class:`Arm` that the route handlers call into.

    Centralising the arm operations here gives us:

    * one place to translate :class:`buddy.arm.Arm` calls into JSON-friendly
      dicts (so the route handlers stay tiny and easy to read),
    * a stable seam for tests — ``ArmService`` can be subclassed or its
      ``arm`` attribute swapped for a mock,
    * graceful degradation when optional sensor reads (temperature, load) are
      not implemented or raise on a particular joint.
    """

    def __init__(self, arm, pose_to_joints=None):
        self.arm = arm
        # Optional Cartesian-IK callable.  Phase 4 plugs in here.
        # Signature: pose_to_joints(pose_dict) -> list[float] | {name: float}
        self.pose_to_joints = pose_to_joints

    # ---- read path ------------------------------------------------------

    def state(self):
        """Return a JSON-serialisable dict describing the whole arm."""
        arm = self.arm
        names = arm.joint_names
        angles, err = _safe_call(arm.read_all)
        joints = []
        for i, name in enumerate(names):
            joint_cfg = arm.joint(i)
            entry = {
                "name": name,
                "servo_id": joint_cfg.servo_id,
                "min_angle": joint_cfg.min_angle,
                "max_angle": joint_cfg.max_angle,
                "is_gripper": bool(joint_cfg.is_gripper),
                "angle": angles[i] if (angles is not None and i < len(angles)) else None,
            }
            # Optional reads — guarded individually so one slow/failing
            # register doesn't blank the whole snapshot.
            temp, _ = _safe_call(self._read_temp, joint_cfg.servo_id)
            entry["temperature"] = temp
            entry["torque"] = self._read_torque(joint_cfg.servo_id)
            joints.append(entry)
        return {
            "joints": joints,
            "gripper": self._gripper_summary(joints),
            "error": err,
        }

    def _read_temp(self, servo_id):
        # Some Arm implementations don't expose temperature via the driver;
        # fall back to None silently.
        driver = getattr(self.arm, "_driver", None)
        if driver is None or not hasattr(driver, "read_temperature"):
            return None
        return driver.read_temperature(servo_id)

    def _read_torque(self, servo_id):
        # Arm doesn't currently cache the torque-enable state and the register
        # isn't part of the standard read path; we expose ``None`` so the UI
        # can render "unknown" rather than guessing.
        return None

    @staticmethod
    def _gripper_summary(joints):
        for j in joints:
            if j.get("is_gripper"):
                return {"name": j["name"], "angle": j["angle"]}
        return None

    # ---- write path -----------------------------------------------------

    def move_joints(self, angles, speed=None, accel=None):
        return _safe_call(self.arm.move_all, angles, speed=speed, accel=accel)

    def move_pose(self, pose, speed=None, accel=None):
        if self.pose_to_joints is None:
            return None, "no IK adapter configured (Phase 4 not yet wired in)"
        try:
            joints = self.pose_to_joints(pose)
        except Exception as exc:
            return None, "IK error: {}: {}".format(type(exc).__name__, exc)
        return _safe_call(self.arm.move_all, joints, speed=speed, accel=accel)

    def set_torque(self, enabled, key="all"):
        if enabled:
            return _safe_call(self.arm.enable_torque, key)
        return _safe_call(self.arm.disable_torque, key)

    def gripper(self, action):
        if action == "open":
            return _safe_call(self.arm.gripper_open)
        if action == "close":
            return _safe_call(self.arm.gripper_close)
        return None, "unknown gripper action: {!r}".format(action)

    def home(self, speed=None, accel=None):
        return _safe_call(self.arm.home, speed=speed, accel=accel)


def state_payload(service):
    """Produce a stable JSON-string representation of the arm state.

    Used by both :func:`create_app`'s ``/state`` handler and the ``/ws``
    stream loop, so they always emit identical schema.  Exposed at module
    level so tests can pin the payload shape without spinning up an HTTP
    client.
    """
    return service.state()


def _parse_joint_key(value):
    """Allow ``"all"`` / null / int / string for joint targeting."""
    if value is None or value == "all":
        return "all"
    if isinstance(value, bool):
        # bools are also ints in Python — explicitly reject to avoid surprises.
        raise ValueError("joint key cannot be a bool")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return value
    raise ValueError("invalid joint key: {!r}".format(value))


def _request_json(req):
    """Best-effort JSON body parser that tolerates empty / non-JSON bodies."""
    body = req.json
    if body is None:
        return {}
    if not isinstance(body, dict):
        raise ValueError("expected JSON object")
    return body


def create_app(arm, pose_to_joints=None, stream_hz=DEFAULT_STREAM_HZ,
               cli_dispatch=None):
    """Build and return a configured :class:`microdot.Microdot` app.

    Parameters
    ----------
    arm : buddy.arm.Arm
        The arm to control.  May be a real :class:`Arm` or any duck-typed
        replacement (tests pass a mock).
    pose_to_joints : callable, optional
        ``pose_dict -> list[float]|dict`` IK adapter for ``/move`` Cartesian
        requests.  Defaults to ``None`` (Cartesian moves return 501).
    stream_hz : int
        WebSocket ``/ws`` broadcast rate.
    cli_dispatch : callable, optional
        ``(command_str, arm, kinematics=...) -> str`` CLI dispatcher.
        When provided, a ``POST /cli`` endpoint is registered.  Pass
        :func:`buddy.cli.dispatch` here to enable the web console.

    Returns
    -------
    (app, service) : tuple
        ``app`` is the microdot instance (call ``.run(host, port)``);
        ``service`` is the :class:`ArmService` so callers / tests can poke
        into the arm without going through HTTP.
    """
    app = Microdot()
    service = ArmService(arm, pose_to_joints=pose_to_joints)
    # Stash on the app so tests can introspect.
    app.service = service
    app.stream_hz = stream_hz

    @app.get("/state")
    def get_state(req):
        return state_payload(service)

    @app.post("/move")
    def post_move(req):
        try:
            body = _request_json(req)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}, 400

        speed = body.get("speed")
        accel = body.get("accel")

        if "angles" in body:
            angles = body["angles"]
            if not isinstance(angles, (list, dict)):
                return {"ok": False, "error": "'angles' must be list or object"}, 400
            _, err = service.move_joints(angles, speed=speed, accel=accel)
            if err is not None:
                return {"ok": False, "error": err}, 500
            return {"ok": True, "mode": "joint"}

        if "pose" in body:
            pose = body["pose"]
            if not isinstance(pose, dict):
                return {"ok": False, "error": "'pose' must be an object"}, 400
            _, err = service.move_pose(pose, speed=speed, accel=accel)
            if err is not None:
                # 501 = "Not Implemented" when no IK is wired in;
                # 500 for genuine runtime failures.
                status = 501 if "no IK adapter" in err else 500
                return {"ok": False, "error": err}, status
            return {"ok": True, "mode": "pose"}

        return {"ok": False, "error": "body must include 'angles' or 'pose'"}, 400

    @app.post("/torque")
    def post_torque(req):
        try:
            body = _request_json(req)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}, 400
        if "enabled" not in body:
            return {"ok": False, "error": "'enabled' (bool) required"}, 400
        try:
            key = _parse_joint_key(body.get("joint", "all"))
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}, 400
        _, err = service.set_torque(bool(body["enabled"]), key=key)
        if err is not None:
            return {"ok": False, "error": err}, 500
        return {"ok": True, "enabled": bool(body["enabled"]), "joint": key}

    @app.post("/gripper")
    def post_gripper(req):
        try:
            body = _request_json(req)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}, 400
        action = body.get("action")
        if action not in ("open", "close"):
            return {"ok": False, "error": "'action' must be 'open' or 'close'"}, 400
        _, err = service.gripper(action)
        if err is not None:
            return {"ok": False, "error": err}, 500
        return {"ok": True, "action": action}

    @app.post("/home")
    def post_home(req):
        try:
            body = _request_json(req)
        except ValueError:
            body = {}
        _, err = service.home(speed=body.get("speed"), accel=body.get("accel"))
        if err is not None:
            return {"ok": False, "error": err}, 500
        return {"ok": True}

    # ---- CLI console endpoint -----------------------------------------------

    @app.post("/cli")
    def post_cli(req):
        if cli_dispatch is None:
            return {"ok": False, "error": "CLI not configured"}, 501
        try:
            body = _request_json(req)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}, 400
        command = body.get("command")
        if not isinstance(command, str) or not command.strip():
            return {"ok": False, "error": "'command' (non-empty string) required"}, 400
        try:
            result = cli_dispatch(command, service.arm, kinematics=service.pose_to_joints)
        except Exception as exc:
            return {"ok": False, "error": "{}: {}".format(type(exc).__name__, exc)}, 500
        if result.startswith("Error:"):
            return {"ok": False, "error": result}
        return {"ok": True, "result": result}

    @app.route("/ws")
    @with_websocket
    def ws_state(req, ws):  # pragma: no cover - exercised via WS test below
        period_ms = max(1, int(1000 / app.stream_hz))
        while True:
            payload = state_payload(service)
            ws.send(json.dumps(payload))
            _sleep_ms(period_ms)

    return app, service


# -- top-level entry point used on-device --------------------------------------

def run(arm, host="0.0.0.0", port=80, **kwargs):  # pragma: no cover - device entry point
    """Convenience wrapper used by ``main.py`` on the microcontroller.

    Builds the app, starts microdot's blocking event loop and keeps the
    arm/service references alive.  Tests use :func:`create_app` directly.
    """
    app, _ = create_app(arm, **kwargs)
    app.run(host=host, port=port)
