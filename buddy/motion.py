"""Time-parameterised motion controller for the Buddy arm.

Layered on top of ``buddy.arm.Arm``: each tick we compute an interpolated
joint-angle vector for the planned trajectory and push it to the arm in a
single sync-write packet (via ``Arm.move_all``).

================================================================
Hardware vs software control decision
================================================================
We rely on the STS3215's *built-in* hardware position controller for the final
joint-level closed loop. The driver writes goal acceleration and goal speed
through ``REG_GOAL_ACC`` / ``REG_GOAL_SPEED`` (see ``sts3215.py``); the servo
firmware does the PID tracking internally. This software layer is therefore
purely a **trajectory generator** -- it shapes the *sequence* of setpoints we
hand to the servo (linear interpolation between the current joint vector and
the target, respecting per-joint velocity / acceleration limits) but does
**not** run a software PID loop on top.

Why no software PID:

* The servo's onboard loop runs faster than we can possibly close one over the
  serial bus (1 Mbaud half-duplex, ~10-15 servos worth of read/write per tick
  at 50 Hz is already tight).
* A software loop would need ``read_all`` every tick, doubling bus traffic and
  jitter for negligible benefit on a position-controlled servo.
* The STS3215's GOAL_SPEED / GOAL_ACC params already express velocity and
  acceleration limits in hardware -- duplicating them in software invites
  fighting between the two controllers.

If hardware control proves inadequate for a given application (e.g. very
slow, very precise tracking) a software PID can be slotted in inside
``_tick`` without changing the public API: ``move_all`` would receive
``current + pid(error)`` instead of the pure interpolated setpoint.
================================================================
"""

# uasyncio on MicroPython, asyncio on CPython -- the two are API-compatible
# for our use (sleep, create_task, Event).
try:
    import uasyncio as asyncio
except ImportError:  # pragma: no cover -- MicroPython-only branch
    import asyncio


DEFAULT_TICK_HZ = 50
DEFAULT_MAX_VELOCITY = 180.0  # degrees per second per joint
DEFAULT_MAX_ACCELERATION = 720.0  # degrees per second^2 per joint


def _interpolate(start, end, t):
    """Linear interpolation between two scalar joint angles.

    ``t`` is the normalised progress in [0, 1] -- callers must clamp before
    calling. Endpoints are exact: ``_interpolate(a, b, 0) == a`` and
    ``_interpolate(a, b, 1) == b``.
    """
    return start + (end - start) * t


def _plan_duration(start_angles, end_angles, requested_duration,
                   max_velocity, max_acceleration):
    """Return the duration we will actually use for this move.

    If the caller-requested duration is too aggressive for the configured
    per-joint velocity or acceleration limits, we *stretch* the move so the
    fastest joint stays inside its envelope. This is friendlier than rejecting
    the move (which would force callers to compute the limits themselves) and
    matches how the STS3215 hardware behaves: speed-limited rather than
    error-out-on-overspeed.

    For each joint with delta = |end - start|:
        v_required = delta / duration
        a_required = v_required / (duration / 2)   # accel half, decel half
                   = 2 * delta / duration^2

    We solve for the minimum duration that keeps both inside the limits.
    """
    if requested_duration <= 0:
        raise ValueError("duration must be > 0")
    if max_velocity <= 0:
        raise ValueError("max_velocity must be > 0")
    if max_acceleration <= 0:
        raise ValueError("max_acceleration must be > 0")

    duration = float(requested_duration)
    for s, e in zip(start_angles, end_angles):
        delta = abs(e - s)
        if delta == 0:
            continue
        # Velocity-limited minimum duration.
        d_vel = delta / max_velocity
        # Acceleration-limited minimum duration (triangular profile).
        # Linear interpolation has constant velocity in the middle, but the
        # *transition* from rest at endpoints implies an effective acceleration
        # of v / (duration/2) = 2*delta/duration^2.
        d_acc = (2.0 * delta / max_acceleration) ** 0.5
        joint_min = max(d_vel, d_acc)
        if joint_min > duration:
            duration = joint_min
    return duration


class MotionController:
    """Non-blocking trajectory generator for an ``Arm``.

    Usage::

        arm = Arm(...)
        mc = MotionController(arm)
        mc.start()                      # spawns the asyncio tick task
        mc.move_to([0, 30, ...], 2.0)   # returns immediately
        await mc.wait()                 # resolves when motion finishes
        mc.stop()                       # cancels the tick task

    The controller never blocks the event loop; it ``asyncio.sleep``s between
    ticks so the future web server / other coroutines run concurrently.
    """

    def __init__(self, arm,
                 tick_hz=DEFAULT_TICK_HZ,
                 max_velocity=DEFAULT_MAX_VELOCITY,
                 max_acceleration=DEFAULT_MAX_ACCELERATION,
                 time_func=None):
        if tick_hz <= 0:
            raise ValueError("tick_hz must be > 0")
        self._arm = arm
        self._tick_hz = tick_hz
        self._tick_period = 1.0 / tick_hz
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        # Tests inject a virtual clock; production uses real time.
        if time_func is None:
            import time as _time
            self._time = _time.monotonic
        else:
            self._time = time_func

        self._task = None
        self._stop = False

        # Trajectory state
        self._start_angles = None
        self._target_angles = None
        self._move_start = 0.0
        self._move_duration = 0.0
        self._moving = False
        self._done_event = asyncio.Event()
        self._done_event.set()  # idle on construction

    # ---- public API --------------------------------------------------

    def move_to(self, target_angles, duration):
        """Plan a linear move from the current position to ``target_angles``.

        Returns immediately. ``duration`` is in seconds and may be stretched
        to respect ``max_velocity`` / ``max_acceleration``. If a move is
        already in progress it is replaced -- the new trajectory starts from
        wherever the arm is *now* (interpolated, not re-read from hardware).
        """
        target_angles = list(target_angles)
        current = self._current_setpoint()
        if len(target_angles) != len(current):
            raise ValueError(
                "target_angles length {} does not match arm joint count {}"
                .format(len(target_angles), len(current))
            )
        actual_duration = _plan_duration(
            current, target_angles, duration,
            self._max_velocity, self._max_acceleration,
        )
        self._start_angles = current
        self._target_angles = target_angles
        self._move_start = self._time()
        self._move_duration = actual_duration
        self._moving = True
        self._done_event.clear()
        return actual_duration

    def is_moving(self):
        """``True`` while a planned trajectory is still progressing."""
        return self._moving

    async def wait(self):
        """Awaitable: resolves once the current move finishes (or immediately
        if no move is in progress)."""
        await self._done_event.wait()

    def start(self):
        """Spawn the background tick task. Idempotent."""
        if self._task is None:
            self._stop = False
            self._task = asyncio.create_task(self._run())
        return self._task

    def stop(self):
        """Stop the background tick task. Pending move (if any) is abandoned."""
        self._stop = True
        if self._task is not None:
            try:
                self._task.cancel()
            except Exception:  # pragma: no cover -- task may already be done
                pass
            self._task = None
        self._moving = False
        self._done_event.set()

    # ---- internals ---------------------------------------------------

    def _current_setpoint(self):
        """Return the joint vector we will treat as the move's starting point.

        While moving: the interpolated setpoint at *now*. When idle: whatever
        we last commanded; if no move has ever been issued, fall back to
        ``arm.read_all()``.
        """
        if self._moving and self._start_angles is not None:
            return self._compute_setpoint(self._time())
        if self._target_angles is not None:
            return list(self._target_angles)
        return list(self._arm.read_all())

    def _compute_setpoint(self, now):
        """Interpolated joint vector at absolute time ``now``. Endpoints
        clamp -- before the move starts, snap to start; after it ends, snap
        to target."""
        if self._move_duration <= 0:
            return list(self._target_angles)
        elapsed = now - self._move_start
        if elapsed <= 0:
            return list(self._start_angles)
        if elapsed >= self._move_duration:
            return list(self._target_angles)
        t = elapsed / self._move_duration
        return [
            _interpolate(s, e, t)
            for s, e in zip(self._start_angles, self._target_angles)
        ]

    def tick(self):
        """One trajectory tick. Pushes the interpolated setpoint to the arm
        via a single sync-write (``Arm.move_all``). Public so tests and
        non-async harnesses can drive the controller deterministically."""
        if not self._moving:
            return False
        now = self._time()
        setpoint = self._compute_setpoint(now)
        # Single sync-write packet for all joints -- crucial for coordinated
        # motion and for keeping bus traffic to ~1 packet per tick.
        self._arm.move_all(setpoint)
        if now - self._move_start >= self._move_duration:
            self._moving = False
            self._done_event.set()
        return True

    async def _run(self):
        """Background task: tick at ``tick_hz`` until ``stop()`` is called."""
        while not self._stop:
            try:
                self.tick()
            except Exception:  # pragma: no cover -- swallow per-tick errors
                # Don't let a transient bus error kill the loop forever.
                pass
            await asyncio.sleep(self._tick_period)
