"""Tests for buddy.motion -- trajectory generator on top of Arm.

Arm is mocked entirely; these tests do not depend on issue #2's arm.py
implementation existing yet. The mock only needs to satisfy the surface
``MotionController`` actually calls: ``move_all(angles)`` and ``read_all()``.
"""
import asyncio

import pytest

from buddy.motion import (
    MotionController,
    _interpolate,
    _plan_duration,
    DEFAULT_TICK_HZ,
)


# ---- mock Arm --------------------------------------------------------------

class FakeArm:
    """Minimal stand-in for the future ``buddy.arm.Arm`` class.

    Records every ``move_all`` call so tests can assert sync-write semantics
    (one packet per tick, never per-joint).
    """

    def __init__(self, joint_count=6, initial_angles=None):
        self.joint_count = joint_count
        if initial_angles is None:
            initial_angles = [0.0] * joint_count
        self._angles = list(initial_angles)
        self.move_all_calls = []  # list of joint vectors, in the order pushed
        self.move_joint_calls = []  # would flag a per-joint regression

    def move_all(self, angles):
        angles = list(angles)
        assert len(angles) == self.joint_count, \
            "move_all called with wrong joint count"
        self.move_all_calls.append(angles)
        self._angles = angles

    def move_joint(self, idx, angle, speed=0):  # pragma: no cover -- guard
        self.move_joint_calls.append((idx, angle, speed))

    def read_all(self):
        return list(self._angles)


# ---- virtual clock ---------------------------------------------------------

class Clock:
    """Hand-cranked monotonic clock for deterministic interpolation tests."""

    def __init__(self, t0=0.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


# ---- fixtures --------------------------------------------------------------

@pytest.fixture
def arm():
    return FakeArm()


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def mc(arm, clock):
    return MotionController(arm, time_func=clock)


# ---- _interpolate ----------------------------------------------------------

def test_interpolate_endpoints_exact():
    assert _interpolate(10.0, 30.0, 0.0) == 10.0
    assert _interpolate(10.0, 30.0, 1.0) == 30.0


def test_interpolate_midpoint():
    assert _interpolate(10.0, 30.0, 0.5) == 20.0


def test_interpolate_negative_delta():
    # going from 30 down to 10
    assert _interpolate(30.0, 10.0, 0.25) == 25.0


# ---- _plan_duration --------------------------------------------------------

def test_plan_duration_returns_request_when_within_limits():
    # 10° move at max_v=180°/s -> v_min = 10/180 = 0.055s, far below 1s
    d = _plan_duration([0], [10], 1.0, max_velocity=180, max_acceleration=720)
    assert d == 1.0


def test_plan_duration_stretches_for_velocity():
    # 360° in 0.1s -> 3600°/s, way above 180°/s limit. Min is 360/180 = 2.0s.
    # Acceleration check: sqrt(2*360/720) = 1.0s -> velocity dominates.
    d = _plan_duration([0], [360], 0.1, max_velocity=180, max_acceleration=720)
    assert d == pytest.approx(2.0)


def test_plan_duration_stretches_for_acceleration():
    # Tiny move (1°) at very low accel -- accel becomes the binding limit.
    # v limit: 1/180 = 0.0056s; a limit: sqrt(2*1/0.5) = 2.0s -> accel wins.
    d = _plan_duration([0], [1], 0.01, max_velocity=180, max_acceleration=0.5)
    assert d == pytest.approx(2.0)


def test_plan_duration_zero_move_keeps_request():
    d = _plan_duration([10, 20], [10, 20], 0.5,
                       max_velocity=180, max_acceleration=720)
    assert d == 0.5


def test_plan_duration_uses_fastest_joint():
    # Two joints: tiny one and a big one. Big one should dominate.
    d = _plan_duration([0, 0], [1, 360], 0.1,
                       max_velocity=180, max_acceleration=720)
    assert d == pytest.approx(2.0)


def test_plan_duration_rejects_zero_or_negative():
    with pytest.raises(ValueError):
        _plan_duration([0], [10], 0, 180, 720)
    with pytest.raises(ValueError):
        _plan_duration([0], [10], -1, 180, 720)


def test_plan_duration_rejects_bad_limits():
    with pytest.raises(ValueError):
        _plan_duration([0], [10], 1, 0, 720)
    with pytest.raises(ValueError):
        _plan_duration([0], [10], 1, 180, 0)


# ---- MotionController construction ----------------------------------------

def test_constructor_rejects_bad_tick_hz(arm):
    with pytest.raises(ValueError):
        MotionController(arm, tick_hz=0)


def test_default_tick_hz_is_50(arm):
    mc = MotionController(arm)
    assert mc._tick_hz == DEFAULT_TICK_HZ


def test_idle_after_construction(arm):
    mc = MotionController(arm)
    assert mc.is_moving() is False


# ---- move_to and interpolation --------------------------------------------

def test_move_to_returns_immediately(mc, arm):
    # A "long" move should return without waiting for completion.
    duration = mc.move_to([10.0] * 6, 1.0)
    assert duration == pytest.approx(1.0)
    assert mc.is_moving() is True
    # No ticks yet -> nothing pushed to the arm.
    assert arm.move_all_calls == []


def test_move_to_rejects_wrong_length(mc):
    with pytest.raises(ValueError):
        mc.move_to([10.0, 20.0], 1.0)  # only 2 joints, arm has 6


def test_move_to_uses_planned_duration_when_too_short(arm, clock):
    # Cap velocity hard so 360° in 0.1s gets stretched to 360/90 = 4s.
    mc = MotionController(arm, max_velocity=90, max_acceleration=10000,
                          time_func=clock)
    target = [360.0] + [0.0] * 5
    actual = mc.move_to(target, 0.1)
    assert actual == pytest.approx(4.0)


def test_tick_pushes_interpolated_setpoint(mc, arm, clock):
    target = [60.0] + [0.0] * 5
    mc.move_to(target, 1.0)

    # t=0 tick: should command the start point (all zeros).
    mc.tick()
    assert arm.move_all_calls[-1] == [0.0] * 6

    # t=0.5 tick: midway through the 1s move -> 30° on joint 0.
    clock.advance(0.5)
    mc.tick()
    assert arm.move_all_calls[-1][0] == pytest.approx(30.0)
    for v in arm.move_all_calls[-1][1:]:
        assert v == pytest.approx(0.0)

    # t=1.0 tick: completes move; setpoint clamps at target.
    clock.advance(0.5)
    mc.tick()
    assert arm.move_all_calls[-1][0] == pytest.approx(60.0)
    assert mc.is_moving() is False


def test_tick_progress_is_monotonic(mc, arm, clock):
    target = [90.0] + [0.0] * 5
    mc.move_to(target, 1.0)
    last = -float("inf")
    for _ in range(11):
        mc.tick()
        v = arm.move_all_calls[-1][0]
        assert v >= last
        last = v
        clock.advance(0.1)
    # Final clamp at the target.
    assert last == pytest.approx(90.0)


def test_tick_clamps_after_overshoot(mc, arm, clock):
    target = [45.0] + [0.0] * 5
    mc.move_to(target, 0.5)
    clock.advance(10.0)  # way past the end
    mc.tick()
    assert arm.move_all_calls[-1][0] == pytest.approx(45.0)
    assert mc.is_moving() is False


def test_tick_does_nothing_when_idle(mc, arm):
    # No move issued -> tick is a no-op (no bus traffic).
    assert mc.tick() is False
    assert arm.move_all_calls == []


def test_tick_uses_single_sync_write(mc, arm, clock):
    """One ``move_all`` per tick, NEVER ``move_joint`` per joint."""
    mc.move_to([10.0] * 6, 1.0)
    for _ in range(5):
        mc.tick()
        clock.advance(0.1)
    # Exactly one move_all per tick (5 ticks above).
    assert len(arm.move_all_calls) == 5
    # And NO per-joint writes anywhere.
    assert arm.move_joint_calls == []


def test_replanning_starts_from_current_interpolated_position(mc, arm, clock):
    """Issuing a new move mid-trajectory should start from the *interpolated*
    current position, not from the original start."""
    mc.move_to([100.0] + [0.0] * 5, 1.0)
    clock.advance(0.5)  # halfway -> joint 0 is at ~50°

    mc.move_to([0.0] * 6, 1.0)  # back to zero
    # The new start angles should be ~50° on joint 0.
    assert mc._start_angles[0] == pytest.approx(50.0)


def test_move_to_uses_arm_read_all_when_no_prior_move(arm, clock):
    arm._angles = [42.0] * 6
    mc = MotionController(arm, time_func=clock)
    mc.move_to([100.0] * 6, 1.0)
    assert mc._start_angles == [42.0] * 6


# ---- non-blocking semantics under asyncio ---------------------------------

def test_move_to_is_non_blocking_under_asyncio(arm):
    """``move_to`` must return immediately; ``wait()`` blocks until the
    background tick loop drains the trajectory."""

    async def scenario():
        # 100Hz so a 0.1s move resolves in ~10 ticks -- quick enough for CI.
        mc = MotionController(arm, tick_hz=100)
        mc.start()
        try:
            mc.move_to([10.0] * 6, 0.1)
            # We did NOT await wait() yet -- still moving.
            assert mc.is_moving() is True
            await mc.wait()
            assert mc.is_moving() is False
            # And the arm received at least one sync-write.
            assert len(arm.move_all_calls) >= 1
            # Final command was at (or extremely close to) the target.
            final = arm.move_all_calls[-1]
            for v in final:
                assert v == pytest.approx(10.0, abs=0.5)
        finally:
            mc.stop()

    asyncio.run(scenario())


def test_wait_returns_immediately_when_idle(arm):
    async def scenario():
        mc = MotionController(arm)
        await mc.wait()  # never moved -> done_event already set

    asyncio.run(scenario())


def test_start_is_idempotent(arm):
    async def scenario():
        mc = MotionController(arm)
        t1 = mc.start()
        t2 = mc.start()
        assert t1 is t2
        mc.stop()

    asyncio.run(scenario())


def test_stop_without_start_is_safe(arm):
    mc = MotionController(arm)
    mc.stop()  # no task to cancel; must not raise
    assert mc.is_moving() is False


def test_stop_aborts_pending_move(arm, clock):
    mc = MotionController(arm, time_func=clock)
    mc.move_to([10.0] * 6, 1.0)
    assert mc.is_moving() is True
    mc.stop()
    assert mc.is_moving() is False


def test_compute_setpoint_before_start_returns_start(arm, clock):
    mc = MotionController(arm, time_func=clock)
    mc.move_to([10.0] * 6, 1.0)
    # rewind clock prior to move_start -> elapsed < 0
    clock.t = mc._move_start - 5.0
    setpoint = mc._compute_setpoint(clock())
    assert setpoint == [0.0] * 6


def test_compute_setpoint_zero_duration_returns_target(arm, clock):
    """Pathological zero-duration trajectory snaps to target on first tick."""
    mc = MotionController(arm, time_func=clock)
    mc._start_angles = [0.0] * 6
    mc._target_angles = [10.0] * 6
    mc._move_duration = 0.0
    mc._move_start = clock()
    assert mc._compute_setpoint(clock()) == [10.0] * 6
