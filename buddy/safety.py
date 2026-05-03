"""Soft-stop safety monitor for the Buddy arm.

Reads temperature and load from each servo during the motion tick and
halts the arm (disabling torque) if either exceeds a configurable
threshold.  A warning callback is invoked when readings approach the
limit (default: 80% of the threshold).

Integration
-----------
The :class:`SafetyMonitor` is designed to be called once per motion tick
(typically 50 Hz).  The :func:`check` method returns ``True`` if the arm
is safe to continue moving, or ``False`` if a soft-stop was triggered.

When a soft-stop fires the monitor:

1. Disables torque on *all* joints (the arm goes limp).
2. Sets :attr:`tripped` to ``True``.
3. Records the reason in :attr:`trip_reason`.

The caller (``main.py`` / ``MotionController``) should stop sending
setpoints until the operator acknowledges the fault and calls
:meth:`reset`.

Thresholds
----------
* ``temp_limit`` -- degrees Celsius.  Default ``70`` (STS3215 rated to
  ~85 C; we stop early).
* ``load_limit`` -- absolute load units (0..1023).  Default ``900``
  (≈88% of the 10-bit range).
* ``warn_fraction`` -- fraction of the limit at which the warning
  callback fires.  Default ``0.8``.
"""


DEFAULT_TEMP_LIMIT = 70       # degrees Celsius
DEFAULT_LOAD_LIMIT = 900      # absolute load magnitude (0..1023)
DEFAULT_WARN_FRACTION = 0.8   # warn at 80% of threshold


class SafetyMonitor:
    """Per-tick safety checker for an STS3215-based arm.

    Parameters
    ----------
    arm : buddy.arm.Arm
        The arm whose joints are monitored.
    driver : sts3215.STS3215
        The low-level driver used to read temperature and load.
    temp_limit : int
        Maximum allowed temperature in degrees Celsius.
    load_limit : int
        Maximum allowed absolute load magnitude.
    warn_fraction : float
        Fraction (0..1) of the limit at which the warning callback fires.
    on_warn : callable, optional
        ``on_warn(message_str)`` -- called when a reading exceeds
        ``warn_fraction * limit`` but is still below the hard limit.
    """

    def __init__(self, arm, driver,
                 temp_limit=DEFAULT_TEMP_LIMIT,
                 load_limit=DEFAULT_LOAD_LIMIT,
                 warn_fraction=DEFAULT_WARN_FRACTION,
                 on_warn=None):
        self._arm = arm
        self._driver = driver
        self._temp_limit = temp_limit
        self._load_limit = load_limit
        self._warn_fraction = warn_fraction
        self._on_warn = on_warn
        self.tripped = False
        self.trip_reason = ""

    # ---- public API --------------------------------------------------

    @property
    def temp_limit(self):
        return self._temp_limit

    @temp_limit.setter
    def temp_limit(self, value):
        self._temp_limit = value

    @property
    def load_limit(self):
        return self._load_limit

    @load_limit.setter
    def load_limit(self, value):
        self._load_limit = value

    @property
    def warn_fraction(self):
        return self._warn_fraction

    @warn_fraction.setter
    def warn_fraction(self, value):
        self._warn_fraction = value

    def check(self):
        """Run one safety check across all joints.

        Returns ``True`` if the arm is safe, ``False`` if a soft-stop
        was triggered (torque disabled, :attr:`tripped` set).
        """
        if self.tripped:
            return False

        for joint in self._arm.joints:
            sid = joint.servo_id
            # --- temperature ---
            try:
                temp = self._driver.read_temperature(sid)
            except Exception:
                temp = None
            if temp is not None:
                if temp >= self._temp_limit:
                    self._trip(
                        "servo {} temperature {} C exceeds limit {} C".format(
                            sid, temp, self._temp_limit
                        )
                    )
                    return False
                warn_temp = self._temp_limit * self._warn_fraction
                if temp >= warn_temp and self._on_warn:
                    self._on_warn(
                        "servo {} temperature {} C approaching limit {} C".format(
                            sid, temp, self._temp_limit
                        )
                    )

            # --- load ---
            try:
                load = self._driver.read_load(sid)
            except Exception:
                load = None
            if load is not None:
                abs_load = abs(load)
                if abs_load >= self._load_limit:
                    self._trip(
                        "servo {} load {} exceeds limit {}".format(
                            sid, abs_load, self._load_limit
                        )
                    )
                    return False
                warn_load = self._load_limit * self._warn_fraction
                if abs_load >= warn_load and self._on_warn:
                    self._on_warn(
                        "servo {} load {} approaching limit {}".format(
                            sid, abs_load, self._load_limit
                        )
                    )

        return True

    def reset(self):
        """Clear the trip state so the arm can be re-enabled."""
        self.tripped = False
        self.trip_reason = ""

    # ---- internals ---------------------------------------------------

    def _trip(self, reason):
        """Disable torque on all joints and record the fault."""
        self.tripped = True
        self.trip_reason = reason
        try:
            self._arm.disable_torque("all")
        except Exception:
            pass  # best-effort; the arm may already be unreachable
