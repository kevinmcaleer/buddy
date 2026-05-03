"""Pytest shim: register fake `machine` and time helpers in sys.modules BEFORE
sts3215 is imported, so the driver loads under CPython.
"""
import os
import sys
import time as _real_time
import types
from collections import deque

import pytest


# ---- fake `machine` module ---------------------------------------------------

class FakeUART:
    """Records every byte written and serves bytes queued via `feed()` for read()."""

    def __init__(self, *args, **kwargs):
        self.written = bytearray()
        self._rx = deque()

    # API used by the driver
    def write(self, data):
        self.written.extend(data)
        return len(data)

    def read(self, n=None):
        if not self._rx:
            return None
        if n is None:
            n = len(self._rx)
        out = bytearray()
        while self._rx and len(out) < n:
            out.append(self._rx.popleft())
        return bytes(out) if out else None

    # Test helpers
    def feed(self, data):
        for b in data:
            self._rx.append(b)

    def clear_written(self):
        self.written = bytearray()


class FakePin:
    """Records value() transitions so tests can assert direction-pin polarity."""

    OUT = 1
    IN = 0

    def __init__(self, *args, **kwargs):
        self._value = 0
        self.history = []  # list of values written, in order

    def value(self, v=None):
        if v is None:
            return self._value
        self._value = int(bool(v))
        self.history.append(self._value)


def _install_fake_machine():
    mod = types.ModuleType("machine")
    mod.UART = FakeUART
    mod.Pin = FakePin
    sys.modules["machine"] = mod


# ---- fake `network` module --------------------------------------------------
# `network` is MicroPython-only.  We register a default stub here so that
# `import network` doesn't blow up when buddy.web.wifi is imported under
# CPython.  Individual tests build per-test FakeWLAN instances and inject them
# via the `network_module` argument to `wifi.connect(...)`, but this stub keeps
# bare `import network` calls safe for any module-level imports added later.

class _StubWLAN:
    STA_IF = 0
    AP_IF = 1

    def __init__(self, *args, **kwargs):
        self._connected = False
        self._active = False

    def active(self, value=None):
        if value is None:
            return self._active
        self._active = bool(value)

    def connect(self, ssid, password):  # pragma: no cover - replaced in tests
        pass

    def isconnected(self):  # pragma: no cover - replaced in tests
        return False

    def ifconfig(self):  # pragma: no cover - replaced in tests
        return ("0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0")

    def config(self, **kwargs):  # pragma: no cover - replaced in tests
        pass


def _install_fake_network():
    if "network" in sys.modules:
        return
    mod = types.ModuleType("network")
    mod.STA_IF = 0
    mod.AP_IF = 1
    mod.WLAN = _StubWLAN
    mod.hostname = lambda *a, **kw: None
    sys.modules["network"] = mod


def _install_time_shims():
    """The driver uses time.sleep_us / sleep_ms / ticks_ms / ticks_add / ticks_diff
    which are MicroPython extensions. Provide CPython-side stand-ins."""
    if not hasattr(_real_time, "sleep_ms"):
        _real_time.sleep_ms = lambda ms: None  # no-op in tests
    if not hasattr(_real_time, "sleep_us"):
        _real_time.sleep_us = lambda us: None
    if not hasattr(_real_time, "ticks_ms"):
        _real_time.ticks_ms = lambda: int(_real_time.monotonic() * 1000)
    if not hasattr(_real_time, "ticks_add"):
        _real_time.ticks_add = lambda t, d: t + d
    if not hasattr(_real_time, "ticks_diff"):
        _real_time.ticks_diff = lambda a, b: a - b


_install_fake_machine()
_install_fake_network()
_install_time_shims()

# Make the package root importable so `import sts3215` works regardless of cwd.
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


# ---- shared fixtures --------------------------------------------------------

@pytest.fixture
def uart():
    return FakeUART()


@pytest.fixture
def pin():
    return FakePin()


@pytest.fixture
def driver(uart):
    from sts3215 import STS3215
    return STS3215(uart, timeout_ms=20)


@pytest.fixture
def driver_with_pin(uart, pin):
    from sts3215 import STS3215
    return STS3215(uart, direction_pin=pin, timeout_ms=20)
