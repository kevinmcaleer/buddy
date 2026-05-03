"""Wi-Fi bring-up helper for the Buddy arm.

Behaviour
---------
1. Read JSON credentials from a file on flash (default
   ``wifi_credentials.json``).  The file is **not** committed to the repo —
   ``wifi_credentials.example.json`` is the template.
2. Try to join the configured SSID in *station* (STA) mode.
3. If STA mode does not associate within ``timeout_s`` seconds, drop into
   *access-point* (AP) mode using the ``ap_fallback`` block from the
   credentials file (or sensible defaults).

Credentials JSON schema
-----------------------
::

    {
      "ssid": "your-wifi-ssid",
      "password": "your-wifi-password",
      "hostname": "buddy",
      "ap_fallback": {
        "ssid": "buddy-arm",
        "password": "buddy1234"
      }
    }

Only ``ssid`` and ``password`` are required.  The ``ap_fallback`` block is
optional; defaults are ``{"ssid": "buddy-arm", "password": "buddy1234"}``.

Imports
-------
``network`` is MicroPython-only.  We import it lazily inside :func:`connect`
so this module can be unit-tested under CPython by injecting a mock
``network`` module via ``sys.modules['network']`` (see
``tests/conftest.py`` for the pattern, mirroring how ``machine`` is mocked).
"""

try:
    import ujson as json   # MicroPython
except ImportError:        # pragma: no cover - CPython
    import json

try:
    import utime as time   # MicroPython
except ImportError:        # pragma: no cover - CPython
    import time


DEFAULT_CRED_PATH = "wifi_credentials.json"
DEFAULT_AP_SSID = "buddy-arm"
DEFAULT_AP_PASSWORD = "buddy1234"
DEFAULT_TIMEOUT_S = 15
# Poll every this many ms while waiting for STA association.
_POLL_INTERVAL_MS = 500


class WiFiError(Exception):
    """Raised when both STA and AP bring-up fail."""


def _sleep_ms(ms):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(ms))
    else:  # pragma: no cover - CPython
        time.sleep(ms / 1000.0)


def load_credentials(path=DEFAULT_CRED_PATH):
    """Load and validate the Wi-Fi credentials JSON file.

    Returns the decoded dict.  Raises :class:`WiFiError` if the file is
    missing or malformed.
    """
    try:
        with open(path, "r") as f:
            data = json.loads(f.read())
    except OSError as exc:
        raise WiFiError("credentials file not found at {!r}: {}".format(path, exc))
    except ValueError as exc:
        raise WiFiError("credentials file is not valid JSON: {}".format(exc))
    if not isinstance(data, dict):
        raise WiFiError("credentials file must contain a JSON object")
    if "ssid" not in data or "password" not in data:
        raise WiFiError("credentials file must include 'ssid' and 'password'")
    return data


def _import_network():
    """Lazy import so tests can stub ``network`` via ``sys.modules``."""
    import network  # noqa: WPS433 - intentional runtime import
    return network


def _start_sta(network_mod, ssid, password, hostname=None, timeout_s=DEFAULT_TIMEOUT_S):
    """Try to connect in STA mode.  Returns the WLAN interface on success,
    or ``None`` on timeout / association failure."""
    sta = network_mod.WLAN(network_mod.STA_IF)
    sta.active(True)
    if hostname:
        # MicroPython's `network.hostname()` is module-scoped on most ports;
        # try both the modern and the per-iface API for compatibility.
        try:
            network_mod.hostname(hostname)
        except (AttributeError, OSError):
            try:
                sta.config(hostname=hostname)
            except (AttributeError, OSError, ValueError):
                pass

    sta.connect(ssid, password)

    deadline_ms = timeout_s * 1000
    waited_ms = 0
    while waited_ms < deadline_ms:
        if sta.isconnected():
            return sta
        _sleep_ms(_POLL_INTERVAL_MS)
        waited_ms += _POLL_INTERVAL_MS
    # Disable so we don't keep the radio in a half-associated state when we
    # fall back to AP mode.
    try:
        sta.active(False)
    except OSError:  # pragma: no cover - device-specific
        pass
    return None


def _start_ap(network_mod, ssid, password):
    """Bring up an open/WPA2 access point.  Returns the AP WLAN interface."""
    ap = network_mod.WLAN(network_mod.AP_IF)
    ap.active(True)
    # ``config`` keyword names match MicroPython's network module.
    try:
        ap.config(essid=ssid, password=password)
    except (TypeError, ValueError):
        # Some ports expose the SSID as ``ssid`` rather than ``essid``.
        ap.config(ssid=ssid, password=password)
    return ap


def connect(path=DEFAULT_CRED_PATH, timeout_s=DEFAULT_TIMEOUT_S, network_module=None):
    """Bring up Wi-Fi according to :doc:`module behaviour <buddy.web.wifi>`.

    Parameters
    ----------
    path : str
        Path to the credentials JSON file on flash.
    timeout_s : int
        How long to wait for STA association before falling back to AP.
    network_module : module, optional
        Override for the ``network`` module — used by tests to inject a
        mock.  Defaults to a real ``import network``.

    Returns
    -------
    dict
        ``{"mode": "sta"|"ap", "ssid": str, "ip": str|None, "wlan": iface}``.

    Raises
    ------
    WiFiError
        If the credentials file is missing/malformed *and* the AP fallback
        also fails to start.
    """
    creds = load_credentials(path)
    network_mod = network_module if network_module is not None else _import_network()

    sta = _start_sta(
        network_mod,
        creds["ssid"],
        creds["password"],
        hostname=creds.get("hostname"),
        timeout_s=timeout_s,
    )
    if sta is not None:
        ip = _ifconfig_ip(sta)
        return {"mode": "sta", "ssid": creds["ssid"], "ip": ip, "wlan": sta}

    # Fallback: AP mode using the configured (or default) credentials.
    ap_cfg = creds.get("ap_fallback") or {}
    ap_ssid = ap_cfg.get("ssid", DEFAULT_AP_SSID)
    ap_pwd = ap_cfg.get("password", DEFAULT_AP_PASSWORD)
    try:
        ap = _start_ap(network_mod, ap_ssid, ap_pwd)
    except Exception as exc:
        raise WiFiError("AP fallback failed: {}: {}".format(type(exc).__name__, exc))
    ip = _ifconfig_ip(ap)
    return {"mode": "ap", "ssid": ap_ssid, "ip": ip, "wlan": ap}


def _ifconfig_ip(wlan):
    """Return the IPv4 address from a WLAN interface, or ``None`` if unavailable."""
    try:
        cfg = wlan.ifconfig()
    except (AttributeError, OSError):
        return None
    if cfg and len(cfg) >= 1:
        return cfg[0]
    return None
