"""Tests for :mod:`buddy.web.wifi`.

A fake :mod:`network` module is injected per-test via the ``network_module``
parameter of :func:`buddy.web.wifi.connect` so we can simulate STA failure /
success and assert that the AP fallback engages cleanly under CPython — no
real radios required.
"""
import json
import types

import pytest

from buddy.web import wifi
from buddy.web.wifi import WiFiError, connect, load_credentials


# ---- in-memory fake `network` module ---------------------------------------


class FakeWLAN:
    """Records every call so tests can assert behaviour."""

    def __init__(self, iface, isconnected_after=0, fail_active=False):
        self.iface = iface
        self.active_state = False
        self.connected_to = None
        self._calls_to_isconnected = 0
        self._isconnected_after = isconnected_after
        self.config_calls = []
        self.ifconfig_value = ("192.168.1.42", "255.255.255.0", "192.168.1.1", "8.8.8.8")
        self._fail_active = fail_active

    def active(self, value=None):
        if value is None:
            return self.active_state
        if self._fail_active and value is False:
            raise OSError("simulated radio shutdown failure")
        self.active_state = bool(value)

    def connect(self, ssid, password):
        self.connected_to = (ssid, password)

    def isconnected(self):
        self._calls_to_isconnected += 1
        return self._calls_to_isconnected > self._isconnected_after

    def ifconfig(self):
        return self.ifconfig_value

    def config(self, **kwargs):
        self.config_calls.append(kwargs)


def _make_network_module(sta_wlan, ap_wlan, hostname_supported=True):
    """Build a stub ``network`` module that hands out the given WLAN fakes."""
    mod = types.ModuleType("network")
    mod.STA_IF = 0
    mod.AP_IF = 1

    instances = {}

    def WLAN(iface):
        if iface == 0:
            instances.setdefault("sta", sta_wlan)
            return sta_wlan
        instances.setdefault("ap", ap_wlan)
        return ap_wlan

    mod.WLAN = WLAN
    if hostname_supported:
        mod.hostname = lambda name: instances.setdefault("hostname", name)
    return mod


# ---- credentials loader ----------------------------------------------------


def test_load_credentials_round_trip(tmp_path):
    p = tmp_path / "wifi.json"
    p.write_text(json.dumps({"ssid": "x", "password": "y", "hostname": "buddy"}))
    creds = load_credentials(str(p))
    assert creds["ssid"] == "x"
    assert creds["password"] == "y"
    assert creds["hostname"] == "buddy"


def test_load_credentials_missing_file_raises(tmp_path):
    with pytest.raises(WiFiError, match="not found"):
        load_credentials(str(tmp_path / "nope.json"))


def test_load_credentials_bad_json_raises(tmp_path):
    p = tmp_path / "wifi.json"
    p.write_text("not json {{{")
    with pytest.raises(WiFiError, match="not valid JSON"):
        load_credentials(str(p))


def test_load_credentials_rejects_non_object(tmp_path):
    p = tmp_path / "wifi.json"
    p.write_text(json.dumps(["a", "b"]))
    with pytest.raises(WiFiError, match="JSON object"):
        load_credentials(str(p))


def test_load_credentials_requires_ssid_and_password(tmp_path):
    p = tmp_path / "wifi.json"
    p.write_text(json.dumps({"ssid": "only"}))
    with pytest.raises(WiFiError, match="must include"):
        load_credentials(str(p))


# ---- STA happy-path --------------------------------------------------------


def _write_creds(tmp_path, **extra):
    body = {"ssid": "home", "password": "secret"}
    body.update(extra)
    p = tmp_path / "wifi.json"
    p.write_text(json.dumps(body))
    return str(p)


def test_connect_sta_success_returns_sta_info(tmp_path):
    sta = FakeWLAN(iface="sta", isconnected_after=0)
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(tmp_path, hostname="buddy")

    result = connect(path=path, timeout_s=2, network_module=netmod)

    assert result["mode"] == "sta"
    assert result["ssid"] == "home"
    assert result["ip"] == "192.168.1.42"
    assert sta.connected_to == ("home", "secret")
    assert sta.active_state is True
    # AP must NOT have been activated when STA succeeds.
    assert ap.active_state is False


def test_connect_sta_falls_back_to_ap_on_timeout(tmp_path):
    # never_connects → isconnected() always False
    sta = FakeWLAN(iface="sta", isconnected_after=10_000)
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(tmp_path)

    # Tiny timeout so the test runs fast.
    result = connect(path=path, timeout_s=0, network_module=netmod)

    assert result["mode"] == "ap"
    # Default AP defaults are used when ap_fallback isn't in the file.
    assert result["ssid"] == "buddy-arm"
    assert ap.active_state is True
    # STA radio should have been deactivated cleanly.
    assert sta.active_state is False


def test_connect_uses_custom_ap_fallback(tmp_path):
    sta = FakeWLAN(iface="sta", isconnected_after=10_000)
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(
        tmp_path,
        ap_fallback={"ssid": "buddy-rescue", "password": "rescuepwd"},
    )

    result = connect(path=path, timeout_s=0, network_module=netmod)

    assert result["mode"] == "ap"
    assert result["ssid"] == "buddy-rescue"
    # Either essid= or ssid= keyword is acceptable; both go through config().
    assert ap.config_calls, "AP config() should have been called"
    last = ap.config_calls[-1]
    assert last.get("essid") == "buddy-rescue" or last.get("ssid") == "buddy-rescue"
    assert last.get("password") == "rescuepwd"


def test_connect_falls_through_when_sta_active_off_raises(tmp_path):
    """If `sta.active(False)` raises after timeout, AP fallback must still
    happen — we must not propagate device-specific shutdown errors."""
    sta = FakeWLAN(iface="sta", isconnected_after=10_000, fail_active=True)
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(tmp_path)
    result = connect(path=path, timeout_s=0, network_module=netmod)
    assert result["mode"] == "ap"


def test_connect_hostname_falls_back_to_iface_config(tmp_path):
    """If module-level network.hostname() is missing, we should fall back to
    sta.config(hostname=...)."""
    sta = FakeWLAN(iface="sta", isconnected_after=0)
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap, hostname_supported=False)
    path = _write_creds(tmp_path, hostname="buddy")

    connect(path=path, timeout_s=2, network_module=netmod)

    # config(hostname=...) must have been called as a fallback.
    assert any("hostname" in call for call in sta.config_calls)


def test_connect_propagates_ap_failure_as_wifi_error(tmp_path, monkeypatch):
    """If even the AP fallback throws, surface it as WiFiError, not a stray
    runtime exception."""
    sta = FakeWLAN(iface="sta", isconnected_after=10_000)

    class BoomWLAN(FakeWLAN):
        def active(self, value=None):
            if value is True:
                raise RuntimeError("AP radio failed")
            return super().active(value)

    ap = BoomWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(tmp_path)

    with pytest.raises(WiFiError, match="AP fallback failed"):
        connect(path=path, timeout_s=0, network_module=netmod)


def test_connect_handles_missing_ifconfig(tmp_path):
    sta = FakeWLAN(iface="sta", isconnected_after=0)
    sta.ifconfig_value = ()  # falsy / empty — IP must come back as None
    ap = FakeWLAN(iface="ap")
    netmod = _make_network_module(sta, ap)
    path = _write_creds(tmp_path)
    result = connect(path=path, timeout_s=2, network_module=netmod)
    assert result["ip"] is None


def test_default_paths_constants_exposed():
    # These must stay stable: they end up in user-facing docs and example file.
    assert wifi.DEFAULT_CRED_PATH == "wifi_credentials.json"
    assert wifi.DEFAULT_AP_SSID == "buddy-arm"
