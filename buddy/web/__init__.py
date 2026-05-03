"""Web backend for the Buddy arm.

Sub-modules
-----------
- :mod:`buddy.web.server`  : HTTP + WebSocket API around an :class:`Arm`
- :mod:`buddy.web.wifi`    : STA + AP-fallback Wi-Fi bring-up helpers
"""

from .server import create_app, ArmService, state_payload
from .wifi import connect, load_credentials, WiFiError

__all__ = [
    "create_app",
    "ArmService",
    "state_payload",
    "connect",
    "load_credentials",
    "WiFiError",
]
