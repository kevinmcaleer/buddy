# Buddy Arm

A MicroPython-based 6-DOF robot arm controller for Feetech STS3215 serial bus servos. Buddy provides everything you need to drive a hobby-class robot arm from a Raspberry Pi Pico W or ESP32: low-level servo communication, coordinated multi-joint motion, analytical inverse kinematics, a built-in web interface with real-time WebSocket streaming, a text CLI, calibration persistence, and safety monitoring.

## Features

- **STS3215 servo driver** -- full Feetech serial protocol implementation (ping, read/write position, temperature, load, ID reassignment, torque control)
- **6-joint arm abstraction** -- per-joint sign/offset transforms, soft limits, sync-write for coordinated motion
- **Analytical inverse kinematics** -- closed-form geometric IK for the 5-DoF positioning chain (base yaw, shoulder, elbow, wrist pitch, wrist roll) with DH parameter table export
- **Forward kinematics** -- compute tool-tip Cartesian pose from joint angles
- **Time-parameterised motion controller** -- async trajectory generator with configurable velocity/acceleration limits and linear interpolation at 50 Hz
- **Web server** -- HTTP + WebSocket backend (via microdot) with endpoints for state queries, joint/Cartesian moves, torque control, gripper, and a 20 Hz WebSocket stream for a 3D viewer
- **Wi-Fi management** -- STA mode with automatic AP fallback, JSON-based credential storage
- **Text CLI** -- command interface usable from the MicroPython REPL or over the network (`move`, `pose`, `home`, `torque`, `grip`, `read`, `calibrate`)
- **Calibration** -- capture current joint positions as home/limits, persist to flash, auto-load on boot
- **Safety monitor** -- per-tick temperature and load checking with configurable thresholds, automatic torque-disable soft-stop, and warning callbacks
- **Sphinx documentation** -- API reference and getting-started guide in `docs/`

## Hardware Requirements

| Component | Notes |
|-----------|-------|
| Microcontroller | Raspberry Pi Pico W, ESP32, or any MicroPython board with UART + Wi-Fi |
| Servos | 6x Feetech STS3215 serial bus servos |
| Bus driver | Half-duplex UART adapter with tri-state buffer (e.g. SN74LVC2G241) or Waveshare servo driver board |
| Power | 6-7 V supply capable of 2 A+ for the servo bus |
| Wiring | Servo bus daisy-chain; UART TX/RX to the bus driver; one GPIO for direction control |

Default pin assignments (editable in `main.py`):

| Function | Pin |
|----------|-----|
| UART TX | GP0 |
| UART RX | GP1 |
| Direction (TX/RX select) | GP2 |

## Software Requirements

- **On-device**: MicroPython firmware (1.20+) with `network` support. Install microdot: `mip install microdot`
- **For development/testing**: Python 3.10+, pip

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/kevinmcaleer/buddy.git
cd buddy
```

### 2. Flash MicroPython firmware

Download the appropriate MicroPython `.uf2` or `.bin` for your board from [micropython.org](https://micropython.org/download/) and flash it.

### 3. Configure Wi-Fi

Copy the example credentials file and fill in your network details:

```bash
cp wifi_credentials.example.json wifi_credentials.json
```

Edit `wifi_credentials.json`:

```json
{
  "ssid": "your-wifi-ssid",
  "password": "your-wifi-password",
  "hostname": "buddy",
  "ap_fallback": {
    "ssid": "buddy-arm",
    "password": "buddy1234"
  }
}
```

### 4. Deploy to the microcontroller

Using `mpremote`:

```bash
mpremote cp sts3215.py :
mpremote cp -r buddy :buddy
mpremote cp main.py :
mpremote cp wifi_credentials.json :
```

Or install microdot on-device:

```bash
mpremote mip install microdot
```

### 5. Boot

Reset the board. Buddy will:

1. Connect to Wi-Fi (or start an AP named `buddy-arm`)
2. Initialise the servo bus and arm
3. Load any saved calibration
4. Home the arm
5. Start the web server on port 80

Navigate to `http://buddy.local` (or the IP printed on the console) to access the web interface.

### 6. CLI usage (over USB serial)

If Wi-Fi is unavailable, Buddy drops into a text REPL:

```
buddy> help
buddy> read
buddy> move J1 90
buddy> home
buddy> grip open
buddy> pose 150 0 200 -45 0
```

## Project Structure

```
buddy/
├── sts3215.py              # Low-level STS3215 servo driver
├── main.py                 # On-device boot entry point
├── wifi_credentials.example.json
├── requirements-dev.txt    # Dev/test dependencies
├── buddy/
│   ├── __init__.py         # Package exports
│   ├── arm.py              # 6-joint Arm class (sync-write, gripper, torque)
│   ├── kinematics.py       # Forward/inverse kinematics (analytical)
│   ├── motion.py           # Async trajectory controller
│   ├── calibration.py      # Capture/save/load calibration to flash
│   ├── safety.py           # Temperature + load safety monitor
│   ├── cli.py              # Text command parser and REPL
│   └── web/
│       ├── __init__.py
│       ├── server.py       # HTTP + WebSocket API (microdot)
│       └── wifi.py         # Wi-Fi STA/AP connection helper
├── tests/                  # pytest test suite (runs on CPython)
│   ├── conftest.py         # MicroPython module mocks
│   ├── test_sts3215.py
│   ├── test_arm.py
│   ├── test_kinematics.py
│   ├── test_motion.py
│   ├── test_calibration.py
│   ├── test_safety.py
│   ├── test_cli.py
│   ├── test_web_server.py
│   ├── test_wifi.py
│   └── test_main.py
└── docs/                   # Sphinx documentation
    ├── conf.py
    ├── index.rst
    ├── api.rst
    ├── getting_started.rst
    └── troubleshooting.rst
```

## Development

### Install dev dependencies

```bash
pip install -r requirements-dev.txt
```

### Run the test suite

```bash
pytest
```

### Run with coverage

```bash
pytest --cov=buddy --cov=sts3215 --cov-report=term-missing
```

Target: 80%+ line coverage.

### Build the docs

```bash
cd docs
make html
# Output in docs/_build/html/
```

## API Overview

| Module | Purpose |
|--------|---------|
| `sts3215` | Low-level UART driver for STS3215 servos (ping, read/write registers, position, temperature, load, ID management) |
| `buddy.arm` | High-level Arm class with per-joint config, sync-write coordinated motion, gripper helpers, torque control |
| `buddy.kinematics` | Analytical forward/inverse kinematics for the 5-DoF chain; DH parameter table export |
| `buddy.motion` | Async `MotionController` -- time-parameterised linear interpolation with velocity/acceleration limits |
| `buddy.calibration` | Capture, save, load, and apply joint calibration data (home positions, limits) |
| `buddy.safety` | `SafetyMonitor` -- per-tick temperature/load checking with auto soft-stop |
| `buddy.cli` | Text command parser (`dispatch`) and interactive REPL for joint moves, IK poses, calibration |
| `buddy.web.server` | HTTP REST API + WebSocket stream (microdot); endpoints: `/state`, `/move`, `/torque`, `/gripper`, `/home`, `/ws` |
| `buddy.web.wifi` | Wi-Fi bring-up with STA + AP fallback; JSON credential loading |

## Contributing

1. Fork the repository and create a feature branch.
2. Write tests for any new functionality (target 80%+ coverage).
3. Run the full test suite: `pytest`
4. Ensure docs build cleanly: `cd docs && make html`
5. Open a pull request describing the change and linking any related issues.

## License

License information will be added in a future release. Please contact the repository owner for licensing inquiries.
