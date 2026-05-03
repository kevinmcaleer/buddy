Troubleshooting
===============

This page covers common issues and their solutions.

Servo not responding (``ServoError: timeout``)
----------------------------------------------

**Symptoms:** ``ServoError: timeout waiting for response`` when
reading position, pinging, or writing to a servo.

**Possible causes:**

1. **Wrong baud rate.** STS3215 servos default to 1 Mbaud. Ensure
   ``UART(baudrate=1_000_000)`` matches.

2. **Direction pin not wired or inverted.** The driver expects
   HIGH = transmit, LOW = receive. If your buffer has opposite polarity,
   wrap the pin or check your schematic.

3. **Servo ID mismatch.** New servos ship with ID 1. If you have
   multiple servos with the same ID, only one will answer and collisions
   corrupt the bus.

4. **Power.** Servos need 6--7.4 V. Under-voltage causes erratic
   behaviour or no response at all.

**Steps:**

* Test with a single servo first.
* Use ``driver.ping(servo_id)`` to verify each ID.
* Check TX/RX wiring with an oscilloscope or logic analyser.

Wi-Fi won't connect
--------------------

**Symptoms:** The board falls back to AP mode or prints
``Wi-Fi failed``.

**Possible causes:**

1. **Missing credentials file.** ``wifi_credentials.json`` must exist on
   flash (not just in the repo).

2. **Wrong SSID or password.** Double-check for trailing spaces.

3. **2.4 GHz only.** The Pico W and ESP32 do not support 5 GHz networks.

**Steps:**

* Verify the file is on flash: ``mpremote ls``
* Try AP mode: connect to ``buddy-arm`` (password ``buddy1234``) at
  ``192.168.4.1``.

Calibration not loading
------------------------

**Symptoms:** The arm homes to default positions even after running
``calibrate``.

**Possible causes:**

1. **File not on flash.** ``calibration.json`` must be in the root of
   the flash filesystem.

2. **Corrupt JSON.** A power loss during save can truncate the file.
   Delete it and re-calibrate.

**Steps:**

* Check the file exists: ``mpremote ls``
* Inspect its contents: ``mpremote cat calibration.json``
* Delete and re-calibrate if corrupt.

Arm goes limp unexpectedly (safety soft-stop)
----------------------------------------------

**Symptoms:** Torque is disabled on all joints mid-motion.

**Cause:** The safety monitor detected a temperature or load reading
above the configured threshold. The default limits are:

* Temperature: 70 degrees Celsius
* Load: 900 (out of 1023)

**Steps:**

* Let the servos cool down before re-enabling torque.
* Check for mechanical binding that would cause high load.
* If the thresholds are too aggressive for your use case, adjust them
  in ``main.py`` or via the :class:`~buddy.safety.SafetyMonitor` API.

Web server not starting
------------------------

**Symptoms:** ``Web server failed`` in the boot log, but CLI works.

**Possible causes:**

1. **microdot not installed.** On the microcontroller, install with::

       import mip
       mip.install("microdot")

2. **Port 80 in use.** Another process may be binding port 80. Try a
   different port in ``main.py``.

3. **Out of memory.** The Pico W has limited RAM. Reduce
   ``stream_hz`` or remove unused modules.

IK errors (``KinematicsError``)
--------------------------------

**Symptoms:** ``target too far`` or ``target too close`` when using the
``pose`` command.

**Cause:** The requested Cartesian position is outside the arm's
reachable workspace.

**Steps:**

* Check the link lengths in :data:`buddy.kinematics.DEFAULT_LINKS`
  against your physical arm.
* Use the ``elbow_up=False`` option to try the alternative IK branch.
* Verify units: all distances are in **millimetres**.
