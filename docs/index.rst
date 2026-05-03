Buddy Arm Documentation
=======================

Welcome to the **Buddy Arm** documentation. Buddy is a MicroPython-based
robot arm controller for 6-servo Feetech STS3215 arms, designed to run on
RP2040/Pico W and ESP32 microcontrollers.

Features
--------

* **STS3215 driver** -- low-level half-duplex UART protocol for Feetech
  serial bus servos (ping, position read/write, sync-write, temperature,
  load).
* **Arm abstraction** -- joint-space control with per-joint sign/offset
  transforms, coordinated sync-write motion, gripper helpers.
* **Motion controller** -- async trajectory generator with velocity and
  acceleration limiting, layered on top of the Arm.
* **Inverse kinematics** -- closed-form geometric IK for the 5-DoF
  kinematic chain (base yaw, shoulder pitch, elbow pitch, wrist pitch,
  wrist roll).
* **Web interface** -- HTTP + WebSocket API via microdot for remote
  control and real-time joint state streaming.
* **CLI** -- text command interface usable from the MicroPython REPL or
  the web console.
* **Calibration** -- capture current joint angles as home positions, save
  to JSON on flash, auto-load on boot.
* **Safety monitor** -- soft-stop on over-temperature or over-load with
  configurable thresholds and warnings.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   getting_started
   api
   troubleshooting
