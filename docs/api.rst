API Reference
=============

This page documents the public API of every module in the Buddy project.

STS3215 Driver (``sts3215``)
----------------------------

The low-level driver for Feetech STS3215 serial bus servos.

.. module:: sts3215

Helpers
^^^^^^^

.. function:: degrees_to_position(deg)

   Map 0--360 degrees to 0--4095 servo ticks. Clamps out-of-range input.

.. function:: position_to_degrees(pos)

   Map 0--4095 servo ticks to 0--360 degrees. Clamps out-of-range input.

.. class:: ServoError

   Raised on any protocol failure: timeout, bad checksum, wrong-ID reply,
   malformed packet, or non-zero servo error byte.

.. class:: STS3215(uart, direction_pin=None, timeout_ms=50)

   Driver for one or more STS3215 servos on a single half-duplex UART bus.

   .. method:: ping(servo_id)

      Return ``True`` if the servo replies, ``False`` on timeout.

   .. method:: read_position(servo_id)

      Return the present position (0--4095).

   .. method:: write_position(servo_id, position, speed=0, accel=0)

      Move servo to *position* at *speed* with *accel*.

   .. method:: enable_torque(servo_id)

      Enable torque on the given servo.

   .. method:: disable_torque(servo_id)

      Disable torque on the given servo.

   .. method:: set_id(servo_id, new_id)

      Change a servo's bus ID (requires EEPROM unlock).

   .. method:: read_temperature(servo_id)

      Return the present temperature in degrees Celsius.

   .. method:: read_load(servo_id)

      Return the signed present load (-1023 to +1023).


Arm (``buddy.arm``)
--------------------

.. module:: buddy.arm

.. class:: JointConfig(servo_id, min_angle=0.0, max_angle=360.0, home=180.0, sign=1, offset=0.0, is_gripper=False, open_angle=None, close_angle=None)

   Static metadata for one servo joint.

   .. classmethod:: from_dict(d)

      Build a :class:`JointConfig` from a plain dict (e.g. JSON-decoded).

.. data:: DEFAULT_JOINT_CONFIGS

   Default 6-joint configuration dict (base, shoulder, elbow, wrist,
   wrist_rot, gripper).

.. class:: Arm(driver, joint_configs=None, default_speed=1000, default_accel=50)

   A coordinated 6-servo arm sharing one half-duplex UART bus.

   .. method:: move_joint(key, angle_deg, speed=None, accel=None)

      Move one joint to *angle_deg* (clamped to limits).

   .. method:: move_all(angles, speed=None, accel=None)

      Move every joint simultaneously via sync-write.

   .. method:: read_all()

      Return a list of present angles for each joint.

   .. method:: home(speed=None, accel=None)

      Move every joint to its configured home angle.

   .. method:: enable_torque(key="all")

      Enable torque on one or all joints.

   .. method:: disable_torque(key="all")

      Disable torque on one or all joints.

   .. method:: gripper_open(speed=None, accel=None)

      Drive the gripper to its open angle.

   .. method:: gripper_close(speed=None, accel=None)

      Drive the gripper to its close angle.

   .. attribute:: joint_names

      List of joint name strings.


Motion Controller (``buddy.motion``)
-------------------------------------

.. module:: buddy.motion

.. class:: MotionController(arm, tick_hz=50, max_velocity=180.0, max_acceleration=720.0, time_func=None)

   Non-blocking trajectory generator for an Arm.

   .. method:: move_to(target_angles, duration)

      Plan a linear move. Returns the (possibly stretched) actual duration.

   .. method:: is_moving()

      ``True`` while a trajectory is progressing.

   .. method:: wait()

      Awaitable: resolves when the current move finishes.

   .. method:: start()

      Spawn the background tick task.

   .. method:: stop()

      Stop the tick task.

   .. method:: tick()

      One trajectory tick (public for deterministic testing).


Kinematics (``buddy.kinematics``)
----------------------------------

.. module:: buddy.kinematics

.. data:: DEFAULT_LINKS

   Default link lengths (mm) for the 5-DoF arm chain.

.. function:: dh_table(links=DEFAULT_LINKS)

   Return a classical Denavit-Hartenberg parameter table.

.. function:: forward(joint_angles, links=DEFAULT_LINKS)

   Forward kinematics: joint angles (5 floats, degrees) to Cartesian pose
   ``(x, y, z, tool_pitch_deg, tool_roll_deg)``.

.. function:: inverse(target_pose, links=DEFAULT_LINKS, joint_configs=None, elbow_up=True)

   Inverse kinematics: Cartesian pose to 5 joint angles (degrees).

.. class:: KinematicsError

   Raised when a pose is unreachable or a joint limit is violated.


Calibration (``buddy.calibration``)
------------------------------------

.. module:: buddy.calibration

.. function:: capture(arm)

   Read current joint angles and return a calibration dict.

.. function:: save(calibration_data, path="calibration.json")

   Persist calibration data to a JSON file.

.. function:: load(path="calibration.json")

   Load calibration data from a JSON file. Returns ``None`` if the file
   does not exist.

.. function:: apply_calibration(arm, calibration_data)

   Apply loaded calibration to an Arm (updates joint configs in-place).

.. function:: calibrate(arm, path="calibration.json")

   One-shot convenience: capture + save.

.. class:: CalibrationError

   Raised on save/load failures.


Safety Monitor (``buddy.safety``)
----------------------------------

.. module:: buddy.safety

.. class:: SafetyMonitor(arm, driver, temp_limit=70, load_limit=900, warn_fraction=0.8, on_warn=None)

   Per-tick safety checker for an STS3215-based arm.

   .. method:: check()

      Run one safety check. Returns ``True`` if safe, ``False`` if
      soft-stop was triggered.

   .. method:: reset()

      Clear the trip state.

   .. attribute:: tripped

      ``True`` after a soft-stop has fired.

   .. attribute:: trip_reason

      String describing why the last soft-stop fired.

   .. attribute:: temp_limit

      Temperature threshold (degrees Celsius, read/write).

   .. attribute:: load_limit

      Load threshold (absolute magnitude 0--1023, read/write).


CLI (``buddy.cli``)
--------------------

.. module:: buddy.cli

.. function:: dispatch(line, arm, kinematics=None)

   Parse and execute a single CLI command string. Returns a
   human-readable result.

.. function:: repl(arm, kinematics=None)

   Interactive REPL loop for the MicroPython console.

Available commands:

* ``move J<n> <deg>`` -- move a joint
* ``pose <x> <y> <z> [pitch roll]`` -- IK move
* ``home`` -- drive all joints home
* ``torque on|off [J<n>]`` -- enable/disable torque
* ``grip open|close`` -- gripper control
* ``read`` -- print joint angles and temperatures
* ``calibrate`` -- capture current pose and save to flash
* ``help`` -- show help


Web Server (``buddy.web.server``)
----------------------------------

.. module:: buddy.web.server

.. function:: create_app(arm, pose_to_joints=None, stream_hz=20, cli_dispatch=None)

   Build a Microdot app with all REST and WebSocket endpoints.

   Returns ``(app, service)`` where *service* is an
   :class:`ArmService` instance.

.. function:: run(arm, host="0.0.0.0", port=80, **kwargs)

   Convenience wrapper: build the app and start the blocking event loop.

Endpoints:

* ``GET /state`` -- JSON snapshot of joint angles, gripper, temperatures.
* ``POST /move`` -- joint-space or Cartesian move.
* ``POST /torque`` -- enable/disable torque.
* ``POST /gripper`` -- open/close gripper.
* ``POST /home`` -- home the arm.
* ``POST /cli`` -- execute a CLI command via the web console.
* ``GET /ws`` -- WebSocket stream of joint state.


Wi-Fi (``buddy.web.wifi``)
----------------------------

.. module:: buddy.web.wifi

.. function:: connect(path="wifi_credentials.json", timeout_s=15, network_module=None)

   Bring up Wi-Fi: STA mode first, AP fallback on timeout.

   Returns ``{"mode": "sta"|"ap", "ssid": str, "ip": str|None, "wlan": iface}``.

.. function:: load_credentials(path="wifi_credentials.json")

   Load and validate the Wi-Fi credentials JSON file.

.. class:: WiFiError

   Raised when both STA and AP bring-up fail.
