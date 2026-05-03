Getting Started
===============

This guide walks you through wiring, Wi-Fi setup, deploying code to the
microcontroller, and performing first-run calibration.

Hardware Wiring
---------------

The Buddy arm uses **Feetech STS3215** serial bus servos on a single
half-duplex UART bus. You need three signal connections plus power.

UART connections
^^^^^^^^^^^^^^^^

===========  ================  ===================================
Signal       Pico W pin        Description
===========  ================  ===================================
TX           GPIO 0            UART0 TX to tri-state buffer input
RX           GPIO 1            UART0 RX from tri-state buffer
DIR          GPIO 2            Direction pin (HIGH = transmit)
===========  ================  ===================================

Tri-state buffer
^^^^^^^^^^^^^^^^

A tri-state buffer (e.g. SN74LVC2G241) switches the bus between transmit
and receive. The direction pin controls this:

* **HIGH** -- transmit enabled (Pico drives the bus)
* **LOW** -- receive enabled (Pico listens for servo replies)

This matches the Waveshare STS bus servo driver board and most community
schematics.

Power
^^^^^

* Servos require **6--7.4 V** DC (2S LiPo or a regulated 6 V supply).
* **Do not power servos from the Pico's 3.3 V or VBUS rail.** Use a
  separate servo power supply with a common ground to the Pico.
* Each STS3215 can draw up to 1.5 A under load; budget 10 A for a
  6-servo arm.

Wi-Fi Setup
-----------

Create a file called ``wifi_credentials.json`` on the microcontroller
flash with your network details:

.. code-block:: json

    {
      "ssid": "your-wifi-ssid",
      "password": "your-wifi-password",
      "hostname": "buddy",
      "ap_fallback": {
        "ssid": "buddy-arm",
        "password": "buddy1234"
      }
    }

Only ``ssid`` and ``password`` are required. If the Pico cannot join your
network within 15 seconds it will create its own access point using the
``ap_fallback`` credentials (default SSID ``buddy-arm``, password
``buddy1234``).

A template is provided in the repository as
``wifi_credentials.example.json``.

Deployment
----------

Use ``mpremote`` to copy the project files to the microcontroller:

.. code-block:: bash

    # Install mpremote if you haven't already
    pip install mpremote

    # Copy the driver, boot script, and package tree
    mpremote cp sts3215.py :
    mpremote cp main.py :
    mpremote cp -r buddy :

    # Copy your Wi-Fi credentials
    mpremote cp wifi_credentials.json :

    # Reset the board to start
    mpremote reset

The ``main.py`` boot script will:

1. Connect to Wi-Fi (or start an AP).
2. Initialise the STS3215 driver and Arm.
3. Load calibration from ``calibration.json`` (if present).
4. Home the arm.
5. Start the web server on port 80.
6. Fall back to the CLI REPL if the web server fails.

First-Run Calibration
---------------------

After deploying and powering on:

1. **Disable torque** so you can move the arm by hand::

       buddy> torque off

2. **Position each joint** at the desired home (centre) position by
   physically moving the arm.

3. **Run the calibrate command**::

       buddy> calibrate

   This reads the current joint angles, saves them as the new home
   positions in ``calibration.json`` on flash, and prints a confirmation.

4. **Re-enable torque** and home the arm::

       buddy> torque on
       buddy> home

On subsequent boots the saved calibration is loaded automatically.

You can also trigger calibration from the web console by posting the
``calibrate`` command to the ``POST /cli`` endpoint.
