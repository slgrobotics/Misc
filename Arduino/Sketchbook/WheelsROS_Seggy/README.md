Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# WheelsROS_Seggy

**Contents:**

- [What is this code for?](#what-is-this-code-for)
- [How Seggy drives](#how-seggy-drives)
- [ROS 2 serial interface](#ros-2-serial-interface)
- [Teensy 4.0 pin assignments](#teensy-40-pin-assignments)
- [Build and upload](#build-and-upload)
- [Serial bench test](#serial-bench-test)
- [ROS 2 bench test](#ros-2-bench-test)

-------------

## What is this code for?

Seggy needs to control two powered wheels to move and steer. This firmware runs on a small computer called a Teensy 4.0. It takes driving commands from a joystick or from ROS 2 on a separate computer, then uses sensors in the wheels to track how fast they turn. For example, when one wheel turns faster than the other, Seggy steers toward the slower wheel.

Related packages:
- https://github.com/slgrobotics/diffdrive_arduino

See:
- https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn#base-control---the-arduino-way
- https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn#bill-of-materials-bom
- https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Seggy

## How Seggy drives

- **Startup.** The Teensy initializes the motor drivers, current sensors, and Hall sensors, then aligns the motors. The wheels can move during alignment. The buzzer gives two short startup beeps, followed by one beep for successful initialization or three longer beeps for failure. The main loop skips motor control if initialization fails.
- **Motor control.** SimpleFOC uses the three Hall signals from each wheel for closed-loop velocity control. Wheel commands are limited to ±100% and mapped to targets of up to ±3.5 rad/s. The firmware runs the motor loop as often as possible and normally updates targets about every 100 ms.
- **Joystick override.** Pulling pin 32 low gives the joystick control. Its X and Y axes are mixed into left and right wheel commands. Pin 33 reads the joystick button for diagnostics.
- **ROS 2 timeout.** If more than two seconds pass without a motor command, the firmware clears the drive setpoints. An active joystick override can still supply wheel commands. Releasing joystick override commands zero until a fresh ROS 2 motor command arrives.
- **Battery monitoring.** The firmware samples the battery voltage through a divider and turns on the buzzer when the measured voltage falls below 10 V.

## ROS 2 serial interface

The Teensy uses USB `Serial` with a configured baud rate of 115200. Send space-separated commands ending with a carriage return (`\r`). The receiver accepts up to 127 bytes per frame before the terminator. It rejects oversized frames and malformed wheel-speed commands. The legacy raw PWM command (`o`) returns an error.

| Command | Effect |
| --- | --- |
| `m <left> <right>` | Set signed wheel speeds as percentages from −100 to 100. Positive values command forward travel. |
| `e` | Read cumulative left and right wheel counts as signed 32-bit values that wrap at the range limits. Seggy's ROS 2 configuration uses 60 counts per revolution. |
| `r` | Reset both wheel counts. |
| `h` | Read battery voltage in millivolts, two zero fields for current and free memory, and a charging flag (0 or 1). |
| `b` | Report the configured baud rate. |

Use a driver version that accepts the fourth health field before flashing this firmware. Seggy enables `has_battery_charging_field` in its ROS hardware configuration. With the flag present, ROS publishes `CHARGING` or `NOT_CHARGING` and reports NaN for unmeasured battery current. At or above `battery_full_voltage` (14.5 V by default), ROS reports `FULL`. Below that threshold, older three-field replies use the legacy voltage heuristic.

See the [ROS 2 diffdrive_arduino driver](https://github.com/slgrobotics/diffdrive_arduino) and [serial command implementation](src/Comm.ino).

## Teensy 4.0 pin assignments

The numbers are Teensy GPIO numbers. Analog aliases appear in parentheses.

| Signal | Left wheel | Right wheel |
| --- | --- | --- |
| Hall sensors A, B, C | 2, 3, 4 | 12, 14 (A0), 15 (A1) |
| Motor driver PWM A, B, C | 5, 6, 7 | 9, 10, 11 |
| Motor driver enable | 8 | 13 |
| Phase current sense A, B, C | 20 (A6), 21 (A7), 22 (A8) | 17 (A3), 18 (A4), 19 (A5) |

| Signal | Pin |
| --- | --- |
| Joystick X | 16 (A2) |
| Joystick Y | 23 (A9) |
| Joystick override, active low | 32 |
| Joystick button, active low | 33 |
| Battery voltage divider | 24 (A10) |
| Charger status, active low | 25 (A11) |
| Buzzer | 26 (A12) |
| ROS 2 serial connection | USB |

Reserve pin 13 for right motor driver enable. The external status LED is disabled by default. To enable it on pin 27 (A13), check that the pin is free on the carrier board, then uncomment `#define STATUS_LED_PIN A13` in [WheelsROS_Seggy.ino](src/WheelsROS_Seggy.ino) and rebuild. A11 uses the Teensy internal pull-up. Connect the charger’s open-collector output to A11 with a shared ground. The transistor pulls A11 LOW while charging and releases it otherwise.

When enabled, the LED blinks at about 5 Hz during normal operation and blinks slowly if motor initialization fails.

## Build and upload

Open the directory containing [platformio.ini](platformio.ini) in VS Code with the PlatformIO extension. The `teensy40` environment uses the Arduino framework and pins SimpleFOC to 2.4.0.

From the project directory, build the release firmware with:

```sh
pio run -e teensy40
```

In VS Code, the equivalent task is **PlatformIO → Project Tasks → teensy40 → General → Build**. Confirm the terminal says `Building in release mode`. The build creates `.pio/build/teensy40/firmware.hex`. Debug and release builds use the same output path.

Connect the Teensy over USB and choose **Upload**, or run:

```sh
pio run -e teensy40 -t upload
```
The project uses the command-line Teensy loader. If the loader waits for the bootloader, press the Teensy's **Program** button once.

Uploading from VS Code:

<img alt="Upload from VS Code" src="https://github.com/user-attachments/assets/618be880-6499-4219-90db-d9e696e28423" />

## Serial bench test

Lift both wheels clear of the bench, disable joystick override, and close the PlatformIO serial monitor. Upload the firmware built without `TRACE` so the USB serial stream contains protocol replies. The test requires Python 3.10 or later and pySerial.

```sh
sss@sss:~/Misc/Arduino/Sketchbook/WheelsROS_Seggy/test$ python3 serial_bench.py --port /dev/ttyACM0
    Battery: 13271 mV
    PASS serial framing, error recovery, health and encoder reset
    PASS motion L=+100% R=+0% encoder delta=(54, 0)
    PASS motion L=-100% R=+0% encoder delta=(-55, 0)
    PASS motion L=+0% R=+100% encoder delta=(0, 55)
    PASS motion L=+0% R=-100% encoder delta=(0, -55)
    PASS motion L=+100% R=+100% encoder delta=(55, 55)
    PASS motion L=-100% R=-100% encoder delta=(-55, -54)
    PASS motion L=+100% R=-100% encoder delta=(55, -54)
    PASS motion L=-100% R=+100% encoder delta=(-57, 57)
    PASS explicit zero command
    PASS watchdog during encoder polling
    PASS watchdog with partial frame
    PASS Seggy serial bench test
    sss@sss:~/Misc/Arduino/Sketchbook/WheelsROS_Seggy/test$ 
```

The test checks serial framing and recovery, battery and encoder replies, and encoder reset. It runs each wheel in both directions at full commanded speed, checks that encoder counts follow the requested directions, and verifies explicit and watchdog stops. It sends a final zero command on exit.

## ROS 2 bench test

Lift both wheels clear of the bench, disable joystick override, close the serial monitor, and stop any other ROS bringup using the Teensy. Use firmware built without `TRACE`.

In both terminals, source your ROS installation and the overlays that provide `articubot_one` and `diffdrive_arduino`. For the local Lyrical setup:

```sh
source /opt/ros/lyrical/setup.bash
source ~/robot_ws/install/local_setup.bash
source ~/tmp_ws/install/local_setup.bash  # where diffdrive_arduino is built
```

From the project directory, start the bench controllers in the first terminal:

```sh
ros2 launch ./test/ros2_bench.launch.py
```

Once both controllers are active, run the test in the second terminal:

```sh
sss@sss:~/Misc/Arduino/Sketchbook/WheelsROS_Seggy$ python3 test/ros2_bench.py
    PASS L=+1.667 R=+0.000 rad/s, encoder delta=[51, 0]
    PASS L=-1.667 R=+0.000 rad/s, encoder delta=[-51, 0]
    PASS L=+0.000 R=+1.667 rad/s, encoder delta=[0, 50]
    PASS L=+0.000 R=-1.667 rad/s, encoder delta=[0, -51]
    PASS L=+1.667 R=+1.667 rad/s, encoder delta=[50, 52]
    PASS L=-1.667 R=-1.667 rad/s, encoder delta=[-52, -51]
    PASS L=+1.667 R=-1.667 rad/s, encoder delta=[52, -50]
    PASS L=-1.667 R=+1.667 rad/s, encoder delta=[-55, 51]
    PASS Seggy ROS 2 wheel bench test
```

The launch loads Seggy's real URDF and `~/robot_ws/src/articubot_one/robots/seggy/config/hardware.yaml`. A direct wheel controller sends joint velocities through `diffdrive_arduino`, and the test checks the encoder-derived joint positions. By default it exercises both directions at 100% firmware command, one wheel at a time and together, then checks that feedback settles after zero commands. Use `--percent 60` for a lower command. To use a different hardware YAML, pass the same path to `hardware_config:=` in the launch and `--hardware-config` in the test. Shut down the launch after testing.

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
