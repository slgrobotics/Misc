Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# WheelsROS_Dragger

**Contents:**

- [What is this package for?](#what-is-this-package-for)
- [How Dragger drives](#how-dragger-drives)
- [Mega 2560 pin assignments](#mega-2560-pin-assignments)
- [ROS 2 serial protocol](#ros-2-serial-protocol)
- [Build and upload](#build-and-upload)
- [Bench tests](#bench-tests)

-------------

## What is this package for?

Dragger needs to turn two powered wheels to move and steer. This program runs on an Arduino Mega 2560 and takes driving commands from a joystick or from ROS 2 on a separate computer. Sensors on the wheels tell the Arduino how fast they are turning. For example, a command to turn in place can make one wheel go forward and the other go backward.

## How Dragger drives

- **Speed control.** The Arduino counts pulses from the wheel encoders and adjusts motor power using feedback. It smooths speed commands and updates the motor outputs on a nominal 100 ms cycle. The IBT-2 motor drivers receive PWM signals that control how much power goes to each wheel.
- **Joystick override.** Pulling pin 22 LOW gives the joystick control. Releasing override clears the motor outputs until a fresh serial drive command arrives.
- **Serial stop timeout.** After two seconds without a motor command, the firmware clears the PWM output when joystick override is inactive. This applies to closed-loop speed and raw PWM commands.
- **Telemetry.** The firmware reads battery voltage, battery current, charger status, and parking sonar data. Wheel counts wrap as signed 32-bit values on the serial link.

## Mega 2560 pin assignments

| Signal | Left wheel | Right wheel |
| --- | --- | --- |
| Encoder channels A, B | D3, D9 | D2, D8 |
| IBT-2 enable | D5 | D4 |
| IBT-2 RPWM, LPWM | D6, D7 | D44, D45 |

| Signal | Mega pin |
| --- | --- |
| Joystick X, Y | A0, A1 |
| Joystick override, button | D22, D24 |
| Battery voltage sense | A4 |
| Battery current sense | A5 |
| Charger status, active low | D37 |
| Buzzer | D13 |
| Diagnostic LEDs, red, green, blue | D33, D35, D31 |
| Parking sonar I²C, SDA and SCL | D20, D21 |
| ROS 2 serial | USB via Serial (UART0) |

D37 uses the Mega’s internal pull-up. Connect the charger’s open-collector output to D37 with a shared ground. The output pulls D37 LOW while charging and releases it otherwise. The output must tolerate the Mega’s 5 V pull-up.

## ROS 2 serial protocol

The firmware uses USB serial at 115200 baud. Commands and replies end with a carriage return (`\r`). Replies begin with a command prefix and a space.

| Command | Behavior |
| --- | --- |
| Empty frame | Return `* OK` for the startup handshake. |
| `m <left> <right>` | Set signed wheel speed targets from −100% to +100%. |
| `o <left> <right>` | Apply signed raw PWM from −255 to +255. |
| `e` | Read left and right encoder counts. Dragger’s ROS configuration uses 13,730 counts per revolution. |
| `r` | Reset both encoder counters. |
| `h` | Read millivolts, measured milliamps, free memory bytes, and the charging flag (1 while charging, 0 otherwise). |
| `p` | Read the four parking sonar values. |
| `b` | Report the configured baud rate. |
| `u <p>:<d>:<i>:<o>` | Acknowledge ROS startup. The firmware uses its compiled PID gains. |

Use a [diffdrive_arduino driver](https://github.com/slgrobotics/diffdrive_arduino) version that accepts four-field health replies. Dragger’s ROS hardware settings enable `has_battery_charging_field` and `battery_current_measured`. ROS reports `FULL` at or above `battery_full_voltage` (14.5 V by default). Below that threshold it uses the charger flag when present. Older three-field firmware uses the driver’s legacy battery logic.

## Build and upload

Open this directory in VS Code with the PlatformIO extension. The [PlatformIO configuration](platformio.ini) targets the Arduino Mega 2560 and pins PID 1.2.1 and digitalWriteFast 1.3.1.

```sh
pio run -e megaatmega2560
pio run -e megaatmega2560 -t upload
```

The equivalent VS Code tasks are **Build** and **Upload** under **PlatformIO → Project Tasks → megaatmega2560**. The build generates `.pio/build/megaatmega2560/firmware.hex`. Leave `TRACE` disabled for ROS and bench tests because diagnostics share the USB serial stream.

## Bench tests

Lift both drive wheels clear of the bench, switch off joystick override, and close the serial monitor. Run one test at a time so only one process owns the Mega’s serial port.

For the [serial test](test/serial_bench.py), install pySerial for Python 3.10 or later. It checks framing, recovery from malformed commands, health, encoder reset, wheel directions, raw PWM, and watchdog stops.

```sh
python3 test/serial_bench.py --port /dev/ttyACM0
```

Pass `--expect-charging 0` with D37 released or `--expect-charging 1` with the charger output pulling it LOW.

For the ROS 2 test, source ROS and the overlays containing `articubot_one` and the updated `diffdrive_arduino` in both terminals. The local ROS 2 Lyrical setup uses:

```sh
source /opt/ros/lyrical/setup.bash
source ~/robot_ws/install/local_setup.bash
source ~/tmp_ws/install/local_setup.bash
```

Start the [bench launch](test/ros2_bench.launch.py) in the first terminal. It loads Dragger’s real robot description and `~/robot_ws/src/articubot_one/robots/dragger/config/hardware.yaml`.

```sh
ros2 launch ./test/ros2_bench.launch.py
```

Once both controllers are active, run the [ROS test](test/ros2_bench.py) in the second terminal. It checks joint feedback for both wheel directions and confirms the wheels settle after zero commands. The default test requests full firmware command. Use `--percent 60` for a lower command.

```sh
python3 test/ros2_bench.py
```

Stop the launch after testing.

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
