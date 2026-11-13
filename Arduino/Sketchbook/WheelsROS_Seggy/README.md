Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# WheelsROS_Seggy

**Contents:**

- [What is this package for?](#what-is-this-package-for)
- [How Seggy drives](#how-seggy-drives)
- [ROS 2 serial interface](#ros-2-serial-interface)
- [Teensy 4.0 pin assignments](#teensy-40-pin-assignments)
- [Build and upload](#build-and-upload)

-------------

## What is this package for?

Seggy needs to control two powered wheels to move and steer. This firmware runs on a small computer called a Teensy 4.0. It takes driving commands from a joystick or from ROS 2 on a separate computer, then uses sensors in the wheels to track how fast they turn. For example, when one wheel turns faster than the other, Seggy steers toward the slower wheel.

## How Seggy drives

- **Startup.** The Teensy initializes the motor drivers, current sensors, and Hall sensors, then aligns the motors. The wheels can move during alignment. The buzzer gives two short startup beeps, followed by one beep for successful initialization or three longer beeps for failure. The main loop skips motor control if initialization fails.
- **Motor control.** SimpleFOC uses the three Hall signals from each wheel for closed-loop velocity control. Wheel commands are limited to ±100% and mapped to targets of up to ±3.5 rad/s. The firmware runs the motor loop as often as possible and normally updates targets about every 100 ms.
- **Joystick override.** Pulling pin 32 low gives the joystick control. Its X and Y axes are mixed into left and right wheel commands. Pin 33 reads the joystick button for diagnostics.
- **ROS 2 timeout.** If more than two seconds pass without a motor command, the firmware clears the drive setpoints. An active joystick override can still supply wheel commands. A partial serial frame can delay the timeout check because the current serial reader waits for the frame terminator.
- **Battery monitoring.** The firmware samples the battery voltage through a divider and turns on the buzzer when the measured voltage falls below 10 V.

## ROS 2 serial interface

The Teensy uses USB `Serial` with a configured baud rate of 115200. Send space-separated commands ending with a carriage return (`\r`). The main commands used for the drive base are below.

| Command | Effect |
| --- | --- |
| `m <left> <right>` | Set signed wheel speeds as percentages from −100 to 100. Positive values command forward travel. |
| `e` | Read cumulative left and right wheel counts. Seggy's ROS 2 configuration uses 60 counts per revolution. |
| `r` | Reset both wheel counts. |
| `h` | Read battery voltage in millivolts, followed by two zero fields in the current firmware. |
| `b` | Report the configured baud rate. |

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
| Buzzer | 26 (A12) |
| ROS 2 serial connection | USB |

Reserve pin 13 for right motor driver enable. The external status LED is disabled by default. To enable it on pin 27 (A13), check that the pin is free on the carrier board, then uncomment `#define STATUS_LED_PIN A13` in [WheelsROS_Seggy.ino](src/WheelsROS_Seggy.ino) and rebuild. Pin 25 (A11) is also unassigned in the sketch and can be used if it is free on the carrier board.

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

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
