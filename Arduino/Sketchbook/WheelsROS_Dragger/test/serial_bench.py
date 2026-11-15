#!/usr/bin/env python3
"""Exercise the Dragger drive protocol with both wheels off the ground."""

import argparse
import time

import serial

"""

sss@sss:~/Misc/Arduino/Sketchbook/WheelsROS_Dragger/test$ python3 serial_bench.py --port /dev/ttyACM0
Battery: 13194 mV, 91 mA, charging=False
PASS serial framing, error recovery, health and encoder reset
PASS motion L=+100% R=+0% encoder delta=(8272, 0)
PASS motion L=-100% R=+0% encoder delta=(-8278, 0)
PASS motion L=+0% R=+100% encoder delta=(0, 8031)
PASS motion L=+0% R=-100% encoder delta=(0, -8414)
PASS motion L=+100% R=+100% encoder delta=(8345, 7983)
PASS motion L=-100% R=-100% encoder delta=(-8237, -8293)
PASS motion L=+100% R=-100% encoder delta=(8360, -8253)
PASS motion L=-100% R=+100% encoder delta=(-8300, 8041)
PASS motion L=+180 PWM R=+0 PWM encoder delta=(7058, 0)
PASS motion L=+0 PWM R=-180 PWM encoder delta=(0, -6957)
PASS explicit zero command
PASS watchdog during encoder polling
PASS watchdog with partial frame
PASS Dragger serial bench test
sergei@sergeiu7:~/Misc/Arduino/Sketchbook/WheelsROS_Dragger/test$ 

 

"""
def expect(condition, message):
    if not condition:
        raise RuntimeError(message)


class Bench:
    def __init__(self, port, expected_charging):
        self.port = port
        self.expected_charging = expected_charging

    def reply(self, frame):
        raw = self.port.read_until(b"\r", size=128)
        expect(raw.endswith(b"\r"), f"{frame!r}: incomplete reply: {raw!r}")
        return raw[:-1].decode("ascii")

    def request(self, frame, prefix):
        self.port.write(frame + b"\r")
        reply = self.reply(frame)
        expect(reply.startswith(prefix), f"{frame!r}: unexpected reply {reply!r}")
        return reply

    def counts(self):
        return tuple(map(int, self.request(b"e", "e ").split()[1:]))

    def drive(self, left, right):
        self.request(f"m {left} {right}".encode(), "m OK")

    def protocol(self):
        self.drive(0, 0)
        expect(self.request(b"b", "b ") == "b 115200", "Unexpected baud reply")
        health = list(map(int, self.request(b"h", "h ").split()[1:]))
        expect(len(health) == 4 and health[0] > 0 and health[3] in (0, 1),
               f"Invalid health reply: {health}")
        print(f"Battery: {health[0]} mV, {health[1]} mA, charging={bool(health[3])}")
        if self.expected_charging is not None:
            expect(health[3] == self.expected_charging, f"Wrong charging flag: {health[3]}")
        self.request(b"u 30:20:0:100", "u OK")
        self.request(b"o 0 0", "o OK")
        self.request(b"", "* OK")
        for invalid in (b"?", b"mm", b"m", b"m 0", b"m x 0", b"m 0 0 extra",
                        b"m 999999999999999999999 0", b"m 0\x00 0",
                        b"m" + b" 0" * 25, b"x" * 128, b"o 0 bad"):
            self.request(invalid, "? Error:")
            expect(self.request(b"b", "b ") == "b 115200", "Parser failed to recover")

        self.port.write(b"m 0")
        expect(self.port.read(1) == b"", "Partial frame produced a response")
        self.request(b" 0", "m OK")
        self.request(b"r", "r OK")
        expect(self.counts() == (0, 0), "Encoder reset failed or a wheel is moving")
        print("PASS serial framing, error recovery, health and encoder reset")

    def motion(self, left, right, raw=False):
        before = self.counts()
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            if raw:
                self.request(f"o {left} {right}".encode(), "o OK")
            else:
                self.drive(left, right)
            time.sleep(0.2)
        self.drive(0, 0)
        time.sleep(0.75)
        after = self.counts()
        delta = tuple(b - a for a, b in zip(before, after, strict=True))
        for side, command, ticks in zip(("left", "right"), (left, right), delta):
            if command:
                expect(ticks * command > 0 and abs(ticks) >= 3,
                       f"{side}: wrong direction or insufficient motion, delta={delta}")
            else:
                expect(abs(ticks) <= 2, f"{side}: inactive wheel moved, delta={delta}")
        units = " PWM" if raw else "%"
        print(f"PASS motion L={left:+d}{units} R={right:+d}{units} encoder delta={delta}")

    def settled(self, label):
        first = self.counts()
        time.sleep(0.5)
        second = self.counts()
        expect(all(abs(b - a) <= 1 for a, b in zip(first, second, strict=True)),
               f"{label}: wheels still moving {first} → {second}")
        print(f"PASS {label}")

    def watchdog(self, partial):
        before = self.counts()
        self.drive(60, 60)
        time.sleep(0.7)
        moving = self.counts()
        expect(all(b > a for a, b in zip(before, moving, strict=True)),
               "Both wheels must move before watchdog test")
        if partial:
            self.port.write(b"m 0")
            time.sleep(3)
            # Finish with an invalid frame so it cannot issue an explicit stop.
            self.request(b" bad", "? Error:")
        else:
            # Encoder traffic must not renew the motor command timeout.
            end = time.monotonic() + 3
            while time.monotonic() < end:
                self.counts()
                time.sleep(0.2)
        self.settled("watchdog with partial frame" if partial else "watchdog during encoder polling")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--expect-charging", type=int, choices=(0, 1))
    args = parser.parse_args()
    try:
        with serial.Serial(args.port, 115200, timeout=0.5, write_timeout=0.5,
                           exclusive=True) as port:
            time.sleep(2)  # USB serial open may reset the Mega.
            port.reset_output_buffer()
            bench = Bench(port, args.expect_charging)
            port.reset_input_buffer()
            try:
                bench.protocol()
                for speeds in ((100, 0), (-100, 0), (0, 100), (0, -100),
                               (100, 100), (-100, -100), (100, -100), (-100, 100)):
                    bench.motion(*speeds)
                bench.motion(180, 0, raw=True)
                bench.motion(0, -180, raw=True)
                bench.settled("explicit zero command")
                bench.watchdog(partial=False)
                bench.watchdog(partial=True)
            finally:
                # Terminate any unfinished frame, then send a complete stop.
                try:
                    port.write(b"\rm 0 0\r")
                    port.flush()
                except (OSError, serial.SerialException) as error:
                    print(f"Could not send final stop: {error}")
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f"FAIL: {error}\n")
    print("PASS Dragger serial bench test")


if __name__ == "__main__":
    main()
