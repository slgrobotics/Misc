#!/usr/bin/env python3
"""Check Dragger wheel commands and feedback through ros2_control."""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml


class Bench(Node):
    def __init__(self):
        super().__init__("dragger_wheel_bench")
        self.commands = self.create_publisher(
            Float64MultiArray, "/dragger_bench/wheel_velocity/commands", 10
        )
        self.create_subscription(JointState, "/dragger_bench/joint_states", self.feedback, 10)
        self.sample = None

    def feedback(self, msg):
        wheels = dict(zip(msg.name, msg.position))
        left = wheels.get("left_wheel_joint")
        right = wheels.get("right_wheel_joint")
        if left is not None and right is not None and all(map(math.isfinite, (left, right))):
            self.sample = (time.monotonic(), (left, right))

    def drive(self, left, right, seconds):
        message = Float64MultiArray(data=[float(left), float(right)])
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.commands.publish(message)
            cycle_end = min(time.monotonic() + 0.05, end)
            while rclpy.ok() and (remaining := cycle_end - time.monotonic()) > 0:
                rclpy.spin_once(self, timeout_sec=remaining)

    def positions(self):
        if self.sample is None or time.monotonic() - self.sample[0] > 0.5:
            raise RuntimeError("Missing fresh wheel positions from joint_states")
        return self.sample[1]

    def check_motion(self, left, right, radians_per_tick):
        before = self.positions()
        self.drive(left, right, 1.5)
        self.drive(0, 0, 0.75)
        after = self.positions()
        ticks = [round((b - a) / radians_per_tick) for a, b in zip(before, after)]
        for command, count in zip((left, right), ticks):
            if command and (command * count <= 0 or abs(count) < 3):
                raise RuntimeError(f"Wrong direction or insufficient motion: {ticks}")
            if not command and abs(count) > 2:
                raise RuntimeError(f"Idle wheel moved: {ticks}")
        self.drive(0, 0, 0.5)
        if any(abs(b - a) > 1.1 * radians_per_tick for a, b in zip(after, self.positions())):
            raise RuntimeError("Wheel feedback continued to change after a zero command")
        print(f"PASS L={left:+.3f} R={right:+.3f} rad/s, encoder delta={ticks}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config", type=Path,
        default=Path.home() / "robot_ws/src/articubot_one/robots/dragger/config/hardware.yaml",
    )
    parser.add_argument("--percent", type=float, default=100, help="firmware command magnitude")
    args = parser.parse_args()
    if not 0 < args.percent <= 100:
        parser.error("--percent must be greater than zero and at most 100")
    config = yaml.safe_load(args.hardware_config.read_text())
    speed = args.percent / float(config["wheels_pwm_factor"])
    radians_per_tick = math.tau / int(config["enc_counts_per_rev"])

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    bench = Bench()
    try:
        bench.drive(0, 0, 2)
        if not bench.commands.get_subscription_count():
            raise RuntimeError("Wheel controller is unavailable")
        bench.positions()
        for left, right in ((1, 0), (-1, 0), (0, 1), (0, -1),
                            (1, 1), (-1, -1), (1, -1), (-1, 1)):
            bench.check_motion(left * speed, right * speed, radians_per_tick)
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted\n")
    except RuntimeError as error:
        parser.exit(1, f"FAIL: {error}\n")
    finally:
        if rclpy.ok():
            bench.drive(0, 0, 0.25)
        bench.destroy_node()
        rclpy.try_shutdown()
    print("PASS Dragger ROS 2 wheel bench test")


if __name__ == "__main__":
    main()
