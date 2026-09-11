#!/usr/bin/env python3
"""Publish a 0.3 m/s step command on /cmd_vel for PID step-response tests."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class StepCmdVelNode(Node):
    def __init__(self):
        super().__init__("step_cmd_vel_node")

        self.declare_parameter("target_v", 0.4)
        self.declare_parameter("target_w", 0.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("duration", 8.0)
        self.declare_parameter("start_delay", 0.0)
        self.declare_parameter("stop_after_duration", True)

        self._target_v = float(self.get_parameter("target_v").value)
        self._target_w = float(self.get_parameter("target_w").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._duration = float(self.get_parameter("duration").value)
        self._start_delay = float(self.get_parameter("start_delay").value)
        self._stop_after_duration = bool(self.get_parameter("stop_after_duration").value)

        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._start_time = self.get_clock().now()
        self.create_timer(1.0 / publish_rate, self._timer_cb)

        if abs(self._target_v) > 0.5:
            self.get_logger().warn(
                f"target_v={self._target_v:.3f} m/s is high for this platform. "
                "Make sure the car is lifted or in a safe test area."
            )

        self.get_logger().info(
            f"Publishing /cmd_vel step: v={self._target_v:.3f} m/s "
            f"w={self._target_w:.3f} rad/s at {publish_rate:.1f} Hz, "
            f"start_delay={self._start_delay:.2f}s duration={self._duration:.2f}s"
        )

    def _timer_cb(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        msg = Twist()
        in_step_window = elapsed >= self._start_delay
        if self._duration > 0.0:
            in_step_window = in_step_window and elapsed < self._start_delay + self._duration

        if in_step_window:
            msg.linear.x = self._target_v
            msg.angular.z = self._target_w
        elif elapsed >= self._start_delay + self._duration and self._stop_after_duration:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self._pub.publish(msg)

        if (
            self._duration > 0.0
            and self._stop_after_duration
            and elapsed >= self._start_delay + self._duration + 0.5
        ):
            self.get_logger().info("Step test finished; published stop command.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StepCmdVelNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            stop = Twist()
            for _ in range(3):
                node._pub.publish(stop)
        node.destroy_node()


if __name__ == "__main__":
    main()
