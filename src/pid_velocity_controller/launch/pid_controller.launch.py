"""
pid_controller.launch.py

Launches the PID velocity controller node on Raspberry Pi.

WARNING: Do NOT launch this together with motor_controller/motor.launch.py.
         Both packages drive the same GPIO pins (ENA/ENB/IN1-IN4).
         Use ONE of the two motor control strategies at a time:
           - motor_driver_node  : open-loop  (cmd_vel → PWM proportionally)
           - pid_controller_node: closed-loop (cmd_vel + encoder → PID → PWM)

Prerequisite:
  encoder_node must be running (provides /encoder/ticks)
  → ros2 launch wheel_odometry odometry.launch.py
    or start encoder_node separately.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("pid_velocity_controller"))
    params_file = str(pkg_share / "config" / "pid_params.yaml")

    pid_node = Node(
        package="pid_velocity_controller",
        executable="pid_controller_node",
        name="pid_controller_node",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([pid_node])
