"""
robot_localization EKF — wheel odom + IMU 융합

입력:
    /odom        (nav_msgs/Odometry, wheel_odometry)
    /imu/data    (sensor_msgs/Imu, madgwick filtered)

출력:
    /odometry/filtered   (nav_msgs/Odometry)
    TF:  odom → base_link
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = str(Path(get_package_share_directory("rc_car_bringup")) / "config" / "ekf.yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    return LaunchDescription([ekf_node])
