"""
전체 센서 통합 launch 파일 (Phase 1-5)

실행되는 노드:
    1. camera_ros (카메라 이미지 퍼블리시)
    2. mpu6050_node (IMU raw 데이터)
    3. imu_filter_madgwick (orientation 필터)
    4. static TF: base_link → camera_link, camera_optical_frame, imu_link
    5. encoder_node + odometry_node (휠 오도메트리)
    6. robot_localization EKF (/odom + /imu/data → /odometry/filtered)

모터 제어 노드는 포함하지 않는다 (motor_driver_node / pid_controller_node 중
하나를 별도 터미널에서 선택 실행 — 둘 다 같은 GPIO 핀을 쓰므로 동시 실행 금지).

사용법:
    ros2 launch rc_car_bringup sensors.launch.py
    # 그 다음 별도 터미널에서:
    ros2 launch pid_velocity_controller pid_controller.launch.py   # closed-loop
    # 또는
    ros2 launch motor_controller motor.launch.py                   # open-loop
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("rc_car_bringup"))
    imu_dir = Path(get_package_share_directory("mpu6050_driver"))
    odom_dir = Path(get_package_share_directory("wheel_odometry"))

    # IMU 파이프라인 (mpu6050 + madgwick filter)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(imu_dir / "launch" / "imu.launch.py"))
    )

    # Static TF (base_link → camera_link, imu_link)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_dir / "launch" / "sensor_tf.launch.py"))
    )

    # 카메라 노드 (camera_ros)
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        parameters=[
            {
                "width": 640,
                "height": 480,
                "format": "RGB888",
            }
        ],
        output="screen",
    )

    # 휠 오도메트리 (encoder + odometry)
    # publish_tf=False — TF는 EKF가 broadcast함
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(odom_dir / "launch" / "odometry.launch.py"))
    )

    # EKF 융합 (wheel odom + IMU → /odometry/filtered, odom→base_link TF)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_dir / "launch" / "ekf.launch.py"))
    )

    # NOTE: 모터 제어 노드(motor_driver_node / pid_controller_node)는 여기서
    # 실행하지 않는다. 두 노드가 같은 L298N GPIO 핀을 제어하므로 둘 중 하나만
    # 별도 터미널에서 골라 실행한다:
    #   ros2 launch motor_controller motor.launch.py            (open-loop)
    #   ros2 launch pid_velocity_controller pid_controller.launch.py  (closed-loop)

    return LaunchDescription(
        [
            imu_launch,
            tf_launch,
            camera_node,
            odom_launch,
            ekf_launch,
        ]
    )
