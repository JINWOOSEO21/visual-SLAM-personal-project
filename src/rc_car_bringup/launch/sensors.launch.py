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
    #
    # camera 파라미터를 반드시 명시한다. camera_ros 는 libcamera 가 찾은 카메라 중
    # 인덱스 0 을 기본으로 고르는데, USB 웹캠(UVC)도 libcamera 가 함께 열거하므로
    # 웹캠이 꽂혀 있으면 0 번이 웹캠이 되어버린다. 실측 (C270 연결 상태):
    #   0: C270 HD WEBCAM (...046d:0825)
    #   1: imx219 (/base/soc/i2c0mux/i2c@1/imx219@10)
    # 이 상태로 camera 를 비워두면 웹캠이 선택되고, C270 은 MJPEG/YUYV 만 내므로
    # 아래 format="RGB888" 에서 `unsupported pixel format "RGB888"` 로 노드가 죽는다.
    #
    # 인덱스 대신 id 문자열로 고정하는 이유: 인덱스 순서는 USB 장치의 유무/열거
    # 순서에 따라 바뀌지만, CSI 카메라의 id 는 device-tree 경로라 고정이다.
    CSI_CAMERA_ID = "/base/soc/i2c0mux/i2c@1/imx219@10"

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        parameters=[
            {
                "camera": CSI_CAMERA_ID,
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
