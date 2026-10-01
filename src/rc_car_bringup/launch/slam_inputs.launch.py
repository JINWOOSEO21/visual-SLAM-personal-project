"""
SLAM 백엔드 공통 입력 (PC 측 실행) — slam.launch.py 가 include 한다.

어떤 백엔드를 고르든 똑같이 필요한 전처리만 여기에 둔다. 오도메트리(휠 + IMU
→ EKF)는 Pi 의 sensors.launch.py 가 이미 만들어 보내므로 PC 쪽 공통 입력은
이미지 decompress 하나다.

입력 (Pi 에서 Wi-Fi 로 수신):
    /camera/image_raw/compressed  (sensor_msgs/CompressedImage)
    /camera/camera_info           (sensor_msgs/CameraInfo)
    /odometry/filtered            (nav_msgs/Odometry)  ← 그대로 백엔드가 구독
    TF odom → base_link, static TF

출력:
    /camera/image_decompressed    (sensor_msgs/Image, RGB)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Wi-Fi 대역폭 절감: Pi 가 보내는 /camera/image_raw/compressed 를
    # PC 측에서 republish 로 풀어 /camera/image_decompressed 로 재발행
    image_republish_node = Node(
        package="image_transport",
        executable="republish",
        name="camera_image_republish",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", "/camera/image_raw/compressed"),
            ("out", "/camera/image_decompressed"),
        ],
        output="screen",
    )

    return LaunchDescription([image_republish_node])
