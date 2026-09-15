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

    # NOTE: CSI 카메라는 정방향으로 장착돼 있다 (2026-09-15 실측: 퍼블리시된
    # 프레임이 똑바로 나온다). 그래서 sensor_tf.launch.py 의
    # base_link -> camera_link 도 roll=0 이다.
    #
    # 여기에 이미지 회전을 넣지 않는다. camera_ros 의 `orientation` 파라미터는
    # libcamera >= 0.2 를 요구하는데 이 Pi 는 ros-humble-libcamera 0.1.0 이라
    # 설정해도 무시된다:
    #   [WARN] parameter 'orientation' not supported on libcamera 0.1
    #
    # 나중에 카메라를 뒤집어 달더라도 회전은 TF 쪽 roll 로 처리한다. 이미지를
    # 돌리면 principal point 가 (W-1-cx, H-1-cy) 로 바뀌어 캘리브레이션을 다시
    # 잡아야 하고, 회전 노드를 끼우면 Pi CPU 와 지연도 는다. 무엇보다 TF 와
    # 이미지를 동시에 뒤집으면 이중 회전이 된다 — 둘 중 하나만 써야 한다.
    # 해상도는 FOV 를 좌우한다. IMX219 는 저해상도를 만드는 방법이 둘인데
    # 결과가 전혀 다르다 (데이터시트 5-2 Pixel Binning Mode):
    #
    #   "Binning read-out can be used to obtain an image of lower resolution
    #    for full field of view."
    #
    #   binning  : 인접 2x2 픽셀을 평균, 배열 전체(3280x2464)를 읽음 -> FOV 유지
    #   cropping : x_addr/y_addr 로 읽는 창 자체를 좁힘            -> FOV 축소
    #
    # 640x480 을 요청하면 libcamera 가 crop 모드를 고른다. 실측: 캘리브레이션에서
    # 나온 수평 FOV 가 27.7도였고, crop 창이 센서 폭의 약 39%(1280/3280)라고 보면
    # 2*atan(0.39*tan(31도)) = 26.4도로 계산과 맞는다. 렌즈 문제가 아니었다.
    #
    # 820x616 은 full-FOV 비닝 모드(1640x1232)의 정확히 1/2 이라 그 모드가 선택되고
    # ISP 가 다운스케일한다. FOV 가 27.7도에서 약 62도로 넓어진다. 원본
    # 1640x1232 를 그대로 쓰지 않는 이유는 RGB888 기준 프레임이 6MB 라
    # DDS 전송이 실패했기 때문이다 (실측: 25초 안에 한 장도 못 받음).
    #
    # NOTE: 해상도를 바꾸면 내부 파라미터가 무효가 된다. camera_info 파일명에도
    # 해상도가 들어가므로(..._820x616.yaml) 반드시 다시 캘리브레이션할 것.
    # intrinsic_calibration.launch.py 의 IMAGE_WIDTH/HEIGHT 와 같은 값이어야 한다.
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        parameters=[
            {
                "camera": CSI_CAMERA_ID,
                "width": 820,
                "height": 616,
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
