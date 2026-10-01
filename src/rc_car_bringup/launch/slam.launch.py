"""
SLAM 진입점 (PC 측 실행) — 공통 입력 + 선택한 SLAM 백엔드 + RViz2

    공통 입력   slam_inputs.launch.py      이미지 decompress
    백엔드      slam_<slam>.launch.py      rtabmap | orbslam3
    뷰어        rviz.launch.py             config/slam.rviz

백엔드 계약 — 새 백엔드를 추가할 때 지켜야 하는 것:
    입력  /camera/image_decompressed, /camera/camera_info, /odometry/filtered,
          TF odom → base_link (Pi 의 EKF), static TF
    출력  TF map → odom
          odom → base_link 는 건드리지 않는다. 백엔드가 추적을 잃어도 EKF 가
          위치를 계속 이어 주고, 백엔드는 map 기준 보정만 얹는다.
    파일  launch/slam_<이름>.launch.py 를 만들고 아래 SLAM_BACKENDS 에 추가한다.

두 백엔드를 동시에 띄우면 map → odom 을 둘 다 발행해 TF 가 충돌한다. 비교는
같은 rosbag 을 백엔드별로 한 번씩 재생하는 방식으로 한다:

    ros2 launch rc_car_bringup slam.launch.py slam:=rtabmap use_sim_time:=true
    ros2 bag play <bag> --clock

카메라 fps / JPEG 품질은 백엔드마다 요구가 다르므로 Pi 쪽 sensors.launch.py 도
같은 slam 인자로 맞춰 띄운다.

사용법 (PC):
    ros2 launch rc_car_bringup slam.launch.py                        # rtabmap (기본)
    ros2 launch rc_car_bringup slam.launch.py slam:=orbslam3
    ros2 launch rc_car_bringup slam.launch.py rviz:=false            # 뷰어 없이

백엔드 전용 인자는 그대로 넘기면 된다 (include 된 launch 가 받는다):
    rtabmap   rtabmap_viz:=true, database_path:=...
    orbslam3  mode:=slam|localization, settings_path:=..., atlas_path:=...

RViz 만 따로 껐다 켜고 싶으면 rviz:=false 로 띄운 뒤
별도 터미널에서 `ros2 launch rc_car_bringup rviz.launch.py` 를 쓰면 된다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SLAM_BACKENDS = ["rtabmap", "orbslam3"]


def _include_backend(context, launch_dir):
    slam = LaunchConfiguration("slam").perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / f"slam_{slam}.launch.py"))
        )
    ]


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("rc_car_bringup")) / "launch"

    inputs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "slam_inputs.launch.py"))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "rviz.launch.py")),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam",
                default_value="rtabmap",
                choices=SLAM_BACKENDS,
                description="SLAM 백엔드",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="RViz2 를 같이 띄운다 (config/slam.rviz)",
            ),
            inputs_launch,
            OpaqueFunction(function=_include_backend, args=[launch_dir]),
            rviz_launch,
        ]
    )
