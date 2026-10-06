"""
RViz2 로 RTAB-Map SLAM 상태를 실시간으로 보는 launch (PC 측 실행)

rtabmap.launch.py 와 별개로 띄운다. RViz 만 껐다 켜도 SLAM 은 계속 돌기 때문에,
같은 launch 에 묶는 것보다 이쪽이 실사용에 편하다.

표시되는 것:
    Map            /map              점유 격자 (2D 맵)
    MapCloud       /mapData          RTAB-Map 그래프의 누적 포인트클라우드
    MapGraph       /mapGraph         노드/링크 그래프 (루프 클로저가 여기 보인다)
    Info           /info             루프 클로저 검출 상태
    Map Cloud      /cloud_map        조밀 포인트클라우드 (gen_depth 결과)
    Odometry       /odometry/filtered EKF 추정 위치
    SLAM Path      /mapPath          최적화된 궤적
    Camera         /camera/image_decompressed

Fixed Frame 은 map 이다. rtabmap 이 map→odom 을 아직 발행하기 전이면
"No transform from [odom] to [map]" 경고가 잠깐 뜨는데, 첫 키프레임이
등록되면 사라진다.

사용법 (PC):
    ros2 launch rc_car_bringup rviz.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("rc_car_bringup")) / "config" / "slam.rviz"
    )

    rviz_config = LaunchConfiguration("rviz_config")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_config,
                description="RViz2 config file path",
            ),
            rviz_node,
        ]
    )
