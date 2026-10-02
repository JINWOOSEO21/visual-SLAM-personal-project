"""
ORB-SLAM3 단안 백엔드 (PC 측 실행) — slam.launch.py 를 통해 띄운다.

    ros2 launch rc_car_bringup slam.launch.py slam:=orbslam3
    ros2 launch rc_car_bringup slam.launch.py slam:=orbslam3 mode:=localization

노드는 orb_slam3_ros2 패키지(ORB-SLAM3 래퍼, 아직 없음)가 제공한다. 패키지가
빌드돼 있지 않으면 여기서 이유를 출력하고 멈춘다. 이 launch 가 그 패키지에
요구하는 인터페이스는 다음과 같다.

    mono_node       ORB-SLAM3 System::TrackMonocular 래퍼
        입력  /camera/image_decompressed (헤더 stamp 를 그대로 사용)
        출력  /orb_slam3/camera_pose  스케일 없는 T_world_cam
              /orb_slam3/state        tracking 상태 + Atlas 맵 id
              /orb_slam3/map_points   PointCloud2 (RViz 용)
        서비스 ~/save_map             atlas_path 로 Atlas 저장

    scale_aligner   단안 포즈를 휠 오도메트리로 미터 단위화
        입력  /orb_slam3/camera_pose, /orb_slam3/state, /odometry/filtered, TF
        출력  /orb_slam3/odometry     map 프레임, 미터 단위
              TF map → odom           (slam.launch.py 의 백엔드 계약)

ORB-SLAM3 는 프레임 간 연속 추적이 전제라 Pi 의 카메라를 RTAB-Map 용 5 Hz 로
두면 추적이 버티지 못한다. Pi 에서 sensors.launch.py slam:=orbslam3 로 띄울 것.

인자:
    mode           slam | localization (localization 은 atlas_path 의 맵을 불러온다)
    vocabulary     ORBvoc.txt 경로
    settings_path  ORB-SLAM3 카메라/특징점 설정 yaml. 비우면 orb_slam3_ros2 의
                   config/imx219_820x616.yaml (Pi 캘리브레이션 결과로 작성)
    atlas_path     Atlas 저장/불러오기 경로 (ORB-SLAM3 가 .osa 를 붙인다)
    use_viewer     ORB-SLAM3 자체 Pangolin 뷰어 (기본 false, RViz 를 쓴다)
    trajectory_path  종료 시 키프레임 궤적(TUM) 저장 경로, 비우면 저장 안 함
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ORB_DIR = Path("~/.ros/orbslam3").expanduser()


def _launch_nodes(context):
    try:
        orb_share = Path(get_package_share_directory("orb_slam3_ros2"))
    except PackageNotFoundError:
        raise RuntimeError(
            "slam:=orbslam3 needs the orb_slam3_ros2 package, which is not built in "
            "this workspace. It is skipped when ORB-SLAM3 is missing (e.g. on the Pi); "
            "build ORB-SLAM3 under ~/workspace/third_party first, or use slam:=rtabmap."
        ) from None

    # settings_path 를 비워두면 패키지에 들어 있는 캘리브레이션 기반 설정을 쓴다.
    settings = LaunchConfiguration("settings_path").perform(context) or str(
        orb_share / "config" / "imx219_820x616.yaml"
    )
    paths = {
        "vocabulary": Path(LaunchConfiguration("vocabulary").perform(context)).expanduser(),
        "settings_path": Path(settings).expanduser(),
    }
    missing = [f"{key}={path}" for key, path in paths.items() if not path.is_file()]
    if missing:
        raise RuntimeError("ORB-SLAM3 input files not found: " + ", ".join(missing))

    use_sim_time = LaunchConfiguration("use_sim_time")

    mono_node = Node(
        package="orb_slam3_ros2",
        executable="mono_node",
        name="orb_slam3",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mode": LaunchConfiguration("mode"),
                "vocabulary": str(paths["vocabulary"]),
                "settings_path": str(paths["settings_path"]),
                "atlas_path": LaunchConfiguration("atlas_path"),
                "use_viewer": LaunchConfiguration("use_viewer"),
                "trajectory_path": LaunchConfiguration("trajectory_path"),
            }
        ],
        remappings=[("image", "/camera/image_decompressed")],
    )

    scale_aligner_node = Node(
        package="orb_slam3_ros2",
        executable="scale_aligner",
        name="orb_scale_aligner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "camera_frame": "camera",
            }
        ],
        remappings=[("odom", "/odometry/filtered")],
    )

    return [mono_node, scale_aligner_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "mode",
                default_value="slam",
                choices=["slam", "localization"],
                description="slam: 새 맵 생성 / localization: atlas_path 의 맵에서 위치 추정",
            ),
            DeclareLaunchArgument(
                "vocabulary",
                default_value=str(ORB_DIR / "ORBvoc.txt"),
                description="ORB-SLAM3 BoW vocabulary (ORB_SLAM3/Vocabulary/ORBvoc.txt)",
            ),
            DeclareLaunchArgument(
                "settings_path",
                default_value="",
                description="ORB-SLAM3 settings yaml (카메라 내부 파라미터 + ORB 추출기)",
            ),
            DeclareLaunchArgument(
                "atlas_path",
                default_value=str(ORB_DIR / "atlas"),
                description="Atlas 저장/불러오기 경로 (확장자 .osa 는 ORB-SLAM3 가 붙인다)",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="false",
                choices=["true", "false"],
                description="ORB-SLAM3 자체 Pangolin 뷰어 (특징점·키프레임 디버깅용)",
            ),
            DeclareLaunchArgument(
                "trajectory_path",
                default_value=str(ORB_DIR / "kf_trajectory.txt"),
                description="종료 시 키프레임 궤적(TUM 형식) 저장 경로. 비우면 저장 안 함",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
