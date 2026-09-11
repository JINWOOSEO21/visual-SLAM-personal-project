"""
RTAB-Map Monocular Visual SLAM launch (PC 측 실행)

입력 토픽 (Pi 에서 Wi-Fi 로 수신):
    /camera/image_raw/compressed  (sensor_msgs/CompressedImage, 30fps)
    /camera/camera_info           (sensor_msgs/CameraInfo)
    /odometry/filtered            (nav_msgs/Odometry, EKF 출력)
    /imu/data                     (sensor_msgs/Imu, imu_kalman_node — orientation 포함)

Wi-Fi 대역폭 이슈로 raw 이미지는 17fps 까지 떨어지므로 compressed 를
PC 측에서 수신 → image_transport/republish 로 decompress → rtabmap 입력.

출력 (노드 네임스페이스가 없으므로 토픽은 전부 루트에 있다):
    /mapData, /cloud_map, /map, /mapGraph, /mapPath, /info
    TF map → odom

이 launch 하나로 decompress + SLAM + 뷰어가 같이 뜬다.

사용법 (PC):
    ros2 launch rc_car_bringup rtabmap.launch.py

    # 뷰어 선택 (기본은 RViz2 만)
    ros2 launch rc_car_bringup rtabmap.launch.py rviz:=false          # 뷰어 없이
    ros2 launch rc_car_bringup rtabmap.launch.py rtabmap_viz:=true    # RTAB-Map GUI 도 같이

RViz 만 따로 껐다 켜고 싶으면 rviz:=false 로 띄운 뒤
별도 터미널에서 `ros2 launch rc_car_bringup rviz.launch.py` 를 쓰면 된다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")
    rviz = LaunchConfiguration("rviz")
    database_path = LaunchConfiguration("database_path")

    bringup_dir = Path(get_package_share_directory("rc_car_bringup"))

    default_db_path = str(Path("~/.ros/rtabmap.db").expanduser())

    # ── RTAB-Map 파라미터 ───────────────────────────────────────────
    # 단안 카메라 + 휠 오도메트리 + IMU 융합 모드
    rtabmap_params = {
        "use_sim_time": use_sim_time,
        # 프레임 / 토픽 구독 모드
        "frame_id": "base_link",
        # odom_frame_id 를 설정하면 rtabmap 이 odom 토픽 대신 TF(odom->base_link)
        # 를 lookup 한다. TF 는 지연 허용치가 없어 Wi-Fi 로 TF 가 조금만 밀려도
        # "extrapolation into the future" 로 매 프레임이 폐기된다 (실측: 지연이
        # 0.6s -> 5.2s 로 계속 증가, 90초간 4프레임만 처리).
        # 비워두면 /odometry/filtered 를 토픽으로 구독해 approx_sync 큐가
        # 지연을 흡수하고, 아래 qos_odom(BEST_EFFORT) 도 비로소 적용된다.
        "map_frame_id": "map",
        "subscribe_depth": False,
        "subscribe_rgb": True,
        "subscribe_rgbd": False,
        "subscribe_stereo": False,
        "subscribe_scan": False,
        "subscribe_odom_info": False,
        # IMU 는 EKF 가 이미 융합해 TF(odom->base_link) 로 내보내므로
        # IMU -> EKF -> rtabmap 단일 경로를 유지한다.
        #
        # 실측 확인 (Humble, rtabmap_slam): 이 `rtabmap` 노드는 subscribe_imu 를
        # 아예 사용하지 않는다. true 로 줘도 /imu/data 구독자는 늘지 않고
        # (ekf_filter_node 하나뿐), 기동 로그의 "subscribed to (approx sync)" 목록도
        # 이미지/카메라인포 그대로다. subscribe_imu 는 rtabmap_odom 계열
        # (rgbd_odometry / stereo_odometry) 노드의 파라미터다.
        # 따라서 이 값은 false 로 두는 것이 실제 동작과 일치한다.
        "subscribe_imu": False,
        # 카메라/odom 타임스탬프 동기화 (서로 다른 주기 → approx)
        "approx_sync": True,
        # odom 이 sync 대상에 추가되므로 50ms 는 너무 좁다. Wi-Fi 지터를
        # 감안해 완화한다 (좁으면 sync 가 성립하지 않아 "Did not receive
        # data since 5 seconds" 가 반복된다).
        "approx_sync_max_interval": 0.2,
        "sync_queue_size": 30,  # 'queue_size' 는 deprecated (기동 시 WARN)
        "qos_image": 2,  # 2 = BEST_EFFORT (Wi-Fi 손실 허용)
        "qos_camera_info": 2,
        "qos_imu": 2,
        "qos_odom": 2,
        # DB 파일
        "database_path": database_path,
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        # feature=0 인 노드를 루프클로저 후보에서 제외 (old=0 reject 방지)
        "Mem/BadSignaturesIgnored": "true",
        # feature descriptor 를 LTM 이동 후에도 보존 (old=0 근본 원인 차단)
        "Mem/BinDataKept": "true",
        # ── 평면 주행 (2D SLAM) ─────────────────────────────────
        "Reg/Force3DoF": "true",
        # NOTE: Optimizer/Slam2D 는 이 rtabmap 버전(Humble)에 존재하지 않는다.
        # `ros2 run rtabmap_slam rtabmap --params` 389개 중 없음 → 설정해도 무시되고
        # 경고만 남으므로 제거했다. 평면 제약은 Reg/Force3DoF + RGBD/ForceOdom3DoF 가 담당한다.
        "Optimizer/Strategy": "1",  # 1 = g2o (중력 제약 지원)
        "RGBD/OptimizeFromGraphEnd": "false",
        # ── 단안 모드 (스케일은 wheel odom 으로 해결) ────────────
        "Vis/EstimationType": "1",  # 1 = PnP (단안에 적합)
        "Vis/MinInliers": "10",  # 15→10: 루프클로저 검증 허용 폭 확대
        "Vis/InlierDistance": "0.1",
        "Vis/MaxFeatures": "600",
        # ORB 과 동일 feature type 으로 맞춰 Mem/UseOdomFeatures 불일치 해소
        "Vis/FeatureType": "8",  # 8 = ORB
        # ── 단안 depth 추정 (Occupancy Grid 생성용) ──────────────
        # monocular triangulation 으로 sparse depth map 생성
        "gen_depth": True,
        "gen_depth_decimation": 4,  # 연산 부하 감소
        # ── 검출/루프클로저 주기 (RPi 부담 ↓, PC 측이지만 보수적) ──
        "Rtabmap/DetectionRate": "1.0",  # Hz
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "false",
        "RGBD/AngularUpdate": "0.05",  # rad
        "RGBD/LinearUpdate": "0.05",  # m
        # ── 특징점: ORB 명시 (xfeatures2d 없는 환경에서 BRIEF 대체) ──
        "Kp/MaxFeatures": "400",
        "Kp/DetectorStrategy": "8",  # 8 = ORB (BRIEF 대신 명시)
    }

    # 토픽 remapping
    rtabmap_remappings = [
        ("rgb/image", "/camera/image_decompressed"),
        ("rgb/camera_info", "/camera/camera_info"),
        # odom_frame_id 를 비워 rtabmap/rtabmap_viz 가 TF 대신 이 토픽을
        # 구독하게 했으므로 이 remap 이 실제로 동작한다.
        ("odom", "/odometry/filtered"),
    ]

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

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        remappings=rtabmap_remappings,
        arguments=["--delete_db_on_start"],  # 매 실행마다 새 맵 (필요시 제거)
    )

    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_id": "base_link",
                # rtabmap 노드와 동일한 이유로 odom_frame_id 를 비운다.
                "subscribe_depth": False,
                "subscribe_rgb": True,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "approx_sync_max_interval": 0.2,
                "qos_image": 2,
                "qos_camera_info": 2,
                "qos_odom": 2,
                "queue_size": 30,
            }
        ],
        remappings=rtabmap_remappings,
        condition=IfCondition(rtabmap_viz),
    )

    # RViz2 (config/slam.rviz). rviz.launch.py 를 그대로 재사용해서
    # 설정 경로가 두 군데로 갈라지지 않게 한다.
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_dir / "launch" / "rviz.launch.py")),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="RViz2 를 같이 띄운다 (config/slam.rviz)",
            ),
            DeclareLaunchArgument(
                "rtabmap_viz",
                default_value="false",
                description="RTAB-Map 자체 GUI 도 같이 띄운다 (뷰어 2개가 된다)",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=default_db_path,
                description="RTAB-Map database file path",
            ),
            image_republish_node,
            rtabmap_node,
            rtabmap_viz_node,
            rviz_launch,
        ]
    )
