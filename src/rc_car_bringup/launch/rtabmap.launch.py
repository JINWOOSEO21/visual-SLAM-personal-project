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
        # NOTE: 아래 두 파라미터는 "LTM 이동 중 데이터가 소실돼 old=0 이 된다"는
        # 가설로 추가했지만, old=0 의 원인이 아니었다. DB 확인 결과 41번 노드는
        # 키포인트 487 개를 온전히 갖고 있었고 3D 좌표만 NULL 이었다 (게다가
        # Rtabmap/TimeThr=0 이라 LTM 이동 자체가 일어나지 않는다).
        # 실제 원인과 처방은 아래 Mem/StereoFromMotion 주석 참고.
        # 두 값 자체는 무해하고 LTM 운용 시 여전히 유효하므로 유지한다.
        "Mem/BadSignaturesIgnored": "true",
        "Mem/BinDataKept": "true",
        # ── 평면 주행 (2D SLAM) ─────────────────────────────────
        "Reg/Force3DoF": "true",
        # NOTE: Optimizer/Slam2D 는 이 rtabmap 버전(Humble)에 존재하지 않는다.
        # `ros2 run rtabmap_slam rtabmap --params` 389개 중 없음 → 설정해도 무시되고
        # 경고만 남으므로 제거했다. 평면 제약은 Reg/Force3DoF + RGBD/ForceOdom3DoF 가 담당한다.
        "Optimizer/Strategy": "1",  # 1 = g2o (중력 제약 지원)
        "RGBD/OptimizeFromGraphEnd": "false",
        # ── 단안 모드 (스케일은 wheel odom 으로 해결) ────────────
        # 1 = PnP. Mem/StereoFromMotion 이 3D 점을 만들어주므로 성립한다.
        # (2 = Epipolar 는 3D 점 없이 돌지만 translation scale 을 못 구해
        #  그래프 최적화에 쓸 metric 제약이 안 나온다.)
        "Vis/EstimationType": "1",
        # 이 값은 두 곳에서 쓰인다: (1) PnP 직전 게이트 — from 의 3D 단어 수와
        # to 의 전체 단어 수가 각각 이 값 이상이어야 매칭이 시도되고,
        # (2) 최종 RANSAC inlier 최소치. 즉 올리면 후보가 더 일찍 잘린다.
        # 우선 10 으로 두고 loop closure 가 잡히는지부터 확인한다. 잡히기
        # 시작하면 오탐(잘못된 링크는 맵을 크게 망가뜨린다)을 걸러내기 위해
        # 15~20 으로 올리는 것을 권장.
        "Vis/MinInliers": "10",
        "Vis/InlierDistance": "0.1",
        "Vis/MaxFeatures": "600",
        # Kp/DetectorStrategy 와 같은 값으로 맞춰 Mem/UseOdomFeatures 불일치 해소
        "Vis/FeatureType": "8",  # 8 = GFTT/ORB
        # ── 단안 3D 점 생성 (loop closure 의 전제 조건) ──────────
        # gen_depth 는 monocular triangulation 이 아니다. "scan_cloud 를 카메라
        # 평면에 투영해 depth 이미지를 만드는" 기능(gen_scan 의 역방향)이라
        # subscribe_scan_cloud=False 인 이 구성에서는 입력이 없어 무동작이다.
        #
        # 실측 (2026-09-12, ~/.ros/rtabmap.db 90 노드):
        #   depth blob 을 가진 노드            0 / 90
        #   3D 좌표를 가진 특징점              0 / 45,840
        #   Link type 1/2 (loop closure)       0 개  (type 0 neighbor 만 58 개)
        # Vis/EstimationType=1(PnP) 은 old 노드의 3D 점 + new 노드의 2D 점을
        # 필요로 하므로, 3D 점이 0 이면 모든 후보가 old=0 으로 기각된다
        # ("Rejected loop closure 41 -> 60: Not enough features (old=0, new=504)").
        "gen_depth": False,
        # 단안에서 3D 점을 얻는 경로: 연속 프레임 사이의 odom 이동량을
        # baseline 삼아 특징점을 triangulate 한다 (stereo from motion).
        "Mem/StereoFromMotion": "true",
        # 위 설정의 필수 동반 옵션. rtabmap 문서: "It would be ignored if
        # Mem/DepthAsMask is true and the feature detector used supports
        # masking." 아래 Kp/DetectorStrategy=8 은 GFTT 계열이라 masking 을
        # 지원하므로, 이 값을 false 로 내리지 않으면 StereoFromMotion 이
        # 조용히 무시된다. depth 이미지가 없으니 마스크로 쓸 것도 없다.
        "Mem/DepthAsMask": "false",
        #
        # NOTE: Vis/MaxDepth 는 의도적으로 설정하지 않는다 (기본값 0 = 비활성).
        # 한때 "짧은 baseline 으로 triangulate 한 먼 점이 PnP 를 오염시킨다"는
        # 이유로 10.0 을 줬지만, 이 파라미터는 PnP 단계가 아니라 그보다 훨씬
        # 앞단인 *매칭 풀 구성* 단계에서 작동한다. RegistrationVis.cpp 가
        # MinDepth/MaxDepth 중 하나라도 0 이 아니면 filterKeypointsByDepth() 로
        # 양쪽 노드의 디스크립터 목록 자체를 잘라내기 때문이다.
        #
        # StereoFromMotion 의 3D 점은 "그 시점의 이동 방향에서 시차가 생긴
        # 특징"만 골라낸 부분집합이라, 같은 장소를 두 번 지나도 두 방문의
        # 부분집합은 서로 많이 다르다. 매칭을 그 부분집합으로 제한하면 이
        # 불일치를 정면으로 맞게 된다.
        #
        # 실측 (2026-09-12 12:01, 172 노드, Vis/MaxDepth=10.0):
        #   BoW 가설 (확률 > Rtabmap/LoopThr=0.11)   54 건
        #     └ 매칭 단계 도달                          6 건  (48 건은 풀 미달로 탈락)
        #        └ Visual_matches                      최대 8  (중앙값 3)
        #           └ PnP inliers > 0                  0 건
        #   예: 10->148 은 거리 0.80m / 시점차 7.2° 의 좋은 재방문인데
        #       풀이 85 vs 97 로 잘려 matches=3 에 그쳤다.
        #
        # 비활성화하면 매칭이 노드당 ~500 개 특징 전체에서 일어나고, PnP 는
        # 그중 from 쪽 3D 가 유효한 쌍만 자동으로 사용한다. 먼 점의 오염은
        # RANSAC + Vis/PnPReprojError=2 가 걸러준다 — 그쪽이 올바른 위치다.
        #
        # ── 검출/루프클로저 주기 ──────────────────────────────────
        # 2.0 -> 1.0 Hz 로 되돌린다. 한때 "키프레임을 촘촘히 해서 프레임 간
        # 회전을 줄이자"는 이유로 2.0 을 줬는데 역효과였다. 키프레임 간격을
        # 절반으로 줄이면 Mem/StereoFromMotion 이 삼각화에 쓰는 baseline 도
        # 절반이 되고, 삼각화는 baseline 이 클수록 유리하다.
        #
        # 실측 (2026-09-15, 148 노드 @2Hz): 노드당 3D 점 중앙값이 8(1Hz) 에서
        # 6 으로 떨어졌고 Visual_matches 최대값도 8 에서 3 으로 내려갔다.
        "Rtabmap/DetectionRate": "1.0",  # Hz
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "false",
        "RGBD/AngularUpdate": "0.05",  # rad
        # 0.05 -> 0.15 m. 5cm 만 움직여도 노드가 추가되는데, 그런 노드는
        # baseline 이 없어 3D 점을 만들지 못한다. 같은 실측에서 스텝 병진
        # 중앙값이 1.1cm 였고 147 스텝 중 109 개가 5cm 미만이었다. 직전 스텝
        # 병진과 삼각화된 3D 점 수의 관계는 뚜렷하다:
        #
        #     < 0.05m : 노드 109 개, 3D 중앙값  5
        #   0.05-0.15m: 노드  18 개, 3D 중앙값 22
        #   >= 0.30m  : 노드   3 개, 3D 중앙값 52
        #
        # 문턱을 올리면 의미 있게 움직였을 때만 노드가 생겨 baseline 이 확보되고,
        # 노드 수가 줄어 처리도 가벼워진다.
        "RGBD/LinearUpdate": "0.15",  # m
        # ── 특징점 (xfeatures2d 없는 환경에서 BRIEF 대체) ────────
        "Kp/MaxFeatures": "400",
        # 8 = GFTT/ORB (ORB 단독은 2). GFTT 균등 검출 + ORB 디스크립터 조합이라
        # 이대로 두면 되지만, GFTT 가 masking 을 지원하는 검출기라는 점 때문에
        # 위 Mem/DepthAsMask=false 가 반드시 같이 필요하다.
        "Kp/DetectorStrategy": "8",
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
