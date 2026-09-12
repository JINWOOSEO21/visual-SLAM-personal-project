"""
Phase 4: 카메라-IMU 외부 캘리브레이션 (Static TF)

TF 프레임 구조:
    base_link
    ├── camera_link   (전방 카메라)
    └── imu_link      (MPU6050)

좌표계 규칙 (REP-103):
    x: 전방, y: 좌측, z: 상방

실측값 (단위: 미터, base_link 원점 = 뒷바퀴 축 중심):
    imu_link:    전방 0.060m, 우측 0.065m(→ y=-0.065), 상방 0.023m
    camera_link: 전방 0.160m, 우측 0.005m(→ y=-0.005), 상방 0.053m

camera_link 값은 IMU 기준 실측에서 유도했다.
IMU 자체 축 기준으로 카메라는 (x=-0.06, y=+0.10, z=+0.03) 에 있고,
IMU 축은 아래 매핑을 따르므로 base_link 축으로는 (전방 0.10, 좌측 0.06, 상방 0.03):

    R_z(-pi/2) @ (-0.06, 0.10, 0.03) = (0.10, 0.06, 0.03)

여기에 imu_link 원점을 더하면
    (0.060, -0.065, 0.023) + (0.10, 0.06, 0.03) = (0.160, -0.005, 0.053)

직접 잰 이전 값 (0.175, 0, 0.050) 과는 1.6cm 차이였고,
IMU 기준 측정이 더 최근이라 이쪽을 채택했다.

IMU 물리 장착 방향:
    IMU x축 → 로봇 우측(base_link -y)
    IMU y축 → 로봇 전방(base_link +x)
    IMU z축 → 로봇 상방(base_link +z)
    ∴ yaw = -π/2 회전 적용
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link → camera_link
    # 카메라가 전방(x)을 바라보되, 광학축 기준으로 180° 뒤집어(상하 반전) 장착돼 있다.
    #
    # 반전을 이미지가 아니라 TF 로 처리하는 이유: camera_ros 의 `orientation`
    # 파라미터는 libcamera >= 0.2 를 요구하는데 이 Pi 는 0.1.0 이라 무시된다
    # ("parameter 'orientation' not supported on libcamera 0.1"). 이미지를 돌리는
    # 별도 노드를 끼우면 Pi CPU 와 지연이 늘고 압축 단계도 한 번 더 타야 한다.
    # TF 로 처리하면 비용이 0 이고 SLAM 기하는 정확하다. 대신 RViz 등에서 보이는
    # 이미지는 뒤집힌 채로 남는다 (표시상의 문제일 뿐 특징점 매칭은 회전 불변이다).
    #
    # 반전을 여기(camera_link)에 넣고 아래 camera_link → camera 는 표준값 그대로
    # 두는 편이 의미가 분명하다. "카메라 몸체가 뒤집혀 달렸다"는 사실은 장착의
    # 속성이지 광학 프레임 규칙이 바뀐 게 아니기 때문이다.
    #
    # 검증 (base_link 기준 optical 축):
    #   roll=0  → 이미지 오른쪽=우측(-y), 이미지 아래=하방(-z), 광축=전방(+x)
    #   roll=pi → 이미지 오른쪽=좌측(+y), 이미지 아래=상방(+z), 광축=전방(+x)
    # 즉 광축은 그대로 전방을 보고 상하좌우만 뒤집힌다.
    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "--x",
            "0.160",  # 전방 16.0cm
            "--y",
            "-0.005",  # 우측 0.5cm (거의 중심선)
            "--z",
            "0.053",  # 상방 5.3cm
            "--roll",
            "3.1415927",  # π : 상하 반전 장착
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "camera_link",
        ],
    )

    # camera_link → camera (optical frame)
    # ROS 카메라 광학 프레임 규칙: z 전방, x 우측, y 하방
    # camera_link(x전방,y좌,z상) → optical(z전방,x우,y하)
    # 회전: roll=-π/2, yaw=-π/2
    # child-frame-id 는 camera_ros 노드가 이미지 헤더에 박는 frame_id ('camera') 와 일치시켜야
    # rtabmap 이 TF lookup 가능. (기존 'camera_optical_frame' → 'camera' 로 변경)
    camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_optical_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.5707963",  # -π/2
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963",  # -π/2
            "--frame-id",
            "camera_link",
            "--child-frame-id",
            "camera",
        ],
    )

    # base_link → imu_link
    # IMU x→로봇우측(-y), y→로봇전방(+x), z→로봇상방(+z)  ∴ yaw=-π/2
    base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "--x",
            "0.060",  # 전방 6.0cm
            "--y",
            "-0.065",  # 우측 6.5cm
            "--z",
            "0.023",  # 상방 2.3cm
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963",  # -π/2 : IMU y축이 로봇 전방
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
    )

    return LaunchDescription(
        [
            base_to_camera,
            camera_to_optical,
            base_to_imu,
        ]
    )
