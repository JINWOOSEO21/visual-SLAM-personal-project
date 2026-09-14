"""카메라 내부 파라미터 캘리브레이션

카메라는 Pi 에 달려 있지만 `cameracalibrator` 는 OpenCV(GTK3) 창을 띄우는 GUI
도구라, 헤드리스인 Pi 에서 돌리면 디스플레이가 없어 SIGSEGV 로 죽는다:

    [ERROR] [cameracalibrator-2]: process has died ... exit code -11

그래서 mode 인자로 둘을 갈라 실행한다. 카메라 노드는 Pi, GUI 는 PC.
토픽 이름이 같으므로 (`/camera_node/image_raw`, `/camera_node`) 두 기기가
같은 ROS_DOMAIN_ID 에 있으면 그대로 붙는다. 실측: 양쪽 다 domain 0,
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, ROS_LOCALHOST_ONLY=0.

사용법 (헤드리스 Pi + PC 조합):
    # Pi — 카메라만
    ros2 launch intrinsic_calibration.launch.py mode:=camera              # CSI (기본)
    ros2 launch intrinsic_calibration.launch.py mode:=camera camera:=usb  # C270

    # PC — GUI 만 (ros-humble-camera-calibration 필요)
    ros2 launch intrinsic_calibration.launch.py mode:=calibrator

사용법 (Pi 에 모니터가 붙어 있을 때):
    ros2 launch intrinsic_calibration.launch.py                 # CSI (imx219), 기본값
    ros2 launch intrinsic_calibration.launch.py camera:=usb     # USB 웹캠 (C270)

    # USB 포트를 옮겨서 id 가 바뀐 경우 직접 지정
    ros2 launch intrinsic_calibration.launch.py camera_id:=<id> format:=YUYV

mode:=calibrator 쪽은 camera/camera_id/format 인자를 쓰지 않는다. GUI 는 어느
카메라가 열렸는지 모르고 토픽만 구독하므로, 카메라 선택은 Pi 쪽에서만 하면 된다.

GUI 의 두 버튼은 저장 위치가 다르다 (camera_calibration 3.0.9 소스 확인):

    COMMIT -> do_upload() 이 set_camera_info 서비스를 호출한다. 파일을 쓰는 건
              GUI 가 아니라 **카메라 드라이버**이므로 결과 yaml 은 camera_ros 가
              도는 **Pi** 의 ~/.ros/camera_info/ 에 저장된다. 분리 실행해도
              복사할 필요가 없다 — 이게 우리가 원하는 동작이다.
    SAVE   -> do_save() 가 **GUI 를 띄운 기기**(PC)의
              /tmp/calibrationdata.tar.gz 로 원본 이미지까지 묶어 저장한다.
              재계산용 백업이지 런타임이 읽는 파일이 아니다.

저장 경로에 카메라 id 와 해상도가 들어가 두 카메라의 결과 파일은
저절로 분리된다. 덮어쓸 걱정은 없다:
    CSI : imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x480.yaml
    USB : C270HDWEBCAM__base_scb_pcie_..._046d_0825_640x480.yaml

체커보드는 7x5(내부 코너 기준), 사각형 25 mm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── 카메라 프리셋 ────────────────────────────────────────────────────
# id 는 libcamera 가 부여하는 문자열이고 camera_ros 는 "완전히 같은 문자열"만
# 받는다 (부분 매칭이나 vid:pid 만 주면 std::runtime_error 로 죽는다).
#
# CSI 의 id 는 device-tree 경로라 고정이다. 반면 USB 의 id 에는 USB 포트 경로
# (`usb@0,0-1.2`)가 들어 있어서 **웹캠을 다른 포트에 꽂으면 바뀐다**. 그럴 때는
# camera_id:= 인자로 직접 넘기면 된다. 현재 id 목록은 camera 파라미터 없이
# camera_node 를 띄우면 기동 로그의 ">> cameras:" 에 출력된다.
#
# format 이 카메라마다 다른 이유: C270 은 libcamera 기준 MJPEG/YUYV 만 내보내고
# RGB888 을 지원하지 않는다. CSI 와 같은 값을 주면 기동 시
# `unsupported pixel format "RGB888"` 로 죽는다.
CAMERA_PRESETS = {
    "csi": {
        "id": "/base/soc/i2c0mux/i2c@1/imx219@10",
        "format": "RGB888",
    },
    "usb": {
        "id": "/base/scb/pcie@7d500000/pci@0,0/usb@0,0-1.2:1.0-046d:0825",
        "format": "YUYV",
    },
}

# 캘리브레이션은 런타임과 같은 해상도로 잡아야 내부 파라미터가 유효하다.
# sensors.launch.py 의 width/height 와 맞춰 둔 값이므로 한쪽만 바꾸지 말 것.
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

BOARD_SIZE = "7x5"  # 내부 코너 개수
SQUARE_SIZE = "0.025"  # m


MODES = ("both", "camera", "calibrator")


def launch_setup(context, *_args, **_kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODES:
        raise RuntimeError(f"mode:={mode} 은 알 수 없는 값이다. 가능한 값: {', '.join(MODES)}")

    nodes = []

    # GUI 만 띄우는 모드에서는 카메라 프리셋을 아예 건드리지 않는다. PC 에는
    # 카메라가 없고, 어느 카메라를 열지는 Pi 쪽 mode:=camera 가 정하기 때문이다.
    if mode in ("both", "camera"):
        nodes.append(_camera_node(context))
    if mode in ("both", "calibrator"):
        nodes.append(_calibration_node(context))

    return nodes


def _camera_node(context):
    preset_name = LaunchConfiguration("camera").perform(context)
    if preset_name not in CAMERA_PRESETS:
        raise RuntimeError(
            f"camera:={preset_name} 은 알 수 없는 값이다. "
            f"가능한 값: {', '.join(sorted(CAMERA_PRESETS))}"
        )
    preset = CAMERA_PRESETS[preset_name]

    # 인자로 넘어온 값이 있으면 프리셋보다 우선한다 (USB 포트 변경 등).
    camera_id = LaunchConfiguration("camera_id").perform(context) or preset["id"]
    image_format = LaunchConfiguration("format").perform(context) or preset["format"]

    return Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        parameters=[
            {
                "camera": camera_id,
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "format": image_format,
            }
        ],
        output="screen",
    )


def _calibration_node(context):
    # 접선 왜곡(p1, p2)을 0 으로 고정할지. CSI 를 세 번 연속 잡아본 실측에서
    # p2 가 -0.0159 -> +0.0060 -> +0.0083 으로 부호까지 바뀌었다. 정상적인
    # 렌즈라면 1e-3 수준이고 시행 간에 안정적이어야 하는데, 한 자릿수 크고
    # 흔들린다는 건 접선 왜곡이 데이터로 구속되지 않아 과적합되고 있다는 뜻이다.
    # 같은 세 시행에서 cx 도 275.7 -> 342.1 -> 352.3 으로 흔들렸다 (fx 와 k1 은
    # 수렴했다). 자유도를 둘 줄이면 남은 파라미터가 더 잘 잡힐 것으로 본다.
    #
    # 끄고 비교하려면 zero_tangent_dist:=false.
    extra_args = []
    if LaunchConfiguration("zero_tangent_dist").perform(context).lower() in ("true", "1"):
        extra_args.append("--zero-tangent-dist")

    # OpenCV(GTK3) 창을 띄우므로 디스플레이가 있는 기기에서만 살아남는다.
    # 헤드리스에서 실행하면 exit code -11 (SIGSEGV) 로 죽는다.
    #
    # PYTHONNOUSERSITE 를 주는 이유: PC 의 ~/.local 에 pip 로 설치된
    # opencv_python_headless 4.13.0.92 가 있는데, user site 가
    # /usr/lib/python3/dist-packages 보다 sys.path 우선순위가 높아서
    # cameracalibrator(shebang /usr/bin/python3)가 그 headless 빌드를 집는다.
    # 이름 그대로 GUI 가 빠진 빌드라 창을 만들 때 죽는다:
    #
    #   cv2.error: ... The function is not implemented.
    #   Rebuild the library with Windows, GTK+ 2.x or Cocoa support.
    #
    # 이 값을 1 로 두면 ~/.local 을 건너뛰고 apt 로 깔린 python3-opencv 4.5.4
    # (GUI: GTK3) 를 쓴다. 실측으로 창 생성과 camera_calibration import 둘 다
    # 확인했다. pip 쪽 opencv 를 지우지 않는 이유는 다른 작업이 그걸 쓸 수
    # 있어서다 — 이 노드만 국소적으로 피해 가는 편이 안전하다.
    return Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="cameracalibrator",
        arguments=[
            "--size",
            BOARD_SIZE,
            "--square",
            SQUARE_SIZE,
            *extra_args,
        ],
        remappings=[
            ("image", "/camera_node/image_raw"),
            # COMMIT 버튼이 호출하는 서비스. 이름을 직접 맞춰줘야 한다.
            #
            # cameracalibrator 는 클라이언트를 상대 이름 "camera/set_camera_info"
            # 로 만들고(camera_calibrator.py:100), camera_ros 는 서버를 상대 이름
            # "set_camera_info" 로 만든다. 둘 다 네임스페이스가 "/" 라 각각
            # /camera/set_camera_info 와 /set_camera_info 로 풀려 어긋난다.
            # 실측 (ros2 node info /camera_node):
            #     Service Servers:  /set_camera_info
            #
            # 이름이 어긋나면 COMMIT 이 존재하지 않는 서비스에 동기 .call() 을
            # 걸고 영원히 블록된다. GUI 스레드에서 호출되므로 창이 굳고, 기다려도
            # 절대 풀리지 않는다.
            #
            # 시작 로그의 "Waiting for service ... OK" 는 이 문제를 전혀
            # 걸러주지 못한다. wait_for_service(timeout_sec=5) 의 반환값을 보지
            # 않고 예외만 잡은 뒤 무조건 "OK" 를 찍기 때문이다
            # (camera_calibrator.py:109-120). 서비스가 없어도 OK 가 나온다.
            ("camera/set_camera_info", "/set_camera_info"),
        ],
        additional_env={"PYTHONNOUSERSITE": "1"},
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="both",
                choices=list(MODES),
                description=(
                    "both = 카메라+GUI 한 기기에서 (Pi 에 모니터가 있을 때), "
                    "camera = 카메라 노드만 (헤드리스 Pi), "
                    "calibrator = GUI 만 (PC)"
                ),
            ),
            DeclareLaunchArgument(
                "zero_tangent_dist",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "접선 왜곡 p1, p2 를 0 으로 고정 (mode 가 both/calibrator 일 때만 의미 있음). "
                    "실측에서 p2 부호가 시행마다 뒤집혀 기본값을 true 로 둔다."
                ),
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="csi",
                choices=sorted(CAMERA_PRESETS),
                description="캘리브레이션할 카메라 (csi = imx219, usb = C270)",
            ),
            DeclareLaunchArgument(
                "camera_id",
                default_value="",
                description="libcamera 카메라 id 직접 지정 (비우면 프리셋 값)",
            ),
            DeclareLaunchArgument(
                "format",
                default_value="",
                description="픽셀 포맷 직접 지정 (비우면 프리셋 값)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
