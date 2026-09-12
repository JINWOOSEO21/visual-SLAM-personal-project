from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # sensors.launch.py 와 동일한 카메라를 동일한 설정으로 띄워야 내부 파라미터가
    # 유효하다. USB 웹캠이 꽂혀 있으면 libcamera 인덱스 0 이 웹캠이 되어버리므로
    # device-tree 경로로 CSI 카메라를 고정한다.
    #
    # 카메라가 상하 반전 장착이라 체커보드도 뒤집혀 보이지만 그대로 잡으면 된다.
    # cameracalibrator 는 체커보드 방향을 가리지 않고, 내부 파라미터는 "실제로
    # 퍼블리시되는 이미지" 기준으로 나와야 런타임과 일치한다. 화면을 바로 세워서
    # 캘리브레이션하면 principal point 가 뒤집힌 값이 나온다.
    CSI_CAMERA_ID = "/base/soc/i2c0mux/i2c@1/imx219@10"

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        parameters=[
            {
                "camera": CSI_CAMERA_ID,
                "width": 640,
                "height": 480,
                "format": "RGB888",
            }
        ],
    )

    calibration_node = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="cameracalibrator",
        arguments=[
            "--size",
            "7x5",
            "--square",
            "0.025",
        ],
        remappings=[
            ("image", "/camera_node/image_raw"),
            ("camera", "/camera_node"),
        ],
    )

    return LaunchDescription(
        [
            camera_node,
            calibration_node,
        ]
    )
