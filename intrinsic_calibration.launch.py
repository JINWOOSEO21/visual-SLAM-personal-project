from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 아래 두 값은 sensors.launch.py 의 CSI_CAMERA_ID / CAMERA_ORIENTATION 과
    # 반드시 같아야 한다. 캘리브레이션은 런타임과 동일한 카메라를, 동일한 방향으로
    # 띄운 상태에서 잡아야 내부 파라미터가 유효하다.
    #
    # camera: USB 웹캠이 꽂혀 있으면 libcamera 인덱스 0 이 웹캠이 되어버리므로
    #         device-tree 경로로 CSI 카메라를 고정한다.
    # orientation: 카메라를 상하 반전해서 장착했으므로 180 으로 되돌린다.
    CSI_CAMERA_ID = "/base/soc/i2c0mux/i2c@1/imx219@10"
    CAMERA_ORIENTATION = 180

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        parameters=[
            {
                "camera": CSI_CAMERA_ID,
                "orientation": CAMERA_ORIENTATION,
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
