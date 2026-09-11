from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        parameters=[
            {
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
