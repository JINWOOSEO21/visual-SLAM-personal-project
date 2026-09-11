from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── 검증 후 이 값을 수정하세요 ──────────────────────────────
    PULSES_PER_REV = 40.0  # 검증 완료: BOTH 엣지, 40 펄스/회전
    # ─────────────────────────────────────────────────────────────

    encoder_node = Node(
        package="wheel_odometry",
        executable="encoder_node",
        name="encoder_node",
        parameters=[
            {
                "left_pin": 17,
                "right_pin": 16,
                "edge_mode": "both",
                "publish_rate": 50.0,
            }
        ],
        output="screen",
    )

    odometry_node = Node(
        package="wheel_odometry",
        executable="odometry_node",
        name="odometry_node",
        parameters=[
            {
                "pulses_per_rev": PULSES_PER_REV,
                "wheel_diameter": 0.065,  # m
                "wheel_base": 0.150,  # m
                "odom_frame": "odom",
                "base_frame": "base_link",
            }
        ],
        output="screen",
    )

    return LaunchDescription([encoder_node, odometry_node])
