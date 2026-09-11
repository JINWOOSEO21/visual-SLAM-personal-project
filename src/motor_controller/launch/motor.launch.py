from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="motor_controller",
                executable="motor_driver_node",
                name="motor_driver_node",
                parameters=[
                    {
                        # GPIO (BCM)
                        "left_ena_pin": 24,
                        "left_in1_pin": 22,
                        "left_in2_pin": 23,
                        "right_enb_pin": 25,
                        "right_in3_pin": 27,
                        "right_in4_pin": 26,
                        # 기구
                        "wheel_base": 0.150,
                        # 배선 극성 반전 (실측: 좌/우 모두 반대로 배선됨)
                        "left_invert": True,
                        "right_invert": True,
                        # SLAM 맵핑용 보수적 속도 상한
                        "max_linear_speed": 0.3,
                        "max_angular_speed": 1.5,
                        # PWM
                        "pwm_frequency": 1000.0,
                        "min_pwm": 50.0,
                        "max_pwm": 100.0,
                        # 안전
                        "cmd_timeout": 0.5,
                        "watchdog_rate": 10.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
