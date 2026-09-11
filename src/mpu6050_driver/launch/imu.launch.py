from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # MPU6050 드라이버 노드
            Node(
                package="mpu6050_driver",
                executable="mpu6050_node",
                name="mpu6050_node",
                parameters=[
                    {
                        "i2c_bus": 1,
                        "device_address": 0x68,
                        "frequency": 50.0,
                        "frame_id": "imu_link",
                        "accel_range": 0,  # ±2g
                        "gyro_range": 0,  # ±250 °/s
                        "calibration_samples": 1000,
                        # 가속도계 bias/scale 은 config/imu_calibration.yaml 에서 자동 적재된다.
                        # 값을 채우려면: python3 src/mpu6050_driver/scripts/calibrate_accel.py
                        # 다른 파일을 쓰려면 'calibration_file' 파라미터로 경로 지정.
                    }
                ],
                output="screen",
            ),
            # Kalman 필터: /imu/data_raw → /imu/data (orientation 포함)
            Node(
                package="mpu6050_driver",
                executable="imu_kalman_node",
                name="imu_filter",
                parameters=[
                    {
                        "q_angle": 0.001,  # 각도 프로세스 노이즈
                        "q_bias": 0.003,  # 자이로 바이어스 프로세스 노이즈
                        "r_measure": 0.03,  # 가속도계 측정 노이즈
                    }
                ],
                remappings=[
                    ("imu/data_raw", "imu/data_raw"),
                    ("imu/data", "imu/data"),
                ],
                output="screen",
            ),
        ]
    )
