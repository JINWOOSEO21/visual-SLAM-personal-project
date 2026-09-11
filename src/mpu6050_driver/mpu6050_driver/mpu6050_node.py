import struct
import time
from pathlib import Path

import rclpy
import smbus
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

# MPU6050 레지스터 주소
_MPU6050_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_ACCEL_XOUT_H = 0x3B
_TEMP_OUT_H = 0x41
_GYRO_XOUT_H = 0x43
_WHO_AM_I = 0x75

# 스케일 팩터
_ACCEL_SCALE = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}  # ±2g, ±4g, ±8g, ±16g
_GYRO_SCALE = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}  # ±250, ±500, ±1000, ±2000 °/s

_GRAVITY = 9.80665
_DEG_TO_RAD = 0.017453292519943


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__("mpu6050_node")

        # 파라미터 선언
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("device_address", _MPU6050_ADDR)
        self.declare_parameter("frequency", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("accel_range", 0)  # 0=±2g, 1=±4g, 2=±8g, 3=±16g
        self.declare_parameter("gyro_range", 0)  # 0=±250, 1=±500, 2=±1000, 3=±2000 °/s
        self.declare_parameter("calibration_samples", 200)

        # 가속도계 보정: a_corr[i] = (a_raw[i] - bias[i]) * scale[i]
        # 파일이 있으면 파일 값이 우선, 없으면 아래 파라미터 값을 쓴다.
        # 빈 문자열이면 패키지 share 의 config/imu_calibration.yaml 을 찾는다.
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("accel_bias", [0.0, 0.0, 0.0])
        self.declare_parameter("accel_scale", [1.0, 1.0, 1.0])

        self._bus_num = self.get_parameter("i2c_bus").value
        self._addr = self.get_parameter("device_address").value
        self._frame_id = self.get_parameter("frame_id").value
        self._accel_range = self.get_parameter("accel_range").value
        self._gyro_range = self.get_parameter("gyro_range").value
        freq = self.get_parameter("frequency").value
        cal_samples = self.get_parameter("calibration_samples").value

        # I2C 초기화
        self._bus = smbus.SMBus(self._bus_num)

        # WHO_AM_I 확인
        who = self._bus.read_byte_data(self._addr, _WHO_AM_I)
        if who != 0x68:
            self.get_logger().error(f"WHO_AM_I mismatch: expected 0x68, got 0x{who:02x}")
            raise RuntimeError("MPU6050 not found")
        self.get_logger().info(f"MPU6050 detected at 0x{self._addr:02x}")

        # 센서 초기화
        self._bus.write_byte_data(self._addr, _PWR_MGMT_1, 0x00)  # 슬립 해제
        time.sleep(0.1)
        self._bus.write_byte_data(self._addr, _SMPLRT_DIV, 0x00)  # 샘플레이트 = gyro rate / (1+div)
        self._bus.write_byte_data(self._addr, _CONFIG, 0x03)  # DLPF 44Hz
        self._bus.write_byte_data(self._addr, _ACCEL_CONFIG, self._accel_range << 3)
        self._bus.write_byte_data(self._addr, _GYRO_CONFIG, self._gyro_range << 3)

        self._accel_scale = _ACCEL_SCALE[self._accel_range]
        self._gyro_scale = _GYRO_SCALE[self._gyro_range]

        # 가속도계 보정 계수 적재
        self._accel_corr_bias, self._accel_corr_scale = self._load_accel_calibration()

        # 퍼블리셔
        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self._temp_pub = self.create_publisher(Temperature, "imu/temperature", 10)

        # 자이로 바이어스 추정 (정지 상태에서) — 온도에 따라 변하므로 매 기동마다 측정.
        # 가속도는 여기서 보정하지 않는다(정지 시에도 중력이 실려 있어 bias 와 구분 불가).
        # 대신 보정 후 |a| 가 9.81 에 맞는지 확인하는 헬스체크로만 쓴다.
        self.get_logger().info(f"Calibrating bias ({cal_samples} samples)... keep sensor still!")
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._accel_rest = [0.0, 0.0, 0.0]
        self._calibrate(cal_samples)
        self.get_logger().info(
            f"Calibration done: "
            f"gyro_bias=[{self._gyro_bias[0]:.4f}, {self._gyro_bias[1]:.4f}, {self._gyro_bias[2]:.4f}] rad/s"
        )
        self._check_gravity()

        # 타이머
        self._timer = self.create_timer(1.0 / freq, self._timer_callback)
        self.get_logger().info(f"Publishing IMU data at {freq} Hz on /imu/data_raw")

    def _load_accel_calibration(self):
        """가속도계 bias/scale 을 YAML 파일 또는 파라미터에서 읽는다.

        파일이 우선하고, 없거나 읽을 수 없으면 파라미터 값(기본 무보정)을 쓴다.
        """
        bias = list(self.get_parameter("accel_bias").value)
        scale = list(self.get_parameter("accel_scale").value)

        configured = self.get_parameter("calibration_file").value
        if configured:
            path = Path(configured)
        else:
            try:
                share_dir = Path(get_package_share_directory("mpu6050_driver"))
                path = share_dir / "config" / "imu_calibration.yaml"
            except Exception:  # 패키지 share 를 못 찾는 경우(소스 직접 실행 등)
                path = None

        if path is not None and path.is_file():
            try:
                with path.open() as f:
                    data = yaml.safe_load(f) or {}
                accel = data.get("accel", {})
                if "bias" in accel:
                    bias = [float(v) for v in accel["bias"]]
                if "scale" in accel:
                    scale = [float(v) for v in accel["scale"]]
                self.get_logger().info(f"Accel calibration loaded from {path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to read {path} ({e}); falling back to parameters")
        elif path is not None:
            self.get_logger().warn(f"Calibration file not found: {path}; using parameter values")

        if len(bias) != 3 or len(scale) != 3:
            self.get_logger().error("accel_bias/accel_scale must have 3 elements; using identity")
            bias, scale = [0.0] * 3, [1.0] * 3
        if any(s == 0.0 for s in scale):
            self.get_logger().error("accel_scale contains 0; using identity scale")
            scale = [1.0] * 3

        if bias == [0.0] * 3 and scale == [1.0] * 3:
            self.get_logger().warn(
                "Accelerometer is UNCALIBRATED (bias=0, scale=1). "
                "Run scripts/calibrate_accel.py — MPU6050 clones are often "
                "10%+ off on accel sensitivity."
            )
        else:
            self.get_logger().info(
                f"Accel correction: bias=[{bias[0]:.4f}, {bias[1]:.4f}, {bias[2]:.4f}] m/s², "
                f"scale=[{scale[0]:.5f}, {scale[1]:.5f}, {scale[2]:.5f}]"
            )
        return bias, scale

    def _read_raw_data(self):
        """가속도(6) + 온도(2) + 자이로(6) = 14바이트 한번에 읽기."""
        data = self._bus.read_i2c_block_data(self._addr, _ACCEL_XOUT_H, 14)
        vals = struct.unpack(">hhhhhhh", bytes(data))
        # vals: ax, ay, az, temp, gx, gy, gz
        return vals

    def _accel_mps2(self, ax_raw, ay_raw, az_raw):
        """원시 카운트 → 보정된 가속도 [m/s²]."""
        b = self._accel_corr_bias
        s = self._accel_corr_scale
        return tuple(
            (raw / self._accel_scale * _GRAVITY - b[i]) * s[i]
            for i, raw in enumerate((ax_raw, ay_raw, az_raw))
        )

    def _calibrate(self, samples):
        # Warmup: 센서 settling을 위해 첫 100샘플 폐기
        for _ in range(100):
            self._read_raw_data()
            time.sleep(0.005)

        gyro = [[], [], []]
        accel = [[], [], []]
        for _ in range(samples):
            ax, ay, az, _, gx, gy, gz = self._read_raw_data()
            axc, ayc, azc = self._accel_mps2(ax, ay, az)
            accel[0].append(axc)
            accel[1].append(ayc)
            accel[2].append(azc)
            gyro[0].append(gx / self._gyro_scale * _DEG_TO_RAD)
            gyro[1].append(gy / self._gyro_scale * _DEG_TO_RAD)
            gyro[2].append(gz / self._gyro_scale * _DEG_TO_RAD)
            time.sleep(0.005)

        def mean(v):
            return sum(v) / len(v)

        def std(v, m):
            return (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5

        for i in range(3):
            self._gyro_bias[i] = mean(gyro[i])
            self._accel_rest[i] = mean(accel[i])

        # 정지 상태 검증: gyro std > 0.01 rad/s면 흔들렸다는 뜻
        gyro_std = [std(gyro[i], self._gyro_bias[i]) for i in range(3)]
        max_std = max(gyro_std)
        if max_std > 0.01:
            self.get_logger().warn(
                f"Sensor was NOT still during calibration (max gyro std={max_std:.4f} rad/s). "
                f"Bias estimate may be inaccurate — restart node with sensor stationary."
            )
        else:
            self.get_logger().info(
                f"Stationary check OK (gyro std=[{gyro_std[0]:.5f}, {gyro_std[1]:.5f}, {gyro_std[2]:.5f}] rad/s)"
            )

    def _check_gravity(self):
        """정지 상태 |a| 가 중력과 일치하는지 확인 (보정이 먹었는지 검증)."""
        mag = sum(v * v for v in self._accel_rest) ** 0.5
        err = mag - _GRAVITY
        if abs(err) <= 0.25:
            self.get_logger().info(f"Gravity check OK: |a|={mag:.3f} m/s² ({err:+.3f} vs 9.807)")
        else:
            self.get_logger().warn(
                f"Gravity check FAILED: |a|={mag:.3f} m/s² ({err:+.3f} vs 9.807, "
                f"{err / _GRAVITY * 100:+.1f}%). "
                f"Run scripts/calibrate_accel.py to (re)calibrate the accelerometer."
            )

    def _timer_callback(self):
        try:
            ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw = self._read_raw_data()
        except OSError as e:
            self.get_logger().warn(f"I2C read error: {e}")
            return

        now = self.get_clock().now().to_msg()

        # 단위 변환: 가속도 → m/s² (bias/scale 보정 포함), 자이로 → rad/s
        ax, ay, az = self._accel_mps2(ax_raw, ay_raw, az_raw)
        gx = (gx_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[0]
        gy = (gy_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[1]
        gz = (gz_raw / self._gyro_scale * _DEG_TO_RAD) - self._gyro_bias[2]

        # IMU 메시지
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        # orientation은 raw 드라이버에서 제공하지 않음
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = 0.0004
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self._imu_pub.publish(msg)

        # 온도 메시지
        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = self._frame_id
        temp_msg.temperature = temp_raw / 340.0 + 36.53
        self._temp_pub.publish(temp_msg)

    def destroy_node(self):
        self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
