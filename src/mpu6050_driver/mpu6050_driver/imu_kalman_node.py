import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Kalman1D:
    """2-state Kalman filter estimating [angle, gyro_bias] for one axis.

    Prediction uses the gyro rate; correction uses the angle derived from
    the accelerometer. Classic formulation (Kristian Lauszus / TKJ).
    """

    def __init__(self, q_angle, q_bias, r_measure):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure

        self.angle = 0.0
        self.bias = 0.0
        # error covariance matrix P
        self.p = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, new_angle, new_rate, dt):
        # --- Predict ---
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle)
        self.p[0][1] -= dt * self.p[1][1]
        self.p[1][0] -= dt * self.p[1][1]
        self.p[1][1] += self.q_bias * dt

        # --- Update ---
        s = self.p[0][0] + self.r_measure
        k0 = self.p[0][0] / s
        k1 = self.p[1][0] / s

        y = new_angle - self.angle
        self.angle += k0 * y
        self.bias += k1 * y

        p00 = self.p[0][0]
        p01 = self.p[0][1]
        self.p[0][0] -= k0 * p00
        self.p[0][1] -= k0 * p01
        self.p[1][0] -= k1 * p00
        self.p[1][1] -= k1 * p01

        return self.angle

    def set_angle(self, angle):
        self.angle = angle


class ImuKalmanNode(Node):
    """Fuses gyro + accel into an orientation estimate with a Kalman filter.

    Subscribes to raw IMU (imu/data_raw) and republishes the same message
    with orientation filled in on imu/data. Roll and pitch are observable
    from gravity and corrected by the filter; yaw has no absolute reference
    (no magnetometer) so it is integrated from the gyro and will drift.
    """

    def __init__(self):
        super().__init__("imu_kalman")

        self.declare_parameter("q_angle", 0.001)
        self.declare_parameter("q_bias", 0.003)
        self.declare_parameter("r_measure", 0.03)

        q_angle = self.get_parameter("q_angle").value
        q_bias = self.get_parameter("q_bias").value
        r_measure = self.get_parameter("r_measure").value

        self._kf_roll = Kalman1D(q_angle, q_bias, r_measure)
        self._kf_pitch = Kalman1D(q_angle, q_bias, r_measure)

        self._yaw = 0.0
        self._last_stamp = None
        self._initialized = False

        self._sub = self.create_subscription(Imu, "imu/data_raw", self._imu_callback, 10)
        self._pub = self.create_publisher(Imu, "imu/data", 10)

        self.get_logger().info("IMU Kalman filter running: imu/data_raw -> imu/data")

    def _imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Roll/pitch from gravity vector (rad).
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        if not self._initialized:
            # Seed the filters with the first accelerometer reading.
            self._kf_roll.set_angle(roll_acc)
            self._kf_pitch.set_angle(pitch_acc)
            self._last_stamp = t
            self._initialized = True
            return

        dt = t - self._last_stamp
        self._last_stamp = t
        if dt <= 0.0 or dt > 0.5:
            # Skip implausible time steps (clock jumps / dropped messages).
            return

        # gx, gy approximate roll/pitch rates for small angles.
        roll = self._kf_roll.update(roll_acc, gx, dt)
        pitch = self._kf_pitch.update(pitch_acc, gy, dt)

        # Yaw: integrate gyro z (no absolute reference without a magnetometer).
        self._yaw += gz * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, self._yaw)

        out = msg
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.orientation_covariance[0] = 0.0025
        out.orientation_covariance[4] = 0.0025
        out.orientation_covariance[8] = 0.01  # yaw less certain (drift)

        self._pub.publish(out)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = ImuKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
