#!/usr/bin/env python3
"""
odometry_node.py
좌/우 엔코더 틱 차이로 차동구동 오도메트리 계산 후 퍼블리시

Subscribed topics:
  /encoder/ticks (std_msgs/Int32MultiArray) : [left_ticks, right_ticks]

Published topics:
  /odom (nav_msgs/Odometry)

TF broadcast:
  odom → base_link

Parameters:
  pulses_per_rev (float, default=20.0) : 엔코더 해상도 (검증 후 수정)
  wheel_diameter (float, default=0.065): 바퀴 직경 [m]
  wheel_base     (float, default=0.150): 휠베이스 [m]
  odom_frame     (str,   default='odom')
  base_frame     (str,   default='base_link')
  publish_tf     (bool,  default=False): odom→base_link TF broadcast 여부
                                         (EKF 사용 시 False 권장)
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        # ─── 파라미터 ────────────────────────────────────────────
        self.declare_parameter("pulses_per_rev", 40.0)  # rising + falling
        self.declare_parameter("wheel_diameter", 0.065)  # m
        self.declare_parameter("wheel_base", 0.150)  # m
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)

        ppr = self.get_parameter("pulses_per_rev").value
        wheel_d = self.get_parameter("wheel_diameter").value
        self.wb = self.get_parameter("wheel_base").value
        self.odom_f = self.get_parameter("odom_frame").value
        self.base_f = self.get_parameter("base_frame").value
        self.pub_tf = self.get_parameter("publish_tf").value

        # 1 펄스당 이동 거리 [m]
        self.dist_per_tick = (math.pi * wheel_d) / ppr

        self.get_logger().info(
            f"OdometryNode 시작: PPR={ppr}, "
            f"wheel_d={wheel_d * 1000:.0f}mm, base={self.wb * 1000:.0f}mm, "
            f"dist/tick={self.dist_per_tick * 1000:.3f}mm"
        )

        # ─── 상태 변수 ───────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._prev_left = None  # 이전 틱 (첫 수신 전 None)
        self._prev_right = None
        self._prev_time = None

        self._last_left = 0
        self._last_right = 0

        # Direction signs inferred from /cmd_vel (+1 forward, -1 backward, 0 stop).
        # Encoder_node publishes unsigned cumulative ticks, so we must track direction here.
        self._dir_left = 1
        self._dir_right = 1

        # ─── 퍼블리셔 / TF ───────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.pub_tf else None

        # ─── 서브스크라이버 ──────────────────────────────────────
        self.create_subscription(Int32MultiArray, "/encoder/ticks", self._cb_ticks, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, 10)

    # ─── cmd_vel 수신: 각 바퀴 방향 부호 갱신 ────────────────────
    def _cb_cmd_vel(self, msg: Twist):
        _DEAD = 0.005  # m/s — stop threshold (matches pid_controller_node)
        v = msg.linear.x
        w = msg.angular.z
        vL = v - w * self.wb / 2.0
        vR = v + w * self.wb / 2.0
        self._dir_left = 0 if abs(vL) < _DEAD else (1 if vL > 0 else -1)
        self._dir_right = 0 if abs(vR) < _DEAD else (1 if vR > 0 else -1)

    # ─── 틱 수신 ─────────────────────────────────────────────────
    def _cb_ticks(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            msg_len = len(msg.data)
            self.get_logger().warn(
                f"/encoder/ticks message must contain [left, right], got {msg_len} values",
                throttle_duration_sec=1.0,
            )
            return
        self._last_left = int(msg.data[0])
        self._last_right = int(msg.data[1])
        self._update()

    # ─── 오도메트리 계산 ──────────────────────────────────────────
    def _update(self):
        now_ros = self.get_clock().now()

        # 첫 수신 시 기준값 설정
        if self._prev_left is None:
            self._prev_left = self._last_left
            self._prev_right = self._last_right
            self._prev_time = now_ros
            return

        # 구간 틱 차이 — 방향 부호 적용 (encoder_node는 unsigned 카운트만 함)
        d_left = (self._last_left - self._prev_left) * self.dist_per_tick * self._dir_left
        d_right = (self._last_right - self._prev_right) * self.dist_per_tick * self._dir_right

        dt = (now_ros - self._prev_time).nanoseconds * 1e-9

        self._prev_left = self._last_left
        self._prev_right = self._last_right
        self._prev_time = now_ros

        if dt <= 0.0:
            return
        # 정지 상태에서도 vx=0, vyaw=0을 EKF에 알려야 위치/yaw 드리프트가 잡힘

        # 차동구동 기구학
        d_center = (d_right + d_left) / 2.0
        d_yaw = (d_right - d_left) / self.wb

        self._yaw += d_yaw
        self._x += d_center * math.cos(self._yaw)
        self._y += d_center * math.sin(self._yaw)

        vx = d_center / dt
        v_yaw = d_yaw / dt

        now = now_ros.to_msg()

        # ─── /odom 퍼블리시 ──────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_f
        odom.child_frame_id = self.base_f

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y

        # yaw → quaternion (z축 회전만)
        odom.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw / 2.0)

        # twist (base_link 기준)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = v_yaw

        # covariance — EKF 입력용 (pose는 드리프트하므로 크게, twist는 상대적으로 작게)
        # 대각성분만 채움: [x, y, z, roll, pitch, yaw]
        LARGE = 1e6  # 사용 안 하는 축
        odom.pose.covariance[0] = 0.05  # x
        odom.pose.covariance[7] = 0.05  # y
        odom.pose.covariance[14] = LARGE  # z (미사용)
        odom.pose.covariance[21] = LARGE  # roll (미사용)
        odom.pose.covariance[28] = LARGE  # pitch (미사용)
        odom.pose.covariance[35] = 0.10  # yaw

        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.001  # vy=0 (비홀로노믹 제약)
        odom.twist.covariance[14] = LARGE  # vz
        odom.twist.covariance[21] = LARGE  # v_roll
        odom.twist.covariance[28] = LARGE  # v_pitch
        odom.twist.covariance[35] = 0.005  # v_yaw (IMU bias drift 보완)

        self._odom_pub.publish(odom)

        # ─── TF 브로드캐스트 (EKF 미사용 시만) ─────────────────
        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_f
            tf.child_frame_id = self.base_f
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = math.sin(self._yaw / 2.0)
            tf.transform.rotation.w = math.cos(self._yaw / 2.0)
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
