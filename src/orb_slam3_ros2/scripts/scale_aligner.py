#!/usr/bin/env python3
"""ORB-SLAM3 단안 포즈를 휠 오도메트리로 미터 단위화하고 map -> odom 을 발행한다.

입력
    /orb_slam3/camera_pose   T_world_cam, 스케일 없음 (mono_node)
    /orb_slam3/state         [tracking 상태, 맵 epoch, 큰 변경 횟수]
    /orb_slam3/map_points    orb_world 프레임 점들, 스케일 없음
    TF odom -> base_link     Pi 의 EKF (휠 + IMU)
    TF base_link -> camera   static TF (sensor_tf.launch.py)
출력
    TF map -> odom           slam.launch.py 의 백엔드 계약
    /orb_slam3/odometry      nav_msgs/Odometry, map 프레임, 미터 단위
    /orb_slam3/map_points_map  map 프레임, 미터 단위 (RViz 용)

스케일
    단안 SLAM 의 이동량은 임의 배율이다. 같은 구간에서 ORB-SLAM3 가 본 카메라 이동
    거리와 오도메트리가 잰 카메라 이동 거리를 비교해 배율 s 를 구한다. 카메라 위치끼리
    비교하므로(오도메트리 쪽은 odom->base_link 에 base_link->camera 를 곱해 얻는다)
    제자리 회전처럼 카메라가 호를 그리는 움직임도 양쪽이 같은 양으로 잰다.
    구간은 오도메트리 기준 min_segment 이상 움직일 때마다 하나씩 쌓고, 최근 window 개
    구간에서 중앙값의 0.5~2 배를 벗어나는 구간(휠 슬립 등)을 빼고 합의 비율로 구한다.
    누적 이동이 min_path 를 넘기 전에는 map -> odom 을 갱신하지 않는다.

map 프레임
    ORB-SLAM3 의 월드 프레임은 맵을 초기화한 키프레임의 카메라 프레임이고, 추적을
    잃고 새 맵을 만들 때마다 바뀐다. 그대로 쓰면 map 이 그때마다 튄다. 그래서 맵마다
    첫 추적 시점을 앵커로 잡는다:

        T_map_base(t) = A · T_base_cam · S(T_cam0_cam(t)) · T_cam_base
        A             = 앵커 시점의 T_map_base = (그때의 map->odom) · T_odom_base(t0)
        T_cam0_cam(t) = inv(T_world_cam(t0)) · T_world_cam(t),  S 는 평행이동에 s 를 곱함

    첫 맵에서는 map->odom 이 항등이므로 map 원점 = 맵을 시작한 순간의 odom 자세가
    된다. 이후 새 맵은 직전의 map->odom 을 이어받아 시작하므로 map 이 연속이다.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32MultiArray

TRACKING_OK = 2  # ORB_SLAM3::Tracking::OK


def quat_to_rot(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(
        math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2]
    )
    y = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0]
    )
    z = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1]
    )
    return x, y, z, w


def make_T(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def T_from_transform(tf):
    q, p = tf.rotation, tf.translation
    return make_T(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def planar(t):
    """x, y, yaw 만 남긴다 (평면 주행 로봇)."""
    yaw = math.atan2(t[1, 0], t[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return make_T(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), [t[0, 3], t[1, 3], 0.0])


class ScaleAligner(Node):
    def __init__(self):
        super().__init__("orb_scale_aligner")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.camera_frame = p("camera_frame", "camera").value
        self.min_segment = p("min_segment", 0.10).value  # m
        self.min_path = p("min_path", 0.5).value  # m
        self.window = p("window", 60).value  # 구간 수
        self.planar = p("planar", True).value
        self.tf_lead = p("tf_lead", 0.1).value  # s, map->odom 을 약간 미래로 찍는다
        rate = p("publish_rate", 20.0).value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.T_base_cam = None
        self.T_map_odom = np.eye(4)  # 스케일이 정해지기 전엔 항등
        self.epoch = None  # mono_node 가 알려준 맵 epoch
        self._reset_map()

        self.odom_pub = self.create_publisher(Odometry, "/orb_slam3/odometry", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/orb_slam3/map_points_map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Int32MultiArray, "/orb_slam3/state", self.on_state, 10)
        self.create_subscription(PoseStamped, "/orb_slam3/camera_pose", self.on_pose, 10)
        self.create_subscription(
            PointCloud2,
            "/orb_slam3/map_points",
            self.on_cloud,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_timer(1.0 / rate, self.publish_tf)

    # ── 맵 단위 상태 ──────────────────────────────────────────────
    def _reset_map(self):
        self.anchor = None  # A = 앵커 시점의 T_map_base
        self.cam0_inv = None  # inv(T_world_cam(t0))
        self.scale = None
        self.segments = []  # (오도메트리 거리, ORB 거리)
        self.last_sample = None
        self.path_len = 0.0

    def on_state(self, msg):
        if len(msg.data) < 2:
            return
        epoch = msg.data[1]
        if epoch != self.epoch:
            if self.epoch is not None:
                self.get_logger().info(
                    f"맵 epoch {self.epoch} -> {epoch}: 스케일을 다시 구한다 "
                    "(map->odom 은 이어받는다)"
                )
            self.epoch = epoch
            self._reset_map()

    # ── 포즈 처리 ────────────────────────────────────────────────
    def _lookup(self, target, source, stamp, timeout=0.05):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=timeout)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return None
        return T_from_transform(tf.transform)

    def on_pose(self, msg):
        if self.T_base_cam is None:
            self.T_base_cam = self._lookup(self.base_frame, self.camera_frame, Time(), timeout=0.5)
            if self.T_base_cam is None:
                self.get_logger().warn(
                    f"static TF {self.base_frame}->{self.camera_frame} 대기 중",
                    throttle_duration_sec=5,
                )
                return

        stamp = Time.from_msg(msg.header.stamp)
        T_odom_base = self._lookup(self.odom_frame, self.base_frame, stamp)
        if T_odom_base is None:
            self.get_logger().warn(
                f"{self.odom_frame}->{self.base_frame} 를 이미지 시각에 못 찾음",
                throttle_duration_sec=5,
            )
            return

        q, t = msg.pose.orientation, msg.pose.position
        T_world_cam = make_T(quat_to_rot(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])

        if self.anchor is None:
            self.anchor = self.T_map_odom @ T_odom_base
            self.cam0_inv = np.linalg.inv(T_world_cam)
            self.get_logger().info("새 맵 앵커 설정 — 스케일 추정 시작")

        self._update_scale(T_world_cam[:3, 3], (T_odom_base @ self.T_base_cam)[:3, 3])
        if self.scale is None:
            return

        rel = self.cam0_inv @ T_world_cam
        rel[:3, 3] *= self.scale
        T_map_base = self.anchor @ self.T_base_cam @ rel @ np.linalg.inv(self.T_base_cam)
        if self.planar:
            T_map_base = planar(T_map_base)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        self.T_map_odom = planar(T_map_odom) if self.planar else T_map_odom

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        x, y, z, w = rot_to_quat(T_map_base[:3, :3])
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = (
            T_map_base[:3, 3]
        )
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = z, w
        self.odom_pub.publish(odom)

    def _update_scale(self, p_orb, p_odom):
        if self.last_sample is None:
            self.last_sample = (p_orb, p_odom)
            return
        d_odom = float(np.linalg.norm(p_odom - self.last_sample[1]))
        if d_odom < self.min_segment:
            return
        d_orb = float(np.linalg.norm(p_orb - self.last_sample[0]))
        self.last_sample = (p_orb, p_odom)
        if d_orb < 1e-6:
            return
        self.segments.append((d_odom, d_orb))
        self.segments = self.segments[-self.window :]
        self.path_len += d_odom
        if self.path_len < self.min_path:
            return

        seg = np.array(self.segments)
        ratio = seg[:, 0] / seg[:, 1]
        med = float(np.median(ratio))
        keep = (ratio > 0.5 * med) & (ratio < 2.0 * med)
        new_scale = float(seg[keep, 0].sum() / seg[keep, 1].sum())
        if self.scale is None:
            self.get_logger().info(
                f"스케일 확정: 1 ORB 단위 = {new_scale:.3f} m ({keep.sum()}/{len(seg)} 구간)"
            )
        self.scale = new_scale

    # ── 출력 ─────────────────────────────────────────────────────
    def publish_tf(self):
        tf = TransformStamped()
        tf.header.stamp = (self.get_clock().now() + Duration(seconds=self.tf_lead)).to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
            self.T_map_odom[:3, 3]
        )
        x, y, z, w = rot_to_quat(self.T_map_odom[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = z, w
        self.tf_pub.sendTransform(tf)

    def on_cloud(self, msg):
        if self.scale is None or self.anchor is None or self.T_base_cam is None or msg.width == 0:
            return
        pts = np.array(
            [
                (p[0], p[1], p[2])
                for p in point_cloud2.read_points(msg, ("x", "y", "z"), skip_nans=True)
            ],
            dtype=np.float64,
        )
        if len(pts) == 0:
            return
        homog = np.hstack([pts, np.ones((len(pts), 1))])
        in_cam0 = (self.cam0_inv @ homog.T).T
        in_cam0[:, :3] *= self.scale
        in_map = (self.anchor @ self.T_base_cam @ in_cam0.T).T[:, :3]
        out = point_cloud2.create_cloud_xyz32(msg.header, in_map.astype(np.float32).tolist())
        out.header.frame_id = self.map_frame
        self.cloud_pub.publish(out)


def main():
    rclpy.init()
    node = ScaleAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
