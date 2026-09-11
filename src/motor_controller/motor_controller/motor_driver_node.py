#!/usr/bin/env python3
"""
motor_driver_node.py
/cmd_vel (geometry_msgs/Twist) 를 구독하여 L298N 모터 드라이버로 PWM 출력.

미분구동 역기구학:
  v_left  = linear.x - angular.z * wheel_base / 2
  v_right = linear.x + angular.z * wheel_base / 2

안전장치:
  - 속도 클램프 (max_linear_speed / max_angular_speed)
  - /cmd_vel 타임아웃 (cmd_timeout 초 동안 미수신 시 자동 정지) — Wi-Fi 끊김 대비
  - 노드 종료 시 모터 정지 + GPIO.cleanup()
  - dead-zone 이하 목표속도는 0 으로, 그 이상은 min_pwm ~ 100 구간으로 매핑

GPIO (BCM):
  Left  : ENA=24, IN1=22, IN2=23
  Right : ENB=25, IN3=27, IN4=26
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from RPi import GPIO


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # ─── 파라미터 ────────────────────────────────────────────
        # GPIO 핀 (BCM)
        self.declare_parameter("left_ena_pin", 24)
        self.declare_parameter("left_in1_pin", 22)
        self.declare_parameter("left_in2_pin", 23)
        self.declare_parameter("right_enb_pin", 25)
        self.declare_parameter("right_in3_pin", 27)
        self.declare_parameter("right_in4_pin", 26)

        # 기구 파라미터
        self.declare_parameter("wheel_base", 0.150)  # m

        # 배선 극성 반전 (L298N IN 핀이 코드 기준과 반대로 배선된 경우 True)
        self.declare_parameter("left_invert", False)
        self.declare_parameter("right_invert", False)

        # 속도 제한 (SLAM 맵핑 중에는 느리게)
        self.declare_parameter("max_linear_speed", 0.3)  # m/s
        self.declare_parameter("max_angular_speed", 2.5)  # rad/s

        # PWM
        self.declare_parameter("pwm_frequency", 1000.0)  # Hz
        self.declare_parameter("min_pwm", 50.0)  # dead-zone 보정 (duty %)
        self.declare_parameter("max_pwm", 100.0)  # 최대 duty %

        # 안전
        self.declare_parameter("cmd_timeout", 0.5)  # s
        self.declare_parameter("watchdog_rate", 10.0)  # Hz

        self.left_ena = self.get_parameter("left_ena_pin").value
        self.left_in1 = self.get_parameter("left_in1_pin").value
        self.left_in2 = self.get_parameter("left_in2_pin").value
        self.right_enb = self.get_parameter("right_enb_pin").value
        self.right_in3 = self.get_parameter("right_in3_pin").value
        self.right_in4 = self.get_parameter("right_in4_pin").value

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.left_invert = bool(self.get_parameter("left_invert").value)
        self.right_invert = bool(self.get_parameter("right_invert").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.pwm_frequency = float(self.get_parameter("pwm_frequency").value)
        self.min_pwm = float(self.get_parameter("min_pwm").value)
        self.max_pwm = float(self.get_parameter("max_pwm").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        # 속도 → PWM 매핑 기준 (최대 linear 속도를 max_pwm 에 대응)
        # 좌/우 휠 목표속도의 절대값을 max_wheel_speed 로 나눠 비율화.
        self.max_wheel_speed = (
            self.max_linear_speed + self.max_angular_speed * self.wheel_base / 2.0
        )

        # ─── GPIO 설정 ───────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (
            self.left_ena,
            self.left_in1,
            self.left_in2,
            self.right_enb,
            self.right_in3,
            self.right_in4,
        ):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self._pwm_left = GPIO.PWM(self.left_ena, self.pwm_frequency)
        self._pwm_right = GPIO.PWM(self.right_enb, self.pwm_frequency)
        self._pwm_left.start(0.0)
        self._pwm_right.start(0.0)

        # ─── 상태 ────────────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self._stopped = True

        # ─── 구독자 / 워치독 ─────────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, 10)

        watchdog_period = 1.0 / float(self.get_parameter("watchdog_rate").value)
        self.create_timer(watchdog_period, self._watchdog)

        self.get_logger().info(
            f"MotorDriverNode 시작: wheel_base={self.wheel_base}m, "
            f"max_lin={self.max_linear_speed}m/s, max_ang={self.max_angular_speed}rad/s, "
            f"min_pwm={self.min_pwm}%, timeout={self.cmd_timeout}s"
        )

    # ─── /cmd_vel 콜백 ──────────────────────────────────────────
    def _cb_cmd_vel(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        # 1) 속도 클램프
        lin = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        ang = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))

        # 2) 차동 역기구학
        v_left = lin - ang * self.wheel_base / 2.0
        v_right = lin + ang * self.wheel_base / 2.0

        # 2-1) 배선 극성 반전 보정
        if self.left_invert:
            v_left = -v_left
        if self.right_invert:
            v_right = -v_right

        # 3) 각 바퀴 PWM + 방향 적용
        self._set_wheel(self.left_in1, self.left_in2, self._pwm_left, v_left)
        self._set_wheel(self.right_in3, self.right_in4, self._pwm_right, v_right)
        self._stopped = False

    # ─── 바퀴 제어 ──────────────────────────────────────────────
    def _set_wheel(self, in_a, in_b, pwm, v):
        # 목표 휠 속도(m/s)를 PWM duty(%) 로 매핑
        # - |v| 가 너무 작으면 정지 (dead-zone 미만은 움직이지 못함)
        # - 그 외엔 min_pwm ~ max_pwm 구간에 선형 매핑
        if self.max_wheel_speed <= 0.0:
            duty = 0.0
        else:
            ratio = min(1.0, abs(v) / self.max_wheel_speed)
            duty = 0.0 if ratio < 0.001 else self.min_pwm + (self.max_pwm - self.min_pwm) * ratio

        if duty <= 0.0:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.LOW)
            pwm.ChangeDutyCycle(0.0)
            return

        if v >= 0.0:
            GPIO.output(in_a, GPIO.HIGH)
            GPIO.output(in_b, GPIO.LOW)
        else:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.HIGH)
        pwm.ChangeDutyCycle(duty)

    # ─── 워치독 ─────────────────────────────────────────────────
    def _watchdog(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout and not self._stopped:
            self.get_logger().warn(f"/cmd_vel 타임아웃 ({elapsed:.2f}s) → 모터 정지")
            self._stop()

    def _stop(self):
        for pin in (self.left_in1, self.left_in2, self.right_in3, self.right_in4):
            GPIO.output(pin, GPIO.LOW)
        self._pwm_left.ChangeDutyCycle(0.0)
        self._pwm_right.ChangeDutyCycle(0.0)
        self._stopped = True

    # ─── 종료 ───────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._stop()
            self._pwm_left.stop()
            self._pwm_right.stop()
        finally:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
