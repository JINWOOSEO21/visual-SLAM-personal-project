#!/usr/bin/env python3
"""
pid_controller_node.py

Closed-loop wheel velocity PID controller for RC car on Raspberry Pi.

Subscriptions:
  /cmd_vel              (geometry_msgs/Twist)  — target v, w from NMPC
  /encoder/ticks        (std_msgs/Int32MultiArray) — [left_ticks, right_ticks]

GPIO output (L298N):
  Left  wheel : ENA=GPIO24 (PWM), IN1=GPIO22, IN2=GPIO23
  Right wheel : ENB=GPIO25 (PWM), IN3=GPIO27, IN4=GPIO26

Control loop:
  1. Differential kinematics: (v, w) → target_vL, target_vR
  2. Velocity measurement: Δticks * dist_per_tick / dt  (moving average for smoothing)
  3. PID error: |target| - measured  (unsigned; direction set separately from target sign)
  4. Feedforward duty from identified velocity-duty model
  5. PID output: raw duty correction
  6. GPIO: set direction pins, apply clipped PWM duty

NOTE: Do NOT run simultaneously with motor_controller/motor_driver_node.
      Both packages control the same GPIO pins (ENA/ENB/IN1-4).

Encoder direction:
  encoder_node publishes unsigned cumulative ticks (no direction encoding).
  Direction is inferred from the commanded velocity sign and applied to
  the L298N IN pins.  On direction reversal the PID integral is reset.
"""

import csv
import datetime
import math
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from RPi import GPIO
from std_msgs.msg import Int32MultiArray

from pid_velocity_controller.pid import PID

# Seconds below which a velocity command is treated as "stop"
_STOP_THRESHOLD = 0.005  # m/s


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__("pid_controller_node")

        # ── Parameters ────────────────────────────────────────────
        # GPIO (BCM numbering)
        self.declare_parameter("left_ena_pin", 24)
        self.declare_parameter("left_in1_pin", 22)
        self.declare_parameter("left_in2_pin", 23)
        self.declare_parameter("right_enb_pin", 25)
        self.declare_parameter("right_in3_pin", 27)
        self.declare_parameter("right_in4_pin", 26)

        # Wheel geometry (must match encoder_node / odometry_node)
        self.declare_parameter("wheel_base", 0.150)  # m
        self.declare_parameter("wheel_diameter", 0.065)  # m
        self.declare_parameter("pulses_per_rev", 40.0)  # ticks/rev (BOTH edge)

        # Wiring inversion (True = L298N IN wiring is reversed for that side)
        self.declare_parameter("left_invert", True)
        self.declare_parameter("right_invert", True)

        # PWM
        self.declare_parameter("pwm_frequency", 1000.0)  # Hz
        self.declare_parameter("max_pwm", 100.0)  # %
        self.declare_parameter("ff_velocity_slope", 0.00446)
        self.declare_parameter("ff_velocity_intercept", 0.217)

        # PID gains — left wheel
        self.declare_parameter("left_kp", 80.0)
        self.declare_parameter("left_ki", 20.0)
        self.declare_parameter("left_kd", 2.0)

        # PID gains — right wheel
        self.declare_parameter("right_kp", 80.0)
        self.declare_parameter("right_ki", 20.0)
        self.declare_parameter("right_kd", 2.0)

        # PID limits
        self.declare_parameter("integral_limit", 40.0)  # duty %
        self.declare_parameter("derivative_tau", 0.05)  # LPF time constant [s]

        # Safety
        self.declare_parameter("cmd_timeout", 0.5)  # s — stop if /cmd_vel stale
        self.declare_parameter("control_rate", 10.0)  # Hz — PID loop frequency

        # Velocity smoothing
        self.declare_parameter("vel_smooth_window", 3)  # ticks window size

        # ── Read parameters ───────────────────────────────────────
        self._left_ena = self.get_parameter("left_ena_pin").value
        self._left_in1 = self.get_parameter("left_in1_pin").value
        self._left_in2 = self.get_parameter("left_in2_pin").value
        self._right_enb = self.get_parameter("right_enb_pin").value
        self._right_in3 = self.get_parameter("right_in3_pin").value
        self._right_in4 = self.get_parameter("right_in4_pin").value

        wheel_base = self.get_parameter("wheel_base").value
        wheel_diameter = self.get_parameter("wheel_diameter").value
        pulses_per_rev = self.get_parameter("pulses_per_rev").value
        self._wheel_base = float(wheel_base)
        self._dist_per_tick = (math.pi * float(wheel_diameter)) / float(pulses_per_rev)

        self._left_invert = bool(self.get_parameter("left_invert").value)
        self._right_invert = bool(self.get_parameter("right_invert").value)

        self._max_pwm = float(self.get_parameter("max_pwm").value)
        self._ff_velocity_slope = float(self.get_parameter("ff_velocity_slope").value)
        self._ff_velocity_intercept = float(self.get_parameter("ff_velocity_intercept").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        if self._ff_velocity_slope <= 0.0:
            raise ValueError("ff_velocity_slope must be > 0")

        smooth_n = int(self.get_parameter("vel_smooth_window").value)
        control_rate = float(self.get_parameter("control_rate").value)
        self._dt = 1.0 / control_rate

        int_lim = float(self.get_parameter("integral_limit").value)
        deriv_tau = float(self.get_parameter("derivative_tau").value)

        # ── PID controllers (one per wheel) ───────────────────────
        # PID returns a raw duty correction. Final actuator clipping is applied
        # after adding the feedforward duty in _apply_wheel().
        self._left_kp = float(self.get_parameter("left_kp").value)
        self._left_ki = float(self.get_parameter("left_ki").value)
        self._left_kd = float(self.get_parameter("left_kd").value)
        self._right_kp = float(self.get_parameter("right_kp").value)
        self._right_ki = float(self.get_parameter("right_ki").value)
        self._right_kd = float(self.get_parameter("right_kd").value)

        self._pid_left = PID(
            kp=self._left_kp,
            ki=self._left_ki,
            kd=self._left_kd,
            integral_limit=int_lim,
            derivative_tau=deriv_tau,
        )
        self._pid_right = PID(
            kp=self._right_kp,
            ki=self._right_ki,
            kd=self._right_kd,
            integral_limit=int_lim,
            derivative_tau=deriv_tau,
        )

        # ── GPIO setup ────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (
            self._left_ena,
            self._left_in1,
            self._left_in2,
            self._right_enb,
            self._right_in3,
            self._right_in4,
        ):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        pwm_freq = float(self.get_parameter("pwm_frequency").value)
        self._pwm_left = GPIO.PWM(self._left_ena, pwm_freq)
        self._pwm_right = GPIO.PWM(self._right_enb, pwm_freq)
        self._pwm_left.start(0.0)
        self._pwm_right.start(0.0)

        # ── State ─────────────────────────────────────────────────
        self._cmd_v = 0.0  # latest cmd_vel linear.x  [m/s]
        self._cmd_w = 0.0  # latest cmd_vel angular.z [rad/s]
        self._target_vL = 0.0  # target left  wheel velocity [m/s], signed
        self._target_vR = 0.0  # target right wheel velocity [m/s], signed
        self._last_cmd_time = self.get_clock().now()

        self._left_ticks_prev = None  # int or None (None until first message)
        self._right_ticks_prev = None
        self._left_ticks_cur = 0
        self._right_ticks_cur = 0

        # Moving-average buffers for smoothing velocity estimates
        self._vel_buf_L = deque([0.0] * smooth_n, maxlen=smooth_n)
        self._vel_buf_R = deque([0.0] * smooth_n, maxlen=smooth_n)

        # Previous direction sign (for detecting reversal and resetting PID)
        self._prev_dir_L = 0  # -1, 0, +1
        self._prev_dir_R = 0

        # ── Result logging ────────────────────────────────────────
        result_dir = Path(__file__).resolve().parent.parent / "result"
        result_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = result_dir / f"{ts}.csv"
        # 노드 수명 내내 열어 두고 _cleanup() 에서 닫으므로 with 를 쓸 수 없다.
        self._csv_file = log_path.open("w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "time_s",
                "cmd_v",
                "cmd_w",
                "target_vL",
                "target_vR",
                "left_ticks",
                "right_ticks",
                "meas_vL",
                "meas_vR",
                "duty_L",
                "duty_R",
            ]
        )
        self._log_start_time = self.get_clock().now()
        self.get_logger().info(f"Logging PID data to {log_path}")

        # ── ROS interfaces ────────────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, 10)
        self.create_subscription(Int32MultiArray, "/encoder/ticks", self._cb_ticks, 10)

        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"PIDControllerNode started: control_rate={control_rate:.0f}Hz "
            f"dist_per_tick={self._dist_per_tick * 1000:.3f}mm "
            f"ff_model: v={self._ff_velocity_slope:.5f}*duty+"
            f"{self._ff_velocity_intercept:.3f} "
            f"max_pwm={self._max_pwm:.1f}%"
        )

    # ── Subscription callbacks ────────────────────────────────────
    def _cb_cmd_vel(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        self._cmd_v = v
        self._cmd_w = w
        self._target_vL = v - w * self._wheel_base / 2.0
        self._target_vR = v + w * self._wheel_base / 2.0

    def _cb_ticks(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            msg_len = len(msg.data)
            self.get_logger().warn(
                f"/encoder/ticks message must contain [left, right], got {msg_len} values",
                throttle_duration_sec=1.0,
            )
            return
        self._left_ticks_cur = int(msg.data[0])
        self._right_ticks_cur = int(msg.data[1])

    # ── Velocity measurement ──────────────────────────────────────
    def _measure_velocities(self) -> tuple:
        """
        Compute instantaneous wheel speeds from tick differences.
        Returns (v_measured_L, v_measured_R) in m/s (always >= 0).
        """
        if self._left_ticks_prev is None:
            # First call: initialise previous tick counters
            self._left_ticks_prev = self._left_ticks_cur
            self._right_ticks_prev = self._right_ticks_cur
            return 0.0, 0.0

        d_ticks_L = self._left_ticks_cur - self._left_ticks_prev
        d_ticks_R = self._right_ticks_cur - self._right_ticks_prev
        self._left_ticks_prev = self._left_ticks_cur
        self._right_ticks_prev = self._right_ticks_cur

        # Unsigned speed (encoder_node has no direction encoding)
        raw_vL = abs(d_ticks_L) * self._dist_per_tick / self._dt
        raw_vR = abs(d_ticks_R) * self._dist_per_tick / self._dt

        # Moving-average smoothing to reduce quantisation noise
        self._vel_buf_L.append(raw_vL)
        self._vel_buf_R.append(raw_vR)
        return (
            sum(self._vel_buf_L) / len(self._vel_buf_L),
            sum(self._vel_buf_R) / len(self._vel_buf_R),
        )

    # ── GPIO helpers ──────────────────────────────────────────────
    def _stop_wheel(self, in_a, in_b, pwm_obj, pid: PID) -> tuple:
        GPIO.output(in_a, GPIO.LOW)
        GPIO.output(in_b, GPIO.LOW)
        pwm_obj.ChangeDutyCycle(0.0)
        pid.reset()
        return 0, 0.0

    def _feedforward_duty(self, target_speed: float) -> float:
        """Invert v = slope*duty + intercept for the requested target speed."""
        duty = (abs(target_speed) - self._ff_velocity_intercept) / self._ff_velocity_slope
        return max(0.0, duty)

    def _apply_wheel(
        self,
        in_a,
        in_b,
        pwm_obj,
        target_v: float,
        pid: PID,
        measured_speed: float,
        prev_dir: int,
        invert: bool,
    ) -> tuple:
        """
        Run one PID step and apply the result to a single wheel.

        Args:
            in_a, in_b   : L298N direction GPIO pins
            pwm_obj      : GPIO.PWM object for this wheel
            target_v     : signed target wheel velocity [m/s]
            pid          : PID instance for this wheel
            measured_speed: unsigned measured speed [m/s]
            prev_dir     : previous direction (-1, 0, +1)
            invert       : True if wiring polarity is reversed

        Returns:
            current direction (-1, 0, +1)
        """
        if abs(target_v) < _STOP_THRESHOLD:
            return self._stop_wheel(in_a, in_b, pwm_obj, pid)

        cur_dir = 1 if target_v > 0 else -1

        # Reset PID on direction reversal
        if cur_dir != prev_dir and prev_dir != 0:
            pid.reset()

        # PID operates on speed magnitude
        error = abs(target_v) - measured_speed
        pid_out = pid.compute(error, self._dt)
        duty_ff = self._feedforward_duty(target_v)
        duty = max(0.0, min(self._max_pwm, duty_ff + pid_out))

        # Direction pin logic (invert swap if wiring is reversed)
        forward = (cur_dir == 1) ^ invert
        if forward:
            GPIO.output(in_a, GPIO.HIGH)
            GPIO.output(in_b, GPIO.LOW)
        else:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.HIGH)

        pwm_obj.ChangeDutyCycle(duty)
        return cur_dir, duty

    # ── control loop ────────────────────────────────────────
    def _control_loop(self):
        # Safety: stop on cmd_vel timeout
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._cmd_timeout:
            if self._target_vL != 0.0 or self._target_vR != 0.0:
                self.get_logger().warn(
                    f"/cmd_vel timeout ({elapsed:.2f}s) — stopping motors",
                    throttle_duration_sec=1.0,
                )
            self._stop_all()
            return

        meas_vL, meas_vR = self._measure_velocities()

        self._prev_dir_L, duty_L = self._apply_wheel(
            self._left_in1,
            self._left_in2,
            self._pwm_left,
            self._target_vL,
            self._pid_left,
            meas_vL,
            self._prev_dir_L,
            self._left_invert,
        )
        self._prev_dir_R, duty_R = self._apply_wheel(
            self._right_in3,
            self._right_in4,
            self._pwm_right,
            self._target_vR,
            self._pid_right,
            meas_vR,
            self._prev_dir_R,
            self._right_invert,
        )

        elapsed_s = (self.get_clock().now() - self._log_start_time).nanoseconds / 1e9
        self._csv_writer.writerow(
            [
                f"{elapsed_s:.3f}",
                f"{self._cmd_v:.4f}",
                f"{self._cmd_w:.4f}",
                f"{self._target_vL:.4f}",
                f"{self._target_vR:.4f}",
                self._left_ticks_cur,
                self._right_ticks_cur,
                f"{meas_vL:.4f}",
                f"{meas_vR:.4f}",
                f"{duty_L:.2f}",
                f"{duty_R:.2f}",
            ]
        )

        self.get_logger().info(
            f"[PID] "
            f"tgt=[{self._target_vL:+.3f},{self._target_vR:+.3f}]m/s  "
            f"meas=[{meas_vL:.3f},{meas_vR:.3f}]m/s  "
            f"err=[{abs(self._target_vL) - meas_vL:+.3f},"
            f"{abs(self._target_vR) - meas_vR:+.3f}]",
            throttle_duration_sec=0.5,
        )

    # ── Stop & cleanup ────────────────────────────────────────────
    def _stop_all(self):
        for pin in (self._left_in1, self._left_in2, self._right_in3, self._right_in4):
            GPIO.output(pin, GPIO.LOW)
        self._pwm_left.ChangeDutyCycle(0.0)
        self._pwm_right.ChangeDutyCycle(0.0)
        self._pid_left.reset()
        self._pid_right.reset()
        self._target_vL = 0.0
        self._target_vR = 0.0

    def destroy_node(self):
        try:
            self._stop_all()
            self._pwm_left.stop()
            self._pwm_right.stop()
        finally:
            GPIO.cleanup()
        if hasattr(self, "_csv_file"):
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
