#!/usr/bin/env python3
"""
test_motor_encoder.py
양쪽 모터를 지정한 duty cycle로 DURATION 동안 직접 구동하며
1초부터 DURATION까지 증가한 왼쪽/오른쪽 엔코더 틱을 출력하는
스탠드얼론 테스트 스크립트.

사용법:
    sudo python3 test_motor_encoder.py
    sudo python3 test_motor_encoder.py --reverse
    (ros2 데몬과 독립적으로 동작. encoder_node 실행 중이면 GPIO 충돌하므로 종료 후 실행)
"""

import argparse
import time

import RPi.GPIO as GPIO

# ─── 핀 정의 (CLAUDE.md 기준) ─────────────────────────────────
# L298N - 왼쪽 모터
ENA = 24
IN1 = 22
IN2 = 23
# L298N - 오른쪽 모터
ENB = 25
IN3 = 27
IN4 = 26
# 엔코더
LEFT_ENC = 17
RIGHT_ENC = 16

# ─── 테스트 파라미터 ──────────────────────────────────────────
DURATION = 3.0  # 구동 시간 (초)
DUTY_CYCLE = 85  # PWM duty (%)
PWM_FREQ = 1000  # PWM 주파수 (Hz)
MEASURE_START = 1.0  # tick 집계 시작 시각 (초)

# ─── 틱 카운터 ────────────────────────────────────────────────
left_ticks = 0
right_ticks = 0


def cb_left(_channel):
    global left_ticks
    left_ticks += 1


def cb_right(_channel):
    global right_ticks
    right_ticks += 1


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # 모터 핀
    for pin in (ENA, IN1, IN2, ENB, IN3, IN4):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # 엔코더 핀
    GPIO.setup(LEFT_ENC, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENC, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENC, GPIO.BOTH, callback=cb_left, bouncetime=1)
    GPIO.add_event_detect(RIGHT_ENC, GPIO.BOTH, callback=cb_right, bouncetime=1)

    pwm_a = GPIO.PWM(ENA, PWM_FREQ)
    pwm_b = GPIO.PWM(ENB, PWM_FREQ)
    pwm_a.start(0)
    pwm_b.start(0)
    return pwm_a, pwm_b


def drive_left(pwm_a, duty, reverse=False):
    GPIO.output(IN1, GPIO.LOW if reverse else GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH if reverse else GPIO.LOW)
    pwm_a.ChangeDutyCycle(duty)


def stop_left(pwm_a):
    pwm_a.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)


def drive_right(pwm_b, duty, reverse=False):
    GPIO.output(IN3, GPIO.LOW if reverse else GPIO.HIGH)
    GPIO.output(IN4, GPIO.HIGH if reverse else GPIO.LOW)
    pwm_b.ChangeDutyCycle(duty)


def stop_right(pwm_b):
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


def drive_both(pwm_a, pwm_b, duty, reverse=False):
    drive_left(pwm_a, duty, reverse)
    drive_right(pwm_b, duty, reverse)


def stop_both(pwm_a, pwm_b):
    stop_left(pwm_a)
    stop_right(pwm_b)


def run_test(pwm_a, pwm_b, reverse=False):
    global left_ticks, right_ticks
    left_ticks = 0
    right_ticks = 0

    if DURATION <= MEASURE_START:
        raise ValueError("DURATION은 MEASURE_START보다 커야 합니다.")

    direction = "역방향" if reverse else "정방향"
    print(f"\n=== 양쪽 바퀴 구동 시작 ({direction}, {DURATION}s @ duty={DUTY_CYCLE}%) ===")
    print(f"  tick 집계 구간: {MEASURE_START:.1f}s ~ {DURATION:.1f}s")

    drive_both(pwm_a, pwm_b, DUTY_CYCLE, reverse)
    start_left = None
    start_right = None
    t0 = time.time()
    while time.time() - t0 < DURATION:
        elapsed = time.time() - t0
        if start_left is None and elapsed >= MEASURE_START:
            start_left = left_ticks
            start_right = right_ticks

        if start_left is None:
            print(
                f"  [{elapsed:4.1f}s] warming up  left={left_ticks:5d}  right={right_ticks:5d}",
                end="\r",
                flush=True,
            )
        else:
            print(
                f"  [{elapsed:4.1f}s] "
                f"left_delta={left_ticks - start_left:5d}  "
                f"right_delta={right_ticks - start_right:5d}",
                end="\r",
                flush=True,
            )
        time.sleep(0.1)

    end_left = left_ticks
    end_right = right_ticks
    stop_both(pwm_a, pwm_b)
    time.sleep(0.3)  # 관성 정지 대기

    if start_left is None:
        start_left = end_left
        start_right = end_right

    print("\n--- 결과 ---")
    print(f"  집계 시작({MEASURE_START:.1f}s) 왼쪽/오른쪽 tick: {start_left} / {start_right}")
    print(f"  집계 종료({DURATION:.1f}s) 왼쪽/오른쪽 tick: {end_left} / {end_right}")
    print(f"  왼쪽 엔코더  증가량: {end_left - start_left}")
    print(f"  오른쪽 엔코더 증가량: {end_right - start_right}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reverse", action="store_true", help="바퀴를 역방향으로 구동")
    args = parser.parse_args()

    pwm_a, pwm_b = setup()
    try:
        run_test(pwm_a, pwm_b, args.reverse)
    except KeyboardInterrupt:
        print("\n[중단됨]")
    finally:
        stop_both(pwm_a, pwm_b)
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()
        print("\nGPIO 정리 완료.")


if __name__ == "__main__":
    main()
