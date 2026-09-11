#!/usr/bin/env python3
"""
calibrate_accel.py — MPU6050 가속도계 6면(6-position) 캘리브레이션.

원리
----
축 i 를 위/아래로 향하게 정지시키면 그 축은 정확히 ±g 를 읽어야 한다.
보정 모델을 a_corr = (a_raw - bias) * scale 로 두면 두 측정값으로 풀린다.

    r_up = bias + g/scale ,  r_dn = bias - g/scale
    →  bias  = (r_up + r_dn) / 2
       scale = 2g / (r_up - r_dn)

축별 bias 와 scale 을 각각 구하므로 "전 축 공통 스케일" 뿐 아니라
축마다 다른 오프셋(자세에 따라 |a| 가 흔들리는 원인)까지 함께 보정된다.

사용법
------
    python3 calibrate_accel.py
    python3 calibrate_accel.py --output /path/to/imu_calibration.yaml
    python3 calibrate_accel.py --samples 400      # 자세당 샘플 수

로봇에서 IMU 보드를 떼거나, 로봇째로 6개 자세를 만들 수 있으면 그대로 두고
진행해도 된다. 중요한 건 각 자세에서 완전히 정지해 있는 것.
"""

import argparse
import math
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import smbus
except ImportError:
    import smbus2 as smbus

ADDR = 0x68
G = 9.80665
ACCEL_LSB = 16384.0  # ±2g 레인지
GYRO_LSB = 131.0  # ±250 °/s

# (키, 사람이 읽는 설명, (중력을 받아야 할 축 인덱스, 부호))
POSITIONS = [
    ("x_up", "X축이 하늘을 향하도록 세우기", (0, +1)),
    ("x_down", "X축이 땅을 향하도록 (바로 위 자세에서 180° 뒤집기)", (0, -1)),
    ("y_up", "Y축이 하늘을 향하도록 세우기", (1, +1)),
    ("y_down", "Y축이 땅을 향하도록 (180° 뒤집기)", (1, -1)),
    ("z_up", "Z축이 하늘을 향하도록 (보드를 평평하게 눕히기)", (2, +1)),
    ("z_down", "Z축이 땅을 향하도록 (평평하게 뒤집기)", (2, -1)),
]


def open_bus(bus_num):
    bus = smbus.SMBus(bus_num)
    who = bus.read_byte_data(ADDR, 0x75)
    if who != 0x68:
        print(f"WHO_AM_I = 0x{who:02x} — MPU6050 응답이 이상합니다.")
        sys.exit(1)
    bus.write_byte_data(ADDR, 0x6B, 0x00)  # sleep 해제
    time.sleep(0.1)
    bus.write_byte_data(ADDR, 0x1C, 0x00)  # accel ±2g
    bus.write_byte_data(ADDR, 0x1B, 0x00)  # gyro ±250 dps
    bus.write_byte_data(ADDR, 0x1A, 0x03)  # DLPF 44Hz
    time.sleep(0.05)
    return bus


def read_once(bus):
    d = bus.read_i2c_block_data(ADDR, 0x3B, 14)
    ax, ay, az, _tp, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(d))
    return (
        ax / ACCEL_LSB * G,
        ay / ACCEL_LSB * G,
        az / ACCEL_LSB * G,
        gx / GYRO_LSB,
        gy / GYRO_LSB,
        gz / GYRO_LSB,
    )


def mean_std(v):
    m = sum(v) / len(v)
    s = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return m, s


def measure(bus, samples, axis_idx, sign):
    """한 자세에서 정지 상태와 자세를 확인하며 축별 평균을 구한다."""
    names = "XYZ"
    for attempt in range(1, 6):
        # 흔들림이 잦아들 때까지 잠깐 대기
        for _ in range(30):
            read_once(bus)
            time.sleep(0.005)

        acc = [[], [], []]
        gyr = [[], [], []]
        step = max(1, samples // 20)
        for i in range(samples):
            ax, ay, az, gx, gy, gz = read_once(bus)
            acc[0].append(ax)
            acc[1].append(ay)
            acc[2].append(az)
            gyr[0].append(gx)
            gyr[1].append(gy)
            gyr[2].append(gz)
            if i % step == 0:
                pct = int(i / samples * 100)
                print(f"\r   측정 중 {pct:3d}%  [{'#' * (pct // 5):<20}]", end="", flush=True)
            time.sleep(0.005)
        print(f"\r   측정 완료 100%  [{'#' * 20}]", flush=True)

        am = [mean_std(acc[i]) for i in range(3)]
        gm = [mean_std(gyr[i]) for i in range(3)]
        max_gyro_std = max(s for _m, s in gm)

        # 1) 정지 확인
        if max_gyro_std > 1.0:
            print(f"   ⚠ 흔들림 감지 (자이로 std={max_gyro_std:.2f} °/s).")
            if attempt < 5:
                input("   보드를 고정한 뒤 Enter: ")
                continue

        # 2) 자세 확인 — 지정한 축이 실제로 중력을 받고 있는지
        dominant = max(range(3), key=lambda i: abs(am[i][0]))
        if dominant != axis_idx or am[axis_idx][0] * sign <= 0:
            print(
                f"   ⚠ 자세가 다릅니다. 기대: {names[axis_idx]}축 "
                f"{'+' if sign > 0 else '-'}g / "
                f"실측: {names[dominant]}축 {am[dominant][0]:+.2f} m/s²"
            )
            if attempt < 5:
                input("   자세를 바로잡고 Enter: ")
                continue

        # 3) 기울어짐 경고 — 나머지 축이 충분히 0에 가까운지
        others = [i for i in range(3) if i != axis_idx]
        tilt = math.degrees(
            math.atan2(math.hypot(am[others[0]][0], am[others[1]][0]), abs(am[axis_idx][0]))
        )
        if tilt > 8.0:
            print(f"   ⚠ 약 {tilt:.1f}° 기울어져 있습니다 (8° 이하 권장).")
            if attempt < 5:
                input("   평평한 면에 다시 대고 Enter: ")
                continue

        print(
            f"   → ax={am[0][0]:+7.3f}  ay={am[1][0]:+7.3f}  az={am[2][0]:+7.3f} m/s²"
            f"   (기울기 {tilt:.1f}°, 자이로 std {max_gyro_std:.2f}°/s)"
        )
        return [am[i][0] for i in range(3)]

    print("   측정에 반복 실패했습니다. 중단합니다.")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="MPU6050 가속도계 6면 캘리브레이션")
    ap.add_argument("--bus", type=int, default=1, help="I2C 버스 번호 (기본 1)")
    ap.add_argument("--samples", type=int, default=300, help="자세당 샘플 수 (기본 300)")
    ap.add_argument("--output", default=None, help="결과 YAML 경로")
    args = ap.parse_args()

    if args.output:
        out_path = Path(args.output)
    else:
        here = Path(__file__).resolve().parent
        out_path = (here.parent / "config" / "imu_calibration.yaml").resolve()

    bus = open_bus(args.bus)

    print("=" * 68)
    print(" MPU6050 가속도계 6면 캘리브레이션")
    print("=" * 68)
    print(" 6개 자세에서 각각 정지 상태로 측정합니다.")
    print(" 각 자세마다 평평하고 단단한 면에 올려놓고 손을 떼주세요.")
    print(" 축 방향은 보드에 인쇄된 X/Y/Z 화살표를 기준으로 합니다.")
    print(" (자세가 틀리면 스크립트가 알려주고 다시 시켜줍니다)")
    print("=" * 68)

    readings = {}
    for n, (key, desc, (axis_idx, sign)) in enumerate(POSITIONS, 1):
        print(f"\n[{n}/6] {desc}")
        input("      준비되면 Enter: ")
        readings[key] = measure(bus, args.samples, axis_idx, sign)

    # ── 축별 bias / scale 계산 ────────────────────────────────
    bias, scale = [0.0] * 3, [1.0] * 3
    for i, up_key, dn_key in [(0, "x_up", "x_down"), (1, "y_up", "y_down"), (2, "z_up", "z_down")]:
        r_up = readings[up_key][i]
        r_dn = readings[dn_key][i]
        span = r_up - r_dn
        if abs(span) < 1e-6:
            print(f"{'XYZ'[i]}축 측정값이 이상합니다 (span={span}). 중단합니다.")
            sys.exit(1)
        bias[i] = (r_up + r_dn) / 2.0
        scale[i] = 2.0 * G / span

    print("\n" + "=" * 68)
    print(" 캘리브레이션 결과")
    print("=" * 68)
    for i, n in enumerate("XYZ"):
        print(
            f"  {n}축:  bias = {bias[i]:+8.4f} m/s²   scale = {scale[i]:.5f}"
            f"   ({(scale[i] - 1) * 100:+.2f}% 보정)"
        )

    # ── 검증: 6개 자세에 보정을 적용했을 때 |a| 가 9.81 에 붙는지 ──
    print("\n 검증 (보정 전 → 보정 후 |a|):")
    errs = []
    for key, _desc, _spec in POSITIONS:
        raw = readings[key]
        cor = [(raw[i] - bias[i]) * scale[i] for i in range(3)]
        m_raw = math.sqrt(sum(v * v for v in raw))
        m_cor = math.sqrt(sum(v * v for v in cor))
        errs.append(abs(m_cor - G))
        print(f"   {key:8s}  {m_raw:6.3f}  →  {m_cor:6.3f} m/s²   (오차 {m_cor - G:+.3f})")
    worst = max(errs)
    print(f"\n 최대 잔차: {worst:.3f} m/s²", end="  ")
    if worst < 0.15:
        print("→ 매우 양호")
    elif worst < 0.35:
        print("→ 양호 (사용 가능)")
    else:
        print("→ 잔차가 큽니다. 자세를 더 정확히 잡고 다시 시도해 보세요.")

    # ── YAML 저장 ─────────────────────────────────────────────
    with out_path.open("w") as f:
        f.write("# MPU6050 가속도계 캘리브레이션 (calibrate_accel.py 자동 생성)\n")
        f.write(f"# 생성 시각: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write("#\n")
        f.write("# 보정식:  a_corrected[i] = (a_raw[i] - bias[i]) * scale[i]\n")
        f.write("#   a_raw 는 ±2g 공칭 스케일(16384 LSB/g)로 변환한 m/s^2 값.\n")
        f.write(f"# 6면 검증 최대 잔차: {worst:.4f} m/s^2\n")
        f.write("accel:\n")
        f.write("  bias:  [{:.6f}, {:.6f}, {:.6f}]\n".format(*bias))
        f.write("  scale: [{:.6f}, {:.6f}, {:.6f}]\n".format(*scale))
    print(f"\n 저장 완료: {out_path}")
    print(" mpu6050_node 를 다시 실행하면 자동으로 적용됩니다.")
    bus.close()


if __name__ == "__main__":
    main()
