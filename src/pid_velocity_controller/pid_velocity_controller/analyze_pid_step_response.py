#!/usr/bin/env python3
"""Analyze PID step-response metrics from pid_controller_node CSV logs."""

import argparse
import csv
import math
from pathlib import Path
from statistics import mean, pstdev


def _latest_log():
    result_dir = Path(__file__).resolve().parent.parent / "result"
    logs = list(result_dir.glob("*.csv"))
    if not logs:
        raise FileNotFoundError(f"No PID logs found: {result_dir / '*.csv'}")
    return max(logs, key=lambda p: p.stat().st_mtime)


def _read_rows(path):
    rows = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            item = {k: float(v) for k, v in row.items()}
            item["v_actual"] = (item["meas_vL"] + item["meas_vR"]) / 2.0
            rows.append(item)
    if not rows:
        raise ValueError(f"Log is empty: {path}")
    return rows


def _first_crossing(rows, target, fraction):
    threshold = abs(target) * fraction
    for row in rows:
        if abs(row["v_actual"]) >= threshold:
            return row["time_s"]
    return None


def _fmt(value, unit="s"):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "not reached"
    return f"{value:.3f} {unit}"


def analyze(path, steady_window):
    rows = _read_rows(path)

    step_rows = [r for r in rows if abs(r["cmd_v"]) > 1e-6]
    if not step_rows:
        raise ValueError("No nonzero cmd_v step found in log")

    step_start = step_rows[0]["time_s"]
    target = step_rows[0]["cmd_v"]
    active_rows = [r for r in rows if r["time_s"] >= step_start and abs(r["cmd_v"]) > 1e-6]

    t10 = _first_crossing(active_rows, target, 0.10)
    t90 = _first_crossing(active_rows, target, 0.90)
    t95 = _first_crossing(active_rows, target, 0.95)

    rise_time_10_90 = None
    if t10 is not None and t90 is not None:
        rise_time_10_90 = t90 - t10

    max_v = max(r["v_actual"] for r in active_rows)
    min_v = min(r["v_actual"] for r in active_rows)
    peak_v = max_v if target >= 0.0 else min_v
    overshoot = max(0.0, (peak_v - target) / abs(target) * 100.0) if target > 0 else 0.0

    end_t = active_rows[-1]["time_s"]
    steady_rows = [r for r in active_rows if r["time_s"] >= end_t - steady_window]
    steady_v = [r["v_actual"] for r in steady_rows]
    steady_mean = mean(steady_v)
    steady_error = target - steady_mean
    steady_error_pct = steady_error / target * 100.0 if abs(target) > 1e-9 else 0.0
    oscillation_std = pstdev(steady_v) if len(steady_v) > 1 else 0.0
    oscillation_pp = max(steady_v) - min(steady_v) if steady_v else 0.0

    return {
        "path": path,
        "target": target,
        "step_start": step_start,
        "samples": len(active_rows),
        "duration": active_rows[-1]["time_s"] - step_start,
        "t10": None if t10 is None else t10 - step_start,
        "t90": None if t90 is None else t90 - step_start,
        "t95": None if t95 is None else t95 - step_start,
        "rise_time_10_90": rise_time_10_90,
        "peak_v": peak_v,
        "overshoot": overshoot,
        "steady_mean": steady_mean,
        "steady_error": steady_error,
        "steady_error_pct": steady_error_pct,
        "oscillation_std": oscillation_std,
        "oscillation_pp": oscillation_pp,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv", nargs="?", default=None, help="PID CSV log path. Defaults to latest pid_log_*.csv."
    )
    parser.add_argument(
        "--steady-window",
        type=float,
        default=2.0,
        help="Seconds from the end of the step used for steady-state metrics.",
    )
    args = parser.parse_args()

    path = args.csv or _latest_log()
    result = analyze(path, args.steady_window)

    print(f"Log: {result['path']}")
    print(f"Target velocity: {result['target']:.3f} m/s")
    print(f"Step start: {result['step_start']:.3f} s")
    print(f"Active samples: {result['samples']} over {result['duration']:.3f} s")
    print(f"Time to 10%: {_fmt(result['t10'])}")
    print(f"Time to 90%: {_fmt(result['t90'])}")
    print(f"Time to 95%: {_fmt(result['t95'])}")
    print(f"Rise time 10-90%: {_fmt(result['rise_time_10_90'])}")
    print(f"Peak velocity: {result['peak_v']:.3f} m/s")
    print(f"Overshoot: {result['overshoot']:.2f} %")
    print(f"Steady-state mean: {result['steady_mean']:.3f} m/s")
    print(
        f"Steady-state error: {result['steady_error']:.3f} m/s ({result['steady_error_pct']:.2f} %)"
    )
    print(f"Steady-state oscillation std: {result['oscillation_std']:.4f} m/s")
    print(f"Steady-state oscillation peak-to-peak: {result['oscillation_pp']:.4f} m/s")


if __name__ == "__main__":
    main()
