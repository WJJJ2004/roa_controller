#!/usr/bin/env python3
"""Compare C++ v2 replay with legacy bags and the offline 300 Hz golden data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rsu_manager.util.core import RSUCore
from rsu_manager.util.rsu_lut import RSULookupTable, RSULookupTableConfig


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def read_bag(path, selected):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    out = {topic: [] for topic in selected}
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic in out:
            out[topic].append((bag_time, deserialize_message(raw, types[topic])))
    return out


def stats(error):
    value = np.abs(np.asarray(error))
    return {key: float(result) for key, result in (
        ("mean_abs", np.mean(value)), ("p95_abs", np.percentile(value, 95)),
        ("p99_abs", np.percentile(value, 99)), ("max_abs", np.max(value)))}


class _Logger:
    def info(self, *_args, **_kwargs): pass


class _Node:
    def get_logger(self): return _Logger()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--v2", type=Path, required=True)
    parser.add_argument("--python-lut", type=Path, required=True)
    parser.add_argument("--golden", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    source = read_bag(args.source, {"/rsu/target", "/rsu/state", "/rsu/imp_solution"})
    v2 = read_bag(args.v2, {"/rsu/state", "/rsu/imp_solution"})
    py = read_bag(args.python_lut, {"/rsu/state", "/rsu/imp_solution"})
    golden = np.load(args.golden)
    golden_stamp = golden["timestamp_ns"]
    golden_arrays = {
        key: golden[key]
        for key in (
            "left_q_exact", "right_q_exact", "left_kp_exact", "right_kp_exact",
            "left_kd_exact", "right_kd_exact",
        )
    }
    golden_index = {int(stamp): i for i, stamp in enumerate(golden_stamp)}
    low, high = int(golden_stamp[0]), int(golden_stamp[-1])

    v2_state = [(t, m) for t, m in v2["/rsu/state"] if low <= stamp_ns(m) <= high and m.feasible]
    v2_solution = [(t, m) for t, m in v2["/rsu/imp_solution"] if low <= stamp_ns(m) <= high and m.feasible]
    # One command can repeat the same latest feedback epoch. Keep the last command per epoch for parity.
    state_by_stamp = {stamp_ns(m): m for _, m in v2_state}
    solution_by_stamp = {stamp_ns(m): m for _, m in v2_solution}
    common_state = sorted(set(state_by_stamp) & set(golden_index))
    common_solution = sorted(set(solution_by_stamp) & set(golden_index))

    q_error, qd_error, kp_error, kd_error = [], [], [], []
    for stamp in common_state:
        m = state_by_stamp[stamp]
        i = golden_index[stamp]
        exact_q = np.r_[golden_arrays["left_q_exact"][i], -golden_arrays["right_q_exact"][i]]
        measured_q = np.array([m.q.left_rsu_roll, m.q.left_rsu_pitch, m.q.right_rsu_roll, m.q.right_rsu_pitch])
        # Golden NPZ does not store qd; qd is compared against the Python LUT online replay below.
        q_error.append(measured_q - exact_q)
    for stamp in common_solution:
        m = solution_by_stamp[stamp]
        i = golden_index[stamp]
        kp = np.array([m.left_actuator_1.kp_eqv, m.left_actuator_2.kp_eqv, m.right_actuator_1.kp_eqv, m.right_actuator_2.kp_eqv])
        kd = np.array([m.left_actuator_1.kd_eqv, m.left_actuator_2.kd_eqv, m.right_actuator_1.kd_eqv, m.right_actuator_2.kd_eqv])
        kp_exact = np.r_[golden_arrays["left_kp_exact"][i], golden_arrays["right_kp_exact"][i]]
        kd_exact = np.r_[golden_arrays["left_kd_exact"][i], golden_arrays["right_kd_exact"][i]]
        kp_error.append(kp - kp_exact)
        kd_error.append(kd - kd_exact)

    core = RSUCore(_Node())
    lut = RSULookupTable(core.solver, RSULookupTableConfig())
    target_records = sorted((stamp_ns(m), m) for _, m in source["/rsu/target"])
    target_by_seq = {int(message.seq): message for _, message in target_records}
    q_target_error = []
    clip_axes = 0
    for stamp in common_solution:
        target = target_by_seq.get(int(solution_by_stamp[stamp].seq))
        if target is None: continue
        internal = np.array([
            [target.left_roll, target.left_pitch],
            [-target.right_roll, -target.right_pitch]], dtype=float)
        clipped = internal.copy()
        clipped[:, 0] = np.clip(clipped[:, 0], lut.cfg.roll_min, lut.cfg.roll_max)
        clipped[:, 1] = np.clip(clipped[:, 1], lut.cfg.pitch_min, lut.cfg.pitch_max)
        clip_axes += int(np.count_nonzero(clipped != internal))
        left = lut.solve(*clipped[0]).alpha
        right = lut.solve(*clipped[1]).alpha
        expected = np.r_[left, -right]
        m = solution_by_stamp[stamp]
        actual = np.array([m.left_actuator_1.q_target, m.left_actuator_2.q_target, m.right_actuator_1.q_target, m.right_actuator_2.q_target])
        q_target_error.append(actual - expected)

    def output_rate(records):
        if len(records) < 2: return 0.0
        return (len(records) - 1) / ((records[-1][0] - records[0][0]) * 1e-9)

    def interval_stats(records):
        dt = np.diff(np.asarray([record[0] for record in records], dtype=np.int64)) * 1e-6
        return {
            "p50_ms": float(np.percentile(dt, 50)),
            "p95_ms": float(np.percentile(dt, 95)),
            "p99_ms": float(np.percentile(dt, 99)),
            "max_ms": float(np.max(dt)),
        }

    legacy_state = source["/rsu/state"]
    legacy_solution = source["/rsu/imp_solution"]
    py_state = py["/rsu/state"]
    py_solution = py["/rsu/imp_solution"]
    py_state_by_stamp = {
        stamp_ns(message): message
        for _, message in py_state
        if low <= stamp_ns(message) <= high and message.feasible
    }
    common_qd = sorted(set(state_by_stamp) & set(py_state_by_stamp))
    for stamp in common_qd:
        cpp = state_by_stamp[stamp]
        python = py_state_by_stamp[stamp]
        cpp_qd = np.array([
            cpp.q_dot.left_rsu_roll, cpp.q_dot.left_rsu_pitch,
            cpp.q_dot.right_rsu_roll, cpp.q_dot.right_rsu_pitch])
        python_qd = np.array([
            python.q_dot.left_rsu_roll, python.q_dot.left_rsu_pitch,
            python.q_dot.right_rsu_roll, python.q_dot.right_rsu_pitch])
        qd_error.append(cpp_qd - python_qd)
    summary = {
        "rates_hz": {
            "legacy_state": output_rate(legacy_state),
            "legacy_solution": output_rate(legacy_solution),
            "python_lut_state": output_rate(py_state),
            "python_lut_solution": output_rate(py_solution),
            "cpp_v2_state_valid_window": output_rate(v2_state),
            "cpp_v2_solution_valid_window": output_rate(v2_solution),
        },
        "cpp_v2": {
            "valid_state_messages": len(v2_state),
            "valid_solution_messages": len(v2_solution),
            "unique_feedback_epochs_state": len(state_by_stamp),
            "unique_feedback_epochs_solution": len(solution_by_stamp),
            "golden_common_state_epochs": len(common_state),
            "golden_common_solution_epochs": len(common_solution),
            "q_error_deg": stats(np.rad2deg(q_error)),
            "qd_error_vs_python_lut_rad_s": stats(qd_error),
            "qd_common_epochs": len(common_qd),
            "kp_error": stats(kp_error),
            "kd_error": stats(kd_error),
            "q_target_error_deg": stats(np.rad2deg(q_target_error)),
            "target_clip_axis_events_in_epoch_comparison": clip_axes,
            "state_output_interval": interval_stats(v2_state),
            "solution_output_interval": interval_stats(v2_solution),
        },
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    names = ["q [deg]", "q target [deg]", "Kp", "Kd"]
    values = [np.rad2deg(q_error), np.rad2deg(q_target_error), kp_error, kd_error]
    for ax, name, value in zip(axes.flat, names, values):
        ax.hist(np.abs(np.asarray(value)).ravel(), bins=100, log=True)
        ax.set(title=f"C++ v2 vs 300 Hz golden: {name}", xlabel="absolute error", ylabel="count")
        ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(args.output / "cpp_v2_parity_histogram.png", dpi=180)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
