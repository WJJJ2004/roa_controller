#!/usr/bin/env python3
"""Validate the conservative RSU clip rectangle and emit the C++ LUT asset."""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rsu_manager.util.core import RSUCore
from rsu_manager.util.rsu_lut import RSULookupTable, RSULookupTableConfig


class _Logger:
    def info(self, *_args, **_kwargs):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace-npz", type=Path, required=True)
    parser.add_argument("--output-bin", type=Path, required=True)
    parser.add_argument("--output-report", type=Path, required=True)
    parser.add_argument("--output-plot", type=Path, required=True)
    args = parser.parse_args()

    source = np.load(args.workspace_npz)
    roll = source["roll_deg"]
    pitch = source["pitch_deg"]
    rectangle = (
        (roll[:, None] >= -18.0) & (roll[:, None] <= 17.5)
        & (pitch[None, :] >= -36.0) & (pitch[None, :] <= 39.5)
    )
    pass_mask = source["pass_mask"].astype(bool)
    failed_inside = int(np.count_nonzero(rectangle & ~pass_mask))
    if failed_inside:
        raise RuntimeError(f"safe clip rectangle contains {failed_inside} failed cells")

    core = RSUCore(_Node())
    lut = RSULookupTable(core.solver, RSULookupTableConfig())
    if not np.all(lut.feasible):
        raise RuntimeError("runtime LUT contains infeasible cells")

    # Header: magic, version, rows, columns, four bounds and resolution.
    args.output_bin.parent.mkdir(parents=True, exist_ok=True)
    with args.output_bin.open("wb") as stream:
        stream.write(struct.pack(
            "<8sIII5d", b"RSULUT2\0", 1,
            lut.roll_axis.size, lut.pitch_axis.size,
            lut.cfg.roll_min, lut.cfg.roll_max,
            lut.cfg.pitch_min, lut.cfg.pitch_max, lut.cfg.resolution,
        ))
        payload = np.concatenate(
            [lut.alpha.reshape(-1, 2), lut.jacobian_table.reshape(-1, 4)], axis=1
        ).astype("<f4", copy=False)
        stream.write(payload.tobytes(order="C"))

    alpha_deg = np.rad2deg(lut.alpha.astype(np.float64))
    report = {
        "source": str(args.workspace_npz),
        "criteria": {
            "virtual_internal_roll_deg": [-18.0, 17.5],
            "virtual_internal_pitch_deg": [-36.0, 39.5],
            "left_external_roll_deg": [-18.0, 17.5],
            "left_external_pitch_deg": [-36.0, 39.5],
            "right_external_roll_deg": [-17.5, 18.0],
            "right_external_pitch_deg": [-39.5, 36.0],
            "feedback_actuator_margin_deg": 2.0,
        },
        "validation": {
            "rectangle_grid_points": int(np.count_nonzero(rectangle)),
            "failed_inside_rectangle": failed_inside,
            "lut_all_cells_feasible": bool(np.all(lut.feasible)),
            "lut_shape": list(lut.feasible.shape),
            "lut_memory_mib": lut.memory_bytes / 1024.0 / 1024.0,
            "actuator_internal_deg": {
                "actuator_1": [float(np.min(alpha_deg[..., 0])), float(np.max(alpha_deg[..., 0]))],
                "actuator_2": [float(np.min(alpha_deg[..., 1])), float(np.max(alpha_deg[..., 1]))],
            },
        },
        "policy": {
            "target": "finite check, clip in external virtual space, mirror right, LUT IK",
            "actuator_output": "clip to LUT-derived actuator extrema as final safety net",
            "feedback": "never clip; reject outside actuator extrema plus 2 degree margin",
        },
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_text(json.dumps(report, indent=2) + "\n")

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.imshow(
        pass_mask.astype(float), origin="lower", aspect="auto",
        extent=[pitch[0], pitch[-1], roll[0], roll[-1]], cmap="RdYlGn",
    )
    ax.add_patch(plt.Rectangle((-36.0, -18.0), 75.5, 35.5, fill=False, color="cyan", linewidth=2))
    ax.set(title="RSU validated workspace and selected clip rectangle", xlabel="Pitch [deg]", ylabel="Roll [deg]")
    fig.tight_layout()
    args.output_plot.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output_plot, dpi=180)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
