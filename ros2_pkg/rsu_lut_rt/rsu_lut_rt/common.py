from __future__ import annotations

import math

import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rsu_manager.util.core import RSUCore
from rsu_manager.util.rsu_lut import RSULookupTable, RSULookupTableConfig


def best_effort_latest_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def declare_lut_parameters(node) -> None:
    node.declare_parameter("lut_resolution_deg", 0.25)
    node.declare_parameter("lut_roll_min_deg", -18.0)
    node.declare_parameter("lut_roll_max_deg", 17.5)
    node.declare_parameter("lut_pitch_min_deg", -36.0)
    node.declare_parameter("lut_pitch_max_deg", 39.5)
    node.declare_parameter("lut_branch_seed", [-0.458105, 0.458105])


def build_lut(node) -> RSULookupTable:
    core = RSUCore(node)
    cfg = RSULookupTableConfig(
        roll_min=math.radians(float(node.get_parameter("lut_roll_min_deg").value)),
        roll_max=math.radians(float(node.get_parameter("lut_roll_max_deg").value)),
        pitch_min=math.radians(float(node.get_parameter("lut_pitch_min_deg").value)),
        pitch_max=math.radians(float(node.get_parameter("lut_pitch_max_deg").value)),
        resolution=math.radians(float(node.get_parameter("lut_resolution_deg").value)),
        branch_seed=tuple(np.asarray(node.get_parameter("lut_branch_seed").value, dtype=float)),
    )
    lut = RSULookupTable(core.solver, cfg)
    node.get_logger().info(
        f"LUT ready shape={lut.alpha.shape[:2]} "
        f"memory={lut.memory_bytes / (1024 * 1024):.3f} MiB "
        f"build={lut.build_seconds:.3f}s"
    )
    return lut
