"""RSU target-to-actuator LUT process using immutable gain snapshots."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from roa_interfaces.msg import RsuImpSol, RsuTarget

from rsu_lut_rt.common import best_effort_latest_qos, build_lut, declare_lut_parameters, stamp_to_ns


class RSULutSolutionNode(Node):
    def __init__(self):
        super().__init__("rsu_lut_solution_node")
        declare_lut_parameters(self)
        self.declare_parameter("compute_rate_hz", 300.0)
        self.declare_parameter("target_topic", "/rsu/target")
        self.declare_parameter("solution_topic", "/rsu/imp_solution")
        self.declare_parameter("gain_snapshot_topic", "/rsu_lut/gain_snapshot")
        self.declare_parameter("frame_id", "rsu_state")
        self.declare_parameter("require_fresh_gain", True)
        self.declare_parameter("gain_max_age_ms", 50.0)
        self.declare_parameter("hold_target_on_infeasible", True)
        self.declare_parameter("default_kp", [9.0, 9.0])
        self.declare_parameter("default_kd", [2.25, 2.25])

        self.compute_rate_hz = float(self.get_parameter("compute_rate_hz").value)
        self.gain_max_age_ns = int(float(self.get_parameter("gain_max_age_ms").value) * 1e6)
        self.require_fresh_gain = bool(self.get_parameter("require_fresh_gain").value)
        self.hold_target_on_infeasible = bool(
            self.get_parameter("hold_target_on_infeasible").value
        )
        if not np.isfinite(self.compute_rate_hz) or self.compute_rate_hz <= 0.0:
            raise RuntimeError("compute_rate_hz must be finite and > 0")
        if self.gain_max_age_ns < 0:
            raise RuntimeError("gain_max_age_ms must be >= 0")

        self.lut = build_lut(self)
        seed = np.asarray(self.get_parameter("lut_branch_seed").value, dtype=float).reshape(2,)
        self.previous_alpha = np.vstack([seed, seed]).astype(float)
        self.last_valid_alpha = self.previous_alpha.copy()
        self.default_kp = np.asarray(self.get_parameter("default_kp").value, dtype=float).reshape(2,)
        self.default_kd = np.asarray(self.get_parameter("default_kd").value, dtype=float).reshape(2,)

        qos = best_effort_latest_qos()
        self.solution_pub = self.create_publisher(
            RsuImpSol, str(self.get_parameter("solution_topic").value), qos
        )
        self.target_sub = self.create_subscription(
            RsuTarget,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            qos,
        )
        self.gain_sub = self.create_subscription(
            RsuImpSol,
            str(self.get_parameter("gain_snapshot_topic").value),
            self._on_gain,
            qos,
        )
        self.timer = self.create_timer(1.0 / self.compute_rate_hz, self._on_timer)
        self._latest_target = None
        self._latest_gain = None
        self._processed_target_seq = None
        self._last_target_seq = None
        self._last_target_stamp_ns = None
        self.get_logger().info(
            f"RSU LUT solution process ready at {self.compute_rate_hz:.1f} Hz; "
            "target/gain callbacks are latch-only"
        )

    def _on_target(self, message):
        self._latest_target = message

    def _on_gain(self, message):
        self._latest_gain = message

    def _on_timer(self):
        target = self._latest_target
        if target is None or int(target.seq) == self._processed_target_seq:
            return
        self._processed_target_seq = int(target.seq)
        target_stamp_ns = stamp_to_ns(target.header.stamp)
        if self._last_target_seq is not None:
            if int(target.seq) <= self._last_target_seq or target_stamp_ns <= self._last_target_stamp_ns:
                self.get_logger().warn(
                    "Ignoring non-increasing RSU target sequence/timestamp",
                    throttle_duration_sec=1.0,
                )
                return
        self._last_target_seq = int(target.seq)
        self._last_target_stamp_ns = target_stamp_ns

        left = self.lut.solve(float(target.left_roll), float(target.left_pitch), self.previous_alpha[0])
        right = self.lut.solve(
            -float(target.right_roll), -float(target.right_pitch), self.previous_alpha[1]
        )
        left_ok = bool(left.feasible and np.all(np.isfinite(left.alpha)))
        right_ok = bool(right.feasible and np.all(np.isfinite(right.alpha)))
        if left_ok:
            self.previous_alpha[0] = left.alpha
            self.last_valid_alpha[0] = left.alpha
        elif not self.hold_target_on_infeasible:
            self.last_valid_alpha[0] = 0.0
        if right_ok:
            self.previous_alpha[1] = right.alpha
            self.last_valid_alpha[1] = right.alpha
        elif not self.hold_target_on_infeasible:
            self.last_valid_alpha[1] = 0.0

        gain = self._latest_gain
        gain_fresh = False
        if gain is not None:
            gain_age_ns = abs(target_stamp_ns - stamp_to_ns(gain.header.stamp))
            gain_fresh = bool(gain.feasible and gain_age_ns <= self.gain_max_age_ns)
        kp_left = self.default_kp.copy()
        kd_left = self.default_kd.copy()
        kp_right = self.default_kp.copy()
        kd_right = self.default_kd.copy()
        if gain_fresh:
            kp_left[:] = [gain.left_actuator_1.kp_eqv, gain.left_actuator_2.kp_eqv]
            kd_left[:] = [gain.left_actuator_1.kd_eqv, gain.left_actuator_2.kd_eqv]
            kp_right[:] = [gain.right_actuator_1.kp_eqv, gain.right_actuator_2.kp_eqv]
            kd_right[:] = [gain.right_actuator_1.kd_eqv, gain.right_actuator_2.kd_eqv]

        output = RsuImpSol()
        output.header = target.header
        output.header.frame_id = str(self.get_parameter("frame_id").value)
        output.seq = target.seq
        output.left_actuator_1.q_target = float(self.last_valid_alpha[0, 0])
        output.left_actuator_2.q_target = float(self.last_valid_alpha[0, 1])
        output.right_actuator_1.q_target = float(-self.last_valid_alpha[1, 0])
        output.right_actuator_2.q_target = float(-self.last_valid_alpha[1, 1])
        for actuator, kp, kd in (
            (output.left_actuator_1, kp_left[0], kd_left[0]),
            (output.left_actuator_2, kp_left[1], kd_left[1]),
            (output.right_actuator_1, kp_right[0], kd_right[0]),
            (output.right_actuator_2, kp_right[1], kd_right[1]),
        ):
            actuator.kp_eqv = float(kp)
            actuator.kd_eqv = float(kd)
        output.feasible = bool(
            left_ok and right_ok and (gain_fresh or not self.require_fresh_gain)
        )
        self.solution_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RSULutSolutionNode()
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
