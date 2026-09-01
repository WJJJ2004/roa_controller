"""RSU feedback reconstruction process.

The subscription callback only replaces an immutable latest-message reference.
All estimator, Jacobian, impedance, and publication work runs in a fixed-rate
timer.  Jacobian state never leaves this process; only a gain snapshot is sent
to the solution process.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from roa_interfaces.msg import MotorStateArray, RsuImpSol, RsuStateArray
from rsu_manager.util.core import RSUEstimatorFactory
from rsu_manager.util.rsu_impedance_mapper import RSUImpedanceMapper, RSUImpedanceMapperConfig

from rsu_lut_rt.common import best_effort_latest_qos, build_lut, declare_lut_parameters, stamp_to_ns


class RSULutStateNode(Node):
    def __init__(self):
        super().__init__("rsu_lut_state_node")
        declare_lut_parameters(self)
        self.declare_parameter("compute_rate_hz", 300.0)
        self.declare_parameter("motor_state_topic", "/hardware_interface/state")
        self.declare_parameter("state_topic", "/rsu/state")
        self.declare_parameter("gain_snapshot_topic", "/rsu_lut/gain_snapshot")
        self.declare_parameter("frame_id", "rsu_state")
        self.declare_parameter("left_motor_ids", [18, 20])
        self.declare_parameter("right_motor_ids", [19, 21])
        self.declare_parameter("left_q_seed", [0.0, -0.5672320])
        self.declare_parameter("right_q_seed", [0.0, -0.5672320])
        self.declare_parameter("left_alpha_seed", [-0.458105, 0.458105])
        self.declare_parameter("right_alpha_seed", [-0.458105, 0.458105])
        self.declare_parameter("require_valid_impedance", False)
        self._declare_impedance_parameters()

        self.compute_rate_hz = float(self.get_parameter("compute_rate_hz").value)
        if not np.isfinite(self.compute_rate_hz) or self.compute_rate_hz <= 0.0:
            raise RuntimeError("compute_rate_hz must be finite and > 0")
        self.left_motor_ids = tuple(int(v) for v in self.get_parameter("left_motor_ids").value)
        self.right_motor_ids = tuple(int(v) for v in self.get_parameter("right_motor_ids").value)
        if len(self.left_motor_ids) != 2 or len(self.right_motor_ids) != 2:
            raise RuntimeError("left_motor_ids/right_motor_ids must each contain two IDs")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.require_valid_impedance = bool(
            self.get_parameter("require_valid_impedance").value
        )

        self.lut = build_lut(self)
        factory = RSUEstimatorFactory(self, self.lut)
        self.left_estimator = factory.make()
        self.right_estimator = factory.make()
        self.left_estimator.reset(
            np.asarray(self.get_parameter("left_q_seed").value, dtype=float),
            np.asarray(self.get_parameter("left_alpha_seed").value, dtype=float),
            initialized=True,
        )
        self.right_estimator.reset(
            np.asarray(self.get_parameter("right_q_seed").value, dtype=float),
            np.asarray(self.get_parameter("right_alpha_seed").value, dtype=float),
            initialized=True,
        )
        self.left_mapper = self._make_mapper()
        self.right_mapper = self._make_mapper()

        qos = best_effort_latest_qos()
        self.state_pub = self.create_publisher(
            RsuStateArray, str(self.get_parameter("state_topic").value), qos
        )
        self.gain_pub = self.create_publisher(
            RsuImpSol, str(self.get_parameter("gain_snapshot_topic").value), qos
        )
        self.state_sub = self.create_subscription(
            MotorStateArray,
            str(self.get_parameter("motor_state_topic").value),
            self._on_motor_state,
            qos,
        )
        self.timer = self.create_timer(1.0 / self.compute_rate_hz, self._on_timer)

        self._latest_message = None
        self._processed_stamp_ns = None
        self._previous_feedback_stamp_ns = None
        self._sequence = 0
        self.get_logger().info(
            f"RSU LUT state process ready at {self.compute_rate_hz:.1f} Hz; "
            "subscription callback is latch-only"
        )

    def _declare_impedance_parameters(self):
        self.declare_parameter("virtual_pitch_kp", 25.0)
        self.declare_parameter("virtual_pitch_kd", 1.2)
        self.declare_parameter("virtual_roll_scale", 1.37)
        self.declare_parameter("actuator_kp_min", [5.0, 5.0])
        self.declare_parameter("actuator_kp_max", [25.0, 25.0])
        self.declare_parameter("actuator_kd_min", [0.2, 0.2])
        self.declare_parameter("actuator_kd_max", [6.0, 6.0])
        self.declare_parameter("default_kp", [9.0, 9.0])
        self.declare_parameter("default_kd", [2.25, 2.25])
        self.declare_parameter("imp_cond_warn", 10.0)
        self.declare_parameter("imp_cond_fail", 50.0)
        self.declare_parameter("imp_sigma_min_thresh", 1e-4)
        self.declare_parameter("fit_error_warn", 0.20)
        self.declare_parameter("fit_error_fail", 0.50)
        self.declare_parameter("gain_lpf_cutoff_hz", 0.0)
        self.declare_parameter("kp_slew_rate", 0.0)
        self.declare_parameter("kd_slew_rate", 0.0)
        self.declare_parameter("imp_dt_min", 1e-5)
        self.declare_parameter("imp_dt_max", 0.2)

    def _array_param(self, name):
        return np.asarray(self.get_parameter(name).value, dtype=float).reshape(2,)

    def _make_mapper(self):
        return RSUImpedanceMapper(
            RSUImpedanceMapperConfig(
                virtual_pitch_kp=float(self.get_parameter("virtual_pitch_kp").value),
                virtual_pitch_kd=float(self.get_parameter("virtual_pitch_kd").value),
                virtual_roll_scale=float(self.get_parameter("virtual_roll_scale").value),
                actuator_kp_min=self._array_param("actuator_kp_min"),
                actuator_kp_max=self._array_param("actuator_kp_max"),
                actuator_kd_min=self._array_param("actuator_kd_min"),
                actuator_kd_max=self._array_param("actuator_kd_max"),
                default_kp=self._array_param("default_kp"),
                default_kd=self._array_param("default_kd"),
                cond_warn=float(self.get_parameter("imp_cond_warn").value),
                cond_fail=float(self.get_parameter("imp_cond_fail").value),
                sigma_min_thresh=float(self.get_parameter("imp_sigma_min_thresh").value),
                relative_fit_error_warn=float(self.get_parameter("fit_error_warn").value),
                relative_fit_error_fail=float(self.get_parameter("fit_error_fail").value),
                gain_lpf_cutoff_hz=float(self.get_parameter("gain_lpf_cutoff_hz").value),
                kp_slew_rate=float(self.get_parameter("kp_slew_rate").value),
                kd_slew_rate=float(self.get_parameter("kd_slew_rate").value),
                dt_min=float(self.get_parameter("imp_dt_min").value),
                dt_max=float(self.get_parameter("imp_dt_max").value),
                hold_last_on_invalid=True,
            )
        )

    def _on_motor_state(self, message):
        self._latest_message = message

    def _on_timer(self):
        message = self._latest_message
        if message is None:
            return
        source_stamp_ns = stamp_to_ns(message.header.stamp)
        if source_stamp_ns == self._processed_stamp_ns:
            return
        self._processed_stamp_ns = source_stamp_ns
        if self._previous_feedback_stamp_ns is None:
            self._previous_feedback_stamp_ns = source_stamp_ns
            return
        dt = (source_stamp_ns - self._previous_feedback_stamp_ns) * 1e-9
        self._previous_feedback_stamp_ns = source_stamp_ns
        if not np.isfinite(dt) or dt <= 0.0:
            return

        states = {int(item.motor_id): item for item in message.states}
        required = set(self.left_motor_ids + self.right_motor_ids)
        if not required.issubset(states):
            self.get_logger().error(
                f"Missing RSU motor IDs: {sorted(required - set(states))}",
                throttle_duration_sec=1.0,
            )
            return

        left_pos = np.asarray([states[i].position for i in self.left_motor_ids], dtype=float)
        left_vel = np.asarray([states[i].velocity for i in self.left_motor_ids], dtype=float)
        right_pos = -np.asarray([states[i].position for i in self.right_motor_ids], dtype=float)
        right_vel = -np.asarray([states[i].velocity for i in self.right_motor_ids], dtype=float)
        left = self.left_estimator.update(left_pos, left_vel, dt)
        right = self.right_estimator.update(right_pos, right_vel, dt)
        left_imp = self.left_mapper.compute(left.J_qx, dt, left.valid, left.degraded)
        right_imp = self.right_mapper.compute(right.J_qx, dt, right.valid, right.degraded)

        state = RsuStateArray()
        state.header = message.header
        state.header.frame_id = self.frame_id
        state.seq = self._sequence
        state.q.left_rsu_roll, state.q.left_rsu_pitch = map(float, left.q_rel)
        # Legacy wire convention: right actuator feedback is mirrored before
        # estimation, then the reconstructed virtual state is mirrored back
        # when published on /rsu/state.
        state.q.right_rsu_roll, state.q.right_rsu_pitch = map(float, -right.q_rel)
        state.q_dot.left_rsu_roll, state.q_dot.left_rsu_pitch = map(float, left.qd_rel)
        state.q_dot.right_rsu_roll, state.q_dot.right_rsu_pitch = map(float, -right.qd_rel)
        state.feasible = bool(left.valid and right.valid and left.feasible and right.feasible)
        self.state_pub.publish(state)

        gain = RsuImpSol()
        gain.header = message.header
        gain.header.frame_id = self.frame_id
        gain.seq = self._sequence
        for output, result in (
            (gain.left_actuator_1, left_imp),
            (gain.right_actuator_1, right_imp),
        ):
            output.kp_eqv = float(result.kp[0])
            output.kd_eqv = float(result.kd[0])
        for output, result in (
            (gain.left_actuator_2, left_imp),
            (gain.right_actuator_2, right_imp),
        ):
            output.kp_eqv = float(result.kp[1])
            output.kd_eqv = float(result.kd[1])
        gain.feasible = bool(
            state.feasible
            and (
                (left_imp.valid and right_imp.valid)
                or not self.require_valid_impedance
            )
        )
        self.gain_pub.publish(gain)
        self._sequence += 1


def main(args=None):
    rclpy.init(args=args)
    node = RSULutStateNode()
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
