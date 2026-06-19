#!/usr/bin/env python3

"""
RSU real-time impedance IK solver and state estimator node.

Inputs:
  /rsu/target
  /hardware_interface/state

Outputs:
  /rsu/imp_solution
  /rsu/state

Flow:
  - Target callback:
      virtual roll/pitch target -> RSU IK -> actuator q_target
      latest current-pose impedance gains -> RsuImpSol publish

  - Motor-state callback:
      motor state -> RSU state estimator
      current-pose Jacobian -> RSU impedance mapper
      cache latest actuator Kp/Kd
      publish virtual RSU state
"""

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from roa_interfaces.msg import (
    RsuTarget,
    RsuImpSol,
    MotorStateArray,
    RsuStateArray,
)

from rsu_manager.util.core import (
    RSUCore,
    RSUEstimatorFactory,
    stamp_to_ns,
)

from rsu_manager.util.rsu_impedance_mapper import (
    RSUImpedanceMapper,
    RSUImpedanceMapperConfig,
)


class RSURtImpedanceSolverNode(Node):
    """
    실시간 RSU IK + state estimator + impedance gain mapper 노드.
    """

    def __init__(self):
        super().__init__("rt_imp_solver_node")

        self.core = RSUCore(self)
        self.solver = self.core.solver

        self._declare_and_get_params()

        self.motor_state = {
            "left_ac1": {
                "pos": None,
                "vel": None,
                "id": self.left_ac1_id,
            },
            "left_ac2": {
                "pos": None,
                "vel": None,
                "id": self.left_ac2_id,
            },
            "right_ac1": {
                "pos": None,
                "vel": None,
                "id": self.right_ac1_id,
            },
            "right_ac2": {
                "pos": None,
                "vel": None,
                "id": self.right_ac2_id,
            },
        }

        # Target ordering
        self._last_seq = None
        self._last_stamp = None

        # IK branch continuity state
        self.prev_alpha_2d = np.vstack([
            self.left_alpha_seed.copy(),
            self.right_alpha_seed.copy(),
        ]).astype(np.float64)

        # Last valid IK targets in internal solver convention
        self.last_left_alpha_target = self.left_alpha_seed.copy()
        self.last_right_alpha_target = self.right_alpha_seed.copy()

        # Target LPF state
        self.target_lpf_2d = np.zeros((2, 2), dtype=np.float64)
        self.target_lpf_initialized = False
        self._last_target_lpf_stamp_ns = None

        # State estimators
        estimator_factory = RSUEstimatorFactory(self, self.solver)

        self.left_estimator = estimator_factory.make()
        self.right_estimator = estimator_factory.make()

        self.left_estimator.reset(
            q_init=self.left_q_seed,
            alpha_seed=self.left_alpha_seed,
            initialized=True,
        )
        self.right_estimator.reset(
            q_init=self.right_q_seed,
            alpha_seed=self.right_alpha_seed,
            initialized=True,
        )

        # Independent mapper state for left and right foot
        imp_cfg = RSUImpedanceMapperConfig(
            virtual_pitch_kp=self.virtual_pitch_kp,
            virtual_pitch_kd=self.virtual_pitch_kd,
            virtual_roll_scale=self.virtual_roll_scale,

            actuator_kp_min=self.actuator_kp_min,
            actuator_kp_max=self.actuator_kp_max,
            actuator_kd_min=self.actuator_kd_min,
            actuator_kd_max=self.actuator_kd_max,

            default_kp=self.default_kp,
            default_kd=self.default_kd,

            cond_warn=self.imp_cond_warn,
            cond_fail=self.imp_cond_fail,
            sigma_min_thresh=self.imp_sigma_min_thresh,

            relative_fit_error_warn=self.fit_error_warn,
            relative_fit_error_fail=self.fit_error_fail,

            gain_lpf_cutoff_hz=self.gain_lpf_cutoff_hz,
            kp_slew_rate=self.kp_slew_rate,
            kd_slew_rate=self.kd_slew_rate,

            dt_min=self.imp_dt_min,
            dt_max=self.imp_dt_max,
            hold_last_on_invalid=True,
        )

        self.left_imp_mapper = RSUImpedanceMapper(imp_cfg)
        self.right_imp_mapper = RSUImpedanceMapper(imp_cfg)

        self.latest_left_imp = None
        self.latest_right_imp = None

        self.latest_left_state = None
        self.latest_right_state = None

        self.left_state_ready = False
        self.right_state_ready = False

        self._last_motor_state_stamp_ns = None
        self._rsu_state_seq = 0

        # QoS
        rsu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        motor_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Publisher / subscriber
        self.pub_imp_solution = self.create_publisher(
            RsuImpSol,
            "/rsu/imp_solution",
            rsu_qos,
        )

        self.sub_both_foot_request = self.create_subscription(
            RsuTarget,
            "/rsu/target",
            self._on_both_foot_request,
            rsu_qos,
        )

        self.sub_motor_status = self.create_subscription(
            MotorStateArray,
            "/hardware_interface/state",
            self._on_motor_state,
            motor_status_qos,
        )

        self.pub_rsu_state = self.create_publisher(
            RsuStateArray,
            "/rsu/state",
            rsu_qos,
        )

        self.get_logger().info(
            "RSU RT Impedance Solver started.\n"
            "Subscribing:\n"
            "  /rsu/target\n"
            "  /hardware_interface/state\n"
            "Publishing:\n"
            "  /rsu/imp_solution\n"
            "  /rsu/state"
        )

    # =========================================================================
    # Parameters
    # =========================================================================
    def _declare_and_get_params(self):
        # Existing solver parameters
        self.declare_parameter("hold_alpha_on_infeasible", True)

        self.declare_parameter("left_ac1_id", 18)
        self.declare_parameter("left_ac2_id", 20)
        self.declare_parameter("right_ac1_id", 19)
        self.declare_parameter("right_ac2_id", 21)

        self.declare_parameter("left_q_seed", [0.0, 0.0])
        self.declare_parameter("right_q_seed", [0.0, 0.0])

        self.declare_parameter("left_alpha_seed", [0.0, 0.0])
        self.declare_parameter("right_alpha_seed", [0.0, 0.0])

        self.declare_parameter("rsu_state_frame_id", "base_link")

        self.declare_parameter("target_lpf_enable", False)
        self.declare_parameter("target_lpf_cutoff_hz", 2.0)

        # Desired virtual impedance
        self.declare_parameter("virtual_pitch_kp", 40.0)
        self.declare_parameter("virtual_pitch_kd", 10.0)
        self.declare_parameter("virtual_roll_scale", 1.37)

        # Actuator gain limits
        self.declare_parameter("actuator_kp_min", [5.0, 5.0])
        self.declare_parameter("actuator_kp_max", [25.0, 25.0])
        self.declare_parameter("actuator_kd_min", [0.2, 0.2])
        self.declare_parameter("actuator_kd_max", [6.0, 6.0])

        self.declare_parameter("default_kp", [18.0, 18.0])
        self.declare_parameter("default_kd", [4.5, 4.5])

        # Impedance mapping validity
        self.declare_parameter("imp_cond_warn", 10.0)
        self.declare_parameter("imp_cond_fail", 50.0)
        self.declare_parameter("imp_sigma_min_thresh", 1e-4)

        self.declare_parameter("fit_error_warn", 0.20)
        self.declare_parameter("fit_error_fail", 0.50)

        # Gain temporal stabilization
        self.declare_parameter("gain_lpf_cutoff_hz", 0.0)
        self.declare_parameter("kp_slew_rate", 0.0)
        self.declare_parameter("kd_slew_rate", 0.0)

        self.declare_parameter("imp_dt_min", 1e-5)
        self.declare_parameter("imp_dt_max", 0.2)

        # Get existing parameters
        self.hold_alpha_on_infeasible = bool(
            self.get_parameter("hold_alpha_on_infeasible").value
        )

        self.left_ac1_id = int(self.get_parameter("left_ac1_id").value)
        self.left_ac2_id = int(self.get_parameter("left_ac2_id").value)
        self.right_ac1_id = int(self.get_parameter("right_ac1_id").value)
        self.right_ac2_id = int(self.get_parameter("right_ac2_id").value)

        self.left_q_seed = np.asarray(
            self.get_parameter("left_q_seed").value,
            dtype=np.float64,
        ).reshape(2,)

        self.right_q_seed = np.asarray(
            self.get_parameter("right_q_seed").value,
            dtype=np.float64,
        ).reshape(2,)

        self.left_alpha_seed = np.asarray(
            self.get_parameter("left_alpha_seed").value,
            dtype=np.float64,
        ).reshape(2,)

        self.right_alpha_seed = np.asarray(
            self.get_parameter("right_alpha_seed").value,
            dtype=np.float64,
        ).reshape(2,)

        self.rsu_state_frame_id = str(
            self.get_parameter("rsu_state_frame_id").value
        )

        self.target_lpf_enable = bool(
            self.get_parameter("target_lpf_enable").value
        )

        self.target_lpf_cutoff_hz = float(
            self.get_parameter("target_lpf_cutoff_hz").value
        )

        # Get virtual impedance parameters
        self.virtual_pitch_kp = float(
            self.get_parameter("virtual_pitch_kp").value
        )
        self.virtual_pitch_kd = float(
            self.get_parameter("virtual_pitch_kd").value
        )
        self.virtual_roll_scale = float(
            self.get_parameter("virtual_roll_scale").value
        )

        # Get actuator bounds
        self.actuator_kp_min = np.asarray(
            self.get_parameter("actuator_kp_min").value,
            dtype=np.float64,
        ).reshape(2,)

        self.actuator_kp_max = np.asarray(
            self.get_parameter("actuator_kp_max").value,
            dtype=np.float64,
        ).reshape(2,)

        self.actuator_kd_min = np.asarray(
            self.get_parameter("actuator_kd_min").value,
            dtype=np.float64,
        ).reshape(2,)

        self.actuator_kd_max = np.asarray(
            self.get_parameter("actuator_kd_max").value,
            dtype=np.float64,
        ).reshape(2,)

        self.default_kp = np.asarray(
            self.get_parameter("default_kp").value,
            dtype=np.float64,
        ).reshape(2,)

        self.default_kd = np.asarray(
            self.get_parameter("default_kd").value,
            dtype=np.float64,
        ).reshape(2,)

        # Get mapper validity parameters
        self.imp_cond_warn = float(
            self.get_parameter("imp_cond_warn").value
        )
        self.imp_cond_fail = float(
            self.get_parameter("imp_cond_fail").value
        )
        self.imp_sigma_min_thresh = float(
            self.get_parameter("imp_sigma_min_thresh").value
        )

        self.fit_error_warn = float(
            self.get_parameter("fit_error_warn").value
        )
        self.fit_error_fail = float(
            self.get_parameter("fit_error_fail").value
        )

        self.gain_lpf_cutoff_hz = float(
            self.get_parameter("gain_lpf_cutoff_hz").value
        )
        self.kp_slew_rate = float(
            self.get_parameter("kp_slew_rate").value
        )
        self.kd_slew_rate = float(
            self.get_parameter("kd_slew_rate").value
        )

        self.imp_dt_min = float(
            self.get_parameter("imp_dt_min").value
        )
        self.imp_dt_max = float(
            self.get_parameter("imp_dt_max").value
        )

        self._validate_params()

        roll_kp = self.virtual_pitch_kp * self.virtual_roll_scale
        roll_kd = self.virtual_pitch_kd * self.virtual_roll_scale

        self.get_logger().info(
            "\n[RT Impedance Solver Params]\n"
            f"  hold_alpha_on_infeasible: {self.hold_alpha_on_infeasible}\n"
            f"  motor IDs: "
            f"L({self.left_ac1_id}, {self.left_ac2_id}), "
            f"R({self.right_ac1_id}, {self.right_ac2_id})\n"
            f"  left_q_seed: {self.left_q_seed.tolist()}\n"
            f"  right_q_seed: {self.right_q_seed.tolist()}\n"
            f"  left_alpha_seed: {self.left_alpha_seed.tolist()}\n"
            f"  right_alpha_seed: {self.right_alpha_seed.tolist()}\n"
            f"  rsu_state_frame_id: {self.rsu_state_frame_id}\n"
            f"  target_lpf_enable: {self.target_lpf_enable}\n"
            f"  target_lpf_cutoff_hz: {self.target_lpf_cutoff_hz}\n"
            f"  desired virtual Kp [roll, pitch]: [{roll_kp}, {self.virtual_pitch_kp}]\n"
            f"  desired virtual Kd [roll, pitch]: [{roll_kd}, {self.virtual_pitch_kd}]\n"
            f"  actuator_kp_min: {self.actuator_kp_min.tolist()}\n"
            f"  actuator_kp_max: {self.actuator_kp_max.tolist()}\n"
            f"  actuator_kd_min: {self.actuator_kd_min.tolist()}\n"
            f"  actuator_kd_max: {self.actuator_kd_max.tolist()}\n"
            f"  default_kp: {self.default_kp.tolist()}\n"
            f"  default_kd: {self.default_kd.tolist()}\n"
            f"  gain_lpf_cutoff_hz: {self.gain_lpf_cutoff_hz}\n"
            f"  kp_slew_rate: {self.kp_slew_rate}\n"
            f"  kd_slew_rate: {self.kd_slew_rate}"
        )

    def _validate_params(self):
        if np.any(self.actuator_kp_min > self.actuator_kp_max):
            raise RuntimeError("actuator_kp_min must be <= actuator_kp_max")

        if np.any(self.actuator_kd_min > self.actuator_kd_max):
            raise RuntimeError("actuator_kd_min must be <= actuator_kd_max")

        if self.imp_cond_warn <= 0.0:
            raise RuntimeError("imp_cond_warn must be > 0")

        if self.imp_cond_fail <= self.imp_cond_warn:
            raise RuntimeError("imp_cond_fail must be > imp_cond_warn")

        if self.fit_error_warn < 0.0:
            raise RuntimeError("fit_error_warn must be >= 0")

        if self.fit_error_fail <= self.fit_error_warn:
            raise RuntimeError("fit_error_fail must be > fit_error_warn")

    # =========================================================================
    # Target handling
    # =========================================================================
    def _accept_target_order(self, seq: int, stamp) -> bool:
        if seq != 0:
            if self._last_seq is None:
                self._last_seq = int(seq)
                return True

            if int(seq) <= int(self._last_seq):
                return False

            self._last_seq = int(seq)
            return True

        if self._last_stamp is None:
            self._last_stamp = stamp
            return True

        if stamp_to_ns(stamp) <= stamp_to_ns(self._last_stamp):
            return False

        self._last_stamp = stamp
        return True

    # def _filter_rsu_target(
    #     self,
    #     target_2d: np.ndarray,
    #     stamp,
    # ) -> np.ndarray:
    #     target_2d = np.asarray(
    #         target_2d,
    #         dtype=np.float64,
    #     ).reshape(2, 2)

    #     stamp_ns = stamp_to_ns(stamp)

    #     if not self.target_lpf_enable:
    #         self.target_lpf_2d = target_2d.copy()
    #         self.target_lpf_initialized = True
    #         self._last_target_lpf_stamp_ns = stamp_ns
    #         return target_2d.copy()

    #     cutoff_hz = float(self.target_lpf_cutoff_hz)

    #     if not np.isfinite(cutoff_hz) or cutoff_hz <= 0.0:
    #         self.get_logger().warn(
    #             f"Invalid target_lpf_cutoff_hz={cutoff_hz}. Bypassing LPF.",
    #             throttle_duration_sec=1.0,
    #         )
    #         self.target_lpf_2d = target_2d.copy()
    #         self.target_lpf_initialized = True
    #         self._last_target_lpf_stamp_ns = stamp_ns
    #         return target_2d.copy()

    #     if (
    #         not self.target_lpf_initialized
    #         or self._last_target_lpf_stamp_ns is None
    #     ):
    #         self.target_lpf_2d = target_2d.copy()
    #         self.target_lpf_initialized = True
    #         self._last_target_lpf_stamp_ns = stamp_ns
    #         return self.target_lpf_2d.copy()

    #     dt = (stamp_ns - self._last_target_lpf_stamp_ns) * 1e-9
    #     self._last_target_lpf_stamp_ns = stamp_ns

    #     if not np.isfinite(dt) or dt <= 0.0:
    #         self.get_logger().warn(
    #             f"Invalid target LPF dt={dt}. Holding previous target.",
    #             throttle_duration_sec=1.0,
    #         )
    #         return self.target_lpf_2d.copy()

    #     tau = 1.0 / (2.0 * np.pi * cutoff_hz)
    #     gamma = dt / (tau + dt)
    #     gamma = float(np.clip(gamma, 0.0, 1.0))

    #     self.target_lpf_2d = (
    #         self.target_lpf_2d
    #         + gamma * (target_2d - self.target_lpf_2d)
    #     )

    #     return self.target_lpf_2d.copy()

    def _on_both_foot_request(self, msg: RsuTarget):
        if not self._accept_target_order(msg.seq, msg.header.stamp):
            self.get_logger().warn(
                "Received /rsu/target with non-increasing seq/stamp. Ignoring.",
                throttle_duration_sec=1.0,
            )
            return

        # External command convention
        l_roll = float(msg.left_roll)
        l_pitch = float(msg.left_pitch)

        # Convert right-foot command to internal solver convention
        r_roll = -float(msg.right_roll)
        r_pitch = -float(msg.right_pitch)

        # target_raw_2d = np.array([
        #     [l_roll_raw, l_pitch_raw],
        #     [r_roll_raw, r_pitch_raw],
        # ], dtype=np.float64)

        # target_cmd_2d = self._filter_rsu_target(
        #     target_2d=target_raw_2d,
        #     stamp=msg.header.stamp,
        # )

        # l_roll = float(target_cmd_2d[0, 0])
        # l_pitch = float(target_cmd_2d[0, 1])
        # r_roll = float(target_cmd_2d[1, 0])
        # r_pitch = float(target_cmd_2d[1, 1])

        l_prev = self.prev_alpha_2d[0, :].copy()
        r_prev = self.prev_alpha_2d[1, :].copy()

        l_res = self.solver.solve(
            l_roll,
            l_pitch,
            l_prev,
        )
        r_res = self.solver.solve(
            r_roll,
            r_pitch,
            r_prev,
        )

        left_ik_ok = bool(
            l_res.feasible
            and np.all(np.isfinite(l_res.alpha))
            # and np.all(l_res.branch >= 0)
        )

        right_ik_ok = bool(
            r_res.feasible
            and np.all(np.isfinite(r_res.alpha))
            # and np.all(r_res.branch >= 0)
        )

        if left_ik_ok:
            left_alpha_cmd = np.asarray(
                l_res.alpha,
                dtype=np.float64,
            ).reshape(2,)

            self.prev_alpha_2d[0, :] = left_alpha_cmd
            self.last_left_alpha_target = left_alpha_cmd.copy()
        else:
            left_alpha_cmd = self.last_left_alpha_target.copy()

            if not self.hold_alpha_on_infeasible:
                left_alpha_cmd = np.zeros(2, dtype=np.float64)
                self.prev_alpha_2d[0, :] = left_alpha_cmd
                self.last_left_alpha_target = left_alpha_cmd.copy()

        if right_ik_ok:
            right_alpha_cmd = np.asarray(
                r_res.alpha,
                dtype=np.float64,
            ).reshape(2,)

            self.prev_alpha_2d[1, :] = right_alpha_cmd
            self.last_right_alpha_target = right_alpha_cmd.copy()
        else:
            right_alpha_cmd = self.last_right_alpha_target.copy()

            if not self.hold_alpha_on_infeasible:
                right_alpha_cmd = np.zeros(2, dtype=np.float64)
                self.prev_alpha_2d[1, :] = right_alpha_cmd
                self.last_right_alpha_target = right_alpha_cmd.copy()

        left_kp, left_kd, left_imp_ok = self._get_cached_impedance(
            self.latest_left_imp
        )
        right_kp, right_kd, right_imp_ok = self._get_cached_impedance(
            self.latest_right_imp
        )

        out = RsuImpSol()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.rsu_state_frame_id
        out.seq = msg.seq

        # Left foot: internal solver convention equals hardware convention
        out.left_actuator_1.q_target = float(left_alpha_cmd[0])
        out.left_actuator_1.kp_eqv = float(left_kp[0])
        out.left_actuator_1.kd_eqv = float(left_kd[0])

        out.left_actuator_2.q_target = float(left_alpha_cmd[1])
        out.left_actuator_2.kp_eqv = float(left_kp[1])
        out.left_actuator_2.kd_eqv = float(left_kd[1])

        # Right foot: convert internal actuator convention back to hardware
        out.right_actuator_1.q_target = float(-right_alpha_cmd[0])
        out.right_actuator_1.kp_eqv = float(right_kp[0])
        out.right_actuator_1.kd_eqv = float(right_kd[0])

        out.right_actuator_2.q_target = float(-right_alpha_cmd[1])
        out.right_actuator_2.kp_eqv = float(right_kp[1])
        out.right_actuator_2.kd_eqv = float(right_kd[1])

        out.feasible = bool(
            left_ik_ok
            and right_ik_ok
            # and left_imp_ok
            # and right_imp_ok
            # and self.left_state_ready
            # and self.right_state_ready
        )

        self.pub_imp_solution.publish(out)

        if not out.feasible:
            self.get_logger().error(
                "[RSU imp solution] fallback/degraded output | "
                f"IK(L={left_ik_ok}, R={right_ik_ok}) | "
                f"IMP(L={left_imp_ok}, R={right_imp_ok}) | "
                f"STATE_READY(L={self.left_state_ready}, "
                f"R={self.right_state_ready})",
                throttle_duration_sec=0.5,
            )

    def _get_cached_impedance(self, imp_result):
        if imp_result is None:
            return (
                self.default_kp.copy(),
                self.default_kd.copy(),
                False,
            )

        kp = np.asarray(
            imp_result.kp,
            dtype=np.float64,
        ).reshape(2,)

        kd = np.asarray(
            imp_result.kd,
            dtype=np.float64,
        ).reshape(2,)

        if not np.all(np.isfinite(kp)) or not np.all(np.isfinite(kd)):
            return (
                self.default_kp.copy(),
                self.default_kd.copy(),
                False,
            )
        kp = np.clip(
            kp,
            self.actuator_kp_min,
            self.actuator_kp_max,
        )

        kd = np.clip(
            kd,
            self.actuator_kd_min,
            self.actuator_kd_max,
        )
        return kp, kd, bool(imp_result.valid)

    # =========================================================================
    # Motor-state handling
    # =========================================================================
    def _on_motor_state(self, msg: MotorStateArray):
        required_ids = {
            self.motor_state["left_ac1"]["id"],
            self.motor_state["left_ac2"]["id"],
            self.motor_state["right_ac1"]["id"],
            self.motor_state["right_ac2"]["id"],
        }

        msg_ids = {int(st.motor_id) for st in msg.states}
        missing_ids = required_ids - msg_ids

        if missing_ids:
            self.get_logger().error(
                f"Missing motor IDs in /hardware_interface/state: {missing_ids}. "
                f"Expected: {required_ids}, Got: {msg_ids}"
            )
            return

        id_to_state = {
            int(st.motor_id): st
            for st in msg.states
        }

        try:
            self.motor_state["left_ac1"]["pos"] = float(
                id_to_state[
                    self.motor_state["left_ac1"]["id"]
                ].position
            )
            self.motor_state["left_ac1"]["vel"] = float(
                id_to_state[
                    self.motor_state["left_ac1"]["id"]
                ].velocity
            )

            self.motor_state["left_ac2"]["pos"] = float(
                id_to_state[
                    self.motor_state["left_ac2"]["id"]
                ].position
            )
            self.motor_state["left_ac2"]["vel"] = float(
                id_to_state[
                    self.motor_state["left_ac2"]["id"]
                ].velocity
            )

            self.motor_state["right_ac1"]["pos"] = float(
                id_to_state[
                    self.motor_state["right_ac1"]["id"]
                ].position
            )
            self.motor_state["right_ac1"]["vel"] = float(
                id_to_state[
                    self.motor_state["right_ac1"]["id"]
                ].velocity
            )

            self.motor_state["right_ac2"]["pos"] = float(
                id_to_state[
                    self.motor_state["right_ac2"]["id"]
                ].position
            )
            self.motor_state["right_ac2"]["vel"] = float(
                id_to_state[
                    self.motor_state["right_ac2"]["id"]
                ].velocity
            )

        except (KeyError, IndexError, AttributeError, TypeError) as exc:
            self.get_logger().error(
                f"Failed to parse motor state: {exc}"
            )
            return

        stamp_ns = stamp_to_ns(msg.header.stamp)

        if self._last_motor_state_stamp_ns is None:
            self._last_motor_state_stamp_ns = stamp_ns
            return

        dt = (
            stamp_ns - self._last_motor_state_stamp_ns
        ) * 1e-9

        self._last_motor_state_stamp_ns = stamp_ns

        if not np.isfinite(dt) or dt <= 0.0:
            self.get_logger().warn(
                f"Invalid dt from /hardware_interface/state: dt={dt}. "
                "Ignoring message.",
                throttle_duration_sec=1.0,
            )
            return

        left_motor_pos = np.array([
            self.motor_state["left_ac1"]["pos"],
            self.motor_state["left_ac2"]["pos"],
        ], dtype=np.float64)

        left_motor_vel = np.array([
            self.motor_state["left_ac1"]["vel"],
            self.motor_state["left_ac2"]["vel"],
        ], dtype=np.float64)

        right_motor_pos = np.array([
            self.motor_state["right_ac1"]["pos"],
            self.motor_state["right_ac2"]["pos"],
        ], dtype=np.float64)

        right_motor_vel = np.array([
            self.motor_state["right_ac1"]["vel"],
            self.motor_state["right_ac2"]["vel"],
        ], dtype=np.float64)

        left_state, left_q, left_qd = self._estimate_one_foot(
            estimator=self.left_estimator,
            motor_pos=left_motor_pos,
            motor_vel=left_motor_vel,
            dt=dt,
            mirror=False,
        )

        right_state, right_q, right_qd = self._estimate_one_foot(
            estimator=self.right_estimator,
            motor_pos=right_motor_pos,
            motor_vel=right_motor_vel,
            dt=dt,
            mirror=True,
        )

        self.latest_left_state = left_state
        self.latest_right_state = right_state

        # Current-pose Jacobian -> scheduled actuator gains
        left_imp = self.left_imp_mapper.compute(
            J_qx=left_state.J_qx,
            dt=dt,
            estimator_valid=left_state.valid,
            estimator_degraded=left_state.degraded,
        )

        right_imp = self.right_imp_mapper.compute(
            J_qx=right_state.J_qx,
            dt=dt,
            estimator_valid=right_state.valid,
            estimator_degraded=right_state.degraded,
        )

        self.latest_left_imp = left_imp
        self.latest_right_imp = right_imp

        self.left_state_ready = bool(left_state.valid)
        self.right_state_ready = bool(right_state.valid)

        # Publish estimated virtual RSU state
        out = RsuStateArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.rsu_state_frame_id
        out.seq = self._rsu_state_seq
        self._rsu_state_seq += 1

        out.q_dot.left_rsu_roll = float(left_qd[0])
        out.q_dot.left_rsu_pitch = float(left_qd[1])
        out.q_dot.right_rsu_roll = float(right_qd[0])
        out.q_dot.right_rsu_pitch = float(right_qd[1])

        out.q.left_rsu_roll = float(left_q[0])
        out.q.left_rsu_pitch = float(left_q[1])
        out.q.right_rsu_roll = float(right_q[0])
        out.q.right_rsu_pitch = float(right_q[1])

        out.feasible = bool(
            left_state.feasible
            and right_state.feasible
            and left_state.valid
            and right_state.valid
        )

        self.pub_rsu_state.publish(out)

        self._log_estimator_status(
            left_state=left_state,
            right_state=right_state,
        )

        self._log_impedance_status(
            left_imp=left_imp,
            right_imp=right_imp,
        )

    def _estimate_one_foot(
        self,
        estimator,
        motor_pos,
        motor_vel,
        dt,
        mirror=False,
    ):
        motor_pos = np.asarray(
            motor_pos,
            dtype=np.float64,
        ).reshape(2,)

        motor_vel = np.asarray(
            motor_vel,
            dtype=np.float64,
        ).reshape(2,)

        if mirror:
            motor_pos = -motor_pos
            motor_vel = -motor_vel

        state = estimator.update(
            motor_pos,
            motor_vel,
            dt,
        )

        q = np.asarray(
            state.q_rel,
            dtype=np.float64,
        ).reshape(2,)

        qd = np.asarray(
            state.qd_rel,
            dtype=np.float64,
        ).reshape(2,)

        if mirror:
            q = -q
            qd = -qd

        return state, q, qd

    # =========================================================================
    # Logging
    # =========================================================================
    def _log_estimator_status(self, left_state, right_state):
        if (not left_state.valid) or (not right_state.valid):
            self.get_logger().warn(
                "[RSU estimator] invalid state | "
                f"L(feasible={left_state.feasible}, "
                f"valid={left_state.valid}, "
                f"res={left_state.residual_norm:.3e}, "
                f"cond={left_state.condJ:.3f}, "
                f"sigma_min={left_state.sigma_min:.3e}) | "
                f"R(feasible={right_state.feasible}, "
                f"valid={right_state.valid}, "
                f"res={right_state.residual_norm:.3e}, "
                f"cond={right_state.condJ:.3f}, "
                f"sigma_min={right_state.sigma_min:.3e})",
                throttle_duration_sec=0.5,
            )
        elif left_state.degraded or right_state.degraded:
            self.get_logger().warn(
                "[RSU estimator] degraded state | "
                f"L(res={left_state.residual_norm:.3e}, "
                f"cond={left_state.condJ:.3f}, "
                f"sigma_min={left_state.sigma_min:.3e}) | "
                f"R(res={right_state.residual_norm:.3e}, "
                f"cond={right_state.condJ:.3f}, "
                f"sigma_min={right_state.sigma_min:.3e})",
                throttle_duration_sec=0.5,
            )

    def _log_impedance_status(self, left_imp, right_imp):
        if (not left_imp.valid) or (not right_imp.valid):
            self.get_logger().warn(
                "[RSU impedance] fallback | "
                f"L(valid={left_imp.valid}, "
                f"fallback={left_imp.fallback_used}, "
                f"cond={left_imp.condJ:.3f}, "
                f"Kerr={left_imp.stiffness_error:.3f}, "
                f"Derr={left_imp.damping_error:.3f}, "
                f"msg='{left_imp.debug_msg}') | "
                f"R(valid={right_imp.valid}, "
                f"fallback={right_imp.fallback_used}, "
                f"cond={right_imp.condJ:.3f}, "
                f"Kerr={right_imp.stiffness_error:.3f}, "
                f"Derr={right_imp.damping_error:.3f}, "
                f"msg='{right_imp.debug_msg}')",
                throttle_duration_sec=0.5,
            )
            return

        if left_imp.degraded or right_imp.degraded:
            self.get_logger().warn(
                "[RSU impedance] degraded | "
                f"L(kp={left_imp.kp.tolist()}, "
                f"kd={left_imp.kd.tolist()}, "
                f"cond={left_imp.condJ:.3f}, "
                f"Kerr={left_imp.stiffness_error:.3f}, "
                f"Derr={left_imp.damping_error:.3f}, "
                f"msg='{left_imp.debug_msg}') | "
                f"R(kp={right_imp.kp.tolist()}, "
                f"kd={right_imp.kd.tolist()}, "
                f"cond={right_imp.condJ:.3f}, "
                f"Kerr={right_imp.stiffness_error:.3f}, "
                f"Derr={right_imp.damping_error:.3f}, "
                f"msg='{right_imp.debug_msg}')",
                throttle_duration_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)
    node = RSURtImpedanceSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()