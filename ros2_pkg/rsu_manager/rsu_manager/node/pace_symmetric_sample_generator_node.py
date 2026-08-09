#!/usr/bin/env python3

'''
ros2_pkg.rsu_manager.rsu_manager.node.pace_symmetric_sample_generator_node의 Docstring

Data Recording Command:

ros2 bag record \
  -o pace_chirp_$(date +%Y%m%d_%H%M%S) \
  /hardware_interface/command \
  /hardware_interface/state \
  /rsu/target \
  /rsu/imp_solution \
  /rsu/state

'''

from __future__ import annotations

import math
import os
import time
import yaml
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from roa_interfaces.msg import (
    RsuTarget,
    RsuImpSol,
    MotorCommand,
    MotorCommandArray,
    MotorStateArray,
)

from rsu_manager.util.hip_roll_pitch_mapper import (
    map_hip_roll_pitch_to_motor_angles,
)

def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return current


def all_finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


cmd_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
rsu_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
state_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class SamplePhase(Enum):
    INITIAL_HOLD = 0
    CHIRP = 1
    RETURN_CENTER = 2
    HOLD_CENTER = 3
    DONE = 4
    FAULT = 5


@dataclass
class RsuActuatorCommand:
    position: float
    kp: float
    kd: float


class SymmetricChirpSampleGeneratorNode(Node):
    UPPER_BODY_IDS = set(range(0, 8))
    TORSO_YAW_ID = 9

    LEFT_HIP_THETA1_ID = 10
    RIGHT_HIP_THETA1_ID = 11
    LEFT_HIP_THETA2_ID = 12
    RIGHT_HIP_THETA2_ID = 13

    LEFT_HIP_YAW_ID = 14
    RIGHT_HIP_YAW_ID = 15
    LEFT_KNEE_ID = 16
    RIGHT_KNEE_ID = 17

    LEFT_RSU_AC1_ID = 18
    RIGHT_RSU_AC1_ID = 19
    LEFT_RSU_AC2_ID = 20
    RIGHT_RSU_AC2_ID = 21

    RSU_MOTOR_IDS = {18, 19, 20, 21}

    def __init__(self) -> None:
        super().__init__("pace_symmetric_sample_generator_node")

        self.load_parameters()

        self.kp_map = {
            9: 50.0,
            10: 150.0,
            11: 150.0,
            12: 200.0,
            13: 200.0,
            14: 100.0,
            15: 100.0,
            16: 150.0,
            17: 150.0,
        }

        self.kd_map = {
            9: 2.0,
            10: 24.722,
            11: 24.722,
            12: 26.387,
            13: 26.387,
            14: 3.419,
            15: 3.419,
            16: 8.654,
            17: 8.654,
        }
        self._validate_parameters()

        self.pub_rsu_target = self.create_publisher(
            RsuTarget, "/rsu/target", rsu_qos
        )
        self.pub_motor_cmd = self.create_publisher(
            MotorCommandArray, "/hardware_interface/command", cmd_qos
        )
        self.sub_rsu_imp_solution = self.create_subscription(
            RsuImpSol,
            "/rsu/imp_solution",
            self.on_rsu_imp_solution,
            rsu_qos,
        )
        self.sub_motor_state = self.create_subscription(
            MotorStateArray,
            "/hardware_interface/state",
            self.on_motor_state,
            state_qos,
        )

        self.motor_ids: List[int] = []
        self.motor_types: List[str] = []
        self.load_motor_yaml()

        self.latest_motor_pos: Dict[int, float] = {}
        self.latest_motor_vel: Dict[int, float] = {}
        self.last_motor_state_t: Optional[float] = None

        self.rsu_actuator_cmd: Dict[int, Optional[RsuActuatorCommand]] = {
            18: None,
            20: None,
            19: None,
            21: None,
        }
        self.rsu_solution_valid = False
        self.last_rsu_solution_t: Optional[float] = None
        self.rsu_target_seq = 1

        self.joint_pairs = self.build_joint_pairs()
        self.configure_pair_frequencies()

        self.phase = SamplePhase.INITIAL_HOLD
        self.phase_start_t = time.monotonic()
        self.last_t = time.monotonic()
        self.pair_idx = 0
        self.offset = 0.0
        self.done_logged = False
        self.fault_reason = ""

        self.previous_final_position: Dict[int, float] = {}
        self.previous_final_position_time: Optional[float] = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.print_test_plan()

    def load_parameters(self) -> None:
        # ────────────── 파라미터 선언 ──────────────
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("initial_hold_sec", 5.0)
        self.declare_parameter("center_hold_sec", 1.0)
        self.declare_parameter("chirp_duration_sec", 20.0)
        self.declare_parameter("min_frequency_hz", 0.1)
        self.declare_parameter("physical_velocity_warn_scale", 1.10)
        self.declare_parameter("return_speed_rad_s", 1.0)
        self.declare_parameter("position_tolerance_rad", 1.0e-5)

        self.declare_parameter("hip_pitch.amp_deg", 5.0)
        self.declare_parameter("hip_pitch.max_frequency_hz", 5.0)
        self.declare_parameter("hip_pitch.max_velocity_rad_s", 2.0)

        self.declare_parameter("hip_roll.amp_deg", 3.0)
        self.declare_parameter("hip_roll.max_frequency_hz", 5.0)
        self.declare_parameter("hip_roll.max_velocity_rad_s", 1.5)

        self.declare_parameter("hip_yaw.amp_deg", 5.0)
        self.declare_parameter("hip_yaw.max_frequency_hz", 5.0)
        self.declare_parameter("hip_yaw.max_velocity_rad_s", 2.0)

        self.declare_parameter("knee_pitch.amp_deg", 10.0)
        self.declare_parameter("knee_pitch.max_frequency_hz", 3.0)
        self.declare_parameter("knee_pitch.max_velocity_rad_s", 3.0)

        self.declare_parameter("ankle_pitch.amp_deg", 8.0)
        self.declare_parameter("ankle_pitch.max_frequency_hz", 3.0)
        self.declare_parameter("ankle_pitch.max_velocity_rad_s", 3.0)

        self.declare_parameter("ankle_roll.amp_deg", 5.0)
        self.declare_parameter("ankle_roll.max_frequency_hz", 4.0)
        self.declare_parameter("ankle_roll.max_velocity_rad_s", 2.0)

        self.declare_parameter("motor_state_timeout_sec", 0.2)
        self.declare_parameter("rsu_solution_timeout_sec", 0.2)
        self.declare_parameter("abort_on_rsu_infeasible", True)
        self.declare_parameter("motor_yaml_path", "")

        self.declare_parameter("upper_body_kp", 40.0)
        self.declare_parameter("upper_body_kd", 1.0)
        self.declare_parameter("default_motor_kp", 20.0)
        self.declare_parameter("default_motor_kd", 0.99)

        # ────────────── 파라미터 로드 ──────────────
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.initial_hold_sec = float(
            self.get_parameter("initial_hold_sec").value
        )
        self.center_hold_sec = float(
            self.get_parameter("center_hold_sec").value
        )
        self.chirp_duration_sec = float(
            self.get_parameter("chirp_duration_sec").value
        )
        self.min_frequency_hz = float(
            self.get_parameter("min_frequency_hz").value
        )
        self.physical_velocity_warn_scale = float(
            self.get_parameter("physical_velocity_warn_scale").value
        )
        self.return_speed_rad_s = float(
            self.get_parameter("return_speed_rad_s").value
        )
        self.position_tolerance_rad = float(
            self.get_parameter("position_tolerance_rad").value
        )

        self.pair_cfg = {
            "hip_pitch": {
                "amp_rad": math.radians(float(
                    self.get_parameter("hip_pitch.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "hip_pitch.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "hip_pitch.max_velocity_rad_s"
                    ).value
                ),
            },
            "hip_roll": {
                "amp_rad": math.radians(float(
                    self.get_parameter("hip_roll.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "hip_roll.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "hip_roll.max_velocity_rad_s"
                    ).value
                ),
            },
            "hip_yaw": {
                "amp_rad": math.radians(float(
                    self.get_parameter("hip_yaw.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "hip_yaw.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "hip_yaw.max_velocity_rad_s"
                    ).value
                ),
            },
            "knee_pitch": {
                "amp_rad": math.radians(float(
                    self.get_parameter("knee_pitch.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "knee_pitch.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "knee_pitch.max_velocity_rad_s"
                    ).value
                ),
            },
            "ankle_pitch": {
                "amp_rad": math.radians(float(
                    self.get_parameter("ankle_pitch.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "ankle_pitch.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "ankle_pitch.max_velocity_rad_s"
                    ).value
                ),
            },
            "ankle_roll": {
                "amp_rad": math.radians(float(
                    self.get_parameter("ankle_roll.amp_deg").value
                )),
                "max_frequency_hz": float(
                    self.get_parameter(
                        "ankle_roll.max_frequency_hz"
                    ).value
                ),
                "max_velocity_rad_s": float(
                    self.get_parameter(
                        "ankle_roll.max_velocity_rad_s"
                    ).value
                ),
            },
        }

        self.motor_state_timeout_sec = float(
            self.get_parameter("motor_state_timeout_sec").value
        )
        self.rsu_solution_timeout_sec = float(
            self.get_parameter("rsu_solution_timeout_sec").value
        )
        self.abort_on_rsu_infeasible = bool(
            self.get_parameter("abort_on_rsu_infeasible").value
        )
        self.motor_yaml_path = str(
            self.get_parameter("motor_yaml_path").value
        )

        self.upper_body_kp = float(
            self.get_parameter("upper_body_kp").value
        )
        self.upper_body_kd = float(
            self.get_parameter("upper_body_kd").value
        )
        self.default_motor_kp = float(
            self.get_parameter("default_motor_kp").value
        )
        self.default_motor_kd = float(
            self.get_parameter("default_motor_kd").value
        )

        # ────────────── 로드 결과 출력 ──────────────
        self.print_loaded_parameters()

    def print_loaded_parameters(self) -> None:
        self.get_logger().info(
            "\n"
            "========== SAMPLE GENERATOR PARAMETERS ==========\n"
            f"rate_hz                     : {self.rate_hz:.3f}\n"
            f"initial_hold_sec            : {self.initial_hold_sec:.3f}\n"
            f"center_hold_sec             : {self.center_hold_sec:.3f}\n"
            f"chirp_duration_sec          : {self.chirp_duration_sec:.3f}\n"
            f"min_frequency_hz            : {self.min_frequency_hz:.3f}\n"
            f"return_speed_rad_s          : {self.return_speed_rad_s:.3f}\n"
            f"position_tolerance_rad      : "
            f"{self.position_tolerance_rad:.8f}\n"
            f"velocity_warn_scale         : "
            f"{self.physical_velocity_warn_scale:.3f}\n"
            "\n"
            f"motor_state_timeout_sec     : "
            f"{self.motor_state_timeout_sec:.3f}\n"
            f"rsu_solution_timeout_sec    : "
            f"{self.rsu_solution_timeout_sec:.3f}\n"
            f"abort_on_rsu_infeasible     : "
            f"{self.abort_on_rsu_infeasible}\n"
            f"motor_yaml_path             : "
            f"{self.motor_yaml_path or '[auto search]'}\n"
            "\n"
            f"upper_body_kp               : {self.upper_body_kp:.3f}\n"
            f"upper_body_kd               : {self.upper_body_kd:.3f}\n"
            f"default_motor_kp            : {self.default_motor_kp:.3f}\n"
            f"default_motor_kd            : {self.default_motor_kd:.3f}\n"
            "================================================="
        )

        for pair_name, cfg in self.pair_cfg.items():
            self.get_logger().info(
                f"[PAIR PARAM] {pair_name:<13} | "
                f"amp={math.degrees(cfg['amp_rad']):.3f} deg | "
                f"max_frequency={cfg['max_frequency_hz']:.3f} Hz | "
                f"max_velocity={cfg['max_velocity_rad_s']:.3f} rad/s"
            )

    def _validate_parameters(self) -> None:
        if self.rate_hz <= 0.0:
            raise RuntimeError("rate_hz must be > 0")
        if self.chirp_duration_sec <= 0.0:
            raise RuntimeError("chirp_duration_sec must be > 0")
        if self.min_frequency_hz <= 0.0:
            raise RuntimeError("min_frequency_hz must be > 0")
        for pair_name, cfg in self.pair_cfg.items():
            if cfg["amp_rad"] <= 0.0:
                raise RuntimeError(f"{pair_name}.amp_deg must be > 0")
            if cfg["max_frequency_hz"] <= self.min_frequency_hz:
                raise RuntimeError(
                    f"{pair_name}.max_frequency_hz must be greater than "
                    f"min_frequency_hz={self.min_frequency_hz}"
                )
            if cfg["max_velocity_rad_s"] <= 0.0:
                raise RuntimeError(
                    f"{pair_name}.max_velocity_rad_s must be > 0"
                )
        if self.return_speed_rad_s <= 0.0:
            raise RuntimeError("return_speed_rad_s must be > 0")

    def load_motor_yaml(self) -> None:
        yaml_path = self.motor_yaml_path.strip()

        if not yaml_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory(
                    "robstride_hardware_interface"
                )
                yaml_path = os.path.join(
                    share_dir, "config", "motor_setting.yaml"
                )
            except Exception:
                yaml_path = os.path.expanduser(
                    "~/colcon_ws/src/roa_controller/ros2_pkg/"
                    "robstride_hardware_interface/config/motor_setting.yaml"
                )

        with open(yaml_path, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)

        params = config["hardware_interface_node"]["ros__parameters"]
        for can_name in params["can_interfaces"]:
            self.motor_ids.extend(
                int(value) for value in params[can_name]["motor_ids"]
            )
            self.motor_types.extend(
                str(value) for value in params[can_name]["motor_type"]
            )

        if len(self.motor_ids) != len(set(self.motor_ids)):
            raise RuntimeError(f"Duplicate motor IDs: {self.motor_ids}")

        self.get_logger().info(f"Loaded motor IDs: {self.motor_ids}")

    def build_joint_pairs(self):
        return [
            # {
            #     "name": "hip_pitch",
            #     "type": "coupled_hip",
            #     "axis": "pitch",
            #     "left_sign": +1.0,
            #     "right_sign": -1.0,
            #     "amp": self.pair_cfg["hip_pitch"]["amp_rad"],
            #     "max_frequency_hz": self.pair_cfg["hip_pitch"]["max_frequency_hz"],
            #     "max_velocity_rad_s": self.pair_cfg["hip_pitch"]["max_velocity_rad_s"],
            #     "directions": [+1.0, -1.0],
            # },
            # {
            #     "name": "hip_roll",
            #     "type": "coupled_hip",
            #     "axis": "roll",
            #     "left_sign": +1.0,
            #     "right_sign": -1.0,
            #     "amp": self.pair_cfg["hip_roll"]["amp_rad"],
            #     "max_frequency_hz": self.pair_cfg["hip_roll"]["max_frequency_hz"],
            #     "max_velocity_rad_s": self.pair_cfg["hip_roll"]["max_velocity_rad_s"],
            #     "directions": [+1.0],
            # },
            # {
            #     "name": "hip_yaw",
            #     "type": "motor_pair",
            #     "left_id": 14,
            #     "right_id": 15,
            #     "left_sign": +1.0,
            #     "right_sign": -1.0,
            #     "amp": self.pair_cfg["hip_yaw"]["amp_rad"],
            #     "max_frequency_hz": self.pair_cfg["hip_yaw"]["max_frequency_hz"],
            #     "max_velocity_rad_s": self.pair_cfg["hip_yaw"]["max_velocity_rad_s"],
            #     "directions": [+1.0, -1.0],
            # },
            {
                "name": "knee_pitch",
                "type": "motor_pair",
                "left_id": 16,
                "right_id": 17,
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.pair_cfg["knee_pitch"]["amp_rad"],
                "max_frequency_hz": self.pair_cfg["knee_pitch"]["max_frequency_hz"],
                "max_velocity_rad_s": self.pair_cfg["knee_pitch"]["max_velocity_rad_s"],
                "directions": [+1.0, -1.0],
            },
            {
                "name": "ankle_pitch",
                "type": "rsu_virtual",
                "axis": "pitch",
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.pair_cfg["ankle_pitch"]["amp_rad"],
                "max_frequency_hz": self.pair_cfg["ankle_pitch"]["max_frequency_hz"],
                "max_velocity_rad_s": self.pair_cfg["ankle_pitch"]["max_velocity_rad_s"],
                "directions": [+1.0, -1.0],
            },
            {
                "name": "ankle_roll",
                "type": "rsu_virtual",
                "axis": "roll",
                "left_sign": +1.0,
                "right_sign": -1.0,
                "amp": self.pair_cfg["ankle_roll"]["amp_rad"],
                "max_frequency_hz": self.pair_cfg["ankle_roll"]["max_frequency_hz"],
                "max_velocity_rad_s": self.pair_cfg["ankle_roll"]["max_velocity_rad_s"],
                "directions": [+1.0, -1.0],
            },
        ]

    @staticmethod
    def is_one_sided_pair(pair) -> bool:
        directions = {float(value) for value in pair["directions"]}
        return directions == {+1.0} or directions == {-1.0}

    @staticmethod
    def _extract_theta_pair(result) -> Tuple[float, float]:
        if hasattr(result, "theta1") and hasattr(result, "theta2"):
            return float(result.theta1), float(result.theta2)
        if isinstance(result, (tuple, list)) and len(result) == 2:
            return float(result[0]), float(result[1])
        raise TypeError(
            "Hip mapper result must expose theta1/theta2 or be length 2"
        )

    def get_hip_virtual_target_for_offset(
        self, pair, offset: float
    ) -> Tuple[float, float, float, float]:
        left_roll = 0.0
        left_pitch = 0.0
        right_roll = 0.0
        right_pitch = 0.0

        if pair is None or pair["type"] != "coupled_hip":
            return left_roll, left_pitch, right_roll, right_pitch

        left_value = float(pair["left_sign"]) * offset
        right_value = float(pair["right_sign"]) * offset

        if pair["axis"] == "pitch":
            left_pitch += left_value
            right_pitch -= right_value
        elif pair["axis"] == "roll":
            left_roll = left_value
            right_roll = right_value

        return left_roll, left_pitch, right_roll, right_pitch

    def _map_hip_pair_to_motor_vector(
        self, pair, offset: float
    ) -> Tuple[float, float, float, float]:
        left_roll, left_pitch, right_roll, right_pitch = (
            self.get_hip_virtual_target_for_offset(pair, offset)
        )

        left_result = map_hip_roll_pitch_to_motor_angles(
            roll=left_roll,
            pitch=left_pitch,
            is_left=True,
        )
        right_result = map_hip_roll_pitch_to_motor_angles(
            roll=right_roll,
            pitch=right_pitch,
            is_left=False,
        )

        left_theta1, left_theta2 = self._extract_theta_pair(left_result)
        right_theta1, right_theta2 = self._extract_theta_pair(right_result)

        return left_theta1, left_theta2, right_theta1, right_theta2

    def estimate_hip_mapper_velocity_gain(self, pair) -> float:
        if pair["type"] != "coupled_hip":
            return 1.0

        amp = float(pair["amp"])
        epsilon = 1.0e-5
        sample_count = 401

        if self.is_one_sided_pair(pair):
            q_values = [
                amp * index / (sample_count - 1)
                for index in range(sample_count)
            ]
        else:
            q_values = [
                -amp + 2.0 * amp * index / (sample_count - 1)
                for index in range(sample_count)
            ]

        max_gain = 1.0

        for q in q_values:
            q_minus = max(q - epsilon, -amp)
            q_plus = min(q + epsilon, amp)
            if q_plus <= q_minus:
                continue

            theta_minus = self._map_hip_pair_to_motor_vector(pair, q_minus)
            theta_plus = self._map_hip_pair_to_motor_vector(pair, q_plus)
            dq = q_plus - q_minus

            for a, b in zip(theta_minus, theta_plus):
                gain = abs((b - a) / dq)
                if math.isfinite(gain):
                    max_gain = max(max_gain, gain)

        return max_gain

    def configure_pair_frequencies(self) -> None:
        for pair in self.joint_pairs:
            amplitude = float(pair["amp"])
            mapper_gain = (
                self.estimate_hip_mapper_velocity_gain(pair)
                if pair["type"] == "coupled_hip"
                else 1.0
            )
            coefficient = (
                math.pi if self.is_one_sided_pair(pair) else 2.0 * math.pi
            )

            requested_frequency = float(pair["max_frequency_hz"])
            velocity_limit = float(pair["max_velocity_rad_s"])

            velocity_based_frequency = (
                velocity_limit
                / (coefficient * amplitude * mapper_gain)
            )

            # Pair마다 두 상한을 모두 관리한다.
            # 1) 사용자가 지정한 최대 주파수
            # 2) pair 최대 속도로부터 계산된 안전 최대 주파수
            # 실제 chirp에는 둘 중 작은 값을 사용한다.
            effective_frequency = min(
                requested_frequency,
                velocity_based_frequency,
            )

            if effective_frequency <= self.min_frequency_hz:
                raise RuntimeError(
                    f"{pair['name']}: effective max frequency "
                    f"{effective_frequency:.3f} Hz must exceed "
                    f"min_frequency_hz={self.min_frequency_hz:.3f} Hz"
                )

            estimated_peak_velocity = (
                coefficient
                * amplitude
                * effective_frequency
                * mapper_gain
            )

            pair["requested_max_frequency_hz"] = requested_frequency
            pair["velocity_based_max_frequency_hz"] = velocity_based_frequency
            pair["max_frequency_hz"] = effective_frequency
            pair["mapper_velocity_gain"] = mapper_gain
            pair["estimated_peak_velocity_rad_s"] = estimated_peak_velocity

    def on_motor_state(self, msg: MotorStateArray) -> None:
        for state in msg.states:
            motor_id = int(state.motor_id)
            position = float(state.position)
            velocity = float(state.velocity)
            if all_finite(position, velocity):
                self.latest_motor_pos[motor_id] = position
                self.latest_motor_vel[motor_id] = velocity
        self.last_motor_state_t = time.monotonic()

    def on_rsu_imp_solution(self, msg: RsuImpSol) -> None:
        if not bool(msg.feasible):
            self.rsu_solution_valid = False
            self.get_logger().error(
                "Received infeasible /rsu/imp_solution.",
                throttle_duration_sec=0.5,
            )
            if (
                self.abort_on_rsu_infeasible
                and self.phase not in (
                    SamplePhase.DONE,
                    SamplePhase.FAULT,
                )
            ):
                self.enter_fault("RSU impedance solution became infeasible")
            return

        commands = {
            18: RsuActuatorCommand(
                float(msg.left_actuator_1.q_target),
                float(msg.left_actuator_1.kp_eqv),
                float(msg.left_actuator_1.kd_eqv),
            ),
            20: RsuActuatorCommand(
                float(msg.left_actuator_2.q_target),
                float(msg.left_actuator_2.kp_eqv),
                float(msg.left_actuator_2.kd_eqv),
            ),
            19: RsuActuatorCommand(
                float(msg.right_actuator_1.q_target),
                float(msg.right_actuator_1.kp_eqv),
                float(msg.right_actuator_1.kd_eqv),
            ),
            21: RsuActuatorCommand(
                float(msg.right_actuator_2.q_target),
                float(msg.right_actuator_2.kp_eqv),
                float(msg.right_actuator_2.kd_eqv),
            ),
        }

        for motor_id, command in commands.items():
            if not all_finite(command.position, command.kp, command.kd):
                self.rsu_solution_valid = False
                self.get_logger().error(
                    f"Non-finite RSU command for ID {motor_id}"
                )
                return

        self.rsu_actuator_cmd.update(commands)
        self.rsu_solution_valid = True
        self.last_rsu_solution_t = time.monotonic()

    def motor_state_ready(self) -> bool:
        return set(self.motor_ids).issubset(self.latest_motor_pos.keys())

    def motor_state_fresh(self) -> bool:
        if self.last_motor_state_t is None:
            return False
        return (
            time.monotonic() - self.last_motor_state_t
            <= self.motor_state_timeout_sec
        )

    def rsu_solution_ready(self) -> bool:
        configured = self.RSU_MOTOR_IDS.intersection(self.motor_ids)
        if not configured:
            return True
        if not self.rsu_solution_valid or self.last_rsu_solution_t is None:
            return False
        if any(self.rsu_actuator_cmd[motor_id] is None for motor_id in configured):
            return False
        return (
            time.monotonic() - self.last_rsu_solution_t
            <= self.rsu_solution_timeout_sec
        )

    def system_ready(self) -> bool:
        return (
            self.motor_state_ready()
            and self.motor_state_fresh()
            and self.rsu_solution_ready()
        )

    def enter_fault(self, reason: str) -> None:
        if self.phase == SamplePhase.FAULT:
            return
        self.phase = SamplePhase.FAULT
        self.phase_start_t = time.monotonic()
        self.offset = 0.0
        self.fault_reason = reason
        self.get_logger().fatal(f"[FAULT] {reason}")

    def current_pair(self):
        if self.pair_idx >= len(self.joint_pairs):
            return None
        return self.joint_pairs[self.pair_idx]

    def phase_elapsed(self) -> float:
        return time.monotonic() - self.phase_start_t

    def set_phase(self, new_phase: SamplePhase) -> None:
        self.phase = new_phase
        self.phase_start_t = time.monotonic()
        pair = self.current_pair()
        if pair is None:
            self.get_logger().warn(f"[PHASE] {new_phase.name}")
        else:
            self.get_logger().warn(
                f"[PHASE] {new_phase.name} | "
                f"pair={pair['name']} | "
                f"f=[{self.min_frequency_hz:.4f}, "
                f"{pair['max_frequency_hz']:.4f}] Hz | "
                f"offset={self.offset:+.6f}"
            )

    def compute_chirp_offset(self, pair, elapsed: float) -> float:
        t = clamp(elapsed, 0.0, self.chirp_duration_sec)
        f0 = self.min_frequency_hz
        f1 = float(pair["max_frequency_hz"])
        sweep_rate = (f1 - f0) / self.chirp_duration_sec

        phase = 2.0 * math.pi * (
            f0 * t + 0.5 * sweep_rate * t * t
        )
        amplitude = float(pair["amp"])

        if self.is_one_sided_pair(pair):
            positive = 0.5 * amplitude * (1.0 - math.cos(phase))
            directions = {float(v) for v in pair["directions"]}
            return -positive if directions == {-1.0} else positive

        return amplitude * math.sin(phase)

    def update_sample_fsm(self, dt: float) -> None:
        pair = self.current_pair()

        if self.phase == SamplePhase.FAULT:
            self.offset = 0.0
            return

        if self.phase == SamplePhase.INITIAL_HOLD:
            # 노드 시작 직후부터 5초 동안 0 target을 발행한다.
            # feedback/RSU readiness를 기다리지 않고 시간이 끝나면 샘플 루프로 진입한다.
            self.offset = 0.0
            if self.phase_elapsed() >= self.initial_hold_sec:
                self.set_phase(SamplePhase.CHIRP)
            return

        if self.phase == SamplePhase.DONE:
            self.offset = 0.0
            if not self.done_logged:
                self.get_logger().warn(
                    "[DONE] All pair chirps completed. Holding zero pose."
                )
                self.done_logged = True
            return

        if pair is None:
            self.set_phase(SamplePhase.DONE)
            return

        if self.phase == SamplePhase.CHIRP:
            elapsed = self.phase_elapsed()

            if elapsed >= self.chirp_duration_sec:
                # 마지막 처프 target을 유지하면서 복귀 단계 진입
                self.set_phase(SamplePhase.RETURN_CENTER)
                return

            self.offset = self.compute_chirp_offset(pair, elapsed)
            return

        if self.phase == SamplePhase.RETURN_CENTER:
            return_speed = min(
                self.return_speed_rad_s,
                float(pair["max_velocity_rad_s"]),
            )

            self.offset = move_toward(
                current=self.offset,
                target=0.0,
                max_delta=return_speed * dt,
            )

            if abs(self.offset) <= self.position_tolerance_rad:
                self.offset = 0.0
                self.set_phase(SamplePhase.HOLD_CENTER)

            return

        if self.phase == SamplePhase.HOLD_CENTER:
            self.offset = 0.0
            if self.phase_elapsed() >= self.center_hold_sec:
                self.pair_idx += 1
                if self.pair_idx >= len(self.joint_pairs):
                    self.set_phase(SamplePhase.DONE)
                else:
                    self.offset = 0.0
                    self.set_phase(SamplePhase.CHIRP)
            return

    def get_hip_motor_targets(self) -> Dict[int, float]:
        pair = self.current_pair()
        active_pair = (
            pair if pair is not None and pair["type"] == "coupled_hip" else None
        )
        values = self._map_hip_pair_to_motor_vector(
            active_pair,
            self.offset if active_pair is not None else 0.0,
        )
        return {
            10: values[0],
            12: values[1],
            11: values[2],
            13: values[3],
        }

    def get_non_rsu_position(self, motor_id: int) -> float:
        if motor_id in self.UPPER_BODY_IDS or motor_id == self.TORSO_YAW_ID:
            return 0.0

        hip_targets = self.get_hip_motor_targets()
        if motor_id in hip_targets:
            return float(hip_targets[motor_id])

        pair = self.current_pair()
        if pair is None or pair["type"] != "motor_pair":
            return 0.0

        if motor_id == int(pair["left_id"]):
            return float(pair["left_sign"]) * self.offset
        if motor_id == int(pair["right_id"]):
            return float(pair["right_sign"]) * self.offset
        return 0.0

    def get_rsu_virtual_target(self) -> Tuple[float, float, float, float]:
        left_roll = left_pitch = right_roll = right_pitch = 0.0
        pair = self.current_pair()

        if pair is None or pair["type"] != "rsu_virtual":
            return left_roll, left_pitch, right_roll, right_pitch

        left_value = float(pair["left_sign"]) * self.offset
        right_value = float(pair["right_sign"]) * self.offset

        if pair["axis"] == "pitch":
            left_pitch = left_value
            right_pitch = right_value
        elif pair["axis"] == "roll":
            left_roll = left_value
            right_roll = right_value

        return left_roll, left_pitch, right_roll, right_pitch

    def publish_rsu_target(self) -> None:
        left_roll, left_pitch, right_roll, right_pitch = (
            self.get_rsu_virtual_target()
        )

        msg = RsuTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "symmetric_chirp_sample_generator"
        msg.seq = int(self.rsu_target_seq)
        self.rsu_target_seq += 1
        msg.left_roll = float(left_roll)
        msg.left_pitch = float(left_pitch)
        msg.right_roll = float(right_roll)
        msg.right_pitch = float(right_pitch)
        self.pub_rsu_target.publish(msg)

    def get_motor_command(self, motor_id: int) -> Optional[MotorCommand]:
        if motor_id in self.RSU_MOTOR_IDS:
            command = self.rsu_actuator_cmd.get(motor_id)
            if command is not None:
                return MotorCommand(
                    motor_id=motor_id,
                    torque=0.0,
                    position=float(command.position),
                    velocity=0.0,
                    kp=float(command.kp),
                    kd=float(command.kd),
                )

            # RT impedance solution이 아직 없더라도 command frame을 막지 않는다.
            # 초기 5초 hold와 RViz/offline trajectory 확인을 위해 zero-effort bootstrap을 보낸다.
            return MotorCommand(
                motor_id=motor_id,
                torque=0.0,
                position=float(self.latest_motor_pos.get(motor_id, 0.0)),
                velocity=0.0,
                kp=0.0,
                kd=0.0,
            )

        position = self.get_non_rsu_position(motor_id)
        if motor_id in self.UPPER_BODY_IDS:
            kp = self.upper_body_kp
            kd = self.upper_body_kd
        else:
            kp = self.kp_map.get(motor_id, self.default_motor_kp)
            kd = self.kd_map.get(motor_id, self.default_motor_kd)

        if not all_finite(position, kp, kd):
            return None

        return MotorCommand(
            motor_id=motor_id,
            torque=0.0,
            position=float(position),
            velocity=0.0,
            kp=float(kp),
            kd=float(kd),
        )

    def monitor_final_command_velocity(self, commands: List[MotorCommand]) -> None:
        now = time.monotonic()
        if self.previous_final_position_time is None:
            self.previous_final_position_time = now
            self.previous_final_position = {
                int(command.motor_id): float(command.position)
                for command in commands
            }
            return

        dt = now - self.previous_final_position_time
        if dt <= 0.0:
            return

        for command in commands:
            motor_id = int(command.motor_id)
            position = float(command.position)
            previous = self.previous_final_position.get(motor_id)
            if previous is None:
                continue
            velocity = abs(position - previous) / dt
            pair = self.current_pair()
            pair_limit = (
                float(pair["max_velocity_rad_s"])
                if pair is not None
                else self.return_speed_rad_s
            )
            warn_limit = pair_limit * self.physical_velocity_warn_scale

            if velocity > warn_limit:
                self.get_logger().warn(
                    f"Final actuator target velocity high: ID={motor_id}, "
                    f"velocity={velocity:.3f} rad/s, "
                    f"pair_limit={pair_limit:.3f} rad/s",
                    throttle_duration_sec=0.5,
                )

        self.previous_final_position_time = now
        self.previous_final_position = {
            int(command.motor_id): float(command.position)
            for command in commands
        }

    def publish_motor_command(self) -> None:
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "SYMMETRIC_CHIRP_SAMPLE_GENERATOR"

        commands: List[MotorCommand] = []
        for motor_id_raw in self.motor_ids:
            motor_id = int(motor_id_raw)
            command = self.get_motor_command(motor_id)
            if command is None:
                self.get_logger().warn(
                    f"Command not ready for motor ID {motor_id}.",
                    throttle_duration_sec=0.5,
                )
                return
            commands.append(command)

        self.monitor_final_command_velocity(commands)
        msg.commands.extend(commands)
        self.pub_motor_cmd.publish(msg)

    def estimate_total_time(self) -> float:
        total = self.initial_hold_sec
        for pair in self.joint_pairs:
            total += self.chirp_duration_sec
            total += float(pair["amp"]) / self.return_speed_rad_s
            total += self.center_hold_sec
        return total

    def print_test_plan(self) -> None:
        self.get_logger().warn("========== CHIRP SAMPLE TEST PLAN ==========")
        self.get_logger().warn(
            f"initial_hold_sec       : {self.initial_hold_sec:.3f}"
        )
        self.get_logger().warn(
            f"chirp_duration_sec     : {self.chirp_duration_sec:.3f}"
        )
        self.get_logger().warn(
            f"estimated total time   : {self.estimate_total_time():.1f} sec"
        )

        for pair in self.joint_pairs:
            self.get_logger().warn(
                f"pair={pair['name']:<13} "
                f"amp={math.degrees(pair['amp']):>6.2f} deg | "
                f"directions={pair['directions']} | "
                f"mapper_gain={pair['mapper_velocity_gain']:.4f} | "
                f"f_req={pair['requested_max_frequency_hz']:.4f} Hz | "
                f"f_vel={pair['velocity_based_max_frequency_hz']:.4f} Hz | "
                f"f_use={pair['max_frequency_hz']:.4f} Hz | "
                f"v_limit={pair['max_velocity_rad_s']:.4f} rad/s | "
                f"v_peak={pair['estimated_peak_velocity_rad_s']:.4f} rad/s"
            )
        self.get_logger().warn("============================================")

    def log_status(self) -> None:
        pair = self.current_pair()
        pair_name = pair["name"] if pair is not None else "none"
        f_max = float(pair["max_frequency_hz"]) if pair is not None else 0.0
        amplitude = float(pair["amp"]) if pair is not None else 0.0

        self.get_logger().info(
            f"[SAMPLE] phase={self.phase.name:<14} "
            f"pair={pair_name:<13} "
            f"f_max={f_max:.4f}Hz "
            f"v_limit={float(pair['max_velocity_rad_s']) if pair is not None else 0.0:.3f}rad/s "
            f"amp={math.degrees(amplitude):.2f}deg "
            f"offset={self.offset:+.6f}rad "
            f"motor_ready={self.motor_state_ready()} "
            f"rsu_ready={self.rsu_solution_ready()}",
            throttle_duration_sec=0.5,
        )

    def on_timer(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self.last_t, 0.0, 0.05)
        self.last_t = now

        # 시작 직후부터 zero virtual ankle target을 지속 발행한다.
        self.publish_rsu_target()
        self.update_sample_fsm(dt)
        self.publish_motor_command()
        self.log_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SymmetricChirpSampleGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()