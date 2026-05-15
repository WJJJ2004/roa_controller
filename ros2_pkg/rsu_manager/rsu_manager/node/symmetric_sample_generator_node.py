#!/usr/bin/env python3

import math
import os
import time
import yaml
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from roa_interfaces.msg import (
    RsuTarget,
    RsuSolution,
    MotorCommand,
    MotorCommandArray,
    MotorStateArray,
)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def move_toward(current: float, target: float, max_delta: float):
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return current


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
    MOVE_TO_BOUNDARY = 1
    HOLD_BOUNDARY = 2
    RETURN_CENTER = 3
    HOLD_CENTER = 4
    DONE = 5


class SymmetricSampleGeneratorNode(Node):
    def __init__(self):
        super().__init__("symmetric_sample_generator_node")

        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)

        self.initial_hold_sec = float(
            self.declare_parameter("initial_hold_sec", 10.0).value
        )
        self.boundary_hold_sec = float(
            self.declare_parameter("boundary_hold_sec", 0.5).value
        )
        self.center_hold_sec = float(
            self.declare_parameter("center_hold_sec", 0.5).value
        )

        self.speed_levels_rad_s = list(
            self.declare_parameter(
                "speed_levels_rad_s",
                [0.3, 0.6, 1.0],
            ).value
        )

        self.default_amp_rad = math.radians(
            float(self.declare_parameter("default_amp_deg", 15.0).value)
        )
        self.ankle_pitch_amp_rad = math.radians(
            float(self.declare_parameter("ankle_pitch_amp_deg", 12.0).value)
        )
        self.ankle_roll_amp_rad = math.radians(
            float(self.declare_parameter("ankle_roll_amp_deg", 10.0).value)
        )

        self.device_path = str(self.declare_parameter("device_path", "").value)

        self.pub_rsu_target = self.create_publisher(
            RsuTarget,
            "/rsu/target",
            rsu_qos,
        )

        self.pub_motor_cmd = self.create_publisher(
            MotorCommandArray,
            "/hardware_interface/command",
            cmd_qos,
        )

        self.sub_rsu_solution = self.create_subscription(
            RsuSolution,
            "/rsu/solution",
            self.on_rsu_solution,
            rsu_qos,
        )

        self.sub_motor_state = self.create_subscription(
            MotorStateArray,
            "/hardware_interface/state",
            self.on_motor_state,
            state_qos,
        )

        self.motor_ids = []
        self.motor_types = []
        self.load_motor_yaml()

        self.latest_motor_pos = {}

        self.actuator_cmd = {
            18: None,
            20: None,
            19: None,
            21: None,
        }

        self.kp_map = {
            9: 50.0,
            10: 150.0,
            12: 200.0,
            14: 100.0,
            16: 150.0,
            11: 150.0,
            13: 200.0,
            15: 100.0,
            17: 150.0,
        }

        self.kd_map = {
            9: 2.0,
            10: 24.722,
            12: 26.387,
            14: 3.419,
            16: 8.654,
            11: 24.722,
            13: 26.387,
            15: 3.419,
            17: 8.654,
        }

        self.default_rsu_kp = float(
            self.declare_parameter("default_rsu_kp", 40.0).value
        )
        self.default_rsu_kd = float(
            self.declare_parameter("default_rsu_kd", 0.99).value
        )

        self.joint_pairs = self.build_joint_pairs()

        self.phase = SamplePhase.INITIAL_HOLD
        self.phase_start_t = time.time()
        self.last_t = time.time()

        self.pair_idx = 0
        self.speed_idx = 0
        self.direction_idx = 0
        self.offset = 0.0
        self.done_logged = False

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().warn(
            "Symmetric sample generator started. "
            "Base target is ZERO pose. "
            "All pair signs use left=+offset, right=-offset. "
            "hip_roll skips left-negative/right-positive direction."
        )

        self.print_test_plan()

    def build_joint_pairs(self):
        return [
            {
                "name": "hip_pitch",
                "type": "motor_pair",
                "left_id": 10,
                "right_id": 11,
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.default_amp_rad,
                "directions": [+1.0, -1.0],
            },
            {
                "name": "hip_roll",
                "type": "motor_pair",
                "left_id": 12,
                "right_id": 13,
                "left_sign": +1.0,
                "right_sign": -1.0,
                "amp": self.default_amp_rad,
                # left=-offset, right=+offset 조합 금지
                # 따라서 direction=-1.0은 사용하지 않음
                "directions": [+1.0],
            },
            {
                "name": "hip_yaw",
                "type": "motor_pair",
                "left_id": 14,
                "right_id": 15,
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.default_amp_rad,
                "directions": [+1.0, -1.0],
            },
            {
                "name": "knee_pitch",
                "type": "motor_pair",
                "left_id": 16,
                "right_id": 17,
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.default_amp_rad,
                "directions": [+1.0, -1.0],
            },
            {
                "name": "ankle_pitch",
                "type": "rsu_virtual",
                "axis": "pitch",
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.ankle_pitch_amp_rad,
                "directions": [+1.0, -1.0],
            },
            {
                "name": "ankle_roll",
                "type": "rsu_virtual",
                "axis": "roll",
                "left_sign": +1.0,
                "right_sign": +1.0,
                "amp": self.ankle_roll_amp_rad,
                "directions": [+1.0, -1.0],
            },
        ]

    def print_test_plan(self):
        total_sec = self.estimate_total_time()

        self.get_logger().warn("========== SAMPLE TEST PLAN ==========")
        self.get_logger().warn(f"initial_hold_sec  : {self.initial_hold_sec:.2f}")
        self.get_logger().warn(f"boundary_hold_sec : {self.boundary_hold_sec:.2f}")
        self.get_logger().warn(f"center_hold_sec   : {self.center_hold_sec:.2f}")
        self.get_logger().warn(f"speed_levels      : {self.speed_levels_rad_s}")
        self.get_logger().warn(f"estimated time    : {total_sec:.1f} sec")

        for pair in self.joint_pairs:
            amp_deg = math.degrees(pair["amp"])
            self.get_logger().warn(
                f"pair={pair['name']:<12} "
                f"amp={amp_deg:.2f} deg "
                f"directions={pair['directions']}"
            )

        self.get_logger().warn("======================================")

    def estimate_total_time(self):
        total = self.initial_hold_sec

        for pair in self.joint_pairs:
            amp = float(pair["amp"])
            num_dir = len(pair["directions"])

            for speed in self.speed_levels_rad_s:
                move_time_one_direction = amp / max(float(speed), 1e-6)

                # 0 -> boundary, boundary -> 0
                # plus boundary hold and center hold
                one_dir_time = (
                    2.0 * move_time_one_direction
                    + self.boundary_hold_sec
                    + self.center_hold_sec
                )

                total += num_dir * one_dir_time

        return total

    def load_motor_yaml(self, yaml_path=None):
        if yaml_path is None:
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory("robstride_hardware_interface")
                yaml_path = os.path.join(share_dir, "config", "motor_setting.yaml")
            except Exception:
                yaml_path = os.path.expanduser(
                    "~/colcon_ws/src/robstride_hardware_interface/config/motor_setting.yaml"
                )

        with open(yaml_path, "r") as f:
            config = yaml.safe_load(f)

        params = config["hardware_interface_node"]["ros__parameters"]
        can_interfaces = params["can_interfaces"]

        for can_name in can_interfaces:
            self.motor_ids += params[can_name]["motor_ids"]
            self.motor_types += params[can_name]["motor_type"]

        self.get_logger().info(f"Loaded motor ids: {self.motor_ids}")

    def on_motor_state(self, msg: MotorStateArray):
        for s in msg.states:
            self.latest_motor_pos[int(s.motor_id)] = float(s.position)

    def on_rsu_solution(self, msg: RsuSolution):
        if not msg.feasible:
            self.get_logger().warn(
                "RSU solution infeasible. Holding previous actuator commands.",
                throttle_duration_sec=1.0,
            )
            return

        self.actuator_cmd[18] = float(msg.left_actuator_1)
        self.actuator_cmd[20] = float(msg.left_actuator_2)
        self.actuator_cmd[19] = float(msg.right_actuator_1)
        self.actuator_cmd[21] = float(msg.right_actuator_2)

    def current_pair(self):
        if self.pair_idx >= len(self.joint_pairs):
            return None
        return self.joint_pairs[self.pair_idx]

    def current_speed(self):
        return float(self.speed_levels_rad_s[self.speed_idx])

    def current_direction(self):
        pair = self.current_pair()
        return float(pair["directions"][self.direction_idx])

    def phase_elapsed(self):
        return time.time() - self.phase_start_t

    def set_phase(self, new_phase: SamplePhase):
        self.phase = new_phase
        self.phase_start_t = time.time()

        pair = self.current_pair()
        if pair is not None:
            self.get_logger().warn(
                f"[PHASE] {self.phase.name} | "
                f"pair={pair['name']} | "
                f"speed={self.current_speed():.3f} rad/s | "
                f"dir={self.current_direction():+.0f} | "
                f"offset={self.offset:+.5f}"
            )

    def advance_sequence(self):
        pair = self.current_pair()
        if pair is None:
            self.set_phase(SamplePhase.DONE)
            return

        self.direction_idx += 1
        if self.direction_idx < len(pair["directions"]):
            self.offset = 0.0
            self.set_phase(SamplePhase.MOVE_TO_BOUNDARY)
            return

        self.direction_idx = 0
        self.speed_idx += 1
        if self.speed_idx < len(self.speed_levels_rad_s):
            self.offset = 0.0
            self.set_phase(SamplePhase.MOVE_TO_BOUNDARY)
            return

        self.speed_idx = 0
        self.pair_idx += 1
        if self.pair_idx < len(self.joint_pairs):
            self.offset = 0.0
            self.set_phase(SamplePhase.MOVE_TO_BOUNDARY)
            return

        self.offset = 0.0
        self.set_phase(SamplePhase.DONE)

    def update_sample_fsm(self, dt):
        pair = self.current_pair()

        if self.phase == SamplePhase.INITIAL_HOLD:
            self.offset = 0.0

            if self.phase_elapsed() >= self.initial_hold_sec:
                self.set_phase(SamplePhase.MOVE_TO_BOUNDARY)

            return

        if self.phase == SamplePhase.DONE:
            self.offset = 0.0

            if not self.done_logged:
                self.get_logger().warn(
                    "[DONE] Sample trajectory finished. Holding zero pose."
                )
                self.done_logged = True

            return

        if pair is None:
            self.set_phase(SamplePhase.DONE)
            return

        amp = float(pair["amp"])
        speed = self.current_speed()
        direction = self.current_direction()
        boundary = direction * amp
        max_delta = speed * dt

        if self.phase == SamplePhase.MOVE_TO_BOUNDARY:
            self.offset = move_toward(self.offset, boundary, max_delta)

            if abs(self.offset - boundary) < 1e-6:
                self.offset = boundary
                self.set_phase(SamplePhase.HOLD_BOUNDARY)

            return

        if self.phase == SamplePhase.HOLD_BOUNDARY:
            self.offset = boundary

            if self.phase_elapsed() >= self.boundary_hold_sec:
                self.set_phase(SamplePhase.RETURN_CENTER)

            return

        if self.phase == SamplePhase.RETURN_CENTER:
            self.offset = move_toward(self.offset, 0.0, max_delta)

            if abs(self.offset) < 1e-6:
                self.offset = 0.0
                self.set_phase(SamplePhase.HOLD_CENTER)

            return

        if self.phase == SamplePhase.HOLD_CENTER:
            self.offset = 0.0

            if self.phase_elapsed() >= self.center_hold_sec:
                self.advance_sequence()

            return

    def get_non_rsu_position(self, motor_id: int):
        # zero pose 기준
        pos = 0.0

        pair = self.current_pair()
        if pair is None:
            return pos

        if pair["type"] != "motor_pair":
            return pos

        if motor_id == int(pair["left_id"]):
            pos = float(pair["left_sign"]) * self.offset

        elif motor_id == int(pair["right_id"]):
            pos = float(pair["right_sign"]) * self.offset

        return pos

    def get_rsu_virtual_target(self):
        left_roll = 0.0
        right_roll = 0.0
        left_pitch = 0.0
        right_pitch = 0.0

        pair = self.current_pair()
        if pair is None:
            return left_roll, left_pitch, right_roll, right_pitch

        if pair["type"] != "rsu_virtual":
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

    def get_command_position(self, motor_id: int):
        if motor_id in self.actuator_cmd:
            if self.actuator_cmd[motor_id] is not None:
                return self.actuator_cmd[motor_id]

            if motor_id in self.latest_motor_pos:
                return self.latest_motor_pos[motor_id]

            return None

        return self.get_non_rsu_position(motor_id)

    def publish_rsu_target(self):
        left_roll, left_pitch, right_roll, right_pitch = self.get_rsu_virtual_target()

        msg = RsuTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "symmetric_sample_generator"
        msg.seq = 0

        msg.left_roll = float(left_roll)
        msg.left_pitch = float(left_pitch)
        msg.right_roll = float(right_roll)
        msg.right_pitch = float(right_pitch)

        self.pub_rsu_target.publish(msg)

    def publish_motor_command(self):
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "SYMMETRIC_SAMPLE_GENERATOR_ZERO_BASE"

        for motor_id_raw in self.motor_ids:
            motor_id = int(motor_id_raw)
            pos = self.get_command_position(motor_id)

            if pos is None:
                self.get_logger().warn(
                    "RSU actuator command is not ready and no encoder hold exists. "
                    "Skipping command publish.",
                    throttle_duration_sec=1.0,
                )
                return

            is_rsu = motor_id in [18, 20, 19, 21]

            msg.commands.append(
                MotorCommand(
                    motor_id=motor_id,
                    torque=0.0,
                    position=float(pos),
                    velocity=0.0,
                    kp=float(self.default_rsu_kp if is_rsu else self.kp_map.get(motor_id, 20.0)),
                    kd=float(self.default_rsu_kd if is_rsu else self.kd_map.get(motor_id, 0.99)),
                )
            )

        self.pub_motor_cmd.publish(msg)

    def log_status(self):
        pair = self.current_pair()

        if pair is None:
            pair_name = "none"
            speed = 0.0
            direction = 0.0
            amp = 0.0
        else:
            pair_name = pair["name"]
            speed = self.current_speed()
            direction = self.current_direction()
            amp = float(pair["amp"])

        self.get_logger().info(
            f"[SAMPLE] phase={self.phase.name:<16} "
            f"pair={pair_name:<12} "
            f"speed={speed:.3f} "
            f"dir={direction:+.0f} "
            f"amp={math.degrees(amp):.2f}deg "
            f"offset={self.offset:+.5f}rad "
            f"offset_deg={math.degrees(self.offset):+.2f}",
            throttle_duration_sec=0.5,
        )

    def on_timer(self):
        now = time.time()
        dt = clamp(now - self.last_t, 0.0, 0.05)
        self.last_t = now

        self.update_sample_fsm(dt)

        self.publish_rsu_target()
        self.publish_motor_command()

        self.log_status()


def main(args=None):
    rclpy.init(args=args)
    node = SymmetricSampleGeneratorNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()