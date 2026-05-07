#!/usr/bin/env python3

import math
import os
import time
import yaml

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

from rsu_manager.util.gamepad_reader import Gamepad


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


HIP_INIT_POS = 0.418879
KNEE_INIT_POS = 0.698131
ANKLE_INIT_POS = 0.458105


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


class InitPoseTuningNode(Node):
    def __init__(self):
        super().__init__("init_pose_tuning_node")

        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)
        self.input_deadzone = float(self.declare_parameter("input_deadzone", 0.08).value)

        # 실기용 저속 튜닝
        self.tune_rate_rad_s = float(self.declare_parameter("tune_rate_rad_s", 0.03).value)

        self.hip_offset_limit = float(self.declare_parameter("hip_offset_limit_rad", 0.20).value)
        self.knee_offset_limit = float(self.declare_parameter("knee_offset_limit_rad", 0.20).value)
        self.ankle_offset_limit = float(self.declare_parameter("ankle_offset_limit_rad", 0.15).value)

        self.device_path = str(self.declare_parameter("device_path", "").value)
        self.vendor_id = int(self.declare_parameter("vendor_id", 0x046D).value)
        self.product_id = int(self.declare_parameter("product_id", 0xC219).value)

        self.gamepad = Gamepad(
            vendor_id=self.vendor_id,
            product_id=self.product_id,
            vel_scale_x=1.0,
            vel_scale_y=1.0,
            vel_scale_rot=1.0,
            device_path=(self.device_path if self.device_path else None),
            prefer_name_contains="Logitech",
        )

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

        # B 버튼으로 선택되는 튜닝 대상
        self.group_order = ["hip", "knee", "ankle"]
        self.selected_group = "hip"
        self.prev_b = False

        # 튜닝 offset
        self.hip_offset = 0.0
        self.knee_offset = 0.0
        self.ankle_offset = 0.0

        # RSU virtual target
        self.left_roll = 0.0
        self.right_roll = 0.0
        self.left_pitch = -ANKLE_INIT_POS
        self.right_pitch = ANKLE_INIT_POS

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

        self.default_rsu_kp = 40.0
        self.default_rsu_kd = 0.99

        self.last_t = time.time()

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "Init pose tuning node started. "
            "B: select hip/knee/ankle, Stick Y: tune selected pitch"
        )

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

    def handle_b_button(self, b_pressed: bool):
        if b_pressed and not self.prev_b:
            idx = self.group_order.index(self.selected_group)
            self.selected_group = self.group_order[(idx + 1) % len(self.group_order)]

            self.get_logger().warn(
                f"[SELECTED GROUP] {self.selected_group.upper()} | "
                f"hip_offset={self.hip_offset:+.5f}, "
                f"knee_offset={self.knee_offset:+.5f}, "
                f"ankle_offset={self.ankle_offset:+.5f}"
            )

        self.prev_b = b_pressed

    def update_tuning_from_gamepad(self, cmd, dt):
        axis_y = float(cmd[0]) if len(cmd) > 0 else 0.0

        if abs(axis_y) < self.input_deadzone:
            return

        delta = axis_y * self.tune_rate_rad_s * dt

        if self.selected_group == "hip":
            self.hip_offset = clamp(
                self.hip_offset + delta,
                -self.hip_offset_limit,
                self.hip_offset_limit,
            )

        elif self.selected_group == "knee":
            self.knee_offset = clamp(
                self.knee_offset + delta,
                -self.knee_offset_limit,
                self.knee_offset_limit,
            )

        elif self.selected_group == "ankle":
            self.ankle_offset = clamp(
                self.ankle_offset + delta,
                -self.ankle_offset_limit,
                self.ankle_offset_limit,
            )

        self.update_rsu_virtual_target()

        self.get_logger().info(
            f"[TUNING] {self.selected_group.upper()} | "
            f"hip={self.hip_offset:+.5f}, "
            f"knee={self.knee_offset:+.5f}, "
            f"ankle={self.ankle_offset:+.5f}",
            throttle_duration_sec=0.3,
        )

    def update_rsu_virtual_target(self):
        self.left_roll = 0.0
        self.right_roll = 0.0

        self.left_pitch = -ANKLE_INIT_POS + self.ankle_offset
        self.right_pitch = ANKLE_INIT_POS - self.ankle_offset

    def get_non_rsu_position(self, motor_id: int):
        if motor_id == 9:
            return 0.0

        # left leg
        if motor_id == 10:
            return -HIP_INIT_POS + self.hip_offset
        if motor_id == 12:
            return 0.0
        if motor_id == 14:
            return 0.0
        if motor_id == 16:
            return KNEE_INIT_POS + self.knee_offset

        # right leg
        if motor_id == 11:
            return HIP_INIT_POS - self.hip_offset
        if motor_id == 13:
            return 0.0
        if motor_id == 15:
            return 0.0
        if motor_id == 17:
            return -KNEE_INIT_POS - self.knee_offset

        return 0.0

    def get_command_position(self, motor_id: int):
        if motor_id in self.actuator_cmd:
            if self.actuator_cmd[motor_id] is not None:
                return self.actuator_cmd[motor_id]

            if motor_id in self.latest_motor_pos:
                return self.latest_motor_pos[motor_id]

            return None

        return self.get_non_rsu_position(motor_id)

    def publish_rsu_target(self):
        msg = RsuTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "init_pose_tuning"
        msg.seq = 0

        msg.left_roll = float(self.left_roll)
        msg.left_pitch = float(self.left_pitch)
        msg.right_roll = float(self.right_roll)
        msg.right_pitch = float(self.right_pitch)

        self.pub_rsu_target.publish(msg)

    def publish_motor_command(self):
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "INIT_POSE_TUNING"

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

    def on_timer(self):
        if not self.gamepad.is_running:
            self.get_logger().error(
                "Gamepad not running. Holding current command.",
                throttle_duration_sec=2.0,
            )
            self.publish_rsu_target()
            self.publish_motor_command()
            return

        now = time.time()
        dt = clamp(now - self.last_t, 0.0, 0.05)
        self.last_t = now

        cmd = self.gamepad.get_command()

        # 기존 발목 테스트 코드 기준 cmd[3]을 B 버튼으로 사용
        b_pressed = bool(cmd[3]) if len(cmd) > 3 else False

        self.handle_b_button(b_pressed)
        self.update_tuning_from_gamepad(cmd, dt)

        self.publish_rsu_target()
        self.publish_motor_command()


def main(args=None):
    rclpy.init(args=args)
    node = InitPoseTuningNode()

    try:
        rclpy.spin(node)
    finally:
        try:
            node.gamepad.stop()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()