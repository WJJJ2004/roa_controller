#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import JointState
from roa_interfaces.msg import MotorCommandArray, RsuTarget


class PaceCommandJointStatePublisher(Node):
    """Merge PACE actuator commands into a URDF-compatible JointState message.

    Conventions:
    - Non-RSU joints use MotorCommandArray.commands[].position directly.
    - RSU ankle joints use the virtual roll/pitch targets from RsuTarget.
    """

    MOTOR_ID_TO_JOINT = {
        9: "torso_yaw",
        10: "left_hip_pitch",
        11: "right_hip_pitch",
        12: "left_hip_roll",
        13: "right_hip_roll",
        14: "left_hip_yaw",
        15: "right_hip_yaw",
        16: "left_knee_pitch",
        17: "right_knee_pitch",
    }

    RSU_JOINT_NAMES = (
        "left_ankle_roll",
        "left_ankle_pitch",
        "right_ankle_roll",
        "right_ankle_pitch",
    )

    def __init__(self) -> None:
        super().__init__("pace_command_joint_state_publisher")

        self.command_topic = str(
            self.declare_parameter(
                "command_topic", "/hardware_interface/command"
            ).value
        )
        self.rsu_target_topic = str(
            self.declare_parameter("rsu_target_topic", "/rsu/target").value
        )
        self.joint_state_topic = str(
            self.declare_parameter("joint_state_topic", "/joint_states").value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 100.0).value
        )
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 1.0).value
        )
        self.rsu_timeout_sec = float(
            self.declare_parameter("rsu_timeout_sec", 1.0).value
        )
        self.publish_before_ready = bool(
            self.declare_parameter("publish_before_ready", True).value
        )
        self.warn_on_unknown_motor_id = bool(
            self.declare_parameter("warn_on_unknown_motor_id", True).value
        )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.command_timeout_sec <= 0.0:
            raise ValueError("command_timeout_sec must be greater than zero")
        if self.rsu_timeout_sec <= 0.0:
            raise ValueError("rsu_timeout_sec must be greater than zero")

        self.joint_positions: Dict[str, float] = {
            joint_name: 0.0
            for joint_name in (
                *self.MOTOR_ID_TO_JOINT.values(),
                *self.RSU_JOINT_NAMES,
            )
        }

        self.last_command_time: float | None = None
        self.last_rsu_time: float | None = None
        self.received_command = False
        self.received_rsu = False

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        rsu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        joint_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_sub = self.create_subscription(
            MotorCommandArray,
            self.command_topic,
            self.on_motor_command,
            command_qos,
        )
        self.rsu_sub = self.create_subscription(
            RsuTarget,
            self.rsu_target_topic,
            self.on_rsu_target,
            rsu_qos,
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            joint_state_qos,
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_joint_state,
        )

        self.get_logger().info(
            "PACE RViz joint-state bridge started: "
            f"{self.command_topic} + {self.rsu_target_topic} "
            f"-> {self.joint_state_topic}"
        )

    @staticmethod
    def is_finite(value: float) -> bool:
        return math.isfinite(float(value))

    def on_motor_command(self, msg: MotorCommandArray) -> None:
        updated_count = 0

        for command in msg.commands:
            motor_id = int(command.motor_id)
            joint_name = self.MOTOR_ID_TO_JOINT.get(motor_id)

            if joint_name is None:
                if self.warn_on_unknown_motor_id:
                    self.get_logger().debug(
                        f"Ignoring unmapped motor ID {motor_id}",
                        throttle_duration_sec=2.0,
                    )
                continue

            position = float(command.position)
            if not self.is_finite(position):
                self.get_logger().error(
                    f"Ignoring non-finite position from motor ID {motor_id}"
                )
                continue

            self.joint_positions[joint_name] = position
            updated_count += 1

        self.last_command_time = time.monotonic()
        self.received_command = True

        if updated_count == 0:
            self.get_logger().warn(
                "MotorCommandArray contained no mapped non-RSU motor commands.",
                throttle_duration_sec=2.0,
            )

    def on_rsu_target(self, msg: RsuTarget) -> None:
        values = {
            "left_ankle_roll": float(msg.left_roll),
            "left_ankle_pitch": float(msg.left_pitch),
            "right_ankle_roll": float(msg.right_roll),
            "right_ankle_pitch": float(msg.right_pitch),
        }

        if not all(self.is_finite(value) for value in values.values()):
            self.get_logger().error(
                "Ignoring /rsu/target because it contains a non-finite value."
            )
            return

        self.joint_positions.update(values)
        self.last_rsu_time = time.monotonic()
        self.received_rsu = True

    def input_is_fresh(self, last_time: float | None, timeout: float) -> bool:
        return (
            last_time is not None
            and time.monotonic() - last_time <= timeout
        )

    def publish_joint_state(self) -> None:
        if not self.publish_before_ready:
            if not self.received_command or not self.received_rsu:
                return

        command_fresh = self.input_is_fresh(
            self.last_command_time, self.command_timeout_sec
        )
        rsu_fresh = self.input_is_fresh(
            self.last_rsu_time, self.rsu_timeout_sec
        )

        if self.received_command and not command_fresh:
            self.get_logger().warn(
                f"No fresh motor command for {self.command_timeout_sec:.2f}s; "
                "holding the last non-RSU pose.",
                throttle_duration_sec=2.0,
            )

        if self.received_rsu and not rsu_fresh:
            self.get_logger().warn(
                f"No fresh RSU target for {self.rsu_timeout_sec:.2f}s; "
                "holding the last ankle pose.",
                throttle_duration_sec=2.0,
            )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Keep a deterministic ordering for plotting and debugging.
        msg.name = list(self.joint_positions.keys())
        msg.position = [
            float(self.joint_positions[name]) for name in msg.name
        ]

        self.joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaceCommandJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
