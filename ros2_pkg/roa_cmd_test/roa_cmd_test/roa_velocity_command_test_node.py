#!/usr/bin/env python3
# Publish the gamepad's velocity command directly, with sim2sim bounds.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np

from roa_cmd_test.gamepad_reader import Gamepad
from roa_cmd_test.gamepad_reader import __file__ as GAMEPAD_SOURCE


SIM2SIM_LIMITS = np.array([1.0, 0.5, 1.0], dtype=np.float32)


def bounded_gamepad_command(raw, running: bool, limits: np.ndarray):
    """Pass through velocity values; fail closed on disable or invalid input."""
    zero = np.zeros(3, dtype=np.float32)
    if not running:
        return zero, "disconnected"
    try:
        values = np.asarray(raw, dtype=np.float32)
    except (TypeError, ValueError):
        return zero, "invalid"
    if values.shape != (4,) or not np.all(np.isfinite(values)) or not 0.0 <= values[3] <= 1.0:
        return zero, "invalid"
    if values[3] <= 0.5:
        return zero, "disabled"
    return np.clip(values[:3], -limits, limits), "enabled"


class RoaVelocityCommandTestNode(Node):
    def __init__(self):
        super().__init__("roa_velocity_command_test_node")

        # ===== ROS params =====
        self.declare_parameter("cmd_topic", "/ctrl/policy_cmd")
        self.declare_parameter("publish_rate_hz", 50.0)

        # Upper bounds are the sim2sim gamepad scales.
        self.declare_parameter("max_vx", 1.0)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_wz", 1.0)

        # gamepad config
        self.declare_parameter("device_path", "")
        self.declare_parameter("vendor_id", 0x046D)
        self.declare_parameter("product_id", 0xC219)
        self.declare_parameter("stick_deadzone", 0.05)

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        configured_limits = np.array(
            [float(self.get_parameter(name).value) for name in ("max_vx", "max_vy", "max_wz")],
            dtype=np.float32,
        )

        self.device_path = str(self.get_parameter("device_path").value)
        self.vendor_id = int(self.get_parameter("vendor_id").value)
        self.product_id = int(self.get_parameter("product_id").value)
        self.stick_deadzone = float(self.get_parameter("stick_deadzone").value)

        # ===== safety check =====
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        if not np.all(np.isfinite(configured_limits)) or np.any(configured_limits <= 0.0):
            raise ValueError("max_vx, max_vy and max_wz must be finite and positive")
        if not math.isfinite(self.stick_deadzone) or not 0.0 <= self.stick_deadzone < 1.0:
            raise ValueError("stick_deadzone must be finite and in [0, 1)")
        self.period = 1.0 / self.publish_rate_hz
        self.limits = np.minimum(configured_limits, SIM2SIM_LIMITS)

        # ===== publisher =====
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # ===== Gamepad =====
        self.gamepad = Gamepad(
            vendor_id=self.vendor_id,
            product_id=self.product_id,
            vel_scale_x=float(SIM2SIM_LIMITS[0]),
            vel_scale_y=float(SIM2SIM_LIMITS[1]),
            vel_scale_rot=float(SIM2SIM_LIMITS[2]),
            device_path=(self.device_path if self.device_path else None),
            prefer_name_contains="Logitech",
            stick_deadzone=self.stick_deadzone,
            logger=self.get_logger(),
        )

        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f"ROA velocity command test node started\n"
            f"  topic        : {cmd_topic}\n"
            f"  rate         : {self.publish_rate_hz:.1f} Hz\n"
            f"  period       : {self.period:.4f} sec\n"
            f"  limits       : vx={self.limits[0]}, vy={self.limits[1]}, wz={self.limits[2]}\n"
            f"  input        : direct-v2; neutral/disabled/disconnected/invalid -> zero\n"
            f"  deadzone     : {self.stick_deadzone:.3f} (normalized, before velocity scaling)\n"
            f"  node source  : {__file__}\n"
            f"  reader source: {GAMEPAD_SOURCE}"
        )

    def timer_callback(self):
        try:
            raw = self.gamepad.get_command() if self.gamepad.is_running else None
        except Exception as exc:
            raw = None
            self.get_logger().warn(f"Gamepad read failed: {exc}", throttle_duration_sec=1.0)
        command, status = bounded_gamepad_command(raw, self.gamepad.is_running, self.limits)
        self.publish_cmd(float(command[0]), float(command[1]), float(command[2]))
        if status == "enabled":
            mode = "neutral" if np.all(command == 0.0) else "enabled"
            self.get_logger().info(
                f"[CMD {mode}] input={np.asarray(raw)[:3].tolist()} "
                f"published=[{command[0]: .6f}, {command[1]: .6f}, {command[2]: .6f}]",
                throttle_duration_sec=0.5,
            )
        else:
            self.get_logger().warn(f"[CMD {status}] publishing zero", throttle_duration_sec=1.0)

    def publish_cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz

        self.cmd_pub.publish(msg)

    def destroy_node(self):
        try:
            self.gamepad.stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RoaVelocityCommandTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
