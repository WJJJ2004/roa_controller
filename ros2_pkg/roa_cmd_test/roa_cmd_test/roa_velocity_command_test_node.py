#!/usr/bin/env python3
# Smooth velocity command test node
# - joy input [-1, 1]
# - target_cmd = joy * max_cmd
# - cmd moves toward target_cmd with rate limit
# - default: 0 -> max command in 1.0 sec

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np

from roa_cmd_test.gamepad_reader import Gamepad


def clamp(value: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(value, vmax))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class RoaVelocityCommandTestNode(Node):
    def __init__(self):
        super().__init__("roa_velocity_command_test_node")

        # ===== ROS params =====
        self.declare_parameter("cmd_topic", "/ctrl/policy_cmd")
        self.declare_parameter("publish_rate_hz", 50.0)

        # sim2sim command range와 동일하게 설정
        self.declare_parameter("max_vx", 1.0)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_wz", 1.0)

        # full stick 기준 0 -> max command까지 걸리는 시간
        # 기본값: 1초 안에 0 -> 1m/s 도달
        self.declare_parameter("rise_time_s", 1.0)

        # stick release 또는 disable 시 0으로 복귀하는 시간
        self.declare_parameter("return_time_s", 1.0)

        # command disable일 때 command를 0 방향으로 publish할지 여부
        self.declare_parameter("publish_zero_when_disabled", True)

        # gamepad config
        self.declare_parameter("device_path", "")
        self.declare_parameter("vendor_id", 0x046D)
        self.declare_parameter("product_id", 0xC219)

        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.rise_time_s = float(self.get_parameter("rise_time_s").value)
        self.return_time_s = float(self.get_parameter("return_time_s").value)

        self.publish_zero_when_disabled = bool(
            self.get_parameter("publish_zero_when_disabled").value
        )

        self.device_path = str(self.get_parameter("device_path").value)
        self.vendor_id = int(self.get_parameter("vendor_id").value)
        self.product_id = int(self.get_parameter("product_id").value)

        # ===== safety check =====
        self.publish_rate_hz = max(self.publish_rate_hz, 1.0)
        self.period = 1.0 / self.publish_rate_hz

        self.max_vx = abs(self.max_vx)
        self.max_vy = abs(self.max_vy)
        self.max_wz = abs(self.max_wz)

        self.rise_time_s = max(self.rise_time_s, 1e-3)
        self.return_time_s = max(self.return_time_s, 1e-3)

        # ===== publisher =====
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # ===== Gamepad =====
        self.gamepad = Gamepad(
            vendor_id=self.vendor_id,
            product_id=self.product_id,
            vel_scale_x=1.0,
            vel_scale_y=1.0,
            vel_scale_rot=1.0,
            device_path=(self.device_path if self.device_path else None),
            prefer_name_contains="Logitech",
        )

        # 현재 publish 중인 velocity command
        # [vx, vy, wz]
        self.cmd = np.zeros(3, dtype=np.float32)

        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f"ROA velocity command test node started\n"
            f"  topic        : {cmd_topic}\n"
            f"  rate         : {self.publish_rate_hz:.1f} Hz\n"
            f"  period       : {self.period:.4f} sec\n"
            f"  max cmd      : vx={self.max_vx}, vy={self.max_vy}, wz={self.max_wz}\n"
            f"  rise_time_s  : {self.rise_time_s}\n"
            f"  return_time_s: {self.return_time_s}"
        )

    def get_rise_delta(self) -> np.ndarray:
        return np.array(
            [
                self.max_vx * self.period / self.rise_time_s,
                self.max_vy * self.period / self.rise_time_s,
                self.max_wz * self.period / self.rise_time_s,
            ],
            dtype=np.float32,
        )

    def get_return_delta(self) -> np.ndarray:
        return np.array(
            [
                self.max_vx * self.period / self.return_time_s,
                self.max_vy * self.period / self.return_time_s,
                self.max_wz * self.period / self.return_time_s,
            ],
            dtype=np.float32,
        )

    def update_cmd_toward_target(self, target_cmd: np.ndarray, max_delta: np.ndarray):
        self.cmd[0] = move_toward(
            float(self.cmd[0]),
            float(target_cmd[0]),
            float(max_delta[0]),
        )
        self.cmd[1] = move_toward(
            float(self.cmd[1]),
            float(target_cmd[1]),
            float(max_delta[1]),
        )
        self.cmd[2] = move_toward(
            float(self.cmd[2]),
            float(target_cmd[2]),
            float(max_delta[2]),
        )

        self.cmd[0] = clamp(float(self.cmd[0]), -self.max_vx, self.max_vx)
        self.cmd[1] = clamp(float(self.cmd[1]), -self.max_vy, self.max_vy)
        self.cmd[2] = clamp(float(self.cmd[2]), -self.max_wz, self.max_wz)

    def return_cmd_to_zero(self):
        target_cmd = np.zeros(3, dtype=np.float32)
        max_delta = self.get_return_delta()
        self.update_cmd_toward_target(target_cmd, max_delta)

    def timer_callback(self):
        if not self.gamepad.is_running:
            self.get_logger().warn(
                "Gamepad is not running. Returning command to zero.",
                throttle_duration_sec=1.0,
            )

            self.return_cmd_to_zero()
            self.publish_cmd(
                float(self.cmd[0]),
                float(self.cmd[1]),
                float(self.cmd[2]),
            )
            return

        raw = self.gamepad.get_command().astype(np.float32)

        # Gamepad API: [vx, vy, wz, enable]
        if raw.shape[0] >= 4:
            joy = raw[:3].copy()
            enabled = bool(raw[3] > 0.5)
        else:
            joy = raw[:3].copy()
            enabled = True

        # deadband는 보수적으로 잡지 않음
        # 단, 이상 입력 방지를 위해 -1.0 ~ 1.0으로만 제한
        joy[0] = np.clip(joy[0], -1.0, 1.0)
        joy[1] = np.clip(joy[1], -1.0, 1.0)
        joy[2] = np.clip(joy[2], -1.0, 1.0)

        if not enabled:
            self.return_cmd_to_zero()

            if self.publish_zero_when_disabled:
                self.publish_cmd(
                    float(self.cmd[0]),
                    float(self.cmd[1]),
                    float(self.cmd[2]),
                )

            self.get_logger().info(
                f"[CMD DISABLED] returning to zero "
                f"vx={self.cmd[0]: .3f}, vy={self.cmd[1]: .3f}, wz={self.cmd[2]: .3f}",
                throttle_duration_sec=1.0,
            )
            return

        # ===== smooth velocity command update =====
        # joy 입력을 최종 목표 velocity로 변환
        target_cmd = np.array(
            [
                joy[0] * self.max_vx,
                joy[1] * self.max_vy,
                joy[2] * self.max_wz,
            ],
            dtype=np.float32,
        )

        # 0 -> max command까지 rise_time_s 안에 도달하도록 tick당 최대 변화량 계산
        max_delta = self.get_rise_delta()

        self.update_cmd_toward_target(target_cmd, max_delta)

        self.publish_cmd(
            float(self.cmd[0]),
            float(self.cmd[1]),
            float(self.cmd[2]),
        )

        self.get_logger().info(
            f"[CMD] vx={self.cmd[0]: .3f}, vy={self.cmd[1]: .3f}, wz={self.cmd[2]: .3f} "
            f"| target=[{target_cmd[0]: .3f}, {target_cmd[1]: .3f}, {target_cmd[2]: .3f}] "
            f"| joy=[{joy[0]: .3f}, {joy[1]: .3f}, {joy[2]: .3f}]",
            throttle_duration_sec=0.5,
        )

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