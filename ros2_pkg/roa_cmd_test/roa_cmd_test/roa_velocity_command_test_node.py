#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np

from roa_cmd_test.gamepad_reader import Gamepad

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

        # 증분 속도 크기
        # 50Hz 기준이면 vx는 1초 동안 최대 약 0.5 증가
        self.declare_parameter("delta_vx", 0.01)
        self.declare_parameter("delta_vy", 0.005)
        self.declare_parameter("delta_wz", 0.01)

        # joystick deadzone 이후 들어온 값이 작아지면 감속시키기 위한 계수
        self.declare_parameter("decay_rate", 0.90)

        # command disable일 때 zero publish 여부
        self.declare_parameter("publish_zero_when_disabled", True)

        cmd_topic = self.get_parameter("cmd_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)

        self.delta_vx = float(self.get_parameter("delta_vx").value)
        self.delta_vy = float(self.get_parameter("delta_vy").value)
        self.delta_wz = float(self.get_parameter("delta_wz").value)

        self.decay_rate = float(self.get_parameter("decay_rate").value)
        self.publish_zero_when_disabled = bool(
            self.get_parameter("publish_zero_when_disabled").value
        )

        # ===== publisher =====
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.device_path = str(self.declare_parameter("device_path", "").value)

        self.vendor_id = int(self.declare_parameter("vendor_id", 0x046D).value)
        self.product_id = int(self.declare_parameter("product_id", 0xC219).value)
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


        self.cmd = np.zeros(3, dtype=np.float32)

        period = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"ROA velocity command test node started\n"
            f"  topic       : {cmd_topic}\n"
            f"  rate        : {publish_rate_hz:.1f} Hz\n"
            f"  max cmd     : vx={self.max_vx}, vy={self.max_vy}, wz={self.max_wz}\n"
            f"  delta cmd   : vx={self.delta_vx}, vy={self.delta_vy}, wz={self.delta_wz}"
        )

    def timer_callback(self):
        if not self.gamepad.is_running:
            self.get_logger().warn("Gamepad is not running. Publishing zero command.", throttle_duration_sec=1.0)
            self.publish_cmd(0.0, 0.0, 0.0)
            return

        raw = self.gamepad.get_command().astype(np.float32)

        # Gamepad API: [vx, vy, wz, enable]
        if raw.shape[0] >= 4:
            joy = raw[:3]
            enabled = bool(raw[3] > 0.5)
        else:
            joy = raw[:3]
            enabled = True

        if not enabled:
            self.cmd[:] = 0.0

            if self.publish_zero_when_disabled:
                self.publish_cmd(0.0, 0.0, 0.0)

            self.get_logger().info(
                "[CMD DISABLED] publish zero command",
                throttle_duration_sec=1.0,
            )
            return

        # ===== incremental velocity update =====
        delta = np.array(
            [
                self.delta_vx * joy[0],
                self.delta_vy * joy[1],
                self.delta_wz * joy[2],
            ],
            dtype=np.float32,
        )

        self.cmd += delta

        # 입력이 거의 없으면 천천히 0으로 감속
        if abs(joy[0]) < 1e-3:
            self.cmd[0] *= self.decay_rate
        if abs(joy[1]) < 1e-3:
            self.cmd[1] *= self.decay_rate
        if abs(joy[2]) < 1e-3:
            self.cmd[2] *= self.decay_rate

        # sim2sim command range와 동일한 clip
        self.cmd[0] = np.clip(self.cmd[0], -self.max_vx, self.max_vx)
        self.cmd[1] = np.clip(self.cmd[1], -self.max_vy, self.max_vy)
        self.cmd[2] = np.clip(self.cmd[2], -self.max_wz, self.max_wz)

        self.publish_cmd(
            float(self.cmd[0]),
            float(self.cmd[1]),
            float(self.cmd[2]),
        )

        self.get_logger().info(
            f"[CMD] vx={self.cmd[0]: .3f}, vy={self.cmd[1]: .3f}, wz={self.cmd[2]: .3f} "
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