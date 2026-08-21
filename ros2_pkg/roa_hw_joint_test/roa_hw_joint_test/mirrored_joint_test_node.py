#!/usr/bin/env python3

"""Play a fixed open-loop hip-yaw test waveform."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from roa_interfaces.msg import MotorCommand, MotorCommandArray, RsuTarget
COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
)
RSU_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

MOTOR_IDS = tuple(range(9, 18))
KP = {
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
KD = {
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


def lerp(start: float, end: float, ratio: float) -> float:
    """Linearly interpolate with a clamped ratio."""
    ratio = max(0.0, min(1.0, ratio))
    return start + (end - start) * ratio


class HipYawWaveformNode(Node):
    """Publish a deterministic zero/+angle/zero/-angle/zero waveform."""

    def __init__(self) -> None:
        super().__init__("mirrored_joint_test_node")

        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)
        self.initial_hold_sec = float(
            self.declare_parameter("initial_hold_sec", 10.0).value
        )
        self.move_duration_sec = float(
            self.declare_parameter("move_duration_sec", 5.0).value
        )
        self.hold_sec = float(
            self.declare_parameter("hold_sec", 3.0).value
        )
        self.amplitude_deg = float(
            self.declare_parameter("amplitude_deg", 30.0).value
        )
        self.repeat = bool(self.declare_parameter("repeat", False).value)

        if self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        if self.initial_hold_sec < 0.0:
            raise ValueError("initial_hold_sec must be non-negative")
        if self.move_duration_sec <= 0.0 or self.hold_sec < 0.0:
            raise ValueError("move_duration_sec must be positive and hold_sec non-negative")
        if not 0.0 < self.amplitude_deg <= 40.0:
            raise ValueError("amplitude_deg must be in (0, 40]")

        self.amplitude = math.radians(self.amplitude_deg)
        self.start_time = time.monotonic()
        self.rsu_seq = 0
        self.last_stage = ""

        self.command_pub = self.create_publisher(
            MotorCommandArray,
            "/hardware_interface/command",
            COMMAND_QOS,
        )
        self.rsu_pub = self.create_publisher(
            RsuTarget,
            "/rsu/target",
            RSU_QOS,
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        total = self.waveform_duration()
        self.get_logger().warn(
            "Open-loop hip-yaw waveform started: "
            f"zero {self.initial_hold_sec:.1f}s -> +{self.amplitude_deg:.1f}deg "
            f"in {self.move_duration_sec:.1f}s -> hold {self.hold_sec:.1f}s -> "
            f"zero -> -{self.amplitude_deg:.1f}deg -> hold -> zero. "
            f"Total {total:.1f}s, repeat={self.repeat}. No feedback is used."
        )

    def waveform_duration(self) -> float:
        """Return total duration of one waveform cycle."""
        return self.initial_hold_sec + 4.0 * self.move_duration_sec + 2.0 * self.hold_sec

    def hip_yaw_at(self, elapsed: float) -> tuple[float, str]:
        """Return the hip-yaw offset and current stage."""
        t = elapsed
        move = self.move_duration_sec
        hold = self.hold_sec
        amp = self.amplitude

        if t < self.initial_hold_sec:
            return 0.0, "INITIAL_ZERO"
        t -= self.initial_hold_sec

        if t < move:
            return lerp(0.0, amp, t / move), "MOVE_TO_POSITIVE"
        t -= move
        if t < hold:
            return amp, "HOLD_POSITIVE"
        t -= hold
        if t < move:
            return lerp(amp, 0.0, t / move), "RETURN_FROM_POSITIVE"
        t -= move
        if t < move:
            return lerp(0.0, -amp, t / move), "MOVE_TO_NEGATIVE"
        t -= move
        if t < hold:
            return -amp, "HOLD_NEGATIVE"
        t -= hold
        if t < move:
            return lerp(-amp, 0.0, t / move), "RETURN_FROM_NEGATIVE"
        return 0.0, "DONE_ZERO"

    def publish_zero_rsu_target(self) -> None:
        """Keep all four virtual ankle targets at zero."""
        msg = RsuTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "roa_hw_joint_test"
        self.rsu_seq += 1
        msg.seq = self.rsu_seq
        msg.left_roll = 0.0
        msg.left_pitch = 0.0
        msg.right_roll = 0.0
        msg.right_pitch = 0.0
        self.rsu_pub.publish(msg)

    def publish_motor_targets(self, hip_yaw: float) -> None:
        """Publish mirrored hip-yaw targets and fix all other joints at zero."""
        msg = MotorCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ROA_HW_HIP_PITCH_WAVEFORM"

        for motor_id in MOTOR_IDS:
            msg.commands.append(MotorCommand(
                motor_id=motor_id,
                torque=0.0,
                position=(
                    hip_yaw if motor_id == 14
                    else -hip_yaw if motor_id == 15
                    else 0.0
                ),
                velocity=0.0,
                kp=KP[motor_id],
                kd=KD[motor_id],
            ))
        self.command_pub.publish(msg)

    def on_timer(self) -> None:
        """Advance solely from monotonic time and publish the fixed waveform."""
        elapsed = time.monotonic() - self.start_time
        total = self.waveform_duration()
        if self.repeat and elapsed >= total:
            elapsed %= total

        hip_yaw, stage = self.hip_yaw_at(elapsed)
        self.publish_zero_rsu_target()
        self.publish_motor_targets(hip_yaw)

        if stage != self.last_stage:
            self.last_stage = stage
            self.get_logger().warn(
                f"[WAVEFORM] {stage}: hip_yaw={math.degrees(hip_yaw):+.3f}deg"
            )
        self.get_logger().info(
            f"stage={stage}, hip_yaw={math.degrees(hip_yaw):+.3f}deg",
            throttle_duration_sec=0.5,
        )


def main(args=None) -> None:
    """Run the open-loop waveform node."""
    rclpy.init(args=args)
    node = HipYawWaveformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
