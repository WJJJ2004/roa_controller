"""Virtual hip roll-pitch to physical motor-angle mapper.

All input and output angles use radians.

Orientation model:

    Rx(27°) Rz(theta1) Rx(243°) Ry(90°) Rz(theta2)
    =
    Rz(pitch) Ry(90°) Rz(-90°) Rz(roll)
"""

from __future__ import annotations

import math
from dataclasses import dataclass


__all__ = [
    "HipMotorAngles",
    "map_hip_roll_pitch_to_motor_angles",
]


_DEG_TO_RAD = math.pi / 180.0

_OFFSET_27_RAD = 27.0 * _DEG_TO_RAD
_OFFSET_243_RAD = 243.0 * _DEG_TO_RAD

_SIN_27 = math.sin(_OFFSET_27_RAD)
_COS_27 = math.cos(_OFFSET_27_RAD)


@dataclass(frozen=True)
class HipMotorAngles:
    """Physical hip motor angles in radians."""

    theta1: float
    theta2: float


def _wrap_to_pi(angle: float) -> float:
    """Normalize an angle to the range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

# TODO : Add left-right mirroring option for the hip roll and pitch mapping.
def map_hip_roll_pitch_to_motor_angles(
    roll: float,
    pitch: float,
    is_left: bool = True,
) -> HipMotorAngles:
    """Map virtual hip roll and pitch to physical motor angles.

    Parameters
    ----------
    roll:
        Desired virtual hip roll in radians.

    pitch:
        Desired virtual hip pitch in radians.

    Returns
    -------
    HipMotorAngles
        ``theta1`` and ``theta2`` in radians.
    """
    sin_pitch = math.sin(pitch)
    cos_pitch = math.cos(pitch)

    sin_roll = math.sin(roll)
    cos_roll = math.cos(roll)

    theta1 = math.atan2(
        _COS_27 * sin_pitch,
        cos_pitch,
    )

    m31 = (
        _SIN_27 * cos_pitch * cos_roll
        - _COS_27 * sin_roll
    )

    m32 = (
        -_SIN_27 * cos_pitch * sin_roll
        - _COS_27 * cos_roll
    )

    theta2 = (
        math.atan2(m32, -m31)
        - _OFFSET_243_RAD
    )

    return HipMotorAngles(
        theta1=_wrap_to_pi(theta1),
        theta2=_wrap_to_pi(theta2),
    )

def main():
    """Test the hip roll-pitch to motor angle mapping."""
    test_cases = [
        (0.0, 0.0),
        (math.pi / 6, math.pi / 6),
        (-math.pi / 6, -math.pi / 6),
        (math.pi / 4, math.pi / 4),
        (-math.pi / 4, -math.pi / 4),
    ]

    for roll, pitch in test_cases:
        motor_angles = map_hip_roll_pitch_to_motor_angles(roll, pitch)
        print(f"Roll: {roll:.3f}, Pitch: {pitch:.3f} -> "
              f"Theta1: {motor_angles.theta1:.3f}, "
              f"Theta2: {motor_angles.theta2:.3f}")