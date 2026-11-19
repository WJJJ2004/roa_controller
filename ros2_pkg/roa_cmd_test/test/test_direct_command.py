"""Offline command mapping checks; no ROS node or hardware is started."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from evdev import ecodes

from roa_cmd_test.gamepad_reader import Gamepad, _interpolate, _normalize_abs

from roa_cmd_test.roa_velocity_command_test_node import (
    RoaVelocityCommandTestNode,
    SIM2SIM_LIMITS,
    bounded_gamepad_command,
)


class DirectCommandTest(unittest.TestCase):
    def test_enabled_movement_then_exact_zero_is_immediate(self):
        command, _ = bounded_gamepad_command([0.4, -0.2, 0.6, 1], True, SIM2SIM_LIMITS)
        self.assertTrue(np.any(command != 0))
        command, status = bounded_gamepad_command([0, 0, 0, 1], True, SIM2SIM_LIMITS)
        np.testing.assert_array_equal(command, [0, 0, 0])
        self.assertEqual(status, "enabled")

    def test_pass_through_and_bounds(self):
        command, status = bounded_gamepad_command([0.4, -0.2, 0.6, 1], True, SIM2SIM_LIMITS)
        np.testing.assert_allclose(command, [0.4, -0.2, 0.6], rtol=0, atol=1e-7)
        self.assertEqual(status, "enabled")
        command, status = bounded_gamepad_command([2, -2, -3, 1], True, SIM2SIM_LIMITS)
        np.testing.assert_array_equal(command, [1, -0.5, -1])
        self.assertEqual(status, "enabled")

    def test_disable_disconnect_and_invalid_are_zero(self):
        for raw, running in (
            ([0.4, 0.2, 0.6, 0], True),
            ([0.4, 0.2, 0.6, 1], False),
            ([np.nan, 0.2, 0.6, 1], True),
            ([0.4, 0.2, 0.6], True),
            ([0.4, 0.2, 0.6, 2], True),
        ):
            command, status = bounded_gamepad_command(raw, running, SIM2SIM_LIMITS)
            np.testing.assert_array_equal(command, [0, 0, 0])
            self.assertNotEqual(status, "enabled")

    def test_timer_publishes_direct_command_then_immediate_zero(self):
        published = []
        gamepad = SimpleNamespace(is_running=True, get_command=lambda: np.array([0.4, -0.2, 0.6, 1]))
        logger = SimpleNamespace(info=lambda *args, **kwargs: None, warn=lambda *args, **kwargs: None)
        harness = SimpleNamespace(
            gamepad=gamepad,
            limits=SIM2SIM_LIMITS,
            get_logger=lambda: logger,
            publish_cmd=lambda *command: published.append(command),
        )
        RoaVelocityCommandTestNode.timer_callback(harness)
        gamepad.get_command = lambda: np.array([0.4, -0.2, 0.6, 0])
        RoaVelocityCommandTestNode.timer_callback(harness)
        gamepad.is_running = False
        RoaVelocityCommandTestNode.timer_callback(harness)
        np.testing.assert_allclose(published[0], [0.4, -0.2, 0.6], rtol=0, atol=1e-7)
        self.assertEqual(published[1:], [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)])


class NeutralInputTest(unittest.TestCase):
    def test_normalized_boundary_device_flat_and_velocity_scales(self):
        info = SimpleNamespace(min=-100, max=100, flat=0)
        for value in (-5, 0, 5):
            self.assertEqual(_normalize_abs(value, info, 0.05), 0.0)
        self.assertEqual(_normalize_abs(6, info, 0.05), 0.06)
        self.assertEqual(_normalize_abs(-6, info, 0.05), -0.06)
        self.assertEqual(_interpolate(0.06, 1.0, 0.5), 0.03)
        info.flat = 10
        self.assertEqual(_normalize_abs(10, info, 0.05), 0.0)
        self.assertEqual(_normalize_abs(11, info, 0.05), 0.11)
        info = SimpleNamespace(min=0, max=255, flat=0)
        for value in (126, 127, 128, 130):
            self.assertEqual(_normalize_abs(value, info), 0.0)
        self.assertEqual(_normalize_abs(0, info), -1.0)
        self.assertEqual(_normalize_abs(255, info), 1.0)

    def test_invalid_deadzone_is_rejected_before_thread_start(self):
        for value in (-0.1, 1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Gamepad(stick_deadzone=value)

    def test_release_events_clear_all_axes_and_remain_zero_without_new_axis_events(self):
        # Mock only the device/thread: exercise the real reader and publisher callback.
        logger = SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
        with patch("roa_cmd_test.gamepad_reader.threading.Thread.start"):
            gamepad = Gamepad(vel_scale_x=1.0, vel_scale_y=0.5,
                              vel_scale_rot=1.0, logger=logger)
        axes = (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX)
        gamepad._axis_codes = dict(zip(("left_x", "left_y", "right_x"), axes))
        gamepad._absinfo = {
            axis: SimpleNamespace(min=0, max=255, flat=0) for axis in axes
        }
        gamepad._connect_device = lambda: True
        published = []
        harness = SimpleNamespace(
            gamepad=gamepad, limits=SIM2SIM_LIMITS, get_logger=lambda: logger,
            publish_cmd=lambda *values: published.append(values),
        )

        def events():
            for axis in axes:
                yield SimpleNamespace(type=ecodes.EV_ABS, code=axis, value=0)
            RoaVelocityCommandTestNode.timer_callback(harness)
            # A held stick must remain valid without additional axis changes.
            yield SimpleNamespace(type=ecodes.EV_SYN, code=ecodes.SYN_REPORT, value=0)
            RoaVelocityCommandTestNode.timer_callback(harness)
            for axis, value in zip(axes, (126, 128, 130)):
                yield SimpleNamespace(type=ecodes.EV_ABS, code=axis, value=value)
            for _ in range(10):
                RoaVelocityCommandTestNode.timer_callback(harness)
                yield SimpleNamespace(type=ecodes.EV_SYN, code=ecodes.SYN_REPORT, value=0)

        gamepad._device = SimpleNamespace(
            read_loop=events, ungrab=lambda: None, close=lambda: None,
        )
        gamepad.read_loop()
        self.assertEqual(published[:2], [(1.0, 0.5, 1.0)] * 2)
        self.assertEqual(published[2:], [(0.0, 0.0, 0.0)] * 10)


if __name__ == "__main__":
    unittest.main()
