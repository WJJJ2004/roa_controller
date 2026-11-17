"""Offline command mapping checks; no ROS node or hardware is started."""

import unittest
from types import SimpleNamespace

import numpy as np

from roa_cmd_test.roa_velocity_command_test_node import (
    RoaVelocityCommandTestNode,
    SIM2SIM_LIMITS,
    bounded_gamepad_command,
)


class DirectCommandTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
