"""Exercise production input callbacks queued across a stop/rearm boundary."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import time
from unittest.mock import Mock, patch
from rclpy._rclpy_pybind11 import InvalidHandle
import yaml
from control_msgs.msg import JointJog
from robot_hardware_runtime.flipper import Reference, NAMES
from robot_hardware_runtime.node import RuntimeNode, spin_runtime


class FlipperInputSession(unittest.TestCase):
    def test_retired_subscription_callback_cannot_replay_after_rearm(self):
        session = {'generation': 1, 'state': 'Running'}
        coordinator = SimpleNamespace(profile='csv',
            generation=lambda _: session['generation'], group_state=lambda _: session['state'])
        limits = yaml.safe_load((Path(__file__).parents[1]/'config/joint_limits.yaml').read_text())
        node = SimpleNamespace(coordinator=coordinator, reference=Reference(limits),
            last_profile=None, jog_active=False, jog_sub=None, trajectory_sub=None,
            retired_subscription_errors=0, last_subscription_retirement=0.0)
        node.create_subscription = lambda kind, topic, callback, depth: callback
        node.destroy_subscription = lambda subscription: True
        node.sync_flipper_session = lambda: RuntimeNode.sync_flipper_session(node)
        node.jog = lambda message, token: RuntimeNode.jog(node, message, token)
        node.trajectory = lambda message, token: RuntimeNode.trajectory(node, message, token)
        node.sync_flipper_session()
        queued = node.jog_sub
        message = JointJog(joint_names=[NAMES[0]], velocities=[.1])
        queued(message)
        self.assertTrue(node.jog_active)
        # Both transitions can occur between reference timer callbacks.
        session.update(generation=3)
        queued(message)
        self.assertFalse(node.jog_active)
        self.assertIsNot(node.jog_sub, queued)
        node.jog_sub(message)
        self.assertTrue(node.jog_active)

    def test_destroyed_take_only_recovers_during_retirement(self):
        node = SimpleNamespace(retired_subscription_errors=1, last_subscription_retirement=time.monotonic())
        executor = Mock()
        executor.spin.side_effect = [InvalidHandle('retired take'), None]
        with patch('robot_hardware_runtime.node.rclpy.ok', return_value=True):
            spin_runtime(executor, node)
        self.assertEqual(executor.spin.call_count, 2)
        self.assertEqual(node.retired_subscription_errors, 0)
        executor.spin.side_effect = InvalidHandle('unrelated destruction')
        with patch('robot_hardware_runtime.node.rclpy.ok', return_value=True):
            with self.assertRaises(InvalidHandle): spin_runtime(executor, node)
        node.retired_subscription_errors = 1
        node.last_subscription_retirement = time.monotonic() - 5
        with patch('robot_hardware_runtime.node.rclpy.ok', return_value=True):
            with self.assertRaises(InvalidHandle): spin_runtime(executor, node)


if __name__ == '__main__': unittest.main()
