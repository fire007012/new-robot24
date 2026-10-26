"""Exercise production input callbacks queued across a stop/rearm boundary."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import yaml
from control_msgs.msg import JointJog
from robot_hardware_runtime.flipper import Reference, NAMES
from robot_hardware_runtime.node import RuntimeNode


class FlipperInputSession(unittest.TestCase):
    def test_retired_subscription_callback_cannot_replay_after_rearm(self):
        session = {'generation': 1, 'state': 'Running'}
        coordinator = SimpleNamespace(profile='csv',
            generation=lambda _: session['generation'], group_state=lambda _: session['state'])
        limits = yaml.safe_load((Path(__file__).parents[1]/'config/joint_limits.yaml').read_text())
        node = SimpleNamespace(coordinator=coordinator, reference=Reference(limits),
            last_profile=None, jog_active=False, jog_sub=None, trajectory_sub=None)
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


if __name__ == '__main__': unittest.main()
