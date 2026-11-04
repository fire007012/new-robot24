"""Exercise a spawner's load/configure gap before runtime activation."""
from types import SimpleNamespace
from concurrent.futures import Future
import unittest
from unittest.mock import Mock, patch

from controller_manager_msgs.srv import ListControllers, SwitchController
from robot_hardware_runtime.node import Adapter


def listed(**states):
    return SimpleNamespace(controller=[SimpleNamespace(name=name, state=state)
                                       for name, state in states.items()])


class ControllerStartupTests(unittest.TestCase):
    def test_waits_for_loaded_controller_to_finish_configuring(self):
        adapter = Adapter(Mock())
        adapter.call = Mock(side_effect=[listed(), listed(wheel_controller='unconfigured'),
                                        listed(wheel_controller='inactive'), SimpleNamespace(ok=True)])
        with patch('robot_hardware_runtime.node.time.sleep'):
            adapter.controllers(['wheel_controller'], [])
        calls = adapter.call.call_args_list
        self.assertEqual([call.args[1] for call in calls],
                         [ListControllers, ListControllers, ListControllers, SwitchController])
        request = calls[-1].args[2]
        self.assertEqual(request.activate_controllers, ['wheel_controller'])
        self.assertEqual(request.strictness, SwitchController.Request.STRICT)

    def test_unconfigured_controller_timeout_never_issues_activation(self):
        adapter = Adapter(Mock())
        adapter.timeout = 0.0
        adapter.call = Mock(return_value=listed(wheel_controller='unconfigured'))
        with self.assertRaisesRegex(RuntimeError, r'wheel_controller\(unconfigured\)'):
            adapter.controllers(['wheel_controller'], [])
        self.assertEqual(adapter.call.call_count, 1)
        self.assertIs(adapter.call.call_args.args[1], ListControllers)

    def test_read_timeout_cancels_only_local_future(self):
        adapter = Adapter(Mock())
        adapter.timeout = 0.0
        future = Future()
        adapter.node.create_client.return_value.call_async.return_value = future
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            adapter.call('/read_only', ListControllers, ListControllers.Request())
        self.assertTrue(future.cancelled())
        self.assertEqual(adapter.pending, {})

    def test_mutation_timeout_preserves_future_and_fences_rearming(self):
        adapter = Adapter(Mock())
        adapter.timeout = 0.0
        adapter.begin_scope('base')
        future = Future()
        adapter.node.create_client.return_value.call_async.return_value = future
        with self.assertRaisesRegex(RuntimeError, 'outcome unknown'):
            adapter.call('/mutation', SwitchController, SwitchController.Request(),
                         mutation=True, resource='resource:CanDriverSystem')
        self.assertFalse(future.cancelled())
        self.assertIn(future, adapter.pending['resource:CanDriverSystem'])
        with self.assertRaisesRegex(RuntimeError, 'still pending'):
            adapter.ensure_settled('base')
        future.set_result(SimpleNamespace(ok=True))
        adapter.ensure_settled('base')


if __name__ == '__main__':
    unittest.main()
