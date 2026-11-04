#!/usr/bin/env python3
"""Check the installed readiness CLI against a lost reply and inactive plant."""
import os
import subprocess
from threading import Event, Thread
import unittest

import rclpy
from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class ReadinessSmoke(unittest.TestCase):
    def setUp(self):
        os.environ.update(ROS_DOMAIN_ID='96', ROS_LOCALHOST_ONLY='1')
        rclpy.init()
        self.node = Node('readiness_fixture')
        self.callbacks = ReentrantCallbackGroup()
        self.release = Event()
        self.calls = 0
        self.drop_first = False
        self.node.create_service(ListControllers, '/controller_manager/list_controllers',
                                 self.controllers, callback_group=self.callbacks)
        self.node.create_service(ListHardwareComponents, '/controller_manager/list_hardware_components',
                                 self.hardware, callback_group=self.callbacks)
        self.state = self.node.create_publisher(String, '/robot_runtime/state', 1)
        self.node.create_timer(0.05, lambda: self.state.publish(String(data='{"state":"Stopped"}')),
                               callback_group=self.callbacks)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = Thread(target=self.executor.spin)
        self.thread.start()

    def tearDown(self):
        self.release.set()
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def controllers(self, _, response):
        self.calls += 1
        if self.drop_first and self.calls == 1:
            self.release.wait(15)
        response.controller = [ControllerState(name=name, state='inactive') for name in
            ('joint_state_broadcaster', 'arm_position_controller', 'flipper_csp_controller',
             'flipper_csv_forward_controller', 'gripper_controller', 'wheel_controller')]
        return response

    def hardware(self, _, response):
        response.component = [HardwareComponentState(name=name, state=State(id=1, label='unconfigured'))
                              for name in ('RobotCanopenSystem', 'CanDriverSystem')]
        return response

    def check(self, *parameters):
        return subprocess.run(['ros2', 'run', 'robot_bringup', 'check_ready', '--ros-args',
                               '-p', 'timeout:=5.0', *parameters],
                              text=True, capture_output=True, timeout=12)

    def test_lost_read_reply_is_retried_before_overall_deadline(self):
        self.drop_first = True
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertGreaterEqual(self.calls, 2)
        self.assertIn('Ready:', result.stdout + result.stderr)

    def test_inactive_plant_is_never_reported_running(self):
        result = self.check('-p', 'require_running:=true')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Readiness timeout', result.stdout + result.stderr)
        self.assertIn('"running": false', result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
