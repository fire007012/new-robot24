#!/usr/bin/env python3
"""Exercise installed top-level bringup and public base/gripper contracts."""
import importlib.util
import math
import os
from pathlib import Path
import time
import subprocess
import unittest
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, Float64, Float64MultiArray
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import SetBool

source = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('hardware_smoke', source / 'robot_hardware/test/hybrid_smoke.py')
hardware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hardware)


class FullSystemSmoke(unittest.TestCase):
    PACKAGE = 'robot_bringup'
    LAUNCH = 'full_system_mock.launch.py'
    ARGUMENTS = ['journal:=/tmp/robot24-full-smoke-maintenance.json', 'enable_moveit:=false']
    DOMAIN = '87'
    setUpClass = classmethod(hardware.HybridSmoke.setUpClass.__func__)
    tearDownClass = classmethod(hardware.HybridSmoke.tearDownClass.__func__)
    joint_state = classmethod(hardware.HybridSmoke.joint_state.__func__)
    runtime_state = classmethod(hardware.HybridSmoke.runtime_state.__func__)
    wait = hardware.HybridSmoke.wait
    result = hardware.HybridSmoke.result
    call = hardware.HybridSmoke.call
    trigger = hardware.HybridSmoke.trigger
    goal = hardware.HybridSmoke.goal

    def prepare(self):
        self.wait(lambda: getattr(self, 'runtime_details', {}).get('diagnostics', {}).get('readable', False))

    def conditions(self):
        for condition in ('arm_secured', 'support_trusted', 'payload_retained'):
            result = self.call('/mock/conditions/' + condition, SetBool, SetBool.Request(data=True))
            self.assertTrue(result.success)

    def test_public_commands_and_session_reset(self):
        def loaded():
            response = self.call('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
            return len(response.controller) == 6 and all(c.state == 'inactive' for c in response.controller)
        self.wait(loaded, 60)
        self.base_out, self.gripper_out = None, None
        self.bad_tf = []
        def transforms(message):
            for tf in message.transforms:
                t, q = tf.transform.translation, tf.transform.rotation
                if not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
                    self.bad_tf.append(tf.child_frame_id)
        self.node.create_subscription(TFMessage, '/tf', transforms, 10)
        self.node.create_subscription(Twist, '/wheel_controller/cmd_vel_unstamped',
            lambda msg: setattr(self, 'base_out', msg.linear.x), 1)
        self.node.create_subscription(Float64MultiArray, '/gripper_controller/command',
            lambda msg: setattr(self, 'gripper_out', msg.data[0]), 1)
        base = self.node.create_publisher(Twist, '/cmd_vel', 1)
        gripper = self.node.create_publisher(Float64, '/arm_control/gripper_velocity', 1)
        opening = self.node.create_publisher(Bool, '/arm_control/gripper_open', 1)
        command = Twist(); command.linear.x = 0.2
        for _ in range(10):
            base.publish(command); gripper.publish(Float64(data=0.01)); time.sleep(0.02)
        self.wait(lambda: self.base_out == 0.0 and self.gripper_out == 0.0)
        self.prepare()
        self.trigger('init')
        self.conditions()
        for operation in ('enable', 'resume'):
            self.trigger(operation)
        self.wait(lambda: self.runtime == 'Running')
        self.wait(lambda: base.get_subscription_count() > 0 and gripper.get_subscription_count() > 0)
        self.wait(lambda: 'left_front_wheel_joint' in self.velocities)
        checked = subprocess.run(['ros2', 'run', 'robot_bringup', 'check_ready', '--ros-args',
                                  '-p', 'require_running:=true', '-p', 'timeout:=5.0'],
                                 text=True, capture_output=True, timeout=10)
        self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)
        def moving():
            base.publish(command); gripper.publish(Float64(data=0.01))
            return self.base_out is not None and self.base_out > 0.02 and self.gripper_out == 0.01
        self.wait(moving)
        self.wait(lambda: self.base_out == 0.0 and self.gripper_out == 0.0, 2)
        self.wait(lambda: abs(self.velocities['left_front_wheel_joint']) < 1e-6, 2)
        # Generated ROS2 action contract executes across both hardware components.
        client, handle = self.goal(0.04)
        self.assertEqual(self.result(handle.get_result_async()).result.error_code, 0)
        self.wait(lambda: opening.get_subscription_count() > 0)
        def opened():
            opening.publish(Bool(data=True))
            return self.gripper_out is not None and self.gripper_out > 0.0
        self.wait(opened)
        self.wait(lambda: self.gripper_out == 0.0, 2)
        self.wait(moving)
        command.linear.x = math.nan
        base.publish(command); gripper.publish(Float64(data=math.nan))
        self.wait(lambda: self.base_out == 0.0 and self.gripper_out == 0.0, 2)
        command.linear.x = 0.2
        self.wait(moving)
        self.trigger('halt')
        self.wait(lambda: self.base_out == 0.0 and self.gripper_out == 0.0)
        # Commands sent during Halted must not become targets after resume.
        for _ in range(10):
            base.publish(command); gripper.publish(Float64(data=0.01)); time.sleep(0.02)
        self.trigger('resume'); self.wait(lambda: self.runtime == 'Running')
        time.sleep(0.4)
        self.assertEqual(self.base_out, 0.0); self.assertEqual(self.gripper_out, 0.0)
        self.trigger('shutdown')
        time.sleep(0.1)
        self.assertEqual(self.bad_tf, [], 'Model consumers must never receive non-finite TF')
        checked = subprocess.run(['ros2', 'run', 'robot_bringup', 'check_ready', '--ros-args',
                                  '-p', 'require_running:=true', '-p', 'timeout:=1.0'],
                                 text=True, capture_output=True, timeout=5)
        self.assertNotEqual(checked.returncode, 0, 'Shut down hardware must not pass Running readiness')


if __name__ == '__main__':
    os.environ.setdefault('ROBOT_OFFLINE_REPORT', '/tmp/robot24-full-smoke.log')
    unittest.main(verbosity=2)
