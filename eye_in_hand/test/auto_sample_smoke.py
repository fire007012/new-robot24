#!/usr/bin/env python3
"""Run the installed sampler against synthetic project motion and marker services."""
import os
from pathlib import Path
import subprocess
import tempfile
from threading import Thread
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml

from arm_control.action import ExecuteArmGoal
from easy_handeye2_msgs.msg import Sample
from easy_handeye2_msgs.srv import TakeSample, ComputeCalibration, SaveCalibration


class AutoSampleSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.update(ROS_DOMAIN_ID='94', ROS_LOCALHOST_ONLY='1')
        rclpy.init()
        cls.node = Node('synthetic_calibration_fixture')
        cls.executor = MultiThreadedExecutor(num_threads=3)
        cls.executor.add_node(cls.node)
        cls.thread = Thread(target=cls.executor.spin)
        cls.goals = []
        cls.samples = []
        cls.calls = []
        cls.publish_marker = True
        cls.motion = ActionServer(cls.node, ExecuteArmGoal, '/arm_control/execute_goal', cls.move)
        prefix = '/easy_handeye2/calibration/'
        cls.services = [
            cls.node.create_service(TakeSample, prefix + 'get_sample_list', cls.sample_list),
            cls.node.create_service(TakeSample, prefix + 'take_sample', cls.sample),
            cls.node.create_service(ComputeCalibration, prefix + 'compute_calibration', cls.compute),
            cls.node.create_service(SaveCalibration, prefix + 'save_calibration', cls.save),
        ]
        cls.marker = cls.node.create_publisher(PoseStamped, '/aruco_single/pose', 10)
        cls.timer = cls.node.create_timer(0.03, cls.broadcast)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.thread.join()
        cls.motion.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def broadcast(cls):
        if cls.publish_marker:
            pose = PoseStamped()
            pose.header.stamp = cls.node.get_clock().now().to_msg()
            pose.header.frame_id = 'paw_camera_color_optical_frame'
            pose.pose.orientation.w = 1.0
            cls.marker.publish(pose)

    @classmethod
    def move(cls, handle):
        cls.goals.append(handle.request)
        handle.succeed()
        return ExecuteArmGoal.Result(success=True)

    @classmethod
    def sample_list(cls, _, response):
        response.samples.samples = cls.samples
        return response

    @classmethod
    def sample(cls, request, response):
        cls.samples.append(Sample())
        return cls.sample_list(request, response)

    @classmethod
    def compute(cls, _, response):
        cls.calls.append('compute')
        response.valid = True
        return response

    @classmethod
    def save(cls, _, response):
        cls.calls.append('save')
        response.success = True
        response.filepath.data = 'synthetic-calibration.yaml'
        return response

    def setUp(self):
        self.goals.clear()
        self.samples.clear()
        self.calls.clear()
        type(self).publish_marker = True

    def run_sampler(self, commissioned=True):
        names = ['shoulder_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                 'wrist_pitch_joint', 'wrist_yaw_joint', 'wrist_roll_joint']
        config = dict(commissioned=commissioned, joint_names=names, home_positions=[0.0]*6,
                      settle_time=0.1, marker_timeout=0.5,
                      poses=[dict(name='first', delta=[0.01]*6),
                             dict(name='second', delta=[0.02]*6)])
        executable = Path(get_package_prefix('eye_in_hand')) / 'lib/eye_in_hand/auto_sample.py'
        with tempfile.TemporaryDirectory(prefix='robot24-auto-sample-') as folder:
            path = Path(folder) / 'synthetic-poses.yaml'
            path.write_text(yaml.safe_dump(config))
            return subprocess.run(['python3', str(executable), '--ros-args', '-p',
                                   'config_path:=' + str(path)],
                                  text=True, capture_output=True, timeout=25)

    def test_project_goals_samples_and_save(self):
        result = self.run_sampler()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(self.samples), 2)
        self.assertEqual(self.calls, ['compute', 'save'])
        self.assertEqual(len(self.goals), 2)
        for goal, position in zip(self.goals, (0.01, 0.02)):
            self.assertEqual(goal.target_type, ExecuteArmGoal.Goal.TARGET_JOINTS)
            self.assertEqual(list(goal.joint_positions), [position]*6)

    def test_missing_marker_stops_before_sampling_or_second_motion(self):
        type(self).publish_marker = False
        result = self.run_sampler()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('no fresh marker', result.stderr)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.samples, [])
        self.assertEqual(self.calls, [])

    def test_uncommissioned_poses_never_command_motion(self):
        result = self.run_sampler(commissioned=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not commissioned', result.stderr)
        self.assertEqual(self.goals, [])
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
