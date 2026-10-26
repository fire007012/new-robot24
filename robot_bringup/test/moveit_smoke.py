#!/usr/bin/env python3
"""Run the installed MoveIt 2 and project action server on mock dual backends."""
import time
import os
from pathlib import Path
import unittest
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from arm_control.action import ExecuteArmGoal
from controller_manager_msgs.srv import ListControllers
from full_system_smoke import FullSystemSmoke


class MoveItSmoke(unittest.TestCase):
    PACKAGE = 'robot_bringup'
    LAUNCH = 'full_system_mock.launch.py'
    ARGUMENTS = ['journal:=/tmp/robot24-moveit-smoke-maintenance.json', 'enable_moveit:=true']
    DOMAIN = '86'
    setUpClass = classmethod(FullSystemSmoke.setUpClass.__func__)
    @classmethod
    def tearDownClass(cls):
        FullSystemSmoke.tearDownClass.__func__(cls)
        log = Path(os.environ.get('ROBOT_OFFLINE_REPORT', '/tmp/robot-hybrid-smoke.log')).read_text()
        if 'exit code -11' in log or 'exit code -6' in log:
            raise AssertionError('Process crashed during shutdown:\n' + log[-8000:])
    joint_state = classmethod(FullSystemSmoke.joint_state.__func__)
    runtime_state = classmethod(FullSystemSmoke.runtime_state.__func__)
    wait = FullSystemSmoke.wait
    result = FullSystemSmoke.result
    call = FullSystemSmoke.call
    trigger = FullSystemSmoke.trigger
    prepare = FullSystemSmoke.prepare
    conditions = FullSystemSmoke.conditions

    def goal(self, kind='named', value='zero'):
        goal = ExecuteArmGoal.Goal()
        if kind == 'named':
            goal.target_type = goal.TARGET_NAMED
            goal.named_target = value
        elif kind == 'pose':
            goal.target_type = goal.TARGET_POSE
            goal.pose_target = value
        else:
            goal.target_type = goal.TARGET_JOINTS
            goal.joint_names = ['shoulder_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                                'wrist_pitch_joint', 'wrist_yaw_joint', 'wrist_roll_joint']
            goal.joint_positions = [float(value)] * 6
        return self.result(self.client.send_goal_async(goal, feedback_callback=self.feedback))

    def feedback(self, message):
        self.phase = message.feedback.phase

    def completed(self, handle):
        self.assertTrue(handle.accepted)
        return self.result(handle.get_result_async(), timeout=90).result

    def test_planning_execution_and_cancel(self):
        self.phase = 0
        self.client = ActionClient(self.node, ExecuteArmGoal, '/arm_control/execute_goal')
        try:
            self.assertTrue(self.client.wait_for_server(timeout_sec=45), self.log.read_text()[-10000:])
            result = self.completed(self.goal(value='does_not_exist'))
            self.assertEqual(result.error_code, result.ERROR_INVALID_GOAL, result.message)
            result = self.completed(self.goal())
            self.assertEqual(result.error_code, result.ERROR_NOT_READY, result.message)
            self.prepare()
            self.trigger('init')
            self.conditions()
            for operation in ('enable', 'resume'):
                self.trigger(operation)
            self.wait(lambda: self.runtime == 'Running')
            self.wait(lambda: len(self.positions) >= 13)
            time.sleep(0.5)
            result = self.completed(self.goal())
            self.assertTrue(result.success, result.message + '\n' + self.log.read_text()[-8000:])
            result = self.completed(self.goal('joints', 0.04))
            self.assertTrue(result.success, result.message + '\n' + self.log.read_text()[-8000:])
            for name in ('shoulder_yaw_joint', 'wrist_roll_joint'):
                self.assertAlmostEqual(self.positions[name], 0.04, delta=0.01)
            from std_srvs.srv import Trigger
            wrist=self.call('/arm_control/wrist_roll_zero',Trigger,Trigger.Request())
            self.assertTrue(wrist.success,wrist.message)
            self.assertAlmostEqual(self.positions['wrist_roll_joint'],0.0,delta=.01)
            from geometry_msgs.msg import PoseStamped
            buffer = Buffer()
            listener = TransformListener(buffer, self.node)
            self.wait(lambda: buffer.can_transform('base_link', 'grasp_tcp', Time()))
            transform = buffer.lookup_transform('base_link', 'grasp_tcp', Time())
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            result = self.completed(self.goal('pose', pose))
            self.assertTrue(result.success, result.message)
            pose.header.frame_id = 'missing_calibration_frame'
            result = self.completed(self.goal('pose', pose))
            self.assertEqual(result.error_code, result.ERROR_TF_FAILED)
            before = self.positions['shoulder_yaw_joint']
            self.phase = 0
            handle = self.goal('joints', 0.2)
            self.assertTrue(handle.accepted)
            duplicate = self.goal()
            self.assertFalse(duplicate.accepted, 'Concurrent project goals must be rejected')
            self.wait(lambda: self.phase == ExecuteArmGoal.Feedback.PHASE_EXECUTING and
                      abs(self.positions['shoulder_yaw_joint'] - before) > 0.002, 10)
            self.assertTrue(self.result(handle.cancel_goal_async()).goals_canceling)
            result = self.completed(handle)
            self.assertFalse(result.success)
            self.assertEqual(result.error_code, result.ERROR_PREEMPTED)
            self.phase = 0
            handle = self.goal('joints', 0.25)
            self.assertTrue(handle.accepted)
            self.wait(lambda: self.phase == ExecuteArmGoal.Feedback.PHASE_EXECUTING, 10)
            self.trigger('halt')
            result = self.completed(handle)
            self.assertFalse(result.success)
            self.wait(lambda: abs(self.velocities['shoulder_yaw_joint']) < 1e-6)
            result = self.completed(self.goal())
            self.assertEqual(result.error_code, result.ERROR_NOT_READY)
            self.trigger('shutdown')
        finally:
            # Node destruction after executor.shutdown owns client cleanup.
            pass


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(MoveItSmoke)
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
