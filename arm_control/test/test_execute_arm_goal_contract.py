#!/usr/bin/env python3

import unittest


def _try_import_action_symbols():
    """The built ROS2 action must expose the preserved public contract."""
    from arm_control.action import ExecuteArmGoal
    from geometry_msgs.msg import PoseStamped
    return (ExecuteArmGoal, ExecuteArmGoal.Goal, ExecuteArmGoal.Result,
            ExecuteArmGoal.Feedback, PoseStamped)


class ExecuteArmGoalContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (
            cls.ExecuteArmGoalAction,
            cls.ExecuteArmGoalGoal,
            cls.ExecuteArmGoalResult,
            cls.ExecuteArmGoalFeedback,
            cls.PoseStamped,
        ) = _try_import_action_symbols()

    def test_action_messages_are_constructible(self):
        self.assertIs(self.ExecuteArmGoalAction.Goal, self.ExecuteArmGoalGoal)
        self.assertIsNotNone(self.ExecuteArmGoalGoal())
        self.assertIsNotNone(self.ExecuteArmGoalResult())
        self.assertIsNotNone(self.ExecuteArmGoalFeedback())

    def test_named_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_NAMED
        goal.named_target = "ready"

        self.assertEqual(goal.target_type, goal.TARGET_NAMED)
        self.assertEqual(goal.named_target, "ready")
        self.assertEqual(list(goal.joint_names), [])
        self.assertEqual(list(goal.joint_positions), [])
        self.assertEqual(goal.pose_target.header.frame_id, "")

    def test_joint_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_JOINTS
        goal.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        goal.joint_positions = [0.0, -0.4, 0.8, 0.0, 0.5, 0.1]

        self.assertEqual(goal.target_type, goal.TARGET_JOINTS)
        self.assertEqual(len(goal.joint_names), 6)
        self.assertEqual(len(goal.joint_positions), 6)
        self.assertEqual(goal.named_target, "")
        self.assertEqual(goal.pose_target.header.frame_id, "")

    def test_pose_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_POSE
        goal.pose_target = self.PoseStamped()
        goal.pose_target.header.frame_id = "base_link"
        goal.pose_target.pose.orientation.w = 1.0
        goal.pose_target.pose.position.x = 0.30
        goal.pose_target.pose.position.y = 0.02
        goal.pose_target.pose.position.z = 0.25

        self.assertEqual(goal.target_type, goal.TARGET_POSE)
        self.assertEqual(goal.pose_target.header.frame_id, "base_link")
        self.assertAlmostEqual(goal.pose_target.pose.orientation.w, 1.0)
        self.assertEqual(goal.named_target, "")
        self.assertEqual(list(goal.joint_names), [])
        self.assertEqual(list(goal.joint_positions), [])

    def test_error_code_constants_exist(self):
        result = self.ExecuteArmGoalResult()
        expected = [
            "ERROR_NONE",
            "ERROR_INVALID_GOAL",
            "ERROR_NOT_READY",
            "ERROR_TF_FAILED",
            "ERROR_PLANNING_FAILED",
            "ERROR_EXECUTION_FAILED",
            "ERROR_PREEMPTED",
        ]

        for value, name in enumerate(expected):
            self.assertEqual(getattr(result, name), value)
            self.assertTrue(
                hasattr(result, name),
                "missing ExecuteArmGoalResult constant %s" % name,
            )


if __name__ == "__main__":
    unittest.main()
