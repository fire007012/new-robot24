#!/usr/bin/env python3

import sys
import threading

import actionlib
import moveit_commander
import rospy
import tf2_geometry_msgs
import tf2_ros
from control_msgs.msg import FollowJointTrajectoryAction
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped

from arm_control.msg import (
    ExecuteArmGoalAction,
    ExecuteArmGoalFeedback,
    ExecuteArmGoalResult,
)


class GoalPreparationError(Exception):
    def __init__(self, error_name, default_error, message):
        super(GoalPreparationError, self).__init__(message)
        self.error_name = error_name
        self.default_error = default_error
        self.message = message


class ArmGoalExecutorNode(object):
    TARGET_JOINTS = 1
    TARGET_NAMED = 2
    TARGET_POSE = 3

    ERROR_NONE = 0
    ERROR_INVALID_GOAL = 1
    ERROR_NOT_READY = 2
    ERROR_TF_FAILED = 3
    ERROR_PLANNING_FAILED = 4
    ERROR_EXECUTION_FAILED = 5
    ERROR_PREEMPTED = 6

    PHASE_VALIDATING = 1
    PHASE_WAITING_READY = 2
    PHASE_PLANNING = 3
    PHASE_EXECUTING = 4
    PHASE_STOPPING = 5

    def __init__(self):
        self.action_name = rospy.get_param("~action_name", "/arm_control/execute_goal")
        self.group_name = rospy.get_param("~group_name", "arm")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.end_effector_link = rospy.get_param("~end_effector_link", "")
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.2))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.2))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.num_planning_attempts = int(rospy.get_param("~num_planning_attempts", 1))
        self.ready_timeout = float(rospy.get_param("~ready_timeout", 2.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.2))
        self.controller_manager_ns = rospy.get_param(
            "~controller_manager_ns", "/controller_manager"
        ).rstrip("/")
        self.joint_state_controller = rospy.get_param(
            "~joint_state_controller", "joint_state_controller"
        )
        self.arm_controller = rospy.get_param("~arm_controller", "arm_position_controller")
        self.arm_action_ns = rospy.get_param(
            "~arm_action_ns", "/arm_position_controller/follow_joint_trajectory"
        )

        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_pose_reference_frame(self.reference_frame)
        if self.end_effector_link:
            self.group.set_end_effector_link(self.end_effector_link)
        self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.num_planning_attempts)

        self.active_joints = list(self.group.get_active_joints())
        self.named_targets = set(self.group.get_named_targets())

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.list_controllers_srv_name = (
            self.controller_manager_ns + "/list_controllers"
        )
        self.list_controllers = rospy.ServiceProxy(
            self.list_controllers_srv_name, ListControllers
        )
        self.arm_action_client = actionlib.SimpleActionClient(
            self.arm_action_ns, FollowJointTrajectoryAction
        )

        self._preempt_requested = threading.Event()
        self._group_lock = threading.RLock()

        self.server = actionlib.SimpleActionServer(
            self.action_name,
            ExecuteArmGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        rospy.loginfo(
            "arm_goal_executor_node started: action=%s group=%s ref=%s ee=%s",
            self.action_name,
            self.group_name,
            self.reference_frame,
            self.end_effector_link or "(MoveIt default)",
        )
        rospy.loginfo(
            "arm_goal_executor_node active_joints=%s named_targets=%s",
            self.active_joints,
            sorted(self.named_targets),
        )

    def _const(self, msg_or_cls, name, default):
        return getattr(msg_or_cls, name, default)

    def _target_const(self, goal, name, default):
        return self._const(goal, name, self._const(type(goal), name, default))

    def _error_const(self, name, default):
        return self._const(ExecuteArmGoalResult, name, default)

    def _phase_const(self, name, default):
        return self._const(ExecuteArmGoalFeedback, name, default)

    def publish_feedback(self, phase, message):
        feedback = ExecuteArmGoalFeedback()
        feedback.phase = phase
        feedback.message = message
        self.server.publish_feedback(feedback)

    def make_result(self, success, error_code, message):
        result = ExecuteArmGoalResult()
        result.success = bool(success)
        result.error_code = error_code
        result.message = message
        return result

    def set_succeeded(self, message):
        result = self.make_result(
            True, self._error_const("ERROR_NONE", self.ERROR_NONE), message
        )
        self.server.set_succeeded(result, message)

    def set_aborted(self, error_name, default_error, message):
        result = self.make_result(
            False, self._error_const(error_name, default_error), message
        )
        self.server.set_aborted(result, message)

    def set_preempted(self, message):
        result = self.make_result(
            False,
            self._error_const("ERROR_PREEMPTED", self.ERROR_PREEMPTED),
            message,
        )
        self.server.set_preempted(result, message)

    def preempt_cb(self):
        self._preempt_requested.set()
        self.publish_feedback(
            self._phase_const("STOPPING", self.PHASE_STOPPING),
            "preempt requested; stopping MoveIt group",
        )
        try:
            with self._group_lock:
                self.group.stop()
        except Exception as exc:
            rospy.logwarn("failed to stop MoveIt group on preempt: %s", exc)

    def execute_cb(self, goal):
        self._preempt_requested.clear()
        self.publish_feedback(
            self._phase_const("VALIDATING", self.PHASE_VALIDATING),
            "validating arm goal",
        )

        try:
            prepared = self.prepare_goal(goal)
        except GoalPreparationError as exc:
            self.set_aborted(exc.error_name, exc.default_error, exc.message)
            return
        except Exception as exc:
            self.set_aborted(
                "ERROR_INVALID_GOAL",
                self.ERROR_INVALID_GOAL,
                "invalid goal: %s" % exc,
            )
            return

        if self.check_preempt("preempted before readiness check"):
            return

        self.publish_feedback(
            self._phase_const("WAITING_READY", self.PHASE_WAITING_READY),
            "checking controller readiness",
        )
        ready, ready_msg = self.check_ready()
        if not ready:
            self.set_aborted("ERROR_NOT_READY", self.ERROR_NOT_READY, ready_msg)
            return

        if self.check_preempt("preempted before planning"):
            return

        self.publish_feedback(
            self._phase_const("PLANNING", self.PHASE_PLANNING),
            "planning arm trajectory",
        )
        try:
            plan = self.plan_goal(prepared)
        except Exception as exc:
            self.set_aborted(
                "ERROR_PLANNING_FAILED",
                self.ERROR_PLANNING_FAILED,
                "planning failed: %s" % exc,
            )
            return

        if self.check_preempt("preempted before execution"):
            return

        self.publish_feedback(
            self._phase_const("EXECUTING", self.PHASE_EXECUTING),
            "executing arm trajectory",
        )
        try:
            with self._group_lock:
                success = self.group.execute(plan, wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
        except Exception as exc:
            self.set_aborted(
                "ERROR_EXECUTION_FAILED",
                self.ERROR_EXECUTION_FAILED,
                "execution failed: %s" % exc,
            )
            return

        if self.check_preempt("preempted during execution"):
            return

        if not success:
            self.set_aborted(
                "ERROR_EXECUTION_FAILED",
                self.ERROR_EXECUTION_FAILED,
                "MoveIt execute() returned false",
            )
            return

        self.set_succeeded("arm goal executed successfully")

    def check_preempt(self, message):
        if not self._preempt_requested.is_set() and not self.server.is_preempt_requested():
            return False

        self.publish_feedback(
            self._phase_const("STOPPING", self.PHASE_STOPPING), message
        )
        try:
            with self._group_lock:
                self.group.stop()
                self.group.clear_pose_targets()
        except Exception as exc:
            rospy.logwarn("failed to cleanly stop MoveIt group after preempt: %s", exc)
        self.set_preempted(message)
        return True

    def prepare_goal(self, goal):
        target_type = int(goal.target_type)
        target_joints = self._target_const(goal, "TARGET_JOINTS", self.TARGET_JOINTS)
        target_named = self._target_const(goal, "TARGET_NAMED", self.TARGET_NAMED)
        target_pose = self._target_const(goal, "TARGET_POSE", self.TARGET_POSE)

        if target_type == target_joints:
            return ("joints", self.prepare_joint_goal(goal))
        if target_type == target_named:
            return ("named", self.prepare_named_goal(goal))
        if target_type == target_pose:
            return ("pose", self.prepare_pose_goal(goal))

        raise ValueError("unknown target_type=%s" % target_type)

    def prepare_joint_goal(self, goal):
        joint_names = list(goal.joint_names)
        joint_positions = list(goal.joint_positions)

        if len(joint_names) != len(joint_positions):
            raise ValueError(
                "joint_names length %d does not match joint_positions length %d"
                % (len(joint_names), len(joint_positions))
            )
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("joint_names contains duplicate names")

        expected = set(self.active_joints)
        incoming = set(joint_names)
        if incoming != expected:
            missing = sorted(expected - incoming)
            extra = sorted(incoming - expected)
            raise ValueError(
                "joint_names must exactly match active joints; missing=%s extra=%s"
                % (missing, extra)
            )

        by_name = dict(zip(joint_names, joint_positions))
        return [float(by_name[name]) for name in self.active_joints]

    def prepare_named_goal(self, goal):
        target = goal.named_target.strip()
        if not target:
            raise ValueError("named_target is empty")

        self.named_targets = set(self.group.get_named_targets())
        if target not in self.named_targets:
            raise ValueError(
                "unknown named_target=%s; available=%s"
                % (target, sorted(self.named_targets))
            )
        return target

    def prepare_pose_goal(self, goal):
        pose = goal.pose_target
        source_frame = pose.header.frame_id.strip()
        if not source_frame:
            raise GoalPreparationError(
                "ERROR_TF_FAILED",
                self.ERROR_TF_FAILED,
                "pose_target.header.frame_id is empty",
            )

        if source_frame == self.reference_frame:
            target_pose = PoseStamped()
            target_pose.header = pose.header
            target_pose.pose = pose.pose
            return target_pose

        stamp = pose.header.stamp if pose.header.stamp != rospy.Time() else rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                source_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            raise GoalPreparationError(
                "ERROR_TF_FAILED",
                self.ERROR_TF_FAILED,
                "failed to transform pose from %s to %s: %s"
                % (source_frame, self.reference_frame, exc),
            )

    def check_ready(self):
        timeout = rospy.Duration(self.ready_timeout)
        try:
            rospy.wait_for_service(self.list_controllers_srv_name, timeout.to_sec())
        except rospy.ROSException as exc:
            return (
                False,
                "controller manager service %s unavailable: %s"
                % (self.list_controllers_srv_name, exc),
            )

        try:
            response = self.list_controllers()
        except rospy.ServiceException as exc:
            return (
                False,
                "failed to call %s: %s" % (self.list_controllers_srv_name, exc),
            )

        states = {controller.name: controller.state for controller in response.controller}
        for name in (self.joint_state_controller, self.arm_controller):
            state = states.get(name)
            if state != "running":
                return (
                    False,
                    "controller %s is not running; current state=%s"
                    % (name, state or "<missing>"),
                )

        if not self.arm_action_client.wait_for_server(timeout):
            return (
                False,
                "arm action server %s unavailable within %.3fs"
                % (self.arm_action_ns, self.ready_timeout),
            )

        return True, "ready"

    def plan_goal(self, prepared):
        target_kind, target = prepared
        with self._group_lock:
            self.group.clear_pose_targets()
            self.group.clear_path_constraints()

            if target_kind == "joints":
                self.group.set_joint_value_target(target)
            elif target_kind == "named":
                self.group.set_named_target(target)
            elif target_kind == "pose":
                if self.end_effector_link:
                    self.group.set_pose_target(target, self.end_effector_link)
                else:
                    self.group.set_pose_target(target)
            else:
                raise ValueError("unsupported prepared target kind=%s" % target_kind)

            plan_result = self.group.plan()

        success, plan, detail = self.normalize_plan_result(plan_result)
        if not success:
            raise RuntimeError("MoveIt plan() failed: %s" % detail)
        if not self.plan_has_points(plan):
            raise RuntimeError("MoveIt plan() returned an empty trajectory")
        return plan

    def normalize_plan_result(self, plan_result):
        if isinstance(plan_result, tuple):
            success = bool(plan_result[0]) if len(plan_result) > 0 else False
            plan = plan_result[1] if len(plan_result) > 1 else None
            detail = "success=%s" % success
            if len(plan_result) > 3:
                detail = "success=%s error_code=%s" % (
                    success,
                    getattr(plan_result[3], "val", plan_result[3]),
                )
            return success, plan, detail

        plan = plan_result
        return self.plan_has_points(plan), plan, "legacy plan result"

    def plan_has_points(self, plan):
        trajectory = getattr(plan, "joint_trajectory", None)
        points = getattr(trajectory, "points", None)
        return bool(points)


def main():
    rospy.init_node("arm_goal_executor_node")
    ArmGoalExecutorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
