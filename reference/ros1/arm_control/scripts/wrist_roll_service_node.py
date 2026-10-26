#!/usr/bin/env python3

import math
import sys
import threading

import moveit_commander
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse


DEFAULT_ARM_JOINTS = [
    "shoulder_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_yaw_joint",
    "wrist_roll_joint",
]


class WristRollServiceNode(object):
    def __init__(self):
        self.group_name = rospy.get_param("~group_name", "arm")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.4))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.4))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.num_planning_attempts = int(rospy.get_param("~num_planning_attempts", 1))
        self.moveit_wait_for_servers = float(
            rospy.get_param("~moveit_wait_for_servers", 5.0)
        )
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self.joint_states_timeout = float(rospy.get_param("~joint_states_timeout", 2.0))
        self.roll_joint_name = rospy.get_param("~roll_joint_name", "wrist_roll_joint")
        self.roll_delta_rad = float(rospy.get_param("~roll_delta_rad", math.pi))
        self.limit_lower = float(rospy.get_param("~limit_lower", -7.069))
        self.limit_upper = float(rospy.get_param("~limit_upper", 7.069))
        self.skip_limit_check = bool(rospy.get_param("~skip_limit_check", False))
        self.positive_service_name = rospy.get_param(
            "~positive_service_name",
            "/arm_control/wrist_roll_positive_180",
        )
        self.negative_service_name = rospy.get_param(
            "~negative_service_name",
            "/arm_control/wrist_roll_negative_180",
        )
        self.zero_service_name = rospy.get_param(
            "~zero_service_name",
            "/arm_control/wrist_roll_zero",
        )

        if self.roll_joint_name not in DEFAULT_ARM_JOINTS:
            raise ValueError(
                "roll_joint_name '{}' must be one of {}".format(
                    self.roll_joint_name, DEFAULT_ARM_JOINTS
                )
            )

        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = None
        self.request_lock = threading.Lock()

        self.positive_service = rospy.Service(
            self.positive_service_name, Trigger, self.handle_positive_180
        )
        self.negative_service = rospy.Service(
            self.negative_service_name, Trigger, self.handle_negative_180
        )
        self.zero_service = rospy.Service(
            self.zero_service_name, Trigger, self.handle_return_to_zero
        )

        rospy.loginfo(
            "wrist_roll_service_node started: group=%s joint=%s services=[%s, %s, %s]",
            self.group_name,
            self.roll_joint_name,
            self.positive_service_name,
            self.negative_service_name,
            self.zero_service_name,
        )

    def handle_positive_180(self, _request):
        return self.handle_request(
            command_name="wrist_roll_positive_180",
            target_roll=None,
            delta_rad=self.roll_delta_rad,
        )

    def handle_negative_180(self, _request):
        return self.handle_request(
            command_name="wrist_roll_negative_180",
            target_roll=None,
            delta_rad=-self.roll_delta_rad,
        )

    def handle_return_to_zero(self, _request):
        return self.handle_request(
            command_name="wrist_roll_zero",
            target_roll=0.0,
            delta_rad=None,
        )

    def handle_request(self, command_name, target_roll, delta_rad):
        if not self.request_lock.acquire(False):
            return TriggerResponse(
                success=False,
                message="wrist roll service node is busy handling another request",
            )

        try:
            target_joint_map, current_roll, resolved_target_roll = self.build_target_joint_map(
                target_roll=target_roll,
                delta_rad=delta_rad,
            )
            move_group = self.ensure_move_group()

            rospy.loginfo(
                "%s request: %s %.6f -> %.6f",
                command_name,
                self.roll_joint_name,
                current_roll,
                resolved_target_roll,
            )
            move_group.set_start_state_to_current_state()
            move_group.set_joint_value_target(target_joint_map)
            success = bool(move_group.go(wait=True))
            move_group.stop()
            move_group.clear_pose_targets()

            if not success:
                return TriggerResponse(
                    success=False,
                    message="{} failed: MoveIt go(wait=True) returned false".format(
                        command_name
                    ),
                )

            return TriggerResponse(
                success=True,
                message=(
                    "{} succeeded: {} {:.6f} -> {:.6f}".format(
                        command_name,
                        self.roll_joint_name,
                        current_roll,
                        resolved_target_roll,
                    )
                ),
            )
        except ValueError as exc:
            rospy.logwarn("%s rejected: %s", command_name, exc)
            return TriggerResponse(success=False, message=str(exc))
        except Exception as exc:
            rospy.logerr("%s failed with unexpected error: %s", command_name, exc)
            return TriggerResponse(
                success=False,
                message="{} unexpected error: {}".format(command_name, exc),
            )
        finally:
            self.request_lock.release()

    def build_target_joint_map(self, target_roll, delta_rad):
        joint_map = self.read_current_joint_map()
        missing = [name for name in DEFAULT_ARM_JOINTS if name not in joint_map]
        if missing:
            raise ValueError(
                "JointState on {} is missing active arm joints: {}".format(
                    self.joint_states_topic, missing
                )
            )

        current_roll = float(joint_map[self.roll_joint_name])
        if target_roll is None:
            resolved_target_roll = current_roll + float(delta_rad)
        else:
            resolved_target_roll = float(target_roll)

        if not self.skip_limit_check:
            if not (self.limit_lower <= resolved_target_roll <= self.limit_upper):
                raise ValueError(
                    "target {}={} rad exceeds local limit check [{}, {}]".format(
                        self.roll_joint_name,
                        resolved_target_roll,
                        self.limit_lower,
                        self.limit_upper,
                    )
                )

        target_joint_map = {
            name: float(joint_map[name]) for name in DEFAULT_ARM_JOINTS
        }
        target_joint_map[self.roll_joint_name] = resolved_target_roll
        return target_joint_map, current_roll, resolved_target_roll

    def ensure_move_group(self):
        if self.move_group is not None:
            return self.move_group

        try:
            try:
                move_group = moveit_commander.MoveGroupCommander(
                    self.group_name,
                    wait_for_servers=self.moveit_wait_for_servers,
                )
            except TypeError:
                move_group = moveit_commander.MoveGroupCommander(self.group_name)
            move_group.set_pose_reference_frame(self.reference_frame)
            move_group.set_max_velocity_scaling_factor(self.velocity_scaling)
            move_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
            move_group.set_planning_time(self.planning_time)
            move_group.set_num_planning_attempts(self.num_planning_attempts)
            self.move_group = move_group
        except Exception as exc:
            raise ValueError(
                "failed to initialize MoveIt group '{}' within {:.3f}s: {}".format(
                    self.group_name,
                    self.moveit_wait_for_servers,
                    exc,
                )
            )
        return self.move_group

    def read_current_joint_map(self):
        try:
            msg = rospy.wait_for_message(
                self.joint_states_topic,
                JointState,
                timeout=self.joint_states_timeout,
            )
        except rospy.ROSException as exc:
            raise ValueError(
                "failed to read current JointState from {} within {:.3f}s: {}".format(
                    self.joint_states_topic,
                    self.joint_states_timeout,
                    exc,
                )
            )

        if len(msg.name) != len(msg.position):
            raise ValueError(
                "JointState on {} has mismatched name/position lengths: {} vs {}".format(
                    self.joint_states_topic,
                    len(msg.name),
                    len(msg.position),
                )
            )

        joint_map = {}
        for name, position in zip(msg.name, msg.position):
            if name not in joint_map:
                joint_map[name] = float(position)
        return joint_map


def main():
    rospy.init_node("wrist_roll_service_node")
    WristRollServiceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
