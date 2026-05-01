#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperCmdNode(object):
    def __init__(self):
        self.position_topic = rospy.get_param(
            "~position_topic", "/arm_control/gripper_position"
        )
        self.open_close_topic = rospy.get_param(
            "~open_close_topic", "/arm_control/gripper_open"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/gripper_controller/command"
        )
        self.joint_name = rospy.get_param("~joint_name", "left_gripper_finger_joint")
        self.min_pos = float(rospy.get_param("~min_position", 0.0))
        self.max_pos = float(rospy.get_param("~max_position", 0.071))
        self.open_pos = float(rospy.get_param("~open_position", 0.071))
        self.close_pos = float(rospy.get_param("~close_position", 0.0))
        self.duration = float(rospy.get_param("~move_duration", 0.8))

        self.pub = rospy.Publisher(self.output_topic, JointTrajectory, queue_size=10)
        self.sub_pos = rospy.Subscriber(
            self.position_topic, Float64, self.pos_cb, queue_size=10
        )
        self.sub_open = rospy.Subscriber(
            self.open_close_topic, Bool, self.open_cb, queue_size=10
        )

        rospy.loginfo(
            "gripper_cmd_node started: %s -> %s",
            self.position_topic,
            self.output_topic,
        )
        rospy.loginfo(
            "gripper_cmd_node open/close topic: %s", self.open_close_topic
        )

    def clamp(self, val):
        return max(self.min_pos, min(self.max_pos, val))

    def publish_target(self, target):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.clamp(target)]
        point.time_from_start = rospy.Duration.from_sec(self.duration)

        traj.points = [point]
        self.pub.publish(traj)

    def pos_cb(self, msg):
        self.publish_target(msg.data)

    def open_cb(self, msg):
        self.publish_target(self.open_pos if msg.data else self.close_pos)


if __name__ == "__main__":
    rospy.init_node("gripper_cmd_node")
    GripperCmdNode()
    rospy.spin()
