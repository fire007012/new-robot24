#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64


class GripperCmdNode(object):
    def __init__(self):
        self.velocity_topic = rospy.get_param(
            "~velocity_topic", "/arm_control/gripper_velocity"
        )
        self.open_close_topic = rospy.get_param(
            "~open_close_topic", "/arm_control/gripper_open"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/gripper_controller/command"
        )
        self.open_velocity = float(rospy.get_param("~open_velocity", 0.8))
        self.close_velocity = float(rospy.get_param("~close_velocity", -0.8))
        self.command_hold_sec = float(rospy.get_param("~command_hold_sec", 0.2))
        self.max_abs_velocity = float(rospy.get_param("~max_abs_velocity", 0.0))
        self.stop_deadline = None
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)

        self.pub = rospy.Publisher(self.output_topic, Float64, queue_size=10)
        self.sub_vel = rospy.Subscriber(
            self.velocity_topic, Float64, self.vel_cb, queue_size=10
        )
        self.sub_open = rospy.Subscriber(
            self.open_close_topic, Bool, self.open_cb, queue_size=10
        )

        rospy.loginfo(
            "gripper_cmd_node started: %s -> %s",
            self.velocity_topic,
            self.output_topic,
        )
        rospy.loginfo(
            "gripper_cmd_node open/close topic: %s", self.open_close_topic
        )
        rospy.loginfo(
            "gripper_cmd_node bool velocities: open=%.3f close=%.3f hold=%.3fs",
            self.open_velocity,
            self.close_velocity,
            self.command_hold_sec,
        )

    def normalize_velocity(self, value):
        velocity = float(value)
        if self.max_abs_velocity > 0.0:
            velocity = max(-self.max_abs_velocity, min(self.max_abs_velocity, velocity))
        return velocity

    def publish_velocity(self, velocity):
        self.pub.publish(Float64(data=self.normalize_velocity(velocity)))

    def vel_cb(self, msg):
        self.stop_deadline = None
        self.publish_velocity(msg.data)

    def open_cb(self, msg):
        velocity = self.open_velocity if msg.data else self.close_velocity
        self.publish_velocity(velocity)
        if self.command_hold_sec > 0.0:
            self.stop_deadline = rospy.Time.now() + rospy.Duration.from_sec(self.command_hold_sec)
        else:
            self.stop_deadline = None

    def timer_cb(self, _event):
        if self.stop_deadline is None:
            return
        if rospy.Time.now() < self.stop_deadline:
            return
        self.stop_deadline = None
        self.publish_velocity(0.0)


if __name__ == "__main__":
    rospy.init_node("gripper_cmd_node")
    GripperCmdNode()
    rospy.spin()
