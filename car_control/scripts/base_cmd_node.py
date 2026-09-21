#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


class BaseCmdNode(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/car_control/cmd_vel')
        self.output_topic = rospy.get_param('~output_topic', '/cmd_vel')
        self.max_linear_x = float(rospy.get_param('~max_linear_x', 0.8))
        self.max_angular_z = float(rospy.get_param('~max_angular_z', 1.5))
        self.timeout_sec = float(rospy.get_param('~timeout_sec', 0.5))

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self.cmd_cb, queue_size=10)

        self.last_cmd_time = rospy.Time(0)
        self.has_cmd = False
        self.timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_cb)

        rospy.loginfo('base_cmd_node started: %s -> %s', self.input_topic, self.output_topic)

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def cmd_cb(self, msg):
        out = Twist()
        out.linear.x = self.clamp(msg.linear.x, -self.max_linear_x, self.max_linear_x)
        out.angular.z = self.clamp(msg.angular.z, -self.max_angular_z, self.max_angular_z)

        self.pub.publish(out)
        self.last_cmd_time = rospy.Time.now()
        self.has_cmd = True

    def watchdog_cb(self, _event):
        if not self.has_cmd:
            return
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.timeout_sec:
            self.pub.publish(Twist())
            self.has_cmd = False


if __name__ == '__main__':
    rospy.init_node('base_cmd_node')
    BaseCmdNode()
    rospy.spin()
