#!/usr/bin/env python3
import math

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class GazeboModelOdomNode(object):
    def __init__(self):
        self.model_states_topic = rospy.get_param('~model_states_topic', '/gazebo/model_states')
        self.model_name = rospy.get_param('~model_name', 'robot')
        self.odom_topic = rospy.get_param('~odom_topic', '/car_urdf/odom')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link_root')
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))
        self.publish_rate = float(rospy.get_param('~publish_rate', 50.0))

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.last_publish_time = rospy.Time(0)
        self.publish_period = rospy.Duration(0.0 if self.publish_rate <= 0.0 else 1.0 / self.publish_rate)

        self.sub = rospy.Subscriber(self.model_states_topic, ModelStates, self.states_cb, queue_size=1)
        rospy.loginfo('gazebo_model_odom_node started: %s[%s] -> %s',
                      self.model_states_topic, self.model_name, self.odom_topic)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def states_cb(self, msg):
        now = rospy.Time.now()
        if self.publish_period.to_sec() > 0.0 and (now - self.last_publish_time) < self.publish_period:
            return

        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(2.0, 'gazebo_model_odom_node: model %s not found in %s',
                                   self.model_name, self.model_states_topic)
            return

        pose = msg.pose[index]
        twist = msg.twist[index]
        yaw = self.yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose

        # Gazebo model_states gives world-frame velocities; rotate planar terms
        # back into the base frame so downstream ROS consumers see standard odom.
        odom.twist.twist.linear.x = cos_yaw * twist.linear.x + sin_yaw * twist.linear.y
        odom.twist.twist.linear.y = -sin_yaw * twist.linear.x + cos_yaw * twist.linear.y
        odom.twist.twist.linear.z = twist.linear.z
        odom.twist.twist.angular.x = cos_yaw * twist.angular.x + sin_yaw * twist.angular.y
        odom.twist.twist.angular.y = -sin_yaw * twist.angular.x + cos_yaw * twist.angular.y
        odom.twist.twist.angular.z = twist.angular.z

        odom.pose.covariance = [
            1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-4, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e12, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e12, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e12, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2,
        ]
        odom.twist.covariance = list(odom.pose.covariance)
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        self.last_publish_time = now


if __name__ == '__main__':
    rospy.init_node('gazebo_model_odom_node')
    GazeboModelOdomNode()
    rospy.spin()
