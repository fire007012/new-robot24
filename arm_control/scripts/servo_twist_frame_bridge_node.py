#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TwistStamped


def rotate_vector(vec, quat):
    qx = quat.x
    qy = quat.y
    qz = quat.z
    qw = quat.w

    # v' = v + 2 * cross(q_vec, cross(q_vec, v) + w * v)
    tx = 2.0 * (qy * vec[2] - qz * vec[1])
    ty = 2.0 * (qz * vec[0] - qx * vec[2])
    tz = 2.0 * (qx * vec[1] - qy * vec[0])

    return (
        vec[0] + qw * tx + (qy * tz - qz * ty),
        vec[1] + qw * ty + (qz * tx - qx * tz),
        vec[2] + qw * tz + (qx * ty - qy * tx),
    )


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


class ServoTwistFrameBridgeNode(object):
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/arm_control/delta_twist_cmds_optical"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/servo_server/delta_twist_cmds"
        )
        self.target_frame = rospy.get_param("~target_frame", "catch_camera")
        self.expected_source_frame = rospy.get_param(
            "~expected_source_frame", "catch_camera_optical_frame"
        )
        self.transform_timeout = float(rospy.get_param("~transform_timeout", 0.05))
        self.drop_unexpected_source_frame = bool(
            rospy.get_param("~drop_unexpected_source_frame", True)
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.output_topic, TwistStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            self.input_topic, TwistStamped, self.twist_cb, queue_size=10
        )

        rospy.loginfo(
            "servo_twist_frame_bridge_node started: %s -> %s (%s -> %s)",
            self.input_topic,
            self.output_topic,
            self.expected_source_frame or "<incoming frame>",
            self.target_frame,
        )

    def twist_cb(self, msg):
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            rospy.logwarn_throttle(
                2.0,
                "servo_twist_frame_bridge_node dropped TwistStamped with empty frame_id",
            )
            return

        if self.expected_source_frame and source_frame != self.expected_source_frame:
            rospy.logwarn_throttle(
                2.0,
                "servo_twist_frame_bridge_node received %s but expected %s",
                source_frame,
                self.expected_source_frame,
            )
            if self.drop_unexpected_source_frame:
                return

        if source_frame == self.target_frame:
            out = TwistStamped()
            out.header = msg.header
            out.header.frame_id = self.target_frame
            out.twist = msg.twist
            self.pub.publish(out)
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                rospy.Duration(self.transform_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "servo_twist_frame_bridge_node failed to transform %s -> %s: %s",
                source_frame,
                self.target_frame,
                exc,
            )
            return

        quat = transform.transform.rotation
        translation = transform.transform.translation

        linear_source = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        )
        angular_source = (
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        )

        angular_target = rotate_vector(angular_source, quat)
        linear_target = rotate_vector(linear_source, quat)

        # Spatial twist frame transform: v_t = R v_s + p x (R w_s)
        angular_offset = cross(
            (translation.x, translation.y, translation.z), angular_target
        )
        linear_target = tuple(
            linear_target[i] + angular_offset[i] for i in range(3)
        )

        out = TwistStamped()
        out.header.stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )
        out.header.frame_id = self.target_frame
        out.twist.linear.x = linear_target[0]
        out.twist.linear.y = linear_target[1]
        out.twist.linear.z = linear_target[2]
        out.twist.angular.x = angular_target[0]
        out.twist.angular.y = angular_target[1]
        out.twist.angular.z = angular_target[2]
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("servo_twist_frame_bridge_node")
    ServoTwistFrameBridgeNode()
    rospy.spin()
