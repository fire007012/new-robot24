#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64


class DS5TeleopNode(object):
    def __init__(self):
        # Topics
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.chassis_topic = rospy.get_param('~chassis_cmd_topic', '/car_control/cmd_vel')
        self.servo_topic = rospy.get_param('~servo_cmd_topic', '/servo_server/delta_twist_cmds')
        self.gripper_velocity_topic = rospy.get_param(
            '~gripper_velocity_topic', '/car_control/gripper_velocity'
        )

        # Servo frame
        self.servo_frame = rospy.get_param('~servo_frame', 'base_link')
        self.servo_command_frame_param = rospy.get_param(
            '~servo_command_frame_param', '/servo_server/robot_link_command_frame'
        )

        # Scales
        self.max_chassis_vx = float(rospy.get_param('~max_chassis_vx', 0.8))
        self.max_chassis_wz = float(rospy.get_param('~max_chassis_wz', 1.5))
        self.chassis_turn_sign = float(rospy.get_param('~chassis_turn_sign', 1.0))
        self.max_arm_linear = float(rospy.get_param('~max_arm_linear', 0.15))
        self.max_arm_angular = float(rospy.get_param('~max_arm_angular', 0.6))
        self.max_gripper_velocity = float(
            rospy.get_param('~max_gripper_velocity', 0.8)
        )
        self.chassis_turn_use_rx_fallback = bool(rospy.get_param('~chassis_turn_use_rx_fallback', False))

        # Safety
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.25))
        self.deadzone = float(rospy.get_param('~deadzone', 0.12))

        # Axis/button indices (configurable for DS4/DS5 differences)
        self.AXIS_L_X = int(rospy.get_param('~axis_lx', 0))
        self.AXIS_L_Y = int(rospy.get_param('~axis_ly', 1))
        self.AXIS_L2 = int(rospy.get_param('~axis_l2', 2))
        self.AXIS_R_X = int(rospy.get_param('~axis_rx', 3))
        self.AXIS_R_Y = int(rospy.get_param('~axis_ry', 4))
        self.AXIS_R2 = int(rospy.get_param('~axis_r2', 5))
        self.AXIS_DPAD_X = int(rospy.get_param('~axis_dpad_x', 6))
        self.AXIS_DPAD_Y = int(rospy.get_param('~axis_dpad_y', 7))
        self.AXIS_L2_RELEASED = float(rospy.get_param('~axis_l2_released', 0.0))
        self.AXIS_L2_PRESSED = float(rospy.get_param('~axis_l2_pressed', -1.0))
        self.AXIS_R2_RELEASED = float(rospy.get_param('~axis_r2_released', 1.0))
        self.AXIS_R2_PRESSED = float(rospy.get_param('~axis_r2_pressed', -1.0))

        self.BTN_SQUARE = int(rospy.get_param('~btn_square', 0))
        self.BTN_CROSS = int(rospy.get_param('~btn_cross', 1))
        self.BTN_CIRCLE = int(rospy.get_param('~btn_circle', 2))
        self.BTN_TRIANGLE = int(rospy.get_param('~btn_triangle', 3))
        self.BTN_L1 = int(rospy.get_param('~btn_l1', 4))
        self.BTN_R1 = int(rospy.get_param('~btn_r1', 5))
        self.BTN_OPTIONS = int(rospy.get_param('~btn_options', 9))
        self.mode_switch_buttons = self.int_list_param('~mode_switch_buttons', [self.BTN_OPTIONS, 10])
        self.chassis_turn_axis_candidates = self.int_list_param(
            '~chassis_turn_axis_candidates',
            [self.AXIS_L_X, self.AXIS_DPAD_X],
        )

        # Mode
        self.MODE_CHASSIS = 0
        self.MODE_ARM = 1
        self.mode = self.MODE_CHASSIS
        self.last_buttons = []

        self.last_joy_time = rospy.Time(0)

        # Pub/Sub
        self.pub_chassis = rospy.Publisher(self.chassis_topic, Twist, queue_size=10)
        self.pub_servo = rospy.Publisher(self.servo_topic, TwistStamped, queue_size=10)
        self.pub_gripper_velocity = rospy.Publisher(
            self.gripper_velocity_topic, Float64, queue_size=10
        )
        self.sub = rospy.Subscriber(self.joy_topic, Joy, self.joy_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_cb)
        self.servo_frame_check_attempts = 0
        self.servo_frame_check_timer = rospy.Timer(
            rospy.Duration(2.0), self.servo_frame_check_cb
        )

        rospy.loginfo(
            'ds5_teleop_node started, mode=CHASSIS, servo_frame=%s, mode_switch_buttons=%s, turn_axis_candidates=%s',
            self.servo_frame,
            self.mode_switch_buttons,
            self.chassis_turn_axis_candidates,
        )
        if self.servo_frame.endswith('_optical_frame'):
            rospy.logwarn(
                'ds5_teleop_node servo_frame=%s looks like an optical frame; '
                'for teleop control you usually want the camera body frame such as catch_camera',
                self.servo_frame,
            )

    def int_list_param(self, name, default):
        raw = rospy.get_param(name, default)
        if isinstance(raw, int):
            return [raw]
        if isinstance(raw, list):
            out = []
            for item in raw:
                try:
                    out.append(int(item))
                except (TypeError, ValueError):
                    pass
            return out if out else list(default)
        if isinstance(raw, str):
            out = []
            for item in raw.split(','):
                item = item.strip()
                if not item:
                    continue
                try:
                    out.append(int(item))
                except ValueError:
                    pass
            return out if out else list(default)
        return list(default)

    def axis(self, msg, idx):
        return self.apply_deadzone(self.axis_raw(msg, idx))

    def axis_raw(self, msg, idx):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return msg.axes[idx]

    def apply_deadzone(self, val):
        return 0.0 if abs(val) < self.deadzone else val

    def button(self, msg, idx):
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return msg.buttons[idx]

    def last_button(self, idx):
        if idx < 0 or idx >= len(self.last_buttons):
            return 0
        return self.last_buttons[idx]

    def button_rising(self, msg, idx):
        return self.button(msg, idx) == 1 and self.last_button(idx) == 0

    def any_button_rising(self, msg, indices):
        for idx in indices:
            if self.button_rising(msg, idx):
                return True
        return False

    def trigger_to_01(self, axis_val, released_val, pressed_val):
        span = pressed_val - released_val
        if abs(span) < 1e-6:
            return 0.0
        normalized = (axis_val - released_val) / span
        normalized = max(0.0, min(1.0, normalized))
        return 0.0 if normalized < self.deadzone else normalized

    def first_active_axis(self, msg, indices):
        for idx in indices:
            candidate = self.axis(msg, idx)
            if abs(candidate) >= self.deadzone:
                return candidate
        return 0.0

    def publish_zero(self):
        self.pub_chassis.publish(Twist())
        self.pub_gripper_velocity.publish(Float64(data=0.0))
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self.servo_frame
        self.pub_servo.publish(ts)

    def joy_cb(self, msg):
        now = rospy.Time.now()
        self.last_joy_time = now

        # Mode switch on configured rising edge(s)
        if self.any_button_rising(msg, self.mode_switch_buttons):
            self.mode = self.MODE_ARM if self.mode == self.MODE_CHASSIS else self.MODE_CHASSIS
            rospy.loginfo('ds5_teleop_node mode=%s', 'ARM_SERVO' if self.mode == self.MODE_ARM else 'CHASSIS')
            self.publish_zero()

        # Gripper hold control: square=open, circle=close
        gripper_velocity = (
            self.button(msg, self.BTN_SQUARE) - self.button(msg, self.BTN_CIRCLE)
        ) * self.max_gripper_velocity
        self.pub_gripper_velocity.publish(Float64(data=gripper_velocity))

        if self.mode == self.MODE_CHASSIS:
            cmd = Twist()
            cmd.linear.x = self.axis(msg, self.AXIS_L_Y) * self.max_chassis_vx
            # Honor candidate priority so LX wins when active; fallback axes
            # only take over when the preferred axis is idle on this driver.
            turn = self.first_active_axis(msg, self.chassis_turn_axis_candidates)
            if self.chassis_turn_use_rx_fallback and abs(turn) < self.deadzone:
                turn = self.axis(msg, self.AXIS_R_X)
            cmd.angular.z = turn * self.max_chassis_wz * self.chassis_turn_sign
            self.pub_chassis.publish(cmd)

            # keep servo side zeroed
            ts = TwistStamped()
            ts.header.stamp = now
            ts.header.frame_id = self.servo_frame
            self.pub_servo.publish(ts)
            self.last_buttons = list(msg.buttons)
            return

        # ARM_SERVO mode
        ts = TwistStamped()
        ts.header.stamp = now
        ts.header.frame_id = self.servo_frame

        # Linear: left stick controls the zy plane, triggers control x.
        ts.twist.linear.y = self.axis(msg, self.AXIS_L_X) * self.max_arm_linear
        ts.twist.linear.z = self.axis(msg, self.AXIS_L_Y) * self.max_arm_linear

        l2 = self.trigger_to_01(
            self.axis_raw(msg, self.AXIS_L2),
            self.AXIS_L2_RELEASED,
            self.AXIS_L2_PRESSED,
        )
        r2 = self.trigger_to_01(
            self.axis_raw(msg, self.AXIS_R2),
            self.AXIS_R2_RELEASED,
            self.AXIS_R2_PRESSED,
        )
        ts.twist.linear.x = (l2 - r2) * self.max_arm_linear

        # Angular: L1/R1 provide digital roll control about x,
        # right stick vertical controls pitch about y with inverted sign for
        # more intuitive stick-up behavior, and right stick horizontal
        # controls yaw about z.
        ts.twist.angular.x = (
            self.button(msg, self.BTN_L1) - self.button(msg, self.BTN_R1)
        ) * self.max_arm_angular
        ts.twist.angular.y = -self.axis(msg, self.AXIS_R_Y) * self.max_arm_angular
        ts.twist.angular.z = self.axis(msg, self.AXIS_R_X) * self.max_arm_angular

        if self.pub_servo.get_num_connections() == 0:
            rospy.logwarn_throttle(2.0, 'No subscribers on %s', self.servo_topic)
        self.pub_servo.publish(ts)
        self.pub_chassis.publish(Twist())
        self.last_buttons = list(msg.buttons)

    def watchdog_cb(self, _event):
        if self.last_joy_time == rospy.Time(0):
            return
        if (rospy.Time.now() - self.last_joy_time).to_sec() > self.cmd_timeout:
            self.publish_zero()

    def servo_frame_check_cb(self, _event):
        self.servo_frame_check_attempts += 1
        expected_frame = rospy.get_param(self.servo_command_frame_param, None)
        if expected_frame is None:
            if self.servo_frame_check_attempts == 1:
                rospy.logwarn(
                    'ds5_teleop_node could not read %s yet; '
                    'will keep using local servo_frame=%s',
                    self.servo_command_frame_param,
                    self.servo_frame,
                )
            if self.servo_frame_check_attempts >= 5:
                self.servo_frame_check_timer.shutdown()
            return

        if expected_frame != self.servo_frame:
            rospy.logwarn(
                'ds5_teleop_node servo_frame=%s but servo_server expects %s; '
                'this can cause axis confusion',
                self.servo_frame,
                expected_frame,
            )
        else:
            rospy.loginfo(
                'ds5_teleop_node confirmed servo command frame: %s',
                expected_frame,
            )

        if expected_frame.endswith('_optical_frame'):
            rospy.logwarn(
                'servo_server is configured with optical frame %s; '
                'for teleop control prefer the camera body frame',
                expected_frame,
            )

        self.servo_frame_check_timer.shutdown()


if __name__ == '__main__':
    rospy.init_node('ds5_teleop_node')
    DS5TeleopNode()
    rospy.spin()
