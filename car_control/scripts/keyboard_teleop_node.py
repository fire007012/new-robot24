#!/usr/bin/env python3

import json

import rospy
from control_msgs.msg import JointJog
from flipper_control.srv import SetControlProfile, SetControlProfileRequest
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class KeyboardTeleopNode(object):
    MODE_BASE = "base"
    MODE_ARM = "arm"
    SPEED_NORMAL = "normal"
    SPEED_FAST = "fast"
    SPEED_SLOW = "slow"

    def __init__(self):
        self.raw_state_topic = rospy.get_param("~raw_state_topic")
        self.status_topic = rospy.get_param("~status_topic")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.input_timeout = float(rospy.get_param("~input_timeout", 0.3))

        self.chassis_cmd_topic = rospy.get_param("~chassis_cmd_topic")
        self.servo_cmd_topic = rospy.get_param("~servo_cmd_topic")
        self.servo_command_frame_param = rospy.get_param(
            "~servo_command_frame_param", "/servo_server/robot_link_command_frame"
        )
        self.servo_frame = rospy.get_param("~servo_frame", "catch_camera")

        self.gripper_position_topic = rospy.get_param("~gripper_position_topic")
        self.gripper_joint_name = rospy.get_param(
            "~gripper_joint_name", "left_gripper_finger_joint"
        )
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self.gripper_min_position = float(rospy.get_param("~gripper_min_position", 0.0))
        self.gripper_max_position = float(rospy.get_param("~gripper_max_position", 0.044))
        self.gripper_target = float(rospy.get_param("~gripper_initial_position", 0.022))
        self.gripper_rates = {
            self.SPEED_NORMAL: float(rospy.get_param("~gripper_rate_normal", 0.03)),
            self.SPEED_FAST: float(rospy.get_param("~gripper_rate_fast", 0.06)),
            self.SPEED_SLOW: float(rospy.get_param("~gripper_rate_slow", 0.015)),
        }
        self.have_gripper_state = False

        self.flipper_jog_topic = rospy.get_param("~flipper_jog_topic")
        self.flipper_profile_service = rospy.get_param("~flipper_profile_service")
        self.flipper_target_profile = rospy.get_param("~flipper_target_profile", "csp_jog")
        self.flipper_profile_retry_sec = float(
            rospy.get_param("~flipper_profile_retry_sec", 2.0)
        )
        self.flipper_joint_names = list(rospy.get_param("~flipper_joint_names", []))
        self.flipper_velocities = {
            self.SPEED_NORMAL: float(rospy.get_param("~flipper_velocity_normal", 0.4)),
            self.SPEED_FAST: float(rospy.get_param("~flipper_velocity_fast", 0.8)),
            self.SPEED_SLOW: float(rospy.get_param("~flipper_velocity_slow", 0.2)),
        }
        self.flipper_jog_duration = float(rospy.get_param("~flipper_jog_duration", 0.15))

        self.base_linear = {
            self.SPEED_NORMAL: float(rospy.get_param("~base_linear_normal", 0.4)),
            self.SPEED_FAST: float(rospy.get_param("~base_linear_fast", 0.8)),
            self.SPEED_SLOW: float(rospy.get_param("~base_linear_slow", 0.2)),
        }
        self.base_angular = {
            self.SPEED_NORMAL: float(rospy.get_param("~base_angular_normal", 0.8)),
            self.SPEED_FAST: float(rospy.get_param("~base_angular_fast", 1.5)),
            self.SPEED_SLOW: float(rospy.get_param("~base_angular_slow", 0.4)),
        }
        self.arm_linear = {
            self.SPEED_NORMAL: float(rospy.get_param("~arm_linear_normal", 0.08)),
            self.SPEED_FAST: float(rospy.get_param("~arm_linear_fast", 0.15)),
            self.SPEED_SLOW: float(rospy.get_param("~arm_linear_slow", 0.04)),
        }
        self.arm_angular = {
            self.SPEED_NORMAL: float(rospy.get_param("~arm_angular_normal", 0.4)),
            self.SPEED_FAST: float(rospy.get_param("~arm_angular_fast", 0.8)),
            self.SPEED_SLOW: float(rospy.get_param("~arm_angular_slow", 0.2)),
        }

        self.mode = self.MODE_BASE
        self.focused = False
        self.current_pressed = set()
        self.last_input_time = rospy.Time(0)
        self.last_status_publish = rospy.Time(0)
        self.last_flipper_profile_attempt = rospy.Time(0)
        self.last_flipper_profile_result = "pending"
        self.last_flipper_profile_message = ""
        self.last_servo_frame_check = rospy.Time(0)

        self.base_pub = rospy.Publisher(self.chassis_cmd_topic, Twist, queue_size=10)
        self.servo_pub = rospy.Publisher(self.servo_cmd_topic, TwistStamped, queue_size=10)
        self.gripper_pub = rospy.Publisher(
            self.gripper_position_topic, Float64, queue_size=10
        )
        self.flipper_pub = rospy.Publisher(self.flipper_jog_topic, JointJog, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)

        self.raw_state_sub = rospy.Subscriber(
            self.raw_state_topic, String, self.raw_state_cb, queue_size=10
        )
        self.joint_state_sub = rospy.Subscriber(
            self.joint_states_topic, JointState, self.joint_state_cb, queue_size=20
        )

        self.flipper_profile_client = rospy.ServiceProxy(
            self.flipper_profile_service, SetControlProfile
        )

        self.last_cycle_time = rospy.Time.now()
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / max(self.publish_rate, 1.0)), self.on_timer
        )

        rospy.loginfo(
            "keyboard_teleop_node started: raw=%s status=%s chassis=%s servo=%s gripper=%s flipper=%s",
            self.raw_state_topic,
            self.status_topic,
            self.chassis_cmd_topic,
            self.servo_cmd_topic,
            self.gripper_position_topic,
            self.flipper_jog_topic,
        )

    def joint_state_cb(self, msg):
        try:
            index = msg.name.index(self.gripper_joint_name)
        except ValueError:
            return

        if index >= len(msg.position):
            return

        measured = self.clamp_gripper(msg.position[index])
        if not self.have_gripper_state:
            self.gripper_target = measured
            self.have_gripper_state = True

    def raw_state_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "keyboard_teleop_node invalid JSON: %s", exc)
            return

        pulses = set(payload.get("pulse", []))
        pressed = set(payload.get("pressed", []))
        focused = bool(payload.get("focused", False))

        self.last_input_time = rospy.Time.now()
        self.focused = focused

        if "enter" in pulses:
            self.current_pressed = set()
            self.publish_zero_outputs(rospy.Time.now())
            rospy.loginfo("keyboard_teleop_node emergency stop")
        else:
            self.current_pressed = pressed if focused else set()

        if "tab" in pulses:
            self.mode = self.MODE_ARM if self.mode == self.MODE_BASE else self.MODE_BASE
            self.publish_zero_outputs(rospy.Time.now())
            rospy.loginfo("keyboard_teleop_node mode=%s", self.mode.upper())
            if self.mode == self.MODE_BASE:
                self.ensure_flipper_profile(force=True)

    def on_timer(self, event):
        now = event.current_real
        dt = max((now - self.last_cycle_time).to_sec(), 0.0)
        self.last_cycle_time = now

        self.refresh_servo_frame(now)

        stale = self.last_input_time == rospy.Time(0) or (
            now - self.last_input_time
        ).to_sec() > self.input_timeout

        if self.mode == self.MODE_BASE:
            self.ensure_flipper_profile(force=False)

        if stale or not self.focused:
            self.publish_zero_outputs(now)
            self.publish_status(now, stale=stale)
            return

        speed_mode = self.current_speed_mode()

        if self.mode == self.MODE_BASE:
            self.publish_base_outputs(now, speed_mode)
        else:
            self.publish_arm_outputs(now, dt, speed_mode)

        self.publish_status(now, stale=False)

    def refresh_servo_frame(self, now):
        if (now - self.last_servo_frame_check).to_sec() < 1.0:
            return
        self.last_servo_frame_check = now
        expected = rospy.get_param(self.servo_command_frame_param, self.servo_frame)
        if expected != self.servo_frame:
            rospy.loginfo(
                "keyboard_teleop_node servo frame updated: %s -> %s",
                self.servo_frame,
                expected,
            )
            self.servo_frame = expected

    def ensure_flipper_profile(self, force):
        if not self.flipper_joint_names:
            return

        now = rospy.Time.now()
        if not force and self.last_flipper_profile_result == "ok":
            return
        if not force and (
            now - self.last_flipper_profile_attempt
        ).to_sec() < self.flipper_profile_retry_sec:
            return

        self.last_flipper_profile_attempt = now
        try:
            self.flipper_profile_client.wait_for_service(timeout=0.1)
            request = SetControlProfileRequest(profile=self.flipper_target_profile)
            response = self.flipper_profile_client(request)
            self.last_flipper_profile_result = (
                "ok" if response.success else "rejected"
            )
            self.last_flipper_profile_message = response.message
            if response.success:
                rospy.loginfo_throttle(
                    5.0,
                    "keyboard_teleop_node flipper profile=%s controller=%s",
                    response.active_profile,
                    response.active_controller,
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "keyboard_teleop_node failed to set flipper profile: %s",
                    response.message,
                )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.last_flipper_profile_result = "unavailable"
            self.last_flipper_profile_message = str(exc)
            rospy.logwarn_throttle(
                2.0,
                "keyboard_teleop_node flipper profile service unavailable: %s",
                exc,
            )

    def publish_base_outputs(self, now, speed_mode):
        cmd = Twist()
        cmd.linear.x = self.axis_value("w", "s") * self.base_linear[speed_mode]
        cmd.angular.z = self.axis_value("a", "d") * self.base_angular[speed_mode]
        self.base_pub.publish(cmd)

        self.publish_zero_servo(now)
        self.publish_flipper_jog(now, speed_mode)

    def publish_arm_outputs(self, now, dt, speed_mode):
        self.base_pub.publish(Twist())
        self.publish_zero_flipper(now)

        linear = self.arm_linear[speed_mode]
        angular = self.arm_angular[speed_mode]

        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = self.servo_frame
        cmd.twist.linear.x = self.axis_value("u", "o") * linear
        cmd.twist.linear.y = self.axis_value("a", "d") * linear
        cmd.twist.linear.z = self.axis_value("w", "s") * linear
        cmd.twist.angular.x = self.axis_value("q", "e") * angular
        cmd.twist.angular.y = self.axis_value("i", "k") * angular
        cmd.twist.angular.z = self.axis_value("j", "l") * angular
        self.servo_pub.publish(cmd)

        gripper_delta = self.axis_value("f", "h") * self.gripper_rates[speed_mode] * dt
        if abs(gripper_delta) > 0.0:
            self.gripper_target = self.clamp_gripper(self.gripper_target + gripper_delta)
            self.gripper_pub.publish(Float64(data=self.gripper_target))

    def publish_zero_outputs(self, now):
        self.base_pub.publish(Twist())
        self.publish_zero_servo(now)
        self.publish_zero_flipper(now)

    def publish_zero_servo(self, now):
        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = self.servo_frame
        self.servo_pub.publish(cmd)

    def publish_zero_flipper(self, now):
        if not self.flipper_joint_names:
            return
        msg = JointJog()
        msg.header.stamp = now
        msg.joint_names = list(self.flipper_joint_names)
        msg.velocities = [0.0] * len(self.flipper_joint_names)
        msg.duration = self.flipper_jog_duration
        self.flipper_pub.publish(msg)

    def publish_flipper_jog(self, now, speed_mode):
        if not self.flipper_joint_names:
            return

        flipper_speed = self.flipper_velocities[speed_mode]
        key_pairs = [
            ("u", "j"),
            ("i", "k"),
            ("o", "l"),
            ("p", "semicolon"),
        ]
        velocities = [
            self.axis_value(pos_key, neg_key) * flipper_speed
            for pos_key, neg_key in key_pairs
        ]

        msg = JointJog()
        msg.header.stamp = now
        msg.joint_names = list(self.flipper_joint_names)
        msg.velocities = velocities
        msg.duration = self.flipper_jog_duration
        self.flipper_pub.publish(msg)

    def publish_status(self, now, stale):
        payload = {
            "mode": self.mode,
            "speed_mode": self.current_speed_mode(),
            "focused": self.focused,
            "stale": stale,
            "pressed": sorted(self.current_pressed),
            "servo_frame": self.servo_frame,
            "gripper_target": round(self.gripper_target, 4),
            "flipper_profile_target": self.flipper_target_profile,
            "flipper_profile_result": self.last_flipper_profile_result,
            "flipper_profile_message": self.last_flipper_profile_message,
            "stamp": now.to_sec(),
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def current_speed_mode(self):
        if "z" in self.current_pressed:
            return self.SPEED_SLOW
        if "shift" in self.current_pressed:
            return self.SPEED_FAST
        return self.SPEED_NORMAL

    def axis_value(self, positive_key, negative_key):
        value = 0.0
        if positive_key in self.current_pressed:
            value += 1.0
        if negative_key in self.current_pressed:
            value -= 1.0
        return value

    def clamp_gripper(self, value):
        return max(self.gripper_min_position, min(self.gripper_max_position, value))


if __name__ == "__main__":
    rospy.init_node("keyboard_teleop_node")
    KeyboardTeleopNode()
    rospy.spin()
