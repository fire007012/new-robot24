#!/usr/bin/env python3

import json
import math
import os
from datetime import datetime

import rospy
from control_msgs.msg import JointJog
from flipper_control.msg import FlipperControlState
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class FlipperDiagnosticNode(object):
    def __init__(self):
        self.status_topic = rospy.get_param(
            "~status_topic", "/car_control/keyboard_teleop/status"
        )
        self.jog_topic = rospy.get_param("~jog_topic", "/flipper_control/jog_cmd")
        self.flipper_state_topic = rospy.get_param(
            "~flipper_state_topic", "/flipper_control/state"
        )
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")

        self.watch_joint = rospy.get_param("~watch_joint", "left_front_arm_joint")
        self.long_travel_sign = float(rospy.get_param("~long_travel_sign", 0.0))
        self.threshold_deg = float(rospy.get_param("~threshold_deg", 30.0))
        self.sample_rate_hz = float(rospy.get_param("~sample_rate_hz", 50.0))
        self.console_rate_hz = float(rospy.get_param("~console_rate_hz", 4.0))
        self.command_epsilon = float(rospy.get_param("~command_epsilon", 0.05))
        self.sticky_ratio_threshold = float(
            rospy.get_param("~sticky_ratio_threshold", 0.65)
        )
        self.sticky_min_command = float(rospy.get_param("~sticky_min_command", 0.2))
        self.sticky_min_effort = float(rospy.get_param("~sticky_min_effort", 0.2))
        self.sticky_min_samples = int(rospy.get_param("~sticky_min_samples", 3))
        self.sticky_cooldown_sec = float(
            rospy.get_param("~sticky_cooldown_sec", 0.75)
        )
        self.output_path = rospy.get_param("~output_path", "")

        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        if self.console_rate_hz <= 0.0:
            raise ValueError("console_rate_hz must be positive")

        self.threshold_rad = math.radians(self.threshold_deg)
        self.threshold_crossed = False
        self.last_console_time = rospy.Time(0)

        self.latest_status = {}
        self.latest_jog = {}
        self.latest_flipper_state = None
        self.latest_joint_state = {}
        self.zero_reference = {}
        self.joint_names = []
        self.sticky_counts = {}
        self.last_sticky_report_time = {}

        if not self.output_path:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_path = "/tmp/flipper_diagnostic_%s.jsonl" % stamp
        output_dir = os.path.dirname(self.output_path)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.log_fp = open(self.output_path, "a", buffering=1)

        rospy.Subscriber(self.status_topic, String, self.status_cb, queue_size=20)
        rospy.Subscriber(self.jog_topic, JointJog, self.jog_cb, queue_size=50)
        rospy.Subscriber(
            self.flipper_state_topic,
            FlipperControlState,
            self.flipper_state_cb,
            queue_size=50,
        )
        rospy.Subscriber(
            self.joint_states_topic, JointState, self.joint_states_cb, queue_size=100
        )

        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.sample_rate_hz), self.on_timer
        )

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "flipper_diagnostic_node started: watch_joint=%s threshold=%.1f deg output=%s",
            self.watch_joint,
            self.threshold_deg,
            self.output_path,
        )

    def on_shutdown(self):
        try:
            self.log_fp.close()
        except Exception:
            pass

    def infer_long_travel_sign(self):
        if abs(self.long_travel_sign) > 1e-6:
            return 1.0 if self.long_travel_sign > 0.0 else -1.0
        lowered = self.watch_joint.lower()
        if "front" in lowered:
            return -1.0
        if "rear" in lowered:
            return 1.0
        return 1.0

    def zip_joint_array(self, joint_names, values):
        result = {}
        for index, joint_name in enumerate(joint_names):
            if index < len(values):
                result[joint_name] = values[index]
        return result

    def status_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "flipper_diagnostic_node invalid status JSON: %s", exc)

    def jog_cb(self, msg):
        velocities = {}
        for index, joint_name in enumerate(msg.joint_names):
            if index < len(msg.velocities):
                velocities[joint_name] = msg.velocities[index]
        self.latest_jog = {
            "stamp": msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec(),
            "duration": msg.duration,
            "velocities": velocities,
        }

    def flipper_state_cb(self, msg):
        measured = self.zip_joint_array(msg.joint_names, msg.measured_positions)
        reference = self.zip_joint_array(msg.joint_names, msg.reference_positions)
        commanded = self.zip_joint_array(msg.joint_names, msg.commanded_velocities)

        if not self.zero_reference and measured:
            self.zero_reference = dict(measured)
            rospy.loginfo(
                "flipper_diagnostic_node captured zero reference for %d joints",
                len(self.zero_reference),
            )

        self.joint_names = list(msg.joint_names)
        self.latest_flipper_state = {
            "stamp": msg.header.stamp.to_sec()
            if msg.header.stamp
            else rospy.Time.now().to_sec(),
            "active_profile": msg.active_profile,
            "active_hardware_mode": msg.active_hardware_mode,
            "active_controller": msg.active_controller,
            "linkage_mode": msg.linkage_mode,
            "switch_state": msg.switch_state,
            "lifecycle_state": msg.lifecycle_state,
            "detail": msg.detail,
            "switching": msg.switching,
            "ready": msg.ready,
            "command_timed_out": msg.command_timed_out,
            "degraded": msg.degraded,
            "measured_positions": measured,
            "reference_positions": reference,
            "commanded_velocities": commanded,
        }

    def joint_states_cb(self, msg):
        joint_state = {}
        for index, joint_name in enumerate(msg.name):
            joint_state[joint_name] = {
                "position": msg.position[index] if index < len(msg.position) else float("nan"),
                "velocity": msg.velocity[index] if index < len(msg.velocity) else float("nan"),
                "effort": msg.effort[index] if index < len(msg.effort) else float("nan"),
            }
        self.latest_joint_state = joint_state

    def compute_watch_state(self):
        measured_position = None
        if self.watch_joint in self.latest_joint_state:
            measured_position = self.latest_joint_state[self.watch_joint]["position"]
        elif self.latest_flipper_state:
            measured_position = self.latest_flipper_state["measured_positions"].get(
                self.watch_joint
            )

        if measured_position is None:
            return {}

        zero = self.zero_reference.get(self.watch_joint, measured_position)
        relative = measured_position - zero
        travel = self.infer_long_travel_sign() * relative
        return {
            "position": measured_position,
            "zero": zero,
            "relative": relative,
            "travel": travel,
            "travel_deg": math.degrees(travel),
            "threshold_crossed": travel >= self.threshold_rad,
        }

    def summarize_joint(self, joint_name):
        summary = {"joint": joint_name}
        if self.latest_jog:
            summary["jog_cmd"] = self.latest_jog["velocities"].get(joint_name, 0.0)
        if self.latest_flipper_state:
            summary["manager_cmd"] = self.latest_flipper_state["commanded_velocities"].get(
                joint_name, 0.0
            )
            summary["manager_reference"] = self.latest_flipper_state[
                "reference_positions"
            ].get(joint_name)
            summary["manager_measured"] = self.latest_flipper_state[
                "measured_positions"
            ].get(joint_name)
        if joint_name in self.latest_joint_state:
            summary.update(self.latest_joint_state[joint_name])
        return summary

    def active_joint_summaries(self):
        active = []
        for joint_name in self.joint_names:
            jog_cmd = self.latest_jog.get("velocities", {}).get(joint_name, 0.0)
            manager_cmd = 0.0
            if self.latest_flipper_state:
                manager_cmd = self.latest_flipper_state["commanded_velocities"].get(
                    joint_name, 0.0
                )
            if abs(jog_cmd) >= self.command_epsilon or abs(manager_cmd) >= self.command_epsilon:
                summary = self.summarize_joint(joint_name)
                cmd_abs = abs(summary.get("manager_cmd", 0.0))
                vel_abs = abs(summary.get("velocity", 0.0))
                summary["velocity_ratio"] = vel_abs / cmd_abs if cmd_abs > 1e-6 else None
                active.append(summary)
        return active

    def log_event(self, record):
        self.log_fp.write(json.dumps(record, sort_keys=True) + "\n")

    def detect_sticky_events(self, active_joints, stamp, watch):
        events = []
        for item in active_joints:
            joint_name = item["joint"]
            cmd_abs = abs(item.get("manager_cmd", 0.0))
            ratio = item.get("velocity_ratio")
            effort_abs = abs(item.get("effort", 0.0))
            if cmd_abs < self.sticky_min_command or ratio is None:
                self.sticky_counts[joint_name] = 0
                continue
            if effort_abs < self.sticky_min_effort:
                self.sticky_counts[joint_name] = 0
                continue

            if ratio < self.sticky_ratio_threshold:
                self.sticky_counts[joint_name] = self.sticky_counts.get(joint_name, 0) + 1
            else:
                self.sticky_counts[joint_name] = 0

            if self.sticky_counts[joint_name] < self.sticky_min_samples:
                continue

            last_report = self.last_sticky_report_time.get(joint_name, rospy.Time(0))
            if (stamp - last_report).to_sec() < self.sticky_cooldown_sec:
                continue

            self.last_sticky_report_time[joint_name] = stamp
            event = {
                "joint": joint_name,
                "count": self.sticky_counts[joint_name],
                "manager_cmd": item.get("manager_cmd"),
                "jog_cmd": item.get("jog_cmd"),
                "velocity": item.get("velocity"),
                "effort": item.get("effort"),
                "velocity_ratio": ratio,
                "travel_deg": watch.get("travel_deg"),
                "pressed": self.latest_status.get("pressed", []),
            }
            events.append(event)
        return events

    def print_console_summary(self, record):
        watch = record.get("watch", {})
        active = record.get("active_joints", [])
        active_text = []
        for item in active:
            active_text.append(
                "%s jog=%.3f mgr=%.3f vel=%.3f ratio=%s eff=%.2f"
                % (
                    item["joint"],
                    item.get("jog_cmd", 0.0),
                    item.get("manager_cmd", 0.0),
                    item.get("velocity", 0.0),
                    (
                        "%.2f" % item["velocity_ratio"]
                        if item.get("velocity_ratio") is not None
                        else "n/a"
                    ),
                    item.get("effort", 0.0),
                )
            )
        state = record.get("flipper_state", {}) or {}
        pressed = record.get("keyboard_status", {}).get("pressed", [])
        rospy.loginfo(
            "watch=%s travel=%.1fdeg crossed=%s ready=%s degraded=%s pressed=%s active=[%s]",
            self.watch_joint,
            watch.get("travel_deg", 0.0),
            watch.get("threshold_crossed", False),
            state.get("ready"),
            state.get("degraded"),
            pressed,
            "; ".join(active_text),
        )

    def on_timer(self, event):
        if not self.latest_flipper_state and not self.latest_joint_state:
            return

        watch = self.compute_watch_state()
        record = {
            "stamp": event.current_real.to_sec(),
            "watch_joint": self.watch_joint,
            "watch": watch,
            "keyboard_status": self.latest_status,
            "jog": self.latest_jog,
            "flipper_state": self.latest_flipper_state,
            "active_joints": self.active_joint_summaries(),
        }
        record["sticky_events"] = self.detect_sticky_events(
            record["active_joints"], event.current_real, watch
        )
        self.log_event(record)

        for sticky_event in record["sticky_events"]:
            rospy.logwarn(
                "flipper_diagnostic_node sticky event: joint=%s ratio=%.2f cmd=%.3f vel=%.3f eff=%.2f travel=%.1fdeg pressed=%s",
                sticky_event["joint"],
                sticky_event["velocity_ratio"],
                sticky_event["manager_cmd"],
                sticky_event["velocity"],
                sticky_event["effort"],
                sticky_event.get("travel_deg", 0.0),
                sticky_event.get("pressed", []),
            )

        crossed = watch.get("threshold_crossed", False)
        if crossed != self.threshold_crossed:
            self.threshold_crossed = crossed
            rospy.logwarn(
                "flipper_diagnostic_node threshold transition: %s travel=%.1f deg",
                "CROSSED" if crossed else "RETURNED",
                watch.get("travel_deg", 0.0),
            )
            self.print_console_summary(record)
            return

        if (event.current_real - self.last_console_time).to_sec() < 1.0 / self.console_rate_hz:
            return
        self.last_console_time = event.current_real
        self.print_console_summary(record)


if __name__ == "__main__":
    rospy.init_node("flipper_diagnostic_node")
    FlipperDiagnosticNode()
    rospy.spin()
