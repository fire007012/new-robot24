#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion


HELP = """
keyboard moveit server (relative end-effector move)

Keys:
  w / s : +X / -X
  a / d : +Y / -Y
  r / f : +Z / -Z
  i / k : +Roll / -Roll
  j / l : +Pitch / -Pitch
  u / o : +Yaw / -Yaw
  + / - : increase / decrease linear step
  ] / [ : increase / decrease angular step
  c     : print current pose
  <space> : stop current motion
  h     : print help
  q     : quit
""".strip()


class KeyboardMoveItServer:
    def __init__(self):
        self.group_name = rospy.get_param("~group_name", "arm")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.end_effector_link = rospy.get_param("~end_effector_link", "")
        self.linear_step = float(rospy.get_param("~linear_step", 0.01))
        self.angular_step_deg = float(rospy.get_param("~angular_step_deg", 5.0))
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.2))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.2))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.num_planning_attempts = int(rospy.get_param("~num_planning_attempts", 1))

        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        if self.reference_frame:
            self.group.set_pose_reference_frame(self.reference_frame)
        if self.end_effector_link:
            self.group.set_end_effector_link(self.end_effector_link)

        self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.num_planning_attempts)

        rospy.loginfo("[keyboard_moveit] group=%s ref=%s ee=%s", self.group_name, self.reference_frame, self.end_effector_link or "(default)")
        rospy.loginfo("[keyboard_moveit] linear_step=%.4f m angular_step=%.2f deg", self.linear_step, self.angular_step_deg)
        rospy.loginfo("\n%s", HELP)

    def _get_current_pose(self):
        if self.end_effector_link:
            return self.group.get_current_pose(self.end_effector_link).pose
        return self.group.get_current_pose().pose

    def _set_target_pose(self, pose):
        self.group.set_pose_target(pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return success

    def _apply_delta(self, dx, dy, dz, droll, dpitch, dyaw):
        pose = self._get_current_pose()

        pose.position.x += dx
        pose.position.y += dy
        pose.position.z += dz

        roll, pitch, yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        roll += droll
        pitch += dpitch
        yaw += dyaw
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        success = self._set_target_pose(pose)
        rospy.loginfo("[keyboard_moveit] move success=%s", str(success))

    def print_pose(self):
        pose = self._get_current_pose()
        roll, pitch, yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        rospy.loginfo("[keyboard_moveit] pose xyz=(%.4f, %.4f, %.4f) rpy=(%.2f, %.2f, %.2f) deg",
                      pose.position.x, pose.position.y, pose.position.z,
                      math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def adjust_linear_step(self, scale):
        self.linear_step = max(0.0005, self.linear_step * scale)
        rospy.loginfo("[keyboard_moveit] linear_step=%.4f", self.linear_step)

    def adjust_angular_step(self, scale):
        self.angular_step_deg = max(0.1, self.angular_step_deg * scale)
        rospy.loginfo("[keyboard_moveit] angular_step_deg=%.2f", self.angular_step_deg)

    def handle_key(self, ch):
        step = self.linear_step
        ang = math.radians(self.angular_step_deg)
        if ch == "w":
            self._apply_delta(step, 0, 0, 0, 0, 0)
        elif ch == "s":
            self._apply_delta(-step, 0, 0, 0, 0, 0)
        elif ch == "a":
            self._apply_delta(0, step, 0, 0, 0, 0)
        elif ch == "d":
            self._apply_delta(0, -step, 0, 0, 0, 0)
        elif ch == "r":
            self._apply_delta(0, 0, step, 0, 0, 0)
        elif ch == "f":
            self._apply_delta(0, 0, -step, 0, 0, 0)
        elif ch == "i":
            self._apply_delta(0, 0, 0, ang, 0, 0)
        elif ch == "k":
            self._apply_delta(0, 0, 0, -ang, 0, 0)
        elif ch == "j":
            self._apply_delta(0, 0, 0, 0, ang, 0)
        elif ch == "l":
            self._apply_delta(0, 0, 0, 0, -ang, 0)
        elif ch == "u":
            self._apply_delta(0, 0, 0, 0, 0, ang)
        elif ch == "o":
            self._apply_delta(0, 0, 0, 0, 0, -ang)
        elif ch == "+":
            self.adjust_linear_step(1.25)
        elif ch == "-":
            self.adjust_linear_step(0.8)
        elif ch == "]":
            self.adjust_angular_step(1.25)
        elif ch == "[":
            self.adjust_angular_step(0.8)
        elif ch == "c":
            self.print_pose()
        elif ch == " ":
            self.group.stop()
            rospy.loginfo("[keyboard_moveit] stop")
        elif ch == "h":
            rospy.loginfo("\n%s", HELP)


def run_keyboard_loop(server):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == "q":
                rospy.loginfo("[keyboard_moveit] quit")
                break
            server.handle_key(ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    rospy.init_node("keyboard_moveit_server")
    server = KeyboardMoveItServer()
    run_keyboard_loop(server)
