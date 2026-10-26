#!/usr/bin/env python3

import argparse
import math
import sys

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from arm_control.msg import ExecuteArmGoalAction, ExecuteArmGoalGoal


DEFAULT_ARM_JOINTS = [
    "shoulder_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_yaw_joint",
    "wrist_roll_joint",
]


def build_parser():
    parser = argparse.ArgumentParser(
        description="Smoke client for /arm_control/execute_goal"
    )
    parser.add_argument(
        "--action-name",
        dest="action_name",
        default=None,
        help="Override action server name. Defaults to ~action_name or /arm_control/execute_goal.",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the action server.",
    )
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for the action result.",
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    named_parser = subparsers.add_parser("named", help="Send a named target goal.")
    named_parser.add_argument(
        "--name",
        default="ready",
        help="Named target defined in MoveIt/SRDF. Default: ready.",
    )

    joints_parser = subparsers.add_parser("joints", help="Send a full joint target goal.")
    joints_parser.add_argument(
        "--joint",
        dest="joints",
        action="append",
        default=[],
        help="Joint target in name=value form. Repeat for each joint.",
    )

    pose_parser = subparsers.add_parser("pose", help="Send a pose target goal.")
    pose_parser.add_argument("--frame", required=True, help="Pose frame id.")
    pose_parser.add_argument("--x", type=float, required=True, help="Position X.")
    pose_parser.add_argument("--y", type=float, required=True, help="Position Y.")
    pose_parser.add_argument("--z", type=float, required=True, help="Position Z.")
    pose_parser.add_argument(
        "--qx", type=float, default=0.0, help="Orientation quaternion x."
    )
    pose_parser.add_argument(
        "--qy", type=float, default=0.0, help="Orientation quaternion y."
    )
    pose_parser.add_argument(
        "--qz", type=float, default=0.0, help="Orientation quaternion z."
    )
    pose_parser.add_argument(
        "--qw", type=float, default=1.0, help="Orientation quaternion w."
    )

    roll_parser = subparsers.add_parser(
        "roll180",
        help="Read current arm joint states, add pi to wrist_roll_joint, and send a full joint goal.",
    )
    roll_parser.add_argument(
        "--joint-states-topic",
        default="/joint_states",
        help="JointState topic used to read the current arm pose. Default: /joint_states.",
    )
    roll_parser.add_argument(
        "--joint-states-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for a JointState message. Default: 2.0.",
    )
    roll_parser.add_argument(
        "--roll-joint",
        default="wrist_roll_joint",
        help="Roll joint name to increment. Default: wrist_roll_joint.",
    )
    roll_parser.add_argument(
        "--delta-rad",
        type=float,
        default=math.pi,
        help="Relative roll increment in radians. Default: pi.",
    )
    roll_parser.add_argument(
        "--limit-lower",
        type=float,
        default=-7.069,
        help="Lower safety check bound for the roll joint. Default: -7.069.",
    )
    roll_parser.add_argument(
        "--limit-upper",
        type=float,
        default=7.069,
        help="Upper safety check bound for the roll joint. Default: 7.069.",
    )
    roll_parser.add_argument(
        "--skip-limit-check",
        action="store_true",
        help="Skip the local roll joint limit check before sending the goal.",
    )
    roll_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the computed absolute joint target and exit without calling the action.",
    )

    return parser


def parse_joint_assignments(assignments):
    joint_map = {}
    for raw in assignments:
        if "=" not in raw:
            raise ValueError("invalid --joint value '{}'".format(raw))
        name, value_text = raw.split("=", 1)
        name = name.strip()
        value_text = value_text.strip()
        if not name:
            raise ValueError("joint name cannot be empty in '{}'".format(raw))
        try:
            value = float(value_text)
        except ValueError:
            raise ValueError("joint '{}' has invalid float '{}'".format(name, value_text))
        if name in joint_map:
            raise ValueError("duplicate joint '{}'".format(name))
        joint_map[name] = value

    if not joint_map:
        raise ValueError("at least one --joint name=value is required")

    return joint_map


def read_current_joint_map(topic_name, timeout_sec):
    try:
        msg = rospy.wait_for_message(topic_name, JointState, timeout=timeout_sec)
    except rospy.ROSException as exc:
        raise ValueError(
            "failed to read current JointState from {} within {:.3f}s: {}".format(
                topic_name, timeout_sec, exc
            )
        )

    if len(msg.name) != len(msg.position):
        raise ValueError(
            "JointState on {} has mismatched name/position lengths: {} vs {}".format(
                topic_name, len(msg.name), len(msg.position)
            )
        )

    joint_map = {}
    for name, position in zip(msg.name, msg.position):
        if name not in joint_map:
            joint_map[name] = float(position)
    return joint_map


def build_roll180_goal(args):
    joint_map = read_current_joint_map(args.joint_states_topic, args.joint_states_timeout)

    if args.roll_joint not in DEFAULT_ARM_JOINTS:
        raise ValueError(
            "roll joint '{}' is not in the default active arm joint set {}".format(
                args.roll_joint, DEFAULT_ARM_JOINTS
            )
        )

    missing = [name for name in DEFAULT_ARM_JOINTS if name not in joint_map]
    if missing:
        raise ValueError(
            "JointState on {} is missing active arm joints: {}".format(
                args.joint_states_topic, missing
            )
        )

    if args.roll_joint not in joint_map:
        raise ValueError(
            "roll joint '{}' was not found on {}".format(
                args.roll_joint, args.joint_states_topic
            )
        )

    current_roll = float(joint_map[args.roll_joint])
    target_roll = current_roll + float(args.delta_rad)
    if not args.skip_limit_check:
        if not (float(args.limit_lower) <= target_roll <= float(args.limit_upper)):
            raise ValueError(
                "target {}={} rad exceeds local limit check [{}, {}]".format(
                    args.roll_joint,
                    target_roll,
                    args.limit_lower,
                    args.limit_upper,
                )
            )

    target_joint_map = {name: float(joint_map[name]) for name in DEFAULT_ARM_JOINTS}
    target_joint_map[args.roll_joint] = target_roll

    goal = ExecuteArmGoalGoal()
    goal.target_type = ExecuteArmGoalGoal.TARGET_JOINTS
    goal.joint_names = list(DEFAULT_ARM_JOINTS)
    goal.joint_positions = [target_joint_map[name] for name in goal.joint_names]
    return goal, current_roll, target_roll


def build_goal(args):
    goal = ExecuteArmGoalGoal()

    if args.mode == "named":
        goal.target_type = ExecuteArmGoalGoal.TARGET_NAMED
        goal.named_target = args.name
        return goal

    if args.mode == "joints":
        joint_map = parse_joint_assignments(args.joints)
        goal.target_type = ExecuteArmGoalGoal.TARGET_JOINTS
        goal.joint_names = list(joint_map.keys())
        goal.joint_positions = [joint_map[name] for name in goal.joint_names]
        return goal

    if args.mode == "pose":
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = args.frame
        pose.pose.position.x = args.x
        pose.pose.position.y = args.y
        pose.pose.position.z = args.z
        pose.pose.orientation.x = args.qx
        pose.pose.orientation.y = args.qy
        pose.pose.orientation.z = args.qz
        pose.pose.orientation.w = args.qw

        goal.target_type = ExecuteArmGoalGoal.TARGET_POSE
        goal.pose_target = pose
        return goal

    if args.mode == "roll180":
        return build_roll180_goal(args)

    raise ValueError("unsupported mode '{}'".format(args.mode))


def resolve_action_name(cli_value):
    if cli_value:
        return cli_value
    return rospy.get_param("~action_name", "/arm_control/execute_goal")


def feedback_cb(feedback):
    phase = getattr(feedback, "phase", "")
    message = getattr(feedback, "message", "")
    rospy.loginfo("feedback phase=%s message=%s", phase, message)


def main():
    parser = build_parser()
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("send_execute_arm_goal", anonymous=True)
    action_name = resolve_action_name(args.action_name)

    try:
        built_goal = build_goal(args)
    except ValueError as exc:
        print("invalid arguments: {}".format(exc), file=sys.stderr)
        return 2

    if args.mode == "roll180":
        goal, current_roll, target_roll = built_goal
        print(
            "computed {}: current={:.6f} target={:.6f} delta={:.6f}".format(
                args.roll_joint, current_roll, target_roll, args.delta_rad
            )
        )
        if args.dry_run:
            print("dry_run=true, not sending action goal")
            print("joint_names={}".format(goal.joint_names))
            print("joint_positions={}".format([float(v) for v in goal.joint_positions]))
            return 0
    else:
        goal = built_goal

    client = actionlib.SimpleActionClient(action_name, ExecuteArmGoalAction)

    print("connecting to action server: {}".format(action_name))
    if not client.wait_for_server(rospy.Duration.from_sec(args.server_timeout)):
        print(
            "timed out waiting for action server after {:.3f}s".format(
                args.server_timeout
            ),
            file=sys.stderr,
        )
        return 3

    client.send_goal(goal, feedback_cb=feedback_cb)

    if not client.wait_for_result(rospy.Duration.from_sec(args.result_timeout)):
        client.cancel_goal()
        print(
            "timed out waiting for action result after {:.3f}s".format(
                args.result_timeout
            ),
            file=sys.stderr,
        )
        return 4

    result = client.get_result()
    if result is None:
        print("action completed without a result payload", file=sys.stderr)
        return 5

    print("success={}".format(result.success))
    print("error_code={}".format(result.error_code))
    print("message={}".format(result.message))
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
