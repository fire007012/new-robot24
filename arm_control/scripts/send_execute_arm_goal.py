#!/usr/bin/env python3
"""ROS2 command-line client; Ctrl-C requests cancellation and waits for stop."""
import argparse
import json
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from arm_control.action import ExecuteArmGoal


def main():
    parser = argparse.ArgumentParser()
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--named')
    target.add_argument('--joints', type=float, nargs=6, metavar='RAD')
    target.add_argument('--pose', type=float, nargs=7, metavar='XYZ_QXYZW')
    parser.add_argument('--frame', default='base_link')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('send_execute_arm_goal')
    client = ActionClient(node, ExecuteArmGoal, '/arm_control/execute_goal')
    handle = None
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('ExecuteArmGoal server unavailable')
        goal = ExecuteArmGoal.Goal()
        if args.named:
            goal.target_type, goal.named_target = goal.TARGET_NAMED, args.named
        elif args.joints:
            goal.target_type = goal.TARGET_JOINTS
            goal.joint_names = ['shoulder_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                                'wrist_pitch_joint', 'wrist_yaw_joint', 'wrist_roll_joint']
            goal.joint_positions = args.joints
        else:
            goal.target_type = goal.TARGET_POSE
            goal.pose_target.header.frame_id = args.frame
            p = goal.pose_target.pose
            p.position.x, p.position.y, p.position.z = args.pose[:3]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = args.pose[3:]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Goal rejected or acceptance timed out')
        handle = future.result()
        outcome = handle.get_result_async()
        rclpy.spin_until_future_complete(node, outcome)
        result = outcome.result().result
        print(json.dumps({'success': result.success, 'error_code': result.error_code, 'message': result.message}))
        return 0 if result.success else 1
    except KeyboardInterrupt:
        if handle is not None and rclpy.ok():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
            outcome = handle.get_result_async()
            rclpy.spin_until_future_complete(node, outcome, timeout_sec=5)
        return 130
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
