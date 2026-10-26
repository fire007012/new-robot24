#!/usr/bin/env python3
"""Read-only graph check; never initializes or enables a hardware component."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from arm_control.action import ExecuteArmGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents


def main():
    rclpy.init()
    node = Node('acceptance_check_ready')
    node.declare_parameter('timeout', 30.0)
    node.declare_parameter('require_running', False)
    node.declare_parameter('require_moveit', False)
    trajectory = ActionClient(node, FollowJointTrajectory, '/arm_position_controller/follow_joint_trajectory')
    project = ActionClient(node, ExecuteArmGoal, '/arm_control/execute_goal')
    latest = {'joint': 0.0, 'state': 0.0, 'runtime': {}}
    def joints(msg):
        if len(msg.name) >= 13 and len(msg.position) == len(msg.name) and all(math.isfinite(q) for q in msg.position):
            latest['joint'] = time.monotonic()
    def state(msg):
        try:
            latest['runtime'] = json.loads(msg.data)
            latest['state'] = time.monotonic()
        except ValueError:
            pass
    node.create_subscription(JointState, '/joint_states', joints, 1)
    node.create_subscription(String, '/robot_runtime/state', state, 1)
    clients = [node.create_client(kind, '/controller_manager/' + name) for kind, name in
               [(ListControllers, 'list_controllers'), (ListHardwareComponents, 'list_hardware_components')]]
    deadline = time.monotonic() + node.get_parameter('timeout').value
    pending = [None, None]
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for i, client in enumerate(clients):
                if pending[i] is None and client.service_is_ready():
                    pending[i] = client.call_async(ListControllers.Request() if i == 0 else ListHardwareComponents.Request())
            if not all(f is not None and f.done() for f in pending):
                continue
            controllers = pending[0].result().controller
            hardware = pending[1].result().component
            required = {'joint_state_broadcaster', 'arm_position_controller', 'flipper_csp_controller',
                        'flipper_csv_forward_controller', 'gripper_controller', 'wheel_controller'}
            known = {c.name for c in controllers if c.state in ('active', 'inactive')}
            components = {h.name for h in hardware}
            fresh = time.monotonic() - latest['state'] < 0.3
            running = latest['runtime'].get('state') == 'Running'
            active = {c.name for c in controllers if c.state == 'active'}
            flipper = 'flipper_csp_controller' if latest['runtime'].get('profile') == 'csp' else 'flipper_csv_forward_controller'
            running = running and trajectory.server_is_ready() and all(h.state.id == 3 for h in hardware) and {
                'joint_state_broadcaster', ('arm_servo_controller' if latest['runtime'].get('arm_profile')=='servo' else 'arm_position_controller'), 'wheel_controller', 'gripper_controller', flipper} <= active
            action_ready = not node.get_parameter('require_moveit').value or project.server_is_ready()
            if action_ready and required <= known and components == {'RobotCanopenSystem', 'CanDriverSystem'} and fresh:
                if not node.get_parameter('require_running').value or (running and time.monotonic() - latest['joint'] < 0.2):
                    node.get_logger().info('Ready: ' + json.dumps({'runtime': latest['runtime'].get('state'),
                        'controllers': {c.name: c.state for c in controllers},
                        'hardware': {h.name: h.state.label for h in hardware}}))
                    result = 0
                    break
            pending = [None, None]
        if result:
            node.get_logger().error('Readiness timeout; init/enable/resume remain explicit operations')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
