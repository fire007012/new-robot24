#!/usr/bin/env python3
"""Bool/Float64 to timed one-element m/s command, fenced by runtime state."""
import json
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray, String


class GripperCmdNode(Node):
    def __init__(self):
        super().__init__('gripper_cmd_node')
        for name, value in dict(open_velocity=0.02, close_velocity=-0.02,
                                max_abs_velocity=0.02, command_hold_sec=0.2).items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in
                       ('open_velocity', 'close_velocity', 'max_abs_velocity', 'command_hold_sec')}
        if (not all(math.isfinite(v) for v in self.params.values()) or
                not 0 < self.params['command_hold_sec'] <= 0.25 or
                not 0 < self.params['max_abs_velocity'] <= 0.02):
            raise ValueError('invalid gripper limits: m/s and 0 < hold <= 0.25 s required')
        self.pub = self.create_publisher(Float64MultiArray, '/gripper_controller/command', 1)
        self.token, self.running, self.state_at = None, False, 0.0
        self.subs = []
        self.velocity, self.deadline = 0.0, 0.0
        self.state_sub = self.create_subscription(String, '/robot_runtime/state', self.state, 1)
        self.timer = self.create_timer(0.02, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def state(self, message):
        try:
            state = json.loads(message.data)
            running = state.get('groups', {}).get('gripper', state['state']) == 'Running'
            token = (running, json.dumps(state.get('operations', {}).get('gripper', {}), sort_keys=True))
        except (ValueError, KeyError, TypeError):
            self.running = False
            self.velocity, self.deadline = 0.0, 0.0
            return
        if token != self.token or time.monotonic() - self.state_at > 0.3:
            self.velocity, self.deadline = 0.0, 0.0
            self.token = token
            for sub in self.subs:
                self.destroy_subscription(sub)
            self.subs = []
            if running:
                self.subs = [
                    self.create_subscription(Float64, '/arm_control/gripper_velocity',
                        lambda msg, session=token: self.command(msg.data, session), 1),
                    self.create_subscription(Bool, '/arm_control/gripper_open',
                        lambda msg, session=token: self.command(self.params[
                            'open_velocity' if msg.data else 'close_velocity'], session), 1)]
            self.publish(0.0)
        self.running, self.state_at = running, time.monotonic()

    def command(self, value, session):
        if session != self.token or not self.running or time.monotonic() - self.state_at > 0.3:
            return
        limit = self.params['max_abs_velocity']
        self.velocity = max(-limit, min(limit, value)) if math.isfinite(value) else 0.0
        self.deadline = time.monotonic() + self.params['command_hold_sec']
        self.publish(self.velocity)

    def publish(self, value):
        self.pub.publish(Float64MultiArray(data=[float(value)]))

    def step(self):
        now = time.monotonic()
        if not self.running or now - self.state_at > 0.3 or now >= self.deadline:
            self.velocity, self.deadline = 0.0, 0.0
        self.publish(self.velocity)


def main():
    rclpy.init()
    node = GripperCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
