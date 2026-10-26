#!/usr/bin/env python3
"""Forward measured finite joints to model consumers without inventing states."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class ModelJointStates(Node):
    def __init__(self):
        super().__init__('model_joint_states')
        self.pub = self.create_publisher(JointState, '/robot_model/joint_states', 1)
        self.sub = self.create_subscription(JointState, '/joint_states', self.received, qos_profile_sensor_data)

    def received(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        indices = [i for i, q in enumerate(message.position) if math.isfinite(q)]
        if not indices:
            return
        out = JointState()
        out.header = message.header
        out.name = [message.name[i] for i in indices]
        out.position = [message.position[i] for i in indices]
        for field in ('velocity', 'effort'):
            values = getattr(message, field)
            if len(values) == len(message.name) and all(math.isfinite(values[i]) for i in indices):
                setattr(out, field, [values[i] for i in indices])
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ModelJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
