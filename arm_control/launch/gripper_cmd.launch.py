from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='arm_control', executable='gripper_cmd_node', output='screen')])
