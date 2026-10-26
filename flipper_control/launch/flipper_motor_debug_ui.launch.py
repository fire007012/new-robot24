from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='flipper_control',executable='flipper_motor_debug_ui.py',output='screen')])
