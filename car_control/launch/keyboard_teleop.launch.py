from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='car_control',executable='keyboard_teleop_gui.py',output='screen')])
