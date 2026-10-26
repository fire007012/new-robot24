from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='joy',executable='joy_node',parameters=[{'autorepeat_rate':30.0}],output='screen'),Node(package='car_control',executable='ds5_teleop_node.py',output='screen')])
