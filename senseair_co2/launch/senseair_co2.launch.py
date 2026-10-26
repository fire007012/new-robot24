from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='senseair_co2',executable='co2_publisher.py',output='screen')])
