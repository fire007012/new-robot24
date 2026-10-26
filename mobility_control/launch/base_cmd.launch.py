from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('mobility_control')) / 'config/base_cmd.yaml'
    return LaunchDescription([Node(package='mobility_control', executable='base_cmd_node',
                                   parameters=[str(config)], output='screen')])
