from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host',default_value='0.0.0.0'),
        DeclareLaunchArgument('port',default_value='9090'),
        DeclareLaunchArgument('camera_config',default_value=''),
        Node(package='host_bridge',executable='host_bridge_node.py',arguments=['--ros','--host',LaunchConfiguration('host'),'--port',LaunchConfiguration('port'),'--camera-config',LaunchConfiguration('camera_config')],output='screen')])
