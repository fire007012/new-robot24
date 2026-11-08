from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id',default_value='robot24'),
        DeclareLaunchArgument('host',default_value='0.0.0.0'),
        DeclareLaunchArgument('port',default_value='9090'),
        DeclareLaunchArgument('control_state_dir',default_value='/tmp/robot24-control-9090'),
        DeclareLaunchArgument('camera_config',default_value=''),
        Node(package='host_bridge',executable='host_bridge_node.py',arguments=['--ros','--control-state-dir',LaunchConfiguration('control_state_dir'),'--robot-id',LaunchConfiguration('robot_id'),'--host',LaunchConfiguration('host'),'--port',LaunchConfiguration('port'),'--camera-config',LaunchConfiguration('camera_config')],output='screen')])
