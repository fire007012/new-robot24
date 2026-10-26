"""ROS2 host/video orchestration; streams come from an explicit site config."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    executable_dir = Path(get_package_prefix('host_bridge')) / 'lib/host_bridge'
    value = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('video_config', description='Absolute path to commissioned video sources YAML'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('manager_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('manager_port', default_value='18081'),
        DeclareLaunchArgument('log_dir', default_value='/tmp/rtsp_logs'),
        DeclareLaunchArgument('start_ros_image_rtsp', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_mediamtx', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mediamtx_bin', default_value='mediamtx'),
        DeclareLaunchArgument('rtsp_port', default_value='8554'),
        ExecuteProcess(cmd=[str(executable_dir / 'mediamtx_launcher.sh'),
                            '--bin', value('mediamtx_bin'), '--port', value('rtsp_port')],
                       condition=IfCondition(value('start_mediamtx')), output='screen'),
        # The manager is plain Python/TCP; launch_ros.Node would append ROS
        # arguments that its argparse interface does not accept.
        ExecuteProcess(cmd=['python3', str(executable_dir / 'video_manager_node.py'),
                            '--config', value('video_config'), '--host', value('manager_host'),
                            '--port', value('manager_port'), '--log-dir', value('log_dir'), '--autostart'],
                       output='screen'),
        Node(package='host_bridge', executable='ros_image_rtsp_node',
             arguments=[value('video_config')],
             condition=IfCondition(value('start_ros_image_rtsp')), output='screen'),
        ExecuteProcess(cmd=['python3', str(executable_dir / 'host_bridge_node.py'),
                            '--ros', '--host', value('host'), '--port', value('port'),
                            '--no-camera-config', '--video-manager',
                            '--video-manager-host', value('manager_host'),
                            '--video-manager-port', value('manager_port')], output='screen'),
    ])
