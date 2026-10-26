from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, RaiseError
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('robot_bringup')) / 'launch/full_system_mock.launch.py'
    check = Node(package='robot_bringup', executable='check_ready', output='screen')
    def checked(event, context):
        if event.returncode:
            return [RaiseError(msg='Startup readiness check failed')]
        return [EmitEvent(event=Shutdown(reason='Read-only startup check passed'))]
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments={'enable_moveit': 'false'}.items()),
        check, RegisterEventHandler(OnProcessExit(target_action=check,
            on_exit=checked))])
