from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path = Path(get_package_share_directory('robot_bringup')) / 'launch/full_system.launch.py'
    options={'enable_cameras':'false','enable_vision':'false','enable_host':'false','enable_co2':'false','enable_thermal':'false','behind_camera_type':'realsense','enable_paw':'false','behind_serial':'','paw_serial':'','model_path':''}
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value) for key,value in options.items()]+[
        DeclareLaunchArgument('enable_servo',default_value='false'),
        DeclareLaunchArgument('ui',default_value='none'),
        DeclareLaunchArgument('enable_moveit', default_value='true'),
        DeclareLaunchArgument('journal', default_value='/tmp/robot24-mock-maintenance.json'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
            launch_arguments={**{key:LaunchConfiguration(key) for key in options},'enable_servo':LaunchConfiguration('enable_servo'),'ui':LaunchConfiguration('ui'),'mock': 'true', 'journal': LaunchConfiguration('journal'), 'enable_moveit': LaunchConfiguration('enable_moveit')}.items())])
