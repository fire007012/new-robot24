"""ROS2 camera entry with hardware parameters inherited from ROS1 vision.launch."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    behind = get('behind_camera_type')
    paw = get('enable_paw') == 'true'
    # Driver accepts an underscore prefix to keep numeric serials YAML strings.
    back = get('behind_serial').lstrip('_')
    if not back:
        back = {'realsense': '112222070518', 'astra': '219323070286'}.get(behind, '')
    hand = get('paw_serial').lstrip('_')
    if paw and not hand:
        raise ValueError('paw_serial must identify the commissioned D405')
    if behind != 'none' and not back:
        raise ValueError('behind_serial must identify the selected rear camera')
    if paw and behind == 'realsense' and hand == back:
        raise ValueError('two RealSense cameras require distinct behind_serial and paw_serial')
    actions = []

    def rs(name, serial, extra):
        path = Path(get_package_share_directory('realsense2_camera')) / 'launch/rs_launch.py'
        arguments = {
            'camera_name': name, 'camera_namespace': '', 'serial_no': '_' + serial,
            'enable_color': 'true', 'enable_depth': 'true',
            'enable_infra': 'false', 'enable_infra1': 'false', 'enable_infra2': 'false',
            'enable_sync': 'false',
        }
        arguments.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())

    if behind == 'realsense':
        actions.append(rs('behind_camera', back, {
            # ROS1 rear camera used driver-selected stream profiles.
            'pointcloud.enable': get('behind_enable_pointcloud'),
            'align_depth.enable': get('behind_align_depth'),
        }))
    elif behind == 'astra':
        path = Path(get_package_share_directory('orbbec_camera')) / 'launch/astra_adv.launch.py'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)), launch_arguments={
                'camera_name': 'behind_camera', 'serial_number': back,
                'enable_point_cloud': 'false', 'enable_ir': 'false',
                'depth_registration': 'true',
            }.items()))
    if paw:
        actions.append(rs('paw_camera', hand, {
            # D405 provides color on its depth module, unlike the rear D435i.
            'depth_module.color_profile': get('paw_color_profile'),
            'depth_module.depth_profile': get('paw_depth_profile'),
            'align_depth.enable': get('paw_align_depth'),
            'pointcloud.enable': get('paw_enable_pointcloud'),
        }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('behind_camera_type', default_value='realsense',
                              choices=['realsense', 'astra', 'none']),
        DeclareLaunchArgument('enable_paw', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('behind_serial', default_value='',
                              description='Empty selects historical serial for behind_camera_type'),
        DeclareLaunchArgument('paw_serial', default_value='260322273334'),
        DeclareLaunchArgument('paw_color_profile', default_value='1280,720,30'),
        DeclareLaunchArgument('paw_depth_profile', default_value='848,480,30'),
        DeclareLaunchArgument('paw_align_depth', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('paw_enable_pointcloud', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('behind_enable_pointcloud', default_value='true', choices=['true', 'false']),
        # Preserve the ROS2 rear aligned-depth consumer contract.
        DeclareLaunchArgument('behind_align_depth', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
