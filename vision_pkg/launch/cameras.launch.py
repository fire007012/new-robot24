from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def setup(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    behind=get('behind_camera_type');paw=get('enable_paw')=='true';back=get('behind_serial');hand=get('paw_serial')
    if paw and behind=='realsense' and (not hand or not back or hand==back):raise ValueError('two RealSense cameras require distinct behind_serial and paw_serial')
    actions=[]
    def rs(name,serial):
        path=Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py'
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments={'camera_name':name,'camera_namespace':'','serial_no':serial,'enable_color':'true','enable_depth':'true','align_depth.enable':'true'}.items())
    if behind=='realsense':actions.append(rs('behind_camera',back))
    elif behind=='astra':
        path=Path(get_package_share_directory('orbbec_camera'))/'launch/astra_adv.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments={'camera_name':'behind_camera','serial_number':back,'enable_point_cloud':'false','enable_ir':'false','depth_registration':'true'}.items()))
    if paw:actions.append(rs('paw_camera',hand))
    return actions

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('behind_camera_type',default_value='realsense',choices=['realsense','astra','none']),DeclareLaunchArgument('enable_paw',default_value='false',choices=['true','false']),DeclareLaunchArgument('behind_serial',default_value=''),DeclareLaunchArgument('paw_serial',default_value=''),OpaqueFunction(function=setup)])
