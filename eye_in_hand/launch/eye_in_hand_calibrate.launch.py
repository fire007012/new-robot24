from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    params={'name':LaunchConfiguration('name'),'calibration_type':'eye_in_hand','robot_base_frame':'base_link','robot_effector_frame':'tool0','tracking_base_frame':'paw_camera_link','tracking_marker_frame':'aruco_marker_frame','freehand_robot_movement':True}
    return LaunchDescription([DeclareLaunchArgument('name',default_value='paw_d405_handeye'),
        DeclareLaunchArgument('marker_id',description='ID of the actual printed marker; confirm detector dictionary and measured size'),
        DeclareLaunchArgument('marker_size',default_value='0.05'),
        Node(package='aruco_ros',executable='single',name='aruco_single',parameters=[{'marker_id':ParameterValue(LaunchConfiguration('marker_id'),value_type=int),'marker_size':ParameterValue(LaunchConfiguration('marker_size'),value_type=float),'camera_frame':'paw_camera_color_optical_frame','marker_frame':'aruco_marker_frame','reference_frame':'paw_camera_color_optical_frame','image_is_rectified':True}],remappings=[('/image','/paw_camera/color/image_raw'),('/camera_info','/paw_camera/color/camera_info')],output='screen'),
        Node(package='easy_handeye2',executable='handeye_server',parameters=[params],output='screen'),
        Node(package='easy_handeye2',executable='rqt_calibrator.py',parameters=[params],output='screen')])
