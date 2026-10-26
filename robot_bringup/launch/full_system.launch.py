"""One controller manager; all movement requires explicit runtime operations."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def peripherals(context):
    get=lambda key:LaunchConfiguration(key).perform(context)
    actions=[]
    def include(package,file,arguments=None):
        path=Path(get_package_share_directory(package))/'launch'/file
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=(arguments or {}).items()))
    if get('enable_cameras')=='true':
        include('vision_pkg','cameras.launch.py',{key:get(key) for key in ('behind_camera_type','enable_paw','behind_serial','paw_serial')})
    if get('enable_vision')=='true':include('vision_pkg','vision.launch.py',{'model_path':get('model_path'),'behind_depth_topic':'/behind_camera/depth/image_raw' if get('behind_camera_type')=='astra' else '/behind_camera/aligned_depth_to_color/image_raw'})
    if get('enable_host')=='true':include('host_bridge','host_bridge.launch.py')
    if get('enable_co2')=='true':include('senseair_co2','senseair_co2.launch.py')
    if get('enable_thermal')=='true':include('thermal_camera','thermal_camera.launch.py')
    return actions

def generate_launch_description():
    hardware = Path(get_package_share_directory('robot_hardware'))
    model = Path(get_package_share_directory('car_urdf')) / 'urdf/robot.urdf.xacro'
    policy = Path(get_package_share_directory('robot_safety')) / 'config/group_policy_v1.yaml'
    actions = [DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('canopen_interface', default_value='canable0'),
        DeclareLaunchArgument('private_interface', default_value='canable1'),
        DeclareLaunchArgument('group_policy', default_value=str(policy)),
        DeclareLaunchArgument('baseline', default_value=str(hardware / 'config/commissioning.yaml')),
        DeclareLaunchArgument('journal', default_value=str(Path.home() / '.ros/robot_maintenance.json')),
        DeclareLaunchArgument('enable_servo',default_value='false'),
        DeclareLaunchArgument('ui',default_value='none',choices=['none','keyboard','flippers','monitor']),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('enable_moveit', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware / 'launch/hardware.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in
                ('enable_servo','mock', 'canopen_interface', 'private_interface', 'group_policy', 'baseline', 'journal')}.items()),
        Node(package='robot_bringup', executable='model_joint_states', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             remappings=[('/joint_states', '/robot_model/joint_states')],
             parameters=[{'robot_description': ParameterValue(Command(['xacro ', str(model)]), value_type=str)}],
             output='screen'),
        Node(package='mobility_control', executable='base_cmd_node', condition=IfCondition(LaunchConfiguration('enable_control')),
             output='screen'),
        Node(package='arm_control', executable='gripper_cmd_node', condition=IfCondition(LaunchConfiguration('enable_control')),
             output='screen')]
    actions.append(Node(package='arm_control',executable='wrist_roll_service_node',condition=IfCondition(LaunchConfiguration('enable_moveit')),output='screen'))
    for package, filename in [('car_moveit_config', 'move_group.launch.py'), ('arm_control', 'arm_goal_executor.launch.py')]:
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
            condition=IfCondition(LaunchConfiguration('enable_moveit'))))
    from launch.substitutions import PythonExpression
    servo=Path(get_package_share_directory('car_control'))/'launch/moveit_servo.launch.py'
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(servo)),condition=IfCondition(LaunchConfiguration('enable_servo'))))
    for role,package,executable in [('keyboard','car_control','keyboard_teleop_gui.py'),('flippers','flipper_control','flipper_motor_debug_ui.py'),('monitor','car_control','flipper_diagnostic_node.py')]:
        actions.append(Node(package=package,executable=executable,condition=IfCondition(PythonExpression(["'",LaunchConfiguration('ui'),"' == '",role,"'"])),output='screen'))
    for key,value in [('enable_cameras','false'),('enable_vision','false'),('enable_host','false'),('enable_co2','false'),('enable_thermal','false'),('behind_camera_type','realsense'),('enable_paw','false'),('behind_serial',''),('paw_serial',''),('model_path','')]:
        actions.append(DeclareLaunchArgument(key,default_value=value))
    actions.append(OpaqueFunction(function=peripherals))
    return LaunchDescription(actions)
