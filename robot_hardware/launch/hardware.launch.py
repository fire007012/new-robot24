from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    share = Path(get_package_share_directory('robot_hardware'))
    canopen = Path(get_package_share_directory('robot_canopen_config'))
    description = ParameterValue(Command(['xacro ', str(share/'urdf/robot.urdf.xacro'),
        ' mock:=', LaunchConfiguration('mock'), ' canopen_interface:=', LaunchConfiguration('canopen_interface'),
        ' private_interface:=', LaunchConfiguration('private_interface'), ' group_policy:="', LaunchConfiguration('group_policy'), '"']), value_type=str)
    actions = [DeclareLaunchArgument('start_controller_manager',default_value='true'),DeclareLaunchArgument('enable_servo',default_value='false'),DeclareLaunchArgument('group_policy', default_value=''),
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('canopen_interface', default_value='canable0'),
        DeclareLaunchArgument('private_interface', default_value='canable1'),
        DeclareLaunchArgument('baseline', default_value=str(share/'config/commissioning.yaml')),
        DeclareLaunchArgument('journal', default_value=str(Path.home()/'.ros/robot_maintenance.json')),
        Node(package='controller_manager', executable='ros2_control_node',condition=IfCondition(LaunchConfiguration('start_controller_manager')),
             parameters=[{'robot_description': description},str(share/'config/controllers.yaml')], output='screen'),
        Node(package='robot_hardware', executable='robot_runtime', parameters=[{
             'enable_servo': ParameterValue(LaunchConfiguration('enable_servo'),value_type=bool),
             'group_policy': ParameterValue(LaunchConfiguration('group_policy'), value_type=str),
             'simulation': ParameterValue(LaunchConfiguration('mock'), value_type=bool), 'bus_config': str(canopen/'config/bus.yml'), 'baseline':LaunchConfiguration('baseline'),
             'journal':LaunchConfiguration('journal')}], output='screen')]
    for controller in ['joint_state_broadcaster','arm_position_controller','flipper_csp_controller',
                       'flipper_csv_forward_controller','gripper_controller','wheel_controller']:
        actions.append(Node(package='controller_manager', executable='spawner',
            arguments=[controller,'--inactive','--controller-manager-timeout','60'], output='screen'))
    actions.append(Node(package='controller_manager',executable='spawner',arguments=['arm_servo_controller','--inactive','--controller-manager-timeout','60'],condition=IfCondition(LaunchConfiguration('enable_servo')),output='screen'))
    return LaunchDescription(actions)
