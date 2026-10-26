from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config=Path(get_package_share_directory('car_moveit_config'))/'config'
    model=Path(get_package_share_directory('car_urdf'))/'urdf/robot.urdf.xacro'
    servo=yaml.safe_load((Path(get_package_share_directory('car_control'))/'config/servo_ros2.yaml').read_text())
    return LaunchDescription([Node(package='moveit_servo',executable='servo_node_main',name='servo_server',parameters=[{'moveit_servo':servo,'robot_description':ParameterValue(Command(['xacro ',str(model)]),value_type=str),'robot_description_semantic':(config/'car_urdf.srdf').read_text(),'robot_description_kinematics':yaml.safe_load((config/'kinematics.yaml').read_text())}],output='screen'),Node(package='car_control',executable='servo_gate.py',output='screen')])
