from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('car_moveit_config'))
    model = Path(get_package_share_directory('car_urdf')) / 'urdf/robot.urdf.xacro'
    return LaunchDescription([Node(package='arm_control', executable='arm_goal_executor', output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', str(model)]), value_type=str),
            'robot_description_semantic': (share / 'config/car_urdf.srdf').read_text(),
            'robot_description_kinematics': yaml.safe_load((share / 'config/kinematics.yaml').read_text())}])])
