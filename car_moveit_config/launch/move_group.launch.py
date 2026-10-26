from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('car_moveit_config'))
    model = Path(get_package_share_directory('car_urdf')) / 'urdf/robot.urdf.xacro'
    load = lambda name: yaml.safe_load((share / 'config' / name).read_text())
    ompl = load('ompl_planning.yaml')
    ompl.update(planning_plugin='ompl_interface/OMPLPlanner',
        request_adapters='default_planner_request_adapters/AddTimeOptimalParameterization '
                         'default_planner_request_adapters/FixWorkspaceBounds '
                         'default_planner_request_adapters/FixStartStateBounds '
                         'default_planner_request_adapters/FixStartStateCollision '
                         'default_planner_request_adapters/FixStartStatePathConstraints',
        start_state_max_bounds_error=0.1)
    description = {'robot_description': ParameterValue(Command(['xacro ', str(model)]), value_type=str),
        'robot_description_semantic': (share / 'config/car_urdf.srdf').read_text(),
        'robot_description_kinematics': load('kinematics.yaml'),
        'robot_description_planning': load('joint_limits.yaml')}
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[description, load('moveit_controllers.yaml'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'allow_trajectory_execution': False,
                'moveit_controller_manager': 'robot24/PlanningOnlyControllerManager',
                'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
                'publish_robot_description': True, 'publish_robot_description_semantic': True,
                'publish_planning_scene': True, 'publish_geometry_updates': True,
                'publish_state_updates': True, 'publish_transforms_updates': True}])])
