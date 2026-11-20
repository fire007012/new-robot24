"""Load only the selected groups, with all components initially unconfigured."""
from pathlib import Path
import subprocess
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from robot_hardware_runtime.control_period import control_rate
from robot_hardware_runtime.selection import parse_groups, select_description, selected_controllers


def launch_selected(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('robot_hardware'))
    canopen = Path(get_package_share_directory('robot_canopen_config')) / 'config'
    groups = parse_groups(get('selected_groups'))
    if not get('group_policy') and groups != {'arm', 'base', 'flippers', 'gripper'}:
        raise ValueError('partial hardware selection requires group_policy')
    policy_path = get('group_policy') or str(Path(get_package_share_directory('robot_safety'))/'config/group_policy_v2.yaml')
    contract = yaml.safe_load(Path(policy_path).read_text())
    raw = subprocess.check_output(['xacro', str(share/'urdf/robot.urdf.xacro'),
        'mock:='+get('mock'), 'canopen_interface:='+get('canopen_interface'),
        'private_interface:='+get('private_interface'), 'group_policy:='+get('group_policy')], text=True)
    backends = {a['backend'] for a in contract['axes'].values() if a['group'] in groups}
    if (get('mock') != 'true' and len(backends) > 1 and
            get('canopen_interface') == get('private_interface')):
        raise ValueError('CAN_TOPOLOGY: shared CANopen/private interface has no commissioned topology')
    description = select_description(raw, contract, groups, canopen)
    controllers = selected_controllers(contract, groups, get('enable_servo') == 'true')
    parameters = yaml.safe_load((share/'config/controllers.yaml').read_text())
    parameters['controller_manager']['ros__parameters']['update_rate'] = control_rate(canopen)
    parameters['controller_manager']['ros__parameters']['hardware_components_initial_state'] = {
        'unconfigured': sorted(backends)}
    # Keep node-scoped YAML so no wildcard parameter overrides hide the selected rate.
    import atexit
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', prefix='robot-selected-', suffix='.yaml', delete=False) as out:
        yaml.safe_dump(parameters, out)
        parameter_path = Path(out.name)
    atexit.register(lambda: parameter_path.unlink(missing_ok=True))
    actions = []
    if get('start_controller_manager') == 'true':
        actions.append(Node(package='controller_manager', executable='ros2_control_node',
            parameters=[{'robot_description': description}, str(parameter_path)], output='screen'))
    actions.append(Node(package='robot_hardware', executable='robot_runtime', parameters=[{
        'enable_servo': get('enable_servo') == 'true', 'group_policy': get('group_policy'),
        'selected_groups': sorted(groups), 'simulation': get('mock') == 'true',
        'bus_config': str(canopen/'bus.yml'), 'baseline': get('baseline'), 'journal': get('journal')}], output='screen'))
    actions.append(Node(package='controller_manager', executable='spawner', arguments=controllers +
        ['--inactive', '--controller-manager-timeout', '60', '--service-call-timeout', '30'], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('robot_hardware'))
    defaults = {'start_controller_manager':'true', 'enable_servo':'false', 'group_policy':'',
        'selected_groups':'arm,base,flippers,gripper', 'mock':'false',
        'canopen_interface':'can1', 'private_interface':'can0',
        'baseline':str(share/'config/commissioning.yaml'),
        'journal':str(Path.home()/'.ros/robot_maintenance.json')}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=launch_selected)])
