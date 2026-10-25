"""Isolated single-axis acceptance; no controller auto-activation and no second master."""
import xml.etree.ElementTree as E
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

AXES={'left_front_arm_joint':1,'right_front_arm_joint':2,'left_rear_arm_joint':3,'right_rear_arm_joint':4,'shoulder_yaw_joint':6,'shoulder_pitch_joint':5}
def setup(context):
    joint=LaunchConfiguration('joint').perform(context)
    if joint not in AXES:raise ValueError('unknown CANopen joint')
    share=Path(get_package_share_directory('robot_hardware'))
    config=Path(get_package_share_directory('robot_canopen_config'))/'config'/f'axis_{AXES[joint]}'
    limits=yaml.safe_load((share/'config/joint_limits.yaml').read_text())[joint]
    root=E.parse(share/'urdf/model.urdf.xacro').getroot()
    component=E.SubElement(root,'ros2_control',{'name':'RobotCanopenSystem','type':'system'})
    hardware=E.SubElement(component,'hardware');E.SubElement(hardware,'plugin').text='canopen_ros2_control/RobotSystem'
    for name,value in {'bus_config':str(config/'bus.yml'),'master_config':str(config/'master.dcf'),'can_interface_name':LaunchConfiguration('can_interface').perform(context)}.items():
        E.SubElement(hardware,'param',{'name':name}).text=value
    entry=E.SubElement(component,'joint',{'name':joint});E.SubElement(entry,'param',{'name':'device_name'}).text=joint
    interface='position' if AXES[joint]>4 else 'velocity'
    command=E.SubElement(entry,'command_interface',{'name':interface})
    for name,value in ({'min':limits['lower'],'max':limits['upper']} if interface=='position' else {'min':-.1,'max':.1}).items():
        E.SubElement(command,'param',{'name':name}).text=str(value)
    for state in ['position','velocity']:E.SubElement(entry,'state_interface',{'name':state})
    controller={'type':'joint_trajectory_controller/JointTrajectoryController'} if interface=='position' else {'type':'robot_hardware/TimedVelocityController'}
    params={'joints':[joint],'command_interfaces':['position'],'state_interfaces':['position','velocity']} if interface=='position' else {'joints':[joint],'interface_name':'velocity','lower':[limits['lower']],'upper':[limits['upper']],'max_velocity':[.1],'command_timeout':.25}
    # Node parameter namespaces must match controller node names, use a launch-owned temporary YAML.
    import tempfile,atexit,os
    fd,path=tempfile.mkstemp(prefix='robot-axis-',suffix='.yaml');os.close(fd);atexit.register(lambda:os.path.exists(path) and os.unlink(path))
    Path(path).write_text(yaml.safe_dump({'controller_manager':{'ros__parameters':{'update_rate':200,'hardware_components_initial_state':{'unconfigured':['RobotCanopenSystem']},'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},'axis_controller':controller}},'axis_controller':{'ros__parameters':params}}))
    return [Node(package='controller_manager',executable='ros2_control_node',parameters=[{'robot_description':E.tostring(root,encoding='unicode')},path],output='screen'),
      *[Node(package='controller_manager',executable='spawner',arguments=[name,'--inactive'],output='screen') for name in ['joint_state_broadcaster','axis_controller']]]
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('joint',default_value='shoulder_yaw_joint'),DeclareLaunchArgument('can_interface',default_value='canable0'),OpaqueFunction(function=setup)])
