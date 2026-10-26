"""Fortress owns the sole controller_manager; runtime permissions stay explicit."""
from pathlib import Path
import copy,subprocess,tempfile,xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter


def setup(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    share=lambda package:Path(get_package_share_directory(package))
    hardware=share('robot_hardware');policy=share('robot_safety')/'config/group_policy_v1.yaml'
    tree=ET.fromstring(subprocess.check_output(['xacro',str(hardware/'urdf/robot.urdf.xacro'),'mock:=true','group_policy:='+str(policy)],text=True))
    # Reuse the canonical geometry; retain exactly the two controlled components.
    for item in list(tree):
        if item.tag!='ros2_control':tree.remove(item)
    model=ET.parse(share('car_urdf')/'urdf/robot.urdf.xacro').getroot()
    for item in model:tree.append(copy.deepcopy(item))
    for source,alias in [('camera_optical_frame','behind_camera_color_optical_frame'),('catch_camera_optical_frame','paw_camera_color_optical_frame')]:
        ET.SubElement(tree,'link',{'name':alias})
        joint=ET.SubElement(tree,'joint',{'name':alias+'_joint','type':'fixed'})
        ET.SubElement(joint,'parent',{'link':source});ET.SubElement(joint,'child',{'link':alias})
    for plugin in tree.findall('./ros2_control/hardware/plugin'):plugin.text='robot_simulation/GazeboSystem'
    for mesh in tree.findall('.//mesh'):
        mesh.set('filename',mesh.get('filename').replace('package://car_urdf/', 'file://'+str(share('car_urdf'))+'/'))
    folder=Path(tempfile.mkdtemp(prefix='robot24-fortress-'))
    controllers=yaml.safe_load((hardware/'config/controllers.yaml').read_text())
    controllers['controller_manager']['ros__parameters'].pop('hardware_components_initial_state',None)
    controllers['controller_manager']['ros__parameters']['use_sim_time']=True
    (folder/'controllers.yaml').write_text(yaml.safe_dump(controllers,sort_keys=False))
    gz=ET.SubElement(tree,'gazebo');plugin=ET.SubElement(gz,'plugin',{'filename':'libgz_ros2_control-system.so','name':'gz_ros2_control::GazeboSimROS2ControlPlugin'})
    ET.SubElement(plugin,'parameters').text=str(folder/'controllers.yaml');ET.SubElement(plugin,'robot_param_node').text='robot_state_publisher';ET.SubElement(plugin,'robot_param').text='robot_description'
    # Sensors use Fortress systems, never Gazebo Classic ROS plugins.
    for link,name in [('camera','behind_camera'),('catch_camera','paw_camera')]:
        gz=ET.SubElement(tree,'gazebo',{'reference':link})
        ET.SubElement(gz,'preserveFixedJoint').text='true'
        for kind,suffix in [('camera','color/image_raw'),('depth_camera','depth/image_raw')]:
            sensor=ET.SubElement(gz,'sensor',{'name':name+'_'+kind,'type':kind});ET.SubElement(sensor,'topic').text='/'+name+'/'+suffix
            ET.SubElement(sensor,'update_rate').text='15';ET.SubElement(sensor,'always_on').text='true'
            camera=ET.SubElement(sensor,'camera');ET.SubElement(camera,'horizontal_fov').text='1.2'
            image=ET.SubElement(camera,'image');ET.SubElement(image,'width').text='320';ET.SubElement(image,'height').text='240';ET.SubElement(image,'format').text='R8G8B8'
            clip=ET.SubElement(camera,'clip');ET.SubElement(clip,'near').text='.05';ET.SubElement(clip,'far').text='10'
            ET.SubElement(camera,'camera_info_topic').text='/'+name+'/color/camera_info'
    gz=ET.SubElement(tree,'gazebo',{'reference':'base_link'});sensor=ET.SubElement(gz,'sensor',{'name':'imu','type':'imu'});ET.SubElement(sensor,'topic').text='/imu/data';ET.SubElement(sensor,'update_rate').text='100';ET.SubElement(sensor,'always_on').text='true'
    description=ET.tostring(tree,encoding='unicode');(folder/'robot.urdf').write_text(description)
    args='-r -s ' if get('gui')=='false' else '-r '
    world=share('robot_simulation')/'worlds/robot24.sdf'
    actions=[SetParameter(name='use_sim_time',value=True),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('ros_gz_sim')/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':args+str(world)}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}],remappings=[('/joint_states','/robot_model/joint_states')],output='screen'),
        Node(package='robot_bringup',executable='model_joint_states',output='screen'),
        Node(package='ros_gz_sim',executable='create',arguments=['-world','robot24','-name','robot24','-file',str(folder/'robot.urdf'),'-z','.15'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock','/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware/'launch/hardware.launch.py')),launch_arguments={'start_controller_manager':'false','mock':'true','group_policy':str(policy),'enable_servo':get('enable_servo'),'journal':get('journal')}.items()),
        Node(package='mobility_control',executable='base_cmd_node',output='screen'),Node(package='arm_control',executable='gripper_cmd_node',output='screen')]
    for name in ('behind_camera','paw_camera'):
        topics=['/'+name+'/color/image_raw','/'+name+'/depth/image_raw','/'+name+'/color/camera_info']
        actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=[topics[0]+'@sensor_msgs/msg/Image[ignition.msgs.Image',topics[1]+'@sensor_msgs/msg/Image[ignition.msgs.Image',topics[2]+'@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],parameters=[{'override_frame_id':name+'_color_optical_frame'}],output='screen'))
    if get('enable_moveit')=='true':
        for package,file in [('car_moveit_config','move_group.launch.py'),('arm_control','arm_goal_executor.launch.py')]:
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share(package)/'launch'/file))))
        actions.append(Node(package='arm_control',executable='wrist_roll_service_node',output='screen'))
    if get('enable_servo')=='true':actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share('car_control')/'launch/moveit_servo.launch.py'))))
    if get('ui')!='none':
        package,exe={'keyboard':('car_control','keyboard_teleop_gui.py'),'flippers':('flipper_control','flipper_motor_debug_ui.py'),'monitor':('car_control','flipper_diagnostic_node.py')}[get('ui')]
        actions.append(Node(package=package,executable=exe,output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='false'),DeclareLaunchArgument('enable_moveit',default_value='true'),DeclareLaunchArgument('enable_servo',default_value='false'),DeclareLaunchArgument('ui',default_value='none',choices=['none','keyboard','flippers','monitor']),DeclareLaunchArgument('journal',default_value='/tmp/robot24-simulation-maintenance.json'),OpaqueFunction(function=setup)])
