from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
NAMES=('forward','back','head','hand','arm')
def setup(context):
    actions=[]
    for name in NAMES:
        device=LaunchConfiguration(name+'_device').perform(context)
        if device:actions.append(Node(package='vision_pkg',executable='usb_camera_node.py',namespace=name+'_camera',parameters=[{'device':device,'frame_id':name+'_camera_optical_frame'}],output='screen'))
    return actions
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name+'_device',default_value='',description='Explicit stable UVC device path; empty disables this camera') for name in NAMES]+[OpaqueFunction(function=setup)])
