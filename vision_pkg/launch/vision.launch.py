from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    get=lambda name:LaunchConfiguration(name).perform(context)
    actions=[]
    for name,enabled,depth in [('paw',get('enable_paw'),get('paw_depth_topic')),('behind',get('enable_behind'),get('behind_depth_topic'))]:
        if enabled!='true':continue
        actions.append(Node(package='vision_pkg',executable='vision_display_node',namespace=name+'_vision',parameters=[{'model_path':get('model_path'),'color_topic':'/'+name+'_camera/color/image_raw','depth_topic':depth,'camera_info_topic':'/'+name+'_camera/color/camera_info'}],output='screen'))
    if get('enable_object_pose')=='true':
        actions.append(Node(package='vision_pkg',executable='object_pose_node',parameters=[{'detection_topic':'/paw_vision/detections','depth_topic':get('paw_depth_topic')}],output='screen'))
    if get('enable_panorama')=='true':actions.append(Node(package='vision_pkg',executable='panorama_node',namespace='panorama',output='screen'))
    if get('enable_h265')=='true':
        for name,topic in [('paw','/paw_vision/vision_image'),('behind','/behind_vision/vision_image'),('panorama','/panorama/panorama_image')]:
            if get('enable_'+name)=='true':actions.append(Node(package='vision_pkg',executable='h265_encoder_node',name=name+'_h265_encoder',parameters=[{'input_topic':topic,'output_topic':topic+'/h265'}],output='screen'))
    return actions

def generate_launch_description():
    defaults={'model_path':'','enable_paw':'true','enable_behind':'true','enable_object_pose':'false','enable_panorama':'false','enable_h265':'false','paw_depth_topic':'/paw_camera/aligned_depth_to_color/image_raw','behind_depth_topic':'/behind_camera/aligned_depth_to_color/image_raw'}
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value) for key,value in defaults.items()]+[OpaqueFunction(function=setup)])
