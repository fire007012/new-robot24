#!/usr/bin/env python3
"""Synthetic RGB/depth inputs through installed ROS2 processing and H265 binaries."""
import os,signal,subprocess,tempfile,time,unittest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo,CompressedImage
from geometry_msgs.msg import PoseStamped
from vision_pkg.msg import Detection
from vision_pkg.srv import SetDetectionEnabled
class VisionSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.update(ROS_DOMAIN_ID='82',ROS_LOCALHOST_ONLY='1')
        cls.folder=tempfile.TemporaryDirectory(prefix='robot24-vision-');cls.log=open('/tmp/robot24-vision.log','w');cls.processes=[]
        rclpy.init();cls.node=Node('vision_smoke');cls.bridge=CvBridge()
        for exe,args in [('vision_display_node',['-r','__ns:=/paw_vision','-p','enable_yolo:=false']),('object_pose_node',[]),('h265_encoder_node',['-p','input_topic:=/paw_camera/color/image_raw']),('h265_decoder_node',[])]:
            cls.processes.append(subprocess.Popen(['ros2','run','vision_pkg',exe,'--ros-args']+args,stdout=cls.log,stderr=subprocess.STDOUT,start_new_session=True))
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node();rclpy.shutdown()
        for p in cls.processes:os.killpg(p.pid,signal.SIGINT)
        for p in cls.processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        cls.log.close();cls.folder.cleanup()
    def wait(self,predicate,timeout=20,publish=None):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.assertTrue(all(p.poll() is None for p in self.processes),open('/tmp/robot24-vision.log').read()[-4000:])
            if publish:publish()
            rclpy.spin_once(self.node,timeout_sec=.05)
            if predicate():return
        self.fail('timeout: '+open('/tmp/robot24-vision.log').read()[-4000:])
    def test_images_depth_services_and_h265(self):
        self.images={};self.poses=[]
        for topic,kind in [('/paw_vision/vision_image',Image),('/panorama/panorama_image/h265',CompressedImage),('/panorama/panorama_image/h265_decoded',Image)]:
            self.node.create_subscription(kind,topic,lambda m,t=topic:self.images.update({t:m}),10)
        self.node.create_subscription(PoseStamped,'/detected_object_pose',self.poses.append,10)
        color=self.node.create_publisher(Image,'/paw_camera/color/image_raw',qos_profile_sensor_data)
        depth=self.node.create_publisher(Image,'/paw_camera/depth/image_rect_raw',qos_profile_sensor_data)
        info=self.node.create_publisher(CameraInfo,'/paw_camera/color/camera_info',qos_profile_sensor_data)
        det=self.node.create_publisher(Detection,'/yolov8/detections',qos_profile_sensor_data)
        def frames(encoding='16UC1'):
            stamp=self.node.get_clock().now().to_msg();frame='paw_camera_color_optical_frame'
            c=self.bridge.cv2_to_imgmsg(np.full((240,320,3),(35,90,170),dtype=np.uint8),'bgr8');c.header.stamp=stamp;c.header.frame_id=frame
            d=self.bridge.cv2_to_imgmsg(np.full((240,320),1250 if encoding=='16UC1' else 1.25,dtype=np.uint16 if encoding=='16UC1' else np.float32),encoding);d.header=c.header
            ci=CameraInfo(header=c.header,width=320,height=240,k=[200.,0.,160.,0.,200.,120.,0.,0.,1.])
            detection=Detection(header=c.header,id=1,class_name='test',confidence=.9,x=150,y=110,width=20,height=20)
            info.publish(ci);depth.publish(d);color.publish(c);det.publish(detection)
        client=self.node.create_client(SetDetectionEnabled,'/paw_vision/set_detection_enabled');self.assertTrue(client.wait_for_service(timeout_sec=20))
        future=client.call_async(SetDetectionEnabled.Request(enable_center_distance=True));self.wait(future.done);self.assertTrue(future.result().success)
        self.wait(lambda:len(self.images)==3 and bool(self.poses),30,frames)
        for topic in ('/paw_vision/vision_image','/panorama/panorama_image/h265_decoded'):
            self.assertEqual((self.images[topic].width,self.images[topic].height),(320,240));self.assertEqual(self.images[topic].header.frame_id,'paw_camera_color_optical_frame')
        pose=self.poses[-1];self.assertAlmostEqual(pose.pose.position.z,1.25,places=3);self.assertAlmostEqual(pose.pose.position.x,0.,places=3)
        self.poses.clear();self.wait(lambda:bool(self.poses),5,lambda:frames('32FC1'));self.assertAlmostEqual(self.poses[-1].pose.position.z,1.25,places=3)
        # Cached output must expire; old detection stamps must not refresh the pose.
        end=time.monotonic()+.7
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.05)
        count=len(self.poses);end=time.monotonic()+.4
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.05)
        self.assertEqual(len(self.poses),count)
        future=client.call_async(SetDetectionEnabled.Request(enable_yolo=True));self.wait(future.done);self.assertFalse(future.result().success)
        print('RGB, depth mm/m, calibrated projection, cache expiry, service refusal without weights, H265 encode/decode passed')
if __name__=='__main__':unittest.main(verbosity=2)
