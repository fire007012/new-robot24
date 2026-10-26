#!/usr/bin/env python3
"""ROS2 UVC capture with explicit device binding and source acquisition stamps."""
import cv2,rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CompressedImage
class UsbCamera(Node):
    def __init__(self):
        super().__init__('usb_camera')
        device=self.declare_parameter('device','').value
        if not device:raise ValueError('device must be an explicit /dev/v4l/by-id path or device path')
        self.frame=self.declare_parameter('frame_id','camera_optical_frame').value
        self.capture=cv2.VideoCapture(device,cv2.CAP_V4L2)
        if not self.capture.isOpened():raise RuntimeError('cannot open camera '+device)
        for key,default,prop in [('width',1280,cv2.CAP_PROP_FRAME_WIDTH),('height',720,cv2.CAP_PROP_FRAME_HEIGHT),('fps',25,cv2.CAP_PROP_FPS)]:
            value=self.declare_parameter(key,default).value
            if value<=0:raise ValueError(key+' must be positive')
            self.capture.set(prop,value)
            if key=='fps':fps=value
        self.bridge=CvBridge();self.raw=self.create_publisher(Image,'image_raw',qos_profile_sensor_data);self.jpeg=self.create_publisher(CompressedImage,'image_raw/compressed',qos_profile_sensor_data)
        self.timer=self.create_timer(1./fps,self.read,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def read(self):
        ok,image=self.capture.read();stamp=self.get_clock().now().to_msg()
        if not ok:self.get_logger().warning('camera read failed');return
        if self.raw.get_subscription_count():
            msg=self.bridge.cv2_to_imgmsg(image,'bgr8');msg.header.stamp=stamp;msg.header.frame_id=self.frame;self.raw.publish(msg)
        if self.jpeg.get_subscription_count():
            msg=self.bridge.cv2_to_compressed_imgmsg(image,'jpg');msg.header.stamp=stamp;msg.header.frame_id=self.frame;self.jpeg.publish(msg)
    def destroy_node(self):
        self.capture.release();return super().destroy_node()
def main():
    rclpy.init();node=None
    try:node=UsbCamera();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
