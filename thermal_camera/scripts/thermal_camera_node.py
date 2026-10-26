#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS node for Xtherm T2S+ thermal camera via IR-Py-Thermal (diminDDL).
   图像处理方式与 pyplot.py 一致：温度帧 + 自动曝光 + colormap。
"""

import sys
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')  # 无 GUI 后端
import matplotlib.cm as cm

# ---- 让 Python 能 import 到 thermal_camera/src 里的库 ----
_lib_path = os.path.join(os.path.dirname(__file__), '..', 'src')
if not os.path.isdir(_lib_path):_lib_path=os.path.dirname(__file__)
if os.path.isdir(_lib_path):
    sys.path.insert(0, os.path.abspath(_lib_path))

try:
    from irpythermal import Camera
except ImportError as e:
    print("ERROR: Cannot import irpythermal.Camera. "
          "确保 IR-Py-Thermal 源码在 thermal_camera/src/ 下。\n" + str(e))
    sys.exit(1)


class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera')

        # ROS 参数
        self.device = self.declare_parameter('device', '').value
        self.camera_raw = self.declare_parameter('raw_mode', True).value
        self.frame_rate = self.declare_parameter('frame_rate', 25).value
        self.fixed_offset = self.declare_parameter('temp_offset', 0.0).value
        self.upscale = self.declare_parameter('upscale', 4).value
        self.colormap = self.declare_parameter('colormap', 'plasma').value  # 与 pyplot 默认一致

        # 自动曝光状态（与 pyplot.py 一致）
        self.T_min = 0.0
        self.T_max = 50.0
        self.T_margin = 2.0

        # matplotlib colormap
        self.cmap = cm.get_cmap(self.colormap)

        # 发布者
        self.pub_color = self.create_publisher(Image,'/thermal_camera/image_raw',1)
        self.bridge = CvBridge()
        self.camera = None

    def open_camera(self):
        """打开 T2S+ 设备"""
        try:
            if self.device:
                from sys import platform
                if platform.startswith('linux'):
                    cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                else:
                    cap = cv2.VideoCapture(self.device)
                if not cap.isOpened():
                    self.get_logger().error("无法打开设备: %s" % (self.device))
                    return False
                video_dev = cap
            else:
                video_dev = None

            self.camera = Camera(
                video_dev=video_dev,
                camera_raw=self.camera_raw,
                fixed_offset=self.fixed_offset,
            )

            w, h = self.camera.get_resolution()
            self.get_logger().info("热成像相机已打开: %dx%d (raw=%s, offset=%.1f)" % (w, h, self.camera_raw, self.fixed_offset))
            return True

        except Exception as e:
            self.get_logger().error("打开热成像相机失败: %s" % (e))
            return False

    def auto_exposure(self, frame):
        """自动曝光，与 pyplot.py 的 utils.autoExposure (ends 模式) 一致"""
        lmin, lmax = float(frame.min()), float(frame.max())
        updated = False

        if self.T_min > lmin:
            self.T_min = lmin - self.T_margin
            updated = True
        if self.T_min + 2 * self.T_margin < lmin:
            self.T_min = lmin - self.T_margin
            updated = True
        if self.T_max < lmax:
            self.T_max = lmax + self.T_margin
            updated = True
        if self.T_max - 2 * self.T_margin > lmax:
            self.T_max = lmax + self.T_margin
            updated = True

        return updated

    def temp_to_color(self, frame):
        """温度帧 → matplotlib colormap → BGR 图像（与 pyplot imshow 效果一致）"""
        # 按曝光范围归一化到 0~1
        span = self.T_max - self.T_min
        if span <= 0:
            span = 1.0
        normalized = (frame - self.T_min) / span
        normalized = np.clip(normalized, 0.0, 1.0)

        # matplotlib colormap 输出 RGBA (float 0~1)
        rgba = self.cmap(normalized)

        # 转 BGR uint8
        rgb = (rgba[:, :, :3] * 255).astype(np.uint8)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return bgr


def main():
    rclpy.init();node=ThermalCameraNode()
    try:
        if not node.open_camera():raise RuntimeError('thermal camera unavailable')
        def frame():
            try:
                temperature=node.camera.get_frame();node.auto_exposure(temperature);colored=node.temp_to_color(temperature)
                if node.upscale>1:colored=cv2.resize(colored,None,fx=node.upscale,fy=node.upscale,interpolation=cv2.INTER_NEAREST)
                msg=node.bridge.cv2_to_imgmsg(colored,encoding='bgr8');msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='thermal_camera';node.pub_color.publish(msg)
            except Exception as error:node.get_logger().warning(str(error))
        node.create_timer(1.0/float(node.frame_rate),frame);rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node.camera is not None:node.camera.release()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
