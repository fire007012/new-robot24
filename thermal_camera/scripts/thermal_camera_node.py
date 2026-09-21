#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS node for Xtherm T2S+ thermal camera via IR-Py-Thermal (diminDDL).
   图像处理方式与 pyplot.py 一致：温度帧 + 自动曝光 + colormap。
"""

import sys
import os
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')  # 无 GUI 后端
import matplotlib.cm as cm

# ---- 让 Python 能 import 到 thermal_camera/src 里的库 ----
_lib_path = os.path.join(os.path.dirname(__file__), '..', 'src')
if os.path.isdir(_lib_path):
    sys.path.insert(0, os.path.abspath(_lib_path))

try:
    from irpythermal import Camera
except ImportError as e:
    print("ERROR: Cannot import irpythermal.Camera. "
          "确保 IR-Py-Thermal 源码在 thermal_camera/src/ 下。\n" + str(e))
    sys.exit(1)


class ThermalCameraNode:
    def __init__(self):
        rospy.init_node('thermal_camera', anonymous=False)

        # ROS 参数
        self.device = rospy.get_param('~device', '')
        self.camera_raw = rospy.get_param('~raw_mode', True)
        self.frame_rate = rospy.get_param('~frame_rate', 25)
        self.fixed_offset = rospy.get_param('~temp_offset', 0.0)
        self.upscale = rospy.get_param('~upscale', 4)
        self.colormap = rospy.get_param('~colormap', 'plasma')  # 与 pyplot 默认一致

        # 自动曝光状态（与 pyplot.py 一致）
        self.T_min = 0.0
        self.T_max = 50.0
        self.T_margin = 2.0

        # matplotlib colormap
        self.cmap = cm.get_cmap(self.colormap)

        # 发布者
        self.pub_color = rospy.Publisher('/thermal_camera/image_raw', Image, queue_size=1)
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
                    rospy.logerr("无法打开设备: %s", self.device)
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
            rospy.loginfo("热成像相机已打开: %dx%d (raw=%s, offset=%.1f)",
                          w, h, self.camera_raw, self.fixed_offset)
            return True

        except Exception as e:
            rospy.logerr("打开热成像相机失败: %s", e)
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

    def run(self):
        """主循环：get_frame() → 自动曝光 → colormap → 放大 → 发布"""
        if not self.open_camera():
            return

        rate = rospy.Rate(self.frame_rate)

        while not rospy.is_shutdown():
            try:
                # 1. 获取温度帧（每个像素 = 摄氏度），与 pyplot animate_func 一致
                temp_frame = self.camera.get_frame()

                # 2. 自动曝光
                self.auto_exposure(temp_frame)

                # 3. colormap 映射（与 pyplot imshow + set_clim 效果一致）
                colored = self.temp_to_color(temp_frame)

                # 4. 放大
                if self.upscale > 1:
                    colored = cv2.resize(
                        colored,
                        (colored.shape[1] * self.upscale, colored.shape[0] * self.upscale),
                        interpolation=cv2.INTER_NEAREST,
                    )

                # 5. 发布
                msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'thermal_camera'
                self.pub_color.publish(msg)

            except Exception as e:
                rospy.logwarn("帧处理异常: %s", e)

            rate.sleep()

        if self.camera is not None:
            try:
                self.camera.release()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        node = ThermalCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

