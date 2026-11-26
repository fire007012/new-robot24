#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS node for Xtherm T2S+ thermal camera via IR-Py-Thermal (diminDDL).
   图像处理方式与 pyplot.py 一致：温度帧 + 自动曝光 + colormap。
"""

import sys
import os
import threading
import time
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
        self._stop_event = threading.Event()
        self._camera_lock = threading.Lock()
        self._worker = None
        self._last_error_log_monotonic = 0.0

    def open_camera(self):
        """打开 T2S+ 设备"""
        cap = None
        try:
            if self.device:
                from sys import platform
                if platform.startswith('linux'):
                    cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                else:
                    cap = cv2.VideoCapture(self.device)
                if not cap.isOpened():
                    cap.release()
                    cap = None
                    now = time.monotonic()
                    if now - self._last_error_log_monotonic >= 10.0:
                        self._last_error_log_monotonic = now
                        self.get_logger().error("无法打开设备: %s" % (self.device))
                    return False
                video_dev = cap
            else:
                video_dev = None

            cam = Camera(
                video_dev=video_dev,
                camera_raw=self.camera_raw,
                fixed_offset=self.fixed_offset,
            )
            with self._camera_lock:
                self.camera = cam

            w, h = cam.get_resolution()
            self.get_logger().info("热成像相机已打开: %dx%d (raw=%s, offset=%.1f)" % (w, h, self.camera_raw, self.fixed_offset))
            return True

        except Exception as e:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            now = time.monotonic()
            if now - self._last_error_log_monotonic >= 10.0:
                self._last_error_log_monotonic = now
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

    def start_capture(self):
        if self._worker is not None and self._worker.is_alive():
            return
        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._capture_loop, name='thermal_capture_worker', daemon=True
        )
        self._worker.start()

    def _release_camera(self):
        with self._camera_lock:
            cam = self.camera
            self.camera = None
        if cam is not None:
            try:
                cam.release()
            except Exception:
                pass

    def _capture_loop(self):
        consecutive_errors = 0
        period = 1.0 / max(1.0, float(self.frame_rate))
        while not self._stop_event.is_set():
            with self._camera_lock:
                cam = self.camera
            if cam is None:
                if not self.open_camera():
                    if self._stop_event.wait(2.0):
                        break
                    continue
                consecutive_errors = 0
                self._last_error_log_monotonic = 0.0
                continue

            started = time.monotonic()
            try:
                temperature = cam.get_frame()
                self.auto_exposure(temperature)
                colored = self.temp_to_color(temperature)
                if self.upscale > 1:
                    colored = cv2.resize(
                        colored, None, fx=self.upscale, fy=self.upscale, interpolation=cv2.INTER_NEAREST
                    )
                if self._stop_event.is_set():
                    break
                msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'thermal_camera'
                self.pub_color.publish(msg)
                consecutive_errors = 0
            except Exception as error:
                consecutive_errors += 1
                now = time.monotonic()
                if now - self._last_error_log_monotonic >= 5.0:
                    self._last_error_log_monotonic = now
                    self.get_logger().warning('thermal capture failed: %s' % (error,))
                if consecutive_errors >= 3:
                    self._release_camera()
                    consecutive_errors = 0
                    if self._stop_event.wait(2.0):
                        break
                    continue

            remaining = period - (time.monotonic() - started)
            if remaining > 0.0 and self._stop_event.wait(remaining):
                break

    def destroy_node(self):
        self._stop_event.set()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.5)
        self._release_camera()
        return super().destroy_node()


def main():
    rclpy.init()
    node = ThermalCameraNode()
    node.start_capture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__=='__main__':main()
