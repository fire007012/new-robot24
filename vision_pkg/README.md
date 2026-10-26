# vision_pkg · ROS2 视觉与相机流水线

launch/ 提供 RealSense/Orbbec/USB 相机、视觉、全景和 H265 启动；src/ 构建检测、深度投影、距离/障碍提醒和图像显示节点，msg/ 与 srv/ 定义接口。YOLO 需要显式有效模型路径，缺失权重时启用失败。深度支持毫米整数与米制浮点，使用 CameraInfo/optical frame；无效或过期数据不能作为有效三维检测。真实镜头、权重、双相机和完整视频链尚待现场验证。

```bash
ros2 launch vision_pkg cameras.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
