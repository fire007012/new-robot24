# thermal_camera · ROS2 热成像接口

scripts/thermal_camera_node.py 是 ROS2 发布入口，src/irpythermal.py 与 utils.py 是运行所需设备库，继续保留。旧 Catkin setup.py 和 XML launch 已移入参考目录。真实热成像设备、标定与图像验证待现场执行。

```bash
ros2 launch thermal_camera thermal_camera.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
