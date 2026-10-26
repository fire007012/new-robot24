# flipper_control · ROS2 原生摆臂调试界面

本包仅安装 `scripts/flipper_motor_debug_ui.py` 和 ROS2 launch。摆臂硬件、联动、CSV/CSP 切换及控制器在 robot_hardware 中，运行许可在 robot_safety 中，不再启动旧 flipper_manager。

```bash
ros2 launch flipper_control flipper_motor_debug_ui.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
