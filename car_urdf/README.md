# car_urdf · ROS2 机器人模型

`urdf/robot.urdf.xacro` 是活动运动学模型，`meshes/` 提供网格，根 frame 为 base_link。整车入口安装并组合模型，过滤后 joint_states 仅用于有限 TF；硬件安全读取原始测量。ros2_control 硬件描述由 robot_hardware 组合。

```bash
ros2 launch robot_bringup full_system_mock.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
