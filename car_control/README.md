# car_control · ROS2 遥控、Servo 与监控

`scripts/` 包含手柄、键盘、Qt 遥控、摆臂监控及 servo_gate；`car_control_runtime/` 处理 RPC 与运行会话。`config/servo_ros2.yaml` 是活动 Servo 配置。JTC 与 Servo 互斥，失焦、断流与会话变更清空目标。

```bash
ros2 launch car_control moveit_servo.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
