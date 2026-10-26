# arm_control · ROS2 机械臂执行与适配

`src/arm_goal_executor.cpp` 提供 ExecuteArmGoal，使用 MoveIt 2 规划后向唯一六轴 JTC 派发，含许可/反馈检查、并发拒绝和取消。`scripts/` 提供夹爪 Bool/Float64 → m/s 适配、腕部 Trigger 服务及目标客户端。

```bash
ros2 run arm_control send_execute_arm_goal --named zero
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
