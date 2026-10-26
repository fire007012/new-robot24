# car_moveit_config · ROS2 MoveIt 2 配置

`config/` 包含 SRDF、KDL、OMPL、限位及六轴控制器映射，`src/` 是 PlanningOnlyControllerManager 插件。move_group 禁止直接执行；项目 ExecuteArmGoal 统一派发至六轴 JTC。

```bash
ros2 launch car_moveit_config move_group.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
