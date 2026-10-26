# mobility_control · ROS2 底盘速度适配

`src/base_cmd_node.cpp` 通过 Ruckig 平滑 Twist，将 /cmd_vel 转发至 /wheel_controller/cmd_vel_unstamped；配置在 config/。几何与方向参数以 robot_hardware/config/controllers.yaml 为准。输入、反馈/会话及输出有超时与非法数值检查，控制器停发看门狗为 300 ms。

```bash
ros2 launch mobility_control base_cmd.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
