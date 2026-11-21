# robot_bringup · ROS2 整车启动

launch/ 提供实机、Mock 入口及只读就绪检查，组合唯一 controller_manager、模型/TF、底盘/夹爪、MoveIt 和可选外围。Fortress 入口位于 `robot_simulation/launch/full_system_simulate.launch.py`，由 Gazebo 插件持有唯一管理器。默认分组策略，显式 init/enable/resume。scripts/check_ready.py 检查原始就绪状态，model_joint_states.py 仅为 TF 转发有限测量；bringup_can*.sh 是现场 CAN 配置辅助脚本。

```bash
ros2 launch robot_bringup full_system_mock.launch.py
```

CAN 插卡自动配置（在下位机安装一次，需要 sudo）：

```bash
bash ~/ros2_ws_car/src/robot_bringup/scripts/install_can_autosetup.sh
```

之后 `can0/can1` 插入或开机自动设置为 1 Mbps、txqueuelen 1000 并启用。使用 udev + systemd，支持拔插重连；安装不打断已 UP 的接口。详情及卸载命令见 [联动部署说明](../docs/上下位机联动迁移说明.md)。

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
