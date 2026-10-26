# Robot24 · ROS2 整车工作区

本项目使用 **Ubuntu 22.04 / ROS2 Humble**，通过一个 `controller_manager` 管理 CANopen 与私有 CAN 两个硬件组件，提供六轴机械臂、履带、四摆臂、夹爪，以及 MoveIt 2、Fortress 仿真、相机视觉、手眼标定和上位机接口。

ROS1 代码已集中到 [reference/ros1](reference/ros1/README.md)，仅供参考；项目不再安排 ROS1 基线回退。参考目录含 `COLCON_IGNORE`，活动包统一使用 ament。

## 目录与职责

| 目录 | 当前职责 |
| --- | --- |
| robot_bringup / robot_full_system | 整车 launch、模型状态转发、就绪检查 / 全量依赖集合 |
| robot_hardware / robot_safety | 唯一控制管理器、控制器、生命周期、维护、分组运行许可与输入会话 |
| can_driver / ros2_canopen / robot_canopen_config | 私有 CAN 后端 / CANopen 后端 / 六轴总线配置生成与校验 |
| car_urdf / car_moveit_config / arm_control | 模型 / MoveIt 2 规划 / ExecuteArmGoal 和腕部、夹爪适配 |
| mobility_control / car_control / flipper_control | 底盘 Ruckig / Servo、遥控与监控 / 原生 Qt 摆臂调试界面 |
| robot_simulation | Gazebo Fortress 物理仿真与传感器 |
| vision_pkg / robot_camera_sdk | 图像、深度、检测、H265 与相机启动 / 安装本地固定版本 SDK |
| realsense-ros / OrbbecSDK_ROS2 | ROS2 相机驱动子模块；后者为支持 Astra+ 的 SDK v1 分支 |
| eye_in_hand / aruco_ros / easy_handeye2 | 手眼采样与启动 / marker 检测 / 求解、保存和外参发布 |
| host_bridge | TCP/JSON 上位机协议、视频管理、ROS2 图像转 RTSP |
| senseair_co2 / thermal_camera | CO₂ 与热成像外围接口 |
| tools / docs / reference | 依赖与回归工具 / 当前架构和现场操作 / 历史参考 |

`host_bridge` 是原目录 `ros1_bridge` 的 ROS2 业务包；`eye_in_hand` 是原 `Eye_in_hand`。源码布局统一使用 `launch/`、`src/`、`scripts/` 和 `config/`。

## 构建与验证

从本仓库根目录（工作区的 `src`）执行，先初始化 [6 个活动子模块](docs/git_submodule_workflow.md)。构建脚本使用系统安装的 Lely，首次环境准备如下：

```bash
bash tools/install_ros2_dependencies.sh --dry-run
bash tools/install_ros2_dependencies.sh
# 仅全量相机集合需要；固定 librealsense 2.58.4 解包到 .deps/，不覆盖系统 SDK。
bash tools/prepare_camera_sdk.sh

# core 默认构建主控制链；full 构建完整的 37 个 ROS2 包。
bash tools/build_ros2.sh /tmp/robot24-ros2 full
source /tmp/robot24-ros2-install/setup.bash
bash tools/validate_ros2.sh /tmp/robot24-ros2 full
```

构建、安装、CTest 和 ROS 日志均写入指定的 `/tmp` 前缀。集成测试需要本机回环 DDS/TCP；Fortress 软件渲染需要 X display。`test/`、`tests/` 和 `tools/test/` 是持续回归源码，应保留；完成批次的日志、截图、evidence 和缓存不放在源码树。

## 启动

```bash
source /tmp/robot24-ros2-install/setup.bash
ros2 launch robot_bringup full_system_mock.launch.py
# 原生 Qt 界面和 Servo 按需打开：enable_servo:=true ui:=keyboard
ros2 launch robot_simulation full_system_simulate.launch.py gui:=false
# 实机入口：先完成设备登记与单轴验收，再在独立终端运行。
# ros2 launch robot_bringup full_system.launch.py baseline:=/实际路径/commissioning.yaml
```

三个入口择一运行。整车入口默认启用 `robot_safety/config/group_policy_v1.yaml`，启动后硬件与运动控制器不会自动进入运行。mock 示例的显式操作见 [主控制链说明](docs/33_ROS2主控制链与MoveIt2实施记录.md)；实机操作见 [现场验收](docs/28_CANopen软件交付与实机验收.md) 和 [分组操作](docs/31_分组隔离与局部恢复实施验收.md)。`init → enable (Armed) → resume (Running)` 建立新会话；故障恢复后也需要显式放行并发送新目标。

六轴规划统一经 `arm_control/ExecuteArmGoal` 发往同一个 JTC；MoveIt 的 `move_group` 仅提供规划，JTC 与 Servo 互斥。模型 TF 使用过滤后的 `/robot_model/joint_states`，安全与就绪检查仍消费原始测量。

## 当前完成范围

37 包全量构建、11 包的 41 个 CTest 入口，以及控制、MoveIt、Servo/UI、合成视觉、上位机/视频控制、模拟 CO₂、合成 TF 标定、Fortress 集成已通过软件验收，见 [记录 34](docs/34_ROS2全量软件集成与验收.md)。本次目录整理的复测结果与连续回归限制见 [整理说明 35](docs/35_ROS2工作区整理说明.md)。实机 CAN 时序/制动/参数、真实相机与 marker、YOLO 权重、真实 RTSP 拉流及外围设备尚待验收；标定自动采样配置保持 `commissioned: false`。

[文档索引](docs/README.md) · [系统架构](docs/06_项目架构图与数据流向图.md) · [实施计划与待办](docs/25_ROS1迁移ROS2实施计划.md)
