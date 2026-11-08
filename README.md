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

`can_driver`、`ros2_canopen` 由项目自主维护；其余四个子模块主要作为第三方依赖，优先使用上游原版，项目适配放在项目包中，非必要不修改依赖源码。具体边界见 [系统架构](docs/06_项目架构图与数据流向图.md#子仓库职责与维护边界)。

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

三个入口择一运行。整车入口默认启用 `robot_safety/config/group_policy_v1.yaml`，启动后硬件与运动控制器不会自动进入运行。mock 示例的显式操作见 [当前运行说明](docs/36_ROS2迁移剩余事项清单.md)；实机操作见 [现场验收](docs/28_CANopen软件交付与实机验收.md) 和 [分组操作](docs/31_分组隔离与局部恢复实施验收.md)。`init → enable (Armed) → resume (Running)` 建立新会话；故障恢复后也需要显式放行并发送新目标。

`full_system.launch.py` 默认启动主控制与 MoveIt，`enable_servo` 默认为 false，`ui` 默认为 none；相机、视觉、上位机桥、CO₂ 和热成像也默认关闭，需分别启用 `enable_cameras/enable_vision/enable_host/enable_co2/enable_thermal`。从整车入口启用 D405 时还须指定 `enable_paw:=true paw_serial:=实际序列号`；该入口的 `paw_serial` 默认为空。相机独立入口及历史参数来源见 [参数说明 37](docs/37_ROS1实机参数迁入ROS2.md)。手眼标定使用 `eye_in_hand` 的独立启动入口。

六轴规划统一经 `arm_control/ExecuteArmGoal` 发往同一个 JTC；MoveIt 的 `move_group` 仅提供规划，JTC 与 Servo 互斥。模型 TF 使用过滤后的 `/robot_model/joint_states`，安全与就绪检查仍消费原始测量。

## 当前完成范围

2026-11-08 恢复第三方上游依赖后的整车 `full` 连续回归已通过：37 包构建、11 包的 43 个 CTest 入口，以及控制、MoveIt、Servo/UI、合成视觉/H265、上位机/视频控制、模拟 CO₂、自动采样、原生标定和 Fortress 全部通过。匹配版本、持久证据和运行方法见 [清单 36](docs/36_ROS2迁移剩余事项清单.md)。无实机现场准备已完成，生成工具为 `tools/prepare_commissioning.py`；真实 CAN、关联条件、相机/marker、YOLO 权重、完整 RTSP 及外围仍待验收。设备和自动采样配置保持 `commissioned: false`。

[文档索引](docs/README.md) · [系统架构](docs/06_项目架构图与数据流向图.md) · [实施计划与待办](docs/25_ROS1迁移ROS2实施计划.md)
