# Robot24 Catkin Workspace

基于 ROS Noetic 的机器人工作空间，当前主链路覆盖底盘、摆臂、夹抓、MoveIt、Gazebo 全机仿真与实机统一 bringup。

当前工作区处于 ROS2 迁移准备阶段：手眼标定子模块已由 ROS1 `easy_handeye` 替换为 ROS2 [`easy_handeye2`](easy_handeye2/README.md)，其余主控制链仍为 ROS1。`Eye_in_hand` 的旧调用端尚待迁移；当前目录不能直接整体作为纯 Catkin 或纯 ROS2 工作空间构建。迁移任务见 [ROS1 迁移 ROS2 实施计划](docs/25_ROS1迁移ROS2实施计划.md)。

## 当前主链路

核心包：

| 包 | 说明 |
| --- | --- |
| `robot_bringup` | 顶层启动入口，统一组织实机与仿真链路 |
| `Eyou_ROS1_Master` | 实机统一硬件外观层与 controller manager bringup |
| `Eyou_Canopen_Master` | CANopen 后端与 ROS 控制适配 |
| `can_driver` | SocketCAN 电机驱动与底盘/夹抓后端 |
| `car_moveit_config` | MoveIt 配置、Move Group、Gazebo 仿真链路 |
| `car_urdf` | 整机 URDF、ros_control 接口与模型配置 |
| `mobility_control` | `/cmd_vel` 到底盘控制器的桥接与限幅 |
| `flipper_control` | 摆臂控制管理节点 |
| `arm_control` | 夹抓/机械臂上行控制接口 |

辅助与历史包：

| 包 | 说明 |
| --- | --- |
| `can_driver_ui` | 驱动相关可视化与调试 UI |
| `vision_pkg` | 视觉感知与拼接/检测节点 |
| `vision_opencv` | OpenCV 相关 ROS 包，上游代码 |
| `realsense-ros` | RealSense ROS 包，上游代码 |

## 包文档入口

- `robot_bringup`: [robot_bringup/README.md](robot_bringup/README.md)
- `Eyou_ROS1_Master`: [Eyou_ROS1_Master/README.md](Eyou_ROS1_Master/README.md)
- `Eyou_Canopen_Master`: [Eyou_Canopen_Master/README.md](Eyou_Canopen_Master/README.md)
- `can_driver`: [can_driver/README.md](can_driver/README.md)
- `mobility_control`: [mobility_control/README.md](mobility_control/README.md)
- `flipper_control`: [flipper_control/README.md](flipper_control/README.md)
- `arm_control`: [arm_control/README.md](arm_control/README.md)
- `car_control`: [car_control/README.md](car_control/README.md)
- `car_moveit_config`: [car_moveit_config/README.md](car_moveit_config/README.md)
- `car_urdf`: [car_urdf/README.md](car_urdf/README.md)
- `vision_pkg`: [vision_pkg/README.md](vision_pkg/README.md)
- `can_driver_ui`: [can_driver_ui/README.md](can_driver_ui/README.md)

## 启动入口

实机入口：

```bash
roslaunch robot_bringup full_system.launch
```

全机仿真入口：

```bash
roslaunch robot_bringup full_system_simulate.launch
```

说明：

- `full_system.launch` 当前是实机链路
- `full_system_simulate.launch` 当前是 Gazebo 全机仿真链路
- `hardware_damiao_system.launch` 已不再作为正式入口

## 目录结构

```text
.
├── Eyou_Canopen_Master/
├── Eyou_ROS1_Master/
├── arm_control/
├── can_driver/
├── can_driver_ui/
├── car_moveit_config/
├── car_urdf/
├── docs/
├── flipper_control/
├── mobility_control/
├── realsense-ros/
├── robot_bringup/
├── vision_opencv/
├── vision_pkg/
└── ...
```

## 依赖与迁移

基础环境：

- Ubuntu 20.04
- ROS Noetic
- Catkin

迁移设备时，优先参考完整依赖清单：

- [docs/24_迁移设备外部依赖清单_2026-04-19.md](docs/24_迁移设备外部依赖清单_2026-04-19.md)

常见核心依赖包括：

- `ros-noetic-roscpp`
- `ros-noetic-rospy`
- `ros-noetic-controller-manager`
- `ros-noetic-diff-drive-controller`
- `ros-noetic-joint-state-controller`
- `ros-noetic-moveit-commander`
- `ros-noetic-moveit-ros-move-group`
- `ros-noetic-rviz`
- `gazebo11`
- `libyaml-cpp-dev`
- `libspdlog-dev`
- Lely CANopen 开发库

推荐先使用 `rosdep`：

```bash
sudo apt-get update
sudo apt-get install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

注意：

- 当前仓库混有视觉上游包与 vendor 示例目录，`rosdep` 结果会比正式主链路更宽
- `Eyou_Canopen_Master` 和 `Eyou_ROS1_Master` 额外需要系统可用的 Lely `pkg-config` 条目
- `can_driver/lib/myactuator_rmd-main` 与 `can_driver/lib/myactuator_rmd_ros-main` 是 vendor 进来的 ROS2/ament 示例树，已通过 `CATKIN_IGNORE` 排除，不参与当前 ROS1 工作空间发现

## 编译

```bash
cd ~/Robot24_catkin_ws
catkin_make
source devel/setup.bash
```

## 测试

```bash
cd ~/Robot24_catkin_ws
catkin_make run_tests -j2
catkin_test_results build
```

## 文档

设计、bringup、修复与迁移说明见：

- [docs/README.md](docs/README.md)
