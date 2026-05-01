# robot_bringup

顶层系统编排包，负责把底盘、摆臂、夹爪、MoveIt 和验收脚本组合成统一启动入口。

## 包职责

- 提供仿真和实机的顶层 launch 入口。
- 提供 MoveIt 键盘微调工具和拓扑就绪检查脚本。
- 约束上层统一入口，减少各子系统单独启动顺序问题。

## 包结构

```text
robot_bringup/
|-- launch/
|   |-- acceptance_smoke.launch
|   |-- full_system.launch
|   |-- full_system_control.launch
|   |-- full_system_hardware.launch
|   |-- full_system_moveit.launch
|   |-- full_system_simulate.launch
|   |-- full_system_vision.launch
|   `-- keyboard_moveit_server.launch
`-- scripts/
    |-- bringup_can_interfaces.sh
    |-- bringup_canable0.sh
    |-- bringup_canable1.sh
    |-- keyboard_moveit_server.py
    |-- start_full_system_split.sh
    `-- wait_for_graph.sh
```

## 快速开始

先跑仿真总入口：

```bash
cd ~/robot24_ws
catkin_make --pkg robot_bringup car_moveit_config car_urdf
source devel/setup.bash
roslaunch robot_bringup full_system_simulate.launch
```

实机入口：

```bash
roslaunch robot_bringup full_system.launch
```

四终端分组启动：

```bash
rosrun robot_bringup start_full_system_split.sh
```

按组传参数时，使用环境变量：

```bash
HARDWARE_ARGS="auto_init:=true auto_enable:=true" \
MOVEIT_ARGS="enable_rviz:=false" \
VISION_ARGS="enable_paw_vision:=true" \
rosrun robot_bringup start_full_system_split.sh
```

分组启动入口已经下放到各自包内维护：

```bash
# 仅启动机械臂（跨包 hybrid）
roslaunch Eyou_ROS1_Master arm_only.launch

# 仅启动摆臂（CANopen 4 轴）
roslaunch Eyou_Canopen_Master flipper_only.launch

# 仅启动履带（can_driver + diff_drive_controller）
roslaunch can_driver tracks_only.launch
```

`full_system.launch` 依赖 `Eyou_ROS1_Master`，并默认从 `Eyou_Canopen_Master` 读取 DCF 和关节配置；如果包名或路径不同，需要显式覆盖 `canopen_dcf_path` 和 `canopen_joints_path`。

## 常用命令

```bash
# 无界面冒烟验收
roslaunch robot_bringup acceptance_smoke.launch

# 启动 MoveIt 键盘微调
roslaunch robot_bringup keyboard_moveit_server.launch

# 仿真入口，关闭 RViz
roslaunch robot_bringup full_system_simulate.launch enable_rviz:=false

# 实机入口，关闭 RViz
roslaunch robot_bringup full_system.launch enable_rviz:=false

# 四终端分组启动
rosrun robot_bringup start_full_system_split.sh

# 同时启动 canable0 和 canable1
rosrun robot_bringup bringup_can_interfaces.sh

# 仅启动 canable0
rosrun robot_bringup bringup_canable0.sh

# 仅启动 canable1
rosrun robot_bringup bringup_canable1.sh
```

## 节点与接口摘要

- `full_system_simulate.launch`
  - 启动 Gazebo、底盘控制、摆臂控制、夹爪命令、MoveIt、RViz。
- `full_system.launch`
  - 以 `Eyou_ROS1_Master` 为硬件统一门面，再接入同一套上层控制节点。
- `full_system_hardware.launch`
  - 电机硬件与控制器管理。
- `full_system_control.launch`
  - 底盘、翻转臂、夹爪控制节点。
- `full_system_moveit.launch`
  - MoveIt、MoveIt Servo、RViz。
- `full_system_vision.launch`
  - 爪端视觉与可选热成像。
- 分组启动入口
  - `Eyou_ROS1_Master/arm_only.launch`
  - `Eyou_Canopen_Master/flipper_only.launch`
  - `can_driver/tracks_only.launch`
- `keyboard_moveit_server.py`
  - 笛卡尔平移：`w/a/s/d/r/f`
  - 姿态调节：`i/j/k/l/u/o`
  - 停止：`Space`
  - 退出：`q`
- `wait_for_graph.sh`
  - 检查 `/gazebo/model_states`
  - 检查 `/wheel_controller/cmd_vel`
  - 检查 `/flipper_control/state`
  - 检查 `/move_group/status`
  - 检查两条 `follow_joint_trajectory` goal 接口
  - 检查 `/joint_states`
- `bringup_can_interfaces.sh`
  - 依次按 `1000000` bitrate 和 `10000` txqueuelen 配置并拉起 `canable0`、`canable1`
- `bringup_canable0.sh`
  - 单独配置并拉起 `canable0`
- `bringup_canable1.sh`
  - 单独配置并拉起 `canable1`

## 详细文档索引

- `README.md`：当前维护入口。
- `docs/README.md`：包内文档索引。
