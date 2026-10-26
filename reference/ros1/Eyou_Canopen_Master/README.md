# Eyou_Canopen_Master

目录名当前是 `Eyou_Canopen_master/`，但包名与 `roslaunch` / `catkin_make --pkg` 使用的名字仍是 `Eyou_Canopen_Master`。这个包负责 CANopen 主站、`ros_control` 适配、生命周期服务和单后端 bringup。

## 包结构

```text
Eyou_Canopen_master/
|-- config/
|   |-- master.yaml / master.dcf
|   |-- joints.yaml
|   |-- master_arm_only.* / joints_arm_only.yaml
|   `-- master_flipper_4axis.* / joints_flipper_4axis.yaml
|-- docs/
|   |-- README.md
|   `-- archive/
|-- launch/
|   |-- canopen_hw.launch
|   |-- bringup.launch
|   |-- flipper_only.launch
|   |-- canopen_hw_flipper_4axis.launch
|   `-- bringup_flipper_4axis.launch
|-- scripts/
|   |-- joint_action_ui.py
|   `-- plot_ruckig_profile.py
|-- src/
|   |-- canopen_hw_ros_node.cpp
|   |-- canopen_master.cpp
|   |-- canopen_robot_hw_ros.cpp
|   |-- service_gateway.cpp
|   `-- controllers/ip_follow_joint_trajectory_executor_*.cpp
|-- srv/
|   |-- SetMode.srv
|   |-- SetZero.srv
|   `-- ApplyLimits.srv
`-- test/
    `-- test_*.cpp
```

## 快速开始

编译：

```bash
cd ~/robot24_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

标准单后端 bringup：

```bash
roslaunch Eyou_Canopen_Master bringup.launch
```

四摆臂 4 轴 bringup：

```bash
roslaunch Eyou_Canopen_Master flipper_only.launch
```

重新生成 DCF：

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh full
rosrun Eyou_Canopen_Master generate_dcf.sh arm_only
rosrun Eyou_Canopen_Master generate_dcf.sh flipper_only
```

启用 IP executor：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory
```

## 常用命令

生命周期：

```bash
rosservice call /canopen_hw_node/init "{}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/halt "{}"
rosservice call /canopen_hw_node/resume "{}"
rosservice call /canopen_hw_node/recover "{}"
rosservice call /canopen_hw_node/shutdown "{}"
```

维护服务：

```bash
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 8}"
rosservice call /canopen_hw_node/set_zero "{axis_index: 0, zero_offset_rad: 0.0, use_current_position_as_zero: true}"
rosservice call /canopen_hw_node/apply_limits "{axis_index: 0, min_position: -1.0, max_position: 1.0, use_urdf_limits: false, require_current_inside_limits: true}"
```

调试：

```bash
rosservice list | grep canopen_hw_node
rostopic echo /diagnostics
rosrun Eyou_Canopen_Master joint_action_ui.py
```

## 接口速查

节点：

- `canopen_hw_node`
  - launch 中实际运行的 ROS 节点是 `canopen_hw_ros_node`

服务：

- `~init`
- `~enable`
- `~disable`
- `~halt`
- `~resume`
- `~recover`
- `~shutdown`
- `~set_mode`
- `~set_zero`
- `~apply_limits`

关键配置：

- `config/master.yaml` / `config/master.dcf`
  - 默认全量 CANopen 电机配置
- `config/joints.yaml`
  - 默认全量 CANopen 关节与限位配置
- `config/master_arm_only.yaml` / `config/master_arm_only.dcf`
  - 机械臂肩部 2 轴专用配置
- `config/joints_arm_only.yaml`
  - 机械臂肩部 2 轴运行时配置
- `launch/bringup.launch`
  - 默认全量 CANopen bringup
- `launch/flipper_only.launch`
  - 四摆臂专用 bringup

关键能力：

- `ros_control` 适配与 controller_manager
- IP `FollowJointTrajectory` executor
- PDO 映射校验
- 生命周期状态机与辅助维护服务

## 使用边界

- 如果系统已经切到统一外观层，优先使用 `Eyou_ROS1_Master`，不要在上层直接绕过 facade 调底层。
- `joint_action_ui.py` 适合动作 / 轨迹链路验证，不覆盖 `flipper_control` 的 `csv_velocity` 调试路径。
- `docs/archive/` 里保留大量历史草案与 bug 记录，当前执行基线以顶层 README 与现行文档为准。

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 当前文档导航、命令速查和归档说明
