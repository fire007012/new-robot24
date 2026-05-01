# Eyou_ROS1_Master

`Eyou_ROS1_Master` 是当前工作区的统一硬件外观层，负责把 `can_driver` 和 `Eyou_Canopen_Master` 两套后端收口到一个 `controller_manager`、一组生命周期服务和一套按关节运行态接口下。

## 包结构

```text
Eyou_ROS1_Master/
|-- config/
|   |-- controllers_jtc.yaml
|   |-- controllers_minimal.yaml
|   `-- joint_mode_mappings.yaml
|-- docs/
|   |-- README.md
|   `-- 归档/
|-- launch/
|   |-- arm_only.launch
|   |-- hybrid_motor_hw.launch
|   `-- hybrid_moveit_servo.launch
|-- msg/
|   |-- JointRuntimeState.msg
|   `-- JointRuntimeStateArray.msg
|-- scripts/
|   `-- hybrid_joint_action_ui.py
|-- src/
|   |-- hybrid_motor_hw_node.cpp
|   |-- hybrid_robot_hw.cpp
|   |-- hybrid_operational_coordinator.cpp
|   |-- hybrid_service_gateway.cpp
|   |-- hybrid_mode_router.cpp
|   `-- hybrid_auto_startup.cpp
|-- srv/
|   |-- SetJointMode.srv
|   |-- SetJointZero.srv
|   `-- ApplyJointLimits.srv
`-- tests/
    `-- test_hybrid_*.cpp
```

## 快速开始

编译时通常一起带上两个后端：

```bash
cd ~/robot24_ws
catkin_make --pkg can_driver Eyou_Canopen_Master Eyou_ROS1_Master
source devel/setup.bash
```

标准启动入口：

```bash
roslaunch Eyou_ROS1_Master hybrid_motor_hw.launch
```

仅启动机械臂：

```bash
roslaunch Eyou_ROS1_Master arm_only.launch
```

覆盖关键配置路径：

```bash
roslaunch Eyou_ROS1_Master hybrid_motor_hw.launch \
  canopen_dcf_path:=$(rospack find Eyou_Canopen_Master)/config/master.dcf \
  canopen_joints_path:=$(rospack find Eyou_Canopen_Master)/config/joints.yaml \
  can_driver_config:=$(rospack find can_driver)/config/can_driver.yaml
```

## 常用命令

生命周期：

```bash
rosservice call /hybrid_motor_hw_node/init "{}"
rosservice call /hybrid_motor_hw_node/enable "{}"
rosservice call /hybrid_motor_hw_node/halt "{}"
rosservice call /hybrid_motor_hw_node/resume "{}"
rosservice call /hybrid_motor_hw_node/recover "{}"
rosservice call /hybrid_motor_hw_node/shutdown "{}"
```

按关节服务：

```bash
rosservice call /hybrid_motor_hw_node/set_joint_mode "{joint_name: 'left_front_arm_joint', mode: 'position'}"
rosservice call /hybrid_motor_hw_node/set_joint_zero "{joint_name: 'left_front_arm_joint', zero_offset_rad: 0.0, use_current_position_as_zero: true}"
rosservice call /hybrid_motor_hw_node/apply_joint_limits "{joint_name: 'left_front_arm_joint', min_position_rad: -1.0, max_position_rad: 1.0, use_urdf_limits: false, require_current_inside_limits: true}"
```

查看运行态：

```bash
rostopic echo /hybrid_motor_hw_node/joint_runtime_states
rosservice list | grep hybrid_motor_hw_node
```

后端原生维护服务：

```bash
rosservice call /hybrid_motor_hw_node/can_driver_node/set_zero "{motor_id: 1, zero_offset_rad: 0.0, use_current_position_as_zero: true, apply_to_motor: false}"
rosservice call /hybrid_motor_hw_node/can_driver_node/apply_limits "{motor_id: 1, min_position_rad: -1.0, max_position_rad: 1.0, use_urdf_limits: false, apply_to_motor: false, require_current_inside_limits: true}"
rosservice call /hybrid_motor_hw_node/canopen_node/set_zero "{axis_index: 0, zero_offset_rad: 0.0, use_current_position_as_zero: true}"
rosservice call /hybrid_motor_hw_node/canopen_node/apply_limits "{axis_index: 0, use_urdf_limits: false, min_position: -1.0, max_position: 1.0, require_current_inside_limits: true}"
```

启动混合关节调试 UI：

```bash
rosrun Eyou_ROS1_Master hybrid_joint_action_ui.py \
  --service-ns /hybrid_motor_hw_node
```

## 接口速查

节点：

- `hybrid_motor_hw_node`
  - 统一硬件门面节点

发布：

- `~joint_runtime_states` `Eyou_ROS1_Master/JointRuntimeStateArray`

服务：

- `~init`
- `~enable`
- `~disable`
- `~halt`
- `~resume`
- `~recover`
- `~shutdown`
- `~set_joint_mode`
- `~set_joint_zero`
- `~apply_joint_limits`

后端原生维护服务：

- `~can_driver_node/set_zero`
- `~can_driver_node/apply_limits`
- `~can_driver_node/set_zero_limit`
- `~canopen_node/set_mode`
- `~canopen_node/set_zero`
- `~canopen_node/apply_limits`

关键配置：

- `config/controllers_jtc.yaml`
  - 默认控制器集合
- `config/joint_mode_mappings.yaml`
  - 统一 mode 字符串到后端实际模式的映射
- `launch/hybrid_motor_hw.launch`
  - 默认启动入口
- `launch/arm_only.launch`
  - 机械臂专用入口，使用 `master_arm_only.dcf + joints_arm_only.yaml + can_driver_arm_only.yaml`

关键消息 / 服务：

- `msg/JointRuntimeState.msg`
- `msg/JointRuntimeStateArray.msg`
- `srv/SetJointMode.srv`
- `srv/SetJointZero.srv`
- `srv/ApplyJointLimits.srv`

## 使用边界

- 当前正式生命周期 authority 在这个包，不要在 `arm_control` / `mobility_control` / `flipper_control` 再重复维护一套状态机。
- 该包依赖 `can_driver` 与 `Eyou_Canopen_Master` 的底层配置正确，自己不重复维护它们的原生 YAML 语义。
- 上层控制包应依赖这里暴露的统一控制器和服务，而不是直接跨过 facade 调底层后端。
- `~set_zero` / `~apply_limits` 不再直接暴露在 `hybrid_motor_hw_node` 根下，避免与两套后端原生服务重名冲突。

## 文档入口

- [`docs/README.md`](docs/README.md)
  - 导航、归档与相关资料入口
