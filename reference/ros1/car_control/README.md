# car_control

`car_control` 目前是迁移期兼容包，保留整机联调仍在使用的遥操作、夹爪和仿真桥接能力。新的底盘控制入口优先看 `mobility_control`，新的机械臂上层入口优先看 `arm_control`；`car_control` 继续可用，但不再作为长期的功能归属包。

## 包职责

- 提供底盘与夹爪的兼容控制节点，维持当前联调入口。
- 提供 DS4/DS5 手柄到底盘、夹爪、MoveIt Servo 的分发逻辑。
- 提供 optical frame `TwistStamped` 桥接，以及 Gazebo 履带/里程计辅助节点。

## 包结构

```text
car_control/
|-- config/
|   |-- control_params.yaml
|   `-- moveit_servo.yaml
|-- docs/
|   |-- README.md
|   |-- runtime_reference.md
|   `-- 归档/
|-- launch/
|   |-- control_nodes.launch
|   |-- moveit_servo.launch
|   |-- servo_twist_frame_bridge.launch
|   |-- sim_test.launch
|   `-- teleop_system.launch
|-- scripts/
|   |-- base_cmd_node.py
|   |-- ds5_teleop_node.py
|   |-- gazebo_model_odom_node.py
|   |-- gripper_cmd_node.py
|   `-- servo_twist_frame_bridge_node.py
`-- src/
    `-- tracked_cmd_bridge_node.cpp
```

## 快速开始

```bash
cd /home/rera/robot24_ws
catkin_make --pkg car_control
source devel/setup.bash
```

```bash
# 底盘 + 夹爪
roslaunch car_control control_nodes.launch

# 手柄 + 底盘 + 夹爪
roslaunch car_control teleop_system.launch joy_dev:=/dev/input/js0

# 仿真联调
roslaunch car_control sim_test.launch joy_dev:=/dev/input/js0
```

如果只需要 MoveIt Servo：

```bash
roslaunch car_control moveit_servo.launch
roslaunch car_control servo_twist_frame_bridge.launch
```

## 常用命令

```bash
# 底盘前进
rostopic pub -1 /car_control/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'

# 夹爪
rostopic pub -1 /car_control/gripper_open std_msgs/Bool 'data: true'
rostopic pub -1 /car_control/gripper_position std_msgs/Float64 'data: 0.02'

# 直接给 MoveIt Servo 发控制量
rostopic pub -1 /servo_server/delta_twist_cmds geometry_msgs/TwistStamped \
  '{header: {frame_id: "catch_camera"}, twist: {linear: {x: 0.05}}}'

# optical frame 输入桥接
rostopic pub -1 /car_control/delta_twist_cmds_optical geometry_msgs/TwistStamped \
  '{header: {frame_id: "catch_camera_optical_frame"}, twist: {linear: {z: 0.05}}}'

# 观察手柄分发后的输出
rostopic echo /car_control/cmd_vel
rostopic echo /servo_server/delta_twist_cmds
```

## 节点 / API 摘要

| 节点 | 接口 | 说明 |
| --- | --- | --- |
| `base_cmd_node.py` | `/car_control/cmd_vel` -> `/car_urdf/cmd_vel` | 底盘速度限幅和超时刹停；迁移期保留实现。 |
| `gripper_cmd_node.py` | 订阅 `/car_control/gripper_position`、`/car_control/gripper_open`；发布 `/gripper_controller/command` | 夹爪命令转 `JointTrajectory`。 |
| `ds5_teleop_node.py` | 订阅 `/joy`；发布 `/car_control/cmd_vel`、`/servo_server/delta_twist_cmds`、`/car_control/gripper_open` | `Options` 切换 `CHASSIS` / `ARM_SERVO`；`Square` 开爪，`Circle` 合爪。 |
| `servo_twist_frame_bridge_node.py` | `/car_control/delta_twist_cmds_optical` -> `/servo_server/delta_twist_cmds` | optical frame 到 `catch_camera` 的 `TwistStamped` 桥接。 |
| `gazebo_model_odom_node.py` | `/gazebo/model_states` -> `/car_urdf/odom` | 从 Gazebo 模型状态重建标准里程计，可选发布 TF。 |
| `tracked_cmd_bridge_node.cpp` | `/car_urdf/cmd_vel` -> Gazebo transport `~/car_urdf/cmd_vel_twist` | 仿真履带桥接；不是默认 launch 入口。 |

坐标系约定：

- 手柄与 MoveIt Servo 默认使用 `catch_camera`
- 视觉链如果输出 `catch_camera_optical_frame`，先经过桥接再送 Servo

## 详细文档索引

- [`docs/README.md`](docs/README.md)：当前文档导航
- [`docs/runtime_reference.md`](docs/runtime_reference.md)：launch 入口、节点关系和迁移说明
- [`config/control_params.yaml`](config/control_params.yaml)：所有节点参数
- [`launch/teleop_system.launch`](launch/teleop_system.launch)：常用联调入口
- [`launch/sim_test.launch`](launch/sim_test.launch)：仿真总入口
- [`docs/归档/car_control_report.md`](docs/归档/car_control_report.md)：历史梳理报告
