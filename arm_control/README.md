# arm_control

`arm_control` 负责机械臂上层控制入口：离散目标执行、夹爪命令和 MoveIt Servo 坐标桥接。它不负责控制器生命周期、硬件写入或 CAN 下发；正式执行链仍在底层 bringup 中。

## 包职责

- 提供统一 Action 接口 `/arm_control/execute_goal`，支持 `named`、完整 6 轴 `joints`、`PoseStamped` 三类目标。
- 将 `/arm_control/gripper_open` 和 `/arm_control/gripper_position` 转成夹爪控制器可执行的 `JointTrajectory`。
- 将 optical frame 下的 `TwistStamped` 转换到 MoveIt Servo 使用的控制坐标系。
- 提供固定 `wrist_roll_joint` 的 3 个无参 service：正转 180 度、反转 180 度、回到零点。

## 包结构

```text
arm_control/
|-- action/
|   `-- ExecuteArmGoal.action
|-- config/
|   |-- control_params.yaml
|   |-- goal_executor.yaml
|   `-- wrist_roll_services.yaml
|-- docs/
|   |-- README.md
|   |-- arm_goal_executor.md
|   `-- 归档/
|-- launch/
|   |-- arm_goal_executor.launch
|   |-- gripper_cmd.launch
|   |-- servo_twist_frame_bridge.launch
|   `-- wrist_roll_services.launch
|-- scripts/
|   |-- arm_goal_executor_node.py
|   |-- gripper_cmd_node.py
|   |-- send_execute_arm_goal.py
|   |-- servo_twist_frame_bridge_node.py
|   `-- wrist_roll_service_node.py
`-- test/
    |-- README.md
    `-- test_execute_arm_goal_contract.py
```

## 快速开始

前置条件：

- 已启动 `move_group`
- `/controller_manager/list_controllers` 可访问
- `joint_state_controller` 与 `arm_position_controller` 处于 `running`
- `/arm_position_controller/follow_joint_trajectory` 可连通
- `Pose` 目标所需 TF 能转换到 `base_link`

```bash
cd /home/rera/robot24_ws
catkin_make --pkg arm_control
source devel/setup.bash
```

```bash
roslaunch arm_control arm_goal_executor.launch
roslaunch arm_control gripper_cmd.launch
roslaunch arm_control servo_twist_frame_bridge.launch
```

```bash
rosrun arm_control send_execute_arm_goal.py named --name ready
```

## 常用命令

```bash
# named target
rosrun arm_control send_execute_arm_goal.py named --name ready

# full joint target
rosrun arm_control send_execute_arm_goal.py joints \
  --joint shoulder_yaw_joint=0.0 \
  --joint shoulder_pitch_joint=-0.2 \
  --joint elbow_pitch_joint=1.0 \
  --joint wrist_pitch_joint=0.0 \
  --joint wrist_roll_joint=0.0 \
  --joint wrist_yaw_joint=0.0

# from current pose, rotate wrist_roll_joint by +pi
rosrun arm_control send_execute_arm_goal.py roll180

# preview the computed absolute target without sending
rosrun arm_control send_execute_arm_goal.py roll180 --dry-run

# wrist roll services
rosservice call /arm_control/wrist_roll_positive_180
rosservice call /arm_control/wrist_roll_negative_180
rosservice call /arm_control/wrist_roll_zero

# pose target
rosrun arm_control send_execute_arm_goal.py pose \
  --frame base_link --x 0.30 --y 0.00 --z 0.25 \
  --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0

# gripper
rostopic pub -1 /arm_control/gripper_open std_msgs/Bool 'data: true'
rostopic pub -1 /arm_control/gripper_position std_msgs/Float64 'data: 0.02'

# Servo optical -> control frame bridge
rostopic pub -1 /arm_control/delta_twist_cmds_optical geometry_msgs/TwistStamped \
  '{header: {frame_id: "catch_camera_optical_frame"}, twist: {linear: {z: 0.05}}}'

# Action status
rostopic echo /arm_control/execute_goal/status
```

## 节点 / API 摘要

| 入口 | 接口 | 说明 |
| --- | --- | --- |
| `arm_goal_executor_node.py` | Action `/arm_control/execute_goal` | 统一离散运动入口；规划参考系默认 `base_link`；执行前检查 controller 和 action server readiness。 |
| `gripper_cmd_node.py` | 订阅 `/arm_control/gripper_position`、`/arm_control/gripper_open`；发布 `/gripper_controller/command` | 夹爪位置/开合命令转 `JointTrajectory`。 |
| `servo_twist_frame_bridge_node.py` | 订阅 `/arm_control/delta_twist_cmds_optical`；发布 `/servo_server/delta_twist_cmds` | optical frame `TwistStamped` 转控制坐标系。 |
| `wrist_roll_service_node.py` | Services `/arm_control/wrist_roll_positive_180`、`/arm_control/wrist_roll_negative_180`、`/arm_control/wrist_roll_zero` | 固定 `wrist_roll_joint`；读取当前 `/joint_states`，保持其余 5 轴不变，再直接通过 MoveIt `MoveGroupCommander` 执行。 |
| `send_execute_arm_goal.py` | CLI | 最小 smoke client，便于快速验证 Action 合同和运行链路。 |

`ExecuteArmGoal.action` 约定：

- `TARGET_NAMED`：使用 `named_target`
- `TARGET_JOINTS`：要求 `joint_names` 与规划组 active joints 完全一致
- `TARGET_POSE`：使用 `pose_target`，且 `header.frame_id` 不能为空
- `ERROR_*`：覆盖 invalid goal、not ready、TF failed、planning failed、execution failed、preempted

## 详细文档索引

- [`docs/README.md`](docs/README.md)：当前文档导航
- [`docs/arm_goal_executor.md`](docs/arm_goal_executor.md)：Action 合同、前置条件和排障要点
- [`action/ExecuteArmGoal.action`](action/ExecuteArmGoal.action)：消息定义
- [`test/README.md`](test/README.md)：合同测试与 smoke 范围
- [`docs/归档/01_arm_control包设计指南_视觉抓取扩展.md`](docs/归档/01_arm_control包设计指南_视觉抓取扩展.md)：历史设计指南
- [`docs/归档/02_arm_goal_executor按commit实施计划.md`](docs/归档/02_arm_goal_executor按commit实施计划.md)：历史实施计划
