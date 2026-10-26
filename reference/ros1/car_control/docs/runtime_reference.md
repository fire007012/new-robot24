# car_control 运行参考

`car_control` 当前主要服务于迁移期联调。新增底盘能力优先放 `mobility_control`，新增机械臂上层能力优先放 `arm_control`。

## Launch 入口

| 命令 | 用途 |
| --- | --- |
| `roslaunch car_control control_nodes.launch` | 仅起底盘安全中继和夹爪控制 |
| `roslaunch car_control teleop_system.launch joy_dev:=/dev/input/js0` | 手柄 + 底盘 + 夹爪；可选带 Servo bridge |
| `roslaunch car_control moveit_servo.launch` | 单独启动 `servo_server` |
| `roslaunch car_control sim_test.launch joy_dev:=/dev/input/js0` | Gazebo + MoveIt + 手柄联调 |

## 运行链路

```text
/joy
  -> ds5_teleop_node
     |-- /car_control/cmd_vel -> base_cmd_node -> /car_urdf/cmd_vel
     |-- /servo_server/delta_twist_cmds
     `-- /car_control/gripper_open -> gripper_cmd_node

/car_control/delta_twist_cmds_optical
  -> servo_twist_frame_bridge_node
  -> /servo_server/delta_twist_cmds
```

## 手柄默认口径

- `Options`：模式切换
- `CHASSIS`：左摇杆控制前进/转向
- `ARM_SERVO`：左摇杆控制 `linear y/z`，`L2/R2` 控 `linear x`，`L1/R1` 控 `roll`
- `Square`：打开夹爪
- `Circle`：闭合夹爪

## 迁移边界

- 底盘正式北向入口应收敛到 `mobility_control/base_cmd_node`
- 机械臂离散目标执行应收敛到 `arm_control/arm_goal_executor_node`
- 本包保留兼容实现，优先保证现有联调可跑，不继续扩展历史设计文档
