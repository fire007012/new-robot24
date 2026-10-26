# arm_goal_executor 运行参考

`arm_goal_executor_node.py` 是 `arm_control` 的主执行入口，Action 名称默认是 `/arm_control/execute_goal`。

## 前置条件

- `move_group` 已启动
- `/controller_manager/list_controllers` 可调用
- `joint_state_controller` 与 `arm_position_controller` 为 `running`
- `/arm_position_controller/follow_joint_trajectory` 可连接
- `Pose` 目标的 `frame_id` 能转换到 `base_link`

## Action 合同

| 类型 | 必填字段 | 约束 |
| --- | --- | --- |
| `TARGET_NAMED` | `named_target` | 必须是 MoveIt 运行时可查询到的 named target |
| `TARGET_JOINTS` | `joint_names`、`joint_positions` | 名称集合必须与规划组 active joints 完全一致 |
| `TARGET_POSE` | `pose_target` | `header.frame_id` 不能为空；节点内部会尝试 TF 转换到 `base_link` |

结果码：

- `ERROR_NONE`
- `ERROR_INVALID_GOAL`
- `ERROR_NOT_READY`
- `ERROR_TF_FAILED`
- `ERROR_PLANNING_FAILED`
- `ERROR_EXECUTION_FAILED`
- `ERROR_PREEMPTED`

反馈阶段：

- `PHASE_VALIDATING`
- `PHASE_WAITING_READY`
- `PHASE_PLANNING`
- `PHASE_EXECUTING`
- `PHASE_STOPPING`

## 常用命令

```bash
roslaunch arm_control arm_goal_executor.launch

rosrun arm_control send_execute_arm_goal.py named --name ready

rosrun arm_control send_execute_arm_goal.py joints \
  --joint shoulder_yaw_joint=0.0 \
  --joint shoulder_pitch_joint=-0.2 \
  --joint elbow_pitch_joint=1.0 \
  --joint wrist_pitch_joint=0.0 \
  --joint wrist_roll_joint=0.0 \
  --joint wrist_yaw_joint=0.0

rosrun arm_control send_execute_arm_goal.py pose \
  --frame base_link --x 0.30 --y 0.00 --z 0.25 \
  --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0
```

## 常见失败点

- `ERROR_NOT_READY`：先检查 controller 是否为 `running`，以及轨迹 action server 是否起来。
- `ERROR_TF_FAILED`：`pose_target.header.frame_id` 为空，或 TF 链路缺失。
- `ERROR_INVALID_GOAL`：关节目标缺轴、重名或名称集合不匹配 active joints。
