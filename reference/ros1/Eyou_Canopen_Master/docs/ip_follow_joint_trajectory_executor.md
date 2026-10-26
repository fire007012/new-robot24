# IP FollowJointTrajectory 执行器（当前实现）

更新时间：2026-03-25  
适用范围：`Eyou_Canopen_Master`

---

## 1. 现状结论

当前 `IpFollowJointTrajectoryExecutor` 已是**多轴实现**，不再是单轴 MVP。

- 接收一个 `FollowJointTrajectory` action goal，内部按 `joint_names` 做映射。
- 支持动态 DOF（`Ruckig<0>`），轴数由 `joints.yaml` 决定。
- 在 `canopen_hw_ros_node` 中以**单个 executor** 挂接全轴，不再“每轴一个 action server”。
- 执行器默认仍由参数开关控制（`use_ip_executor`）。

---

## 2. 代码位置

公开头文件：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`

实现拆分：

- `src/controllers/ip_follow_joint_trajectory_executor_validation.cpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `src/controllers/ip_follow_joint_trajectory_executor_ros.cpp`
- `src/controllers/ip_follow_joint_trajectory_executor_internal.hpp`

节点集成：

- `src/canopen_hw_ros_node.cpp`

单元测试：

- `test/test_ip_follow_joint_trajectory_executor.cpp`

---

## 3. 数据与执行语义

### 3.1 输入语义

`ValidateGoal()` 当前要求：

1. `goal.trajectory.joint_names` 数量与配置轴数一致。
2. 关节名必须都能映射到配置关节，且不允许重复。
3. 每个轨迹点 `positions` 长度必须等于 DOF。
4. `velocities/accelerations` 允许为空；非空时长度必须等于 DOF。
5. `time_from_start` 必须单调非递减。

### 3.2 起点语义

新 goal 起点来自当前硬件实际状态：

- `current_position` ← `CanopenRobotHwRos::joint_position`
- `current_velocity` ← `CanopenRobotHwRos::joint_velocity`

不是从旧 goal 或旧 desired 延续。

### 3.3 多轴映射语义

执行器构造 `goal_joint_index -> config_axis_index` 映射，允许 goal 关节顺序与配置顺序不同。  
Ruckig 与下发命令都按配置轴顺序写入，确保与下层索引一致。

### 3.4 终点判定

完成判定按每轴容差进行：

- `abs(actual - target) <= goal_tolerances[axis]`

若 Ruckig 段完成但尚未全轴到位，执行器继续保持终点命令（速度/加速度置零）直到到位。

---

## 4. 运行参数

ROS 参数（由 `canopen_hw_ros_node` 读取）：

- `use_ip_executor`：是否启用执行器（默认 `false`）
- `ip_executor_rate_hz`：执行器更新频率
- `ip_executor_action_ns`：action namespace（默认 `arm_position_controller/follow_joint_trajectory`）

每轴约束来源（`joints.yaml`）：

- `ip_max_velocity`
- `ip_max_acceleration`
- `ip_max_jerk`
- `ip_goal_tolerance`

说明：DOF 与每轴约束长度由 `joints.yaml` 的 `joints` 列表决定。

---

## 5. 主循环接线

在 `canopen_hw_ros_node` 控制循环中的顺序为：

1. `robot_hw_ros.read(now, period)`
2. `cm.update(now, period)`
3. `ip_executor->update(now, period)`（仅启用时）
4. `robot_hw_ros.write(now, period)`

该顺序保证 executor 在控制器更新后仍可覆盖最终外部位置命令。

---

## 6. 已知边界

1. 当前执行器目标是 IP 点流执行，不等价于完整 `JointTrajectoryController` 语义。
2. MoveIt 可继续对接标准 `FollowJointTrajectory` action，但复杂 tolerance 语义仍需按现场策略补充。
3. 启用前建议先完成 `CanopenRobotHwRos.*` 与 `IpFollowJointTrajectoryExecutor.*` 回归测试。

---

## 7. 最小验证命令

```bash
cd ~/Robot24_catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=Eyou_Canopen_Master --no-color
ctest -R '^(CanopenRobotHwRos|IpFollowJointTrajectoryExecutor)\.' --output-on-failure
```
