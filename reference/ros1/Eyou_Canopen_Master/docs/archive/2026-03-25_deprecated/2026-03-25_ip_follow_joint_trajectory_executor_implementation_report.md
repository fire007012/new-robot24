# 实现报告：IP FollowJointTrajectory 执行器（单轴阶段）

日期：2026-03-25  
范围：`Eyou_Canopen_Master`

---

## 1. 报告目的

本文档记录本轮单轴 `IP FollowJointTrajectory` 执行器的实际落地情况，回答两个问题：

1. 已经具体做了什么；
2. 当前代码状态距离“现场可用”还差什么。

本文档描述的是**当前实现状态**，不是设计目标。

---

## 2. 结论摘要

当前已经完成：

- 单轴执行器的代码骨架；
- `FollowJointTrajectory` goal 基本校验；
- 基于 Ruckig 的单轴“从实际状态起步 + 周期更新”执行核心；
- 在 `canopen_hw_ros_node` 中加入执行器参数和主循环挂点；
- 相关单元测试；
- 构建链路接入 `Ruckig` 动态库；
- 默认行为保持不变：**执行器默认关闭**。

当前还未完成：

- 完整的 action feedback 周期发布；
- 完整的 cancel / preempt / abort 现场行为验证；
- 单轴现场抓包闭环验收；
- 6 轴扩展；
- 文档级使用手册更新。

所以当前状态应评估为：

- **代码骨架与核心执行逻辑已落地**
- **默认不启用，不影响现有系统**
- **尚未达到“现场替换 JTC”的最终完成态**

---

## 3. 已完成提交

### 3.1 `dc8eac7`

`feat(ip_exec): add single-axis action skeleton`

本提交完成：

- 新增执行器头文件与源文件骨架
- 新增最小单元测试
- 为包增加 actionlib/control_msgs/trajectory_msgs 依赖
- 将执行器源文件接入 `canopen_ros_adapter`

涉及文件：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `test/test_ip_follow_joint_trajectory_executor.cpp`
- `package.xml`
- `CMakeLists.txt`

编译与验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^OperationalCoordinator\\.|^CiA402Protocol\\.|^AxisLogicTest\\.|^RobotHw\\.|^SharedState\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

结果：通过。

### 3.2 `6bd8c8f`

`feat(ip_exec): add actual-state start and update core`

本提交完成：

- executor 内部状态结构 `State`
- `startGoal()`：从当前实际状态启动
- `step()`：单轴 Ruckig 周期更新
- `cancelGoal()` / `hasActiveGoal()`
- 扩展单元测试覆盖：
  - goal 从实际状态起步
  - step 能推进目标
  - cancel 生效

涉及文件：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `test/test_ip_follow_joint_trajectory_executor.cpp`

编译与验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^IpFollowJointTrajectoryExecutor\\." --output-on-failure
ctest -R "^OperationalCoordinator\\.|^CiA402Protocol\\.|^AxisLogicTest\\.|^RobotHw\\.|^SharedState\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

结果：通过。

### 3.3 `dbcab0f`

`feat(ip_exec): integrate single-axis executor into ros node`

本提交完成：

- 给 `CanopenRobotHwRos` 增加最小执行器接口：
  - `SetExternalPositionCommand()`
  - `joint_position()`
  - `joint_velocity()`
  - `joint_effort()`
- 在 executor 中加入主循环 `update()`、终态通知、运行时等待逻辑
- 在 `canopen_hw_ros_node` 中增加：
  - `use_ip_executor`
  - `ip_executor_action_ns`
  - `ip_executor_joint_name`
  - `ip_executor_rate_hz`
  等参数
- 将 executor 接入主循环顺序：
  - `read -> cm.update -> ip_executor.update -> write`

重要说明：

- 当前执行器只是在节点中**可启用**
- 仍然默认关闭

涉及文件：

- `include/canopen_hw/canopen_robot_hw_ros.hpp`
- `src/canopen_robot_hw_ros.cpp`
- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `src/canopen_hw_ros_node.cpp`

编译与验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^IpFollowJointTrajectoryExecutor\\.|^CanopenRobotHwRos\\." --output-on-failure
ctest -R "^OperationalCoordinator\\.|^CiA402Protocol\\.|^AxisLogicTest\\.|^RobotHw\\.|^SharedState\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

结果：通过。

### 3.4 `e033c94`

`feat(ip_exec): add node parameters for single-axis executor`

本提交完成：

- executor 的关键约束参数全部从 ROS 参数读入，不再写死：
  - `ip_executor_max_velocity`
  - `ip_executor_max_acceleration`
  - `ip_executor_max_jerk`
  - `ip_executor_goal_tolerance`
- 当 `ip_executor_joint_name` 不存在于 `joints.yaml` 时，节点启动直接拒绝

涉及文件：

- `src/canopen_hw_ros_node.cpp`

编译与验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^OperationalCoordinator\\.|^CiA402Protocol\\.|^AxisLogicTest\\.|^RobotHw\\.|^SharedState\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

结果：通过。

---

## 4. 当前代码里真正已经存在的能力

### 4.1 可以接收单轴 FollowJointTrajectory goal

当前执行器已经具备 action server 骨架，可接收：

- `joint_names == ['joint_1']`
- 至少 1 个轨迹点

不符合条件的 goal 会被拒绝。

### 4.2 会从实际状态起步

`startGoal()` 已经从当前 `actual position / velocity / acceleration` 初始化 Ruckig 输入，而不是从旧 `desired` 起步。

### 4.3 能做单轴 Ruckig 周期推进

`step()` 已经具备：

- 单轴 Ruckig 更新
- 当前位置推进
- 终点完成判定
- 错误转移

### 4.4 已接到 ROS 主循环

当 `use_ip_executor:=true` 时，执行器会在：

```text
read -> cm.update -> ip_executor.update -> write
```

这个顺序里运行。

这样设计的目的，是让 executor 在 `cm.update()` 之后仍能覆盖最终位置命令，避免旧控制器输出直接落到总线。

### 4.5 默认不启用

当前默认参数是：

- `use_ip_executor = false`

因此，现有系统默认行为没有变化。

---

## 5. 当前还没做完的部分

### 5.1 Action 行为还不够完整

虽然 action server 已经存在，但仍有缺口：

- feedback 未按周期发布；
- result/error code 还比较粗；
- preempt/cancel 的现场行为还未按实际控制循环完整验证；
- 还没有与 MoveIt 做现场联调闭环。

### 5.2 还没有单轴现场抓包闭环

本轮验证都是：

- 编译级；
- 单元测试级；
- 现有关键回归级。

还没有完成这一步：

- 在现场启用 `use_ip_executor:=true`
- 通过 `follow_joint_trajectory` action 下发单轴轨迹
- 抓包确认 `0x305` 连续变化、且起点来自当前实际位置

### 5.3 还没有 6 轴扩展

当前实现是严格单轴版本：

- 默认 joint 名 `joint_1`
- 只接受单关节 goal
- 只做单轴 Ruckig

### 5.4 还没有替换现有 JTC

当前 executor 是“可选后端”，不是默认后端。

这意味着：

- 现有 `arm_position_controller` 仍保留；
- 还没有做 launch 层切换、controller 配置清理和现场迁移说明。

---

## 6. 代码改动清单

当前 executor 实现涉及的核心文件如下：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `test/test_ip_follow_joint_trajectory_executor.cpp`
- `include/canopen_hw/canopen_robot_hw_ros.hpp`
- `src/canopen_robot_hw_ros.cpp`
- `src/canopen_hw_ros_node.cpp`
- `CMakeLists.txt`
- `package.xml`

这些改动全部已经进入本地提交历史。

---

## 7. 已验证结果

本轮反复执行的验证命令：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^IpFollowJointTrajectoryExecutor\\.|^CanopenRobotHwRos\\." --output-on-failure
ctest -R "^OperationalCoordinator\\.|^CiA402Protocol\\.|^AxisLogicTest\\.|^RobotHw\\.|^SharedState\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

最终结果：

- `IpFollowJointTrajectoryExecutor.*` 通过
- `CanopenRobotHwRos.*` 通过
- 关键回归集 `65/65` 通过

---

## 8. 当前风险判断

### 8.1 风险已控制

- executor 默认关闭，不会影响当前现网默认路径；
- 关键回归集未退化；
- Ruckig 已成功接入构建链。

### 8.2 主要剩余风险

- action 语义尚未做现场闭环验证；
- 真正启用 executor 后，MoveIt / action 客户端的时序行为仍需抓包确认；
- 单轴版本虽然已可运行，但距离“替代现有 JTC”还差现场验收。

---

## 9. 下一步建议

下一阶段建议只做一件事：

- 让 executor 真正跑通单轴现场闭环

具体包括：

1. 完整 action feedback/result；
2. 单轴现场启用参数与 launch 说明；
3. action 下发轨迹并抓包验证；
4. 关闭单轴 JTC 后端，避免语义冲突。

完成这一步后，再决定是否扩到 6 轴。

---

## 10. 结论

本轮工作不是“只写了设计文档”，而是已经把单轴 IP executor 的关键骨架和执行核心落进代码，并且接进了 ROS 节点主循环。

但也不能把当前状态说成“已经可直接替换现网 JTC”。

准确的状态表述应当是：

- **单轴 executor 的基础框架、执行核心和节点接入已完成**
- **默认关闭，关键回归全绿**
- **下一步进入单轴现场动作闭环与 action 语义完善阶段**
