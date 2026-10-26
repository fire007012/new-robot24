# 设计方案：MoveIt 到 IP 模式的轨迹执行器（单轴先行）

日期：2026-03-25  
范围：`Eyou_Canopen_Master`

---

## 1. 背景

当前系统已经确认以下事实：

- `IP(mode=7)` 通道有效，位置目标优先经 `0x60C1:01` 下发，并运行时镜像到 `0x607A`。
- `JointTrajectoryController` 可接收 `FollowJointTrajectory` goal，但其执行语义与当前系统需求不完全一致。
- 在现场约束下，总线带宽只有 `1 Mbps`，整机控制周期上限约 `250 Hz`，后续还要控制 `6` 轴。

本系统真正需要的上层执行语义是：

- 每次新 goal 到来时，从**当前实际位置**重新起步；
- 对 `resume` / `recover` / 新使能会话敏感，必须与现有安全门控兼容；
- 以总线可承受的固定频率连续输出位置点；
- 将“细颗粒连续性”更多交给驱动器的 IP 插补器，而不是 ROS 侧高频采样器。

---

## 2. 问题定义

### 2.1 当前 JTC 直驱 IP 的问题

`JointTrajectoryController` 的目标是“轨迹执行”，不是“IP 点流执行”。

在当前场景下，直接用 JTC 驱动 IP 会暴露三个问题：

1. 新 goal 的桥接语义更偏“从当前执行轨迹继续”，而不是每次都从 `actual` 重新起步。
2. ROS 侧采样出的 `desired` 不一定等于驱动器当前最适合消费的安全点。
3. 在 `1 Mbps + 250 Hz + 后续 6 轴` 的预算下，不宜继续让 ROS 侧承担高频细颗粒轨迹采样职责。

### 2.2 目标执行器要解决的问题

新的执行器必须保证：

- MoveIt 继续使用标准 `FollowJointTrajectory` 接口；
- 新 goal 从 `actual position / actual velocity` 起步；
- 输出频率可控（建议 `100~125 Hz` 起步）；
- 兼容现有 `command_sync_sequence / epoch / guard / health gate` 逻辑；
- 在 `IP(mode=7)` 下持续输出平滑点流，而不是一次性终点。

---

## 3. 设计目标

### 3.1 必须实现

- 单轴 `joint_1` 的 `FollowJointTrajectory` action 执行；
- 轨迹生成使用 Ruckig；
- 起点使用当前实际状态；
- 新 goal 到来时支持 preempt；
- cancel 时停在当前位置；
- 与现有 `CanopenRobotHwRos` 集成，不破坏当前 CANopen 主链。

### 3.2 暂不实现

- 完整替代 `JointTrajectoryController` 的所有行为；
- 通用 ros_control controller plugin；
- 6 轴版本；
- 完整容差矩阵和复杂路径误差处理；
- 高级 jerk 策略调优。

---

## 4. 总体架构

### 4.1 数据流

```text
MoveIt / Action Client
        |
        v
FollowJointTrajectory Action Server
  (IpFollowJointTrajectoryExecutor)
        |
        v
Ruckig 单轴轨迹生成
        |
        v
CanopenRobotHwRos::SetExternalPositionCommand()
        |
        v
CanopenRobotHw::WriteToSharedState()
        |
        v
SharedState.commands
        |
        v
AxisDriver / AxisLogic
        |
        +--> 0x60C1:01 (IP 主通道)
        +--> 0x607A    (legacy 镜像通道)
```

### 4.2 集成位置

执行器直接放在 `Eyou_Canopen_Master` 包内：

- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- `test/test_ip_follow_joint_trajectory_executor.cpp`

第一阶段不做单独 package，也不做 ros_control plugin。

原因：

- 强依赖 `OperationalCoordinator`、`CanopenRobotHwRos`、`command_sync_sequence`、IP 位置双通道策略；
- 先在当前包内验证行为，再决定是否抽象为独立模块。

---

## 5. 核心执行语义

### 5.1 goal 接收

执行器接收 `control_msgs/FollowJointTrajectoryAction`。

单轴 MVP 只接受：

- `joint_names == ['joint_1']`
- 至少 1 个 waypoint

其余输入直接 reject。

### 5.2 轨迹起点

新 goal 到来时，Ruckig 当前状态取自：

- `current_position = hw current actual position`
- `current_velocity = hw current actual velocity`

而不是：

- 旧 `desired`
- 旧 `goal` 的终点
- controller 内部缓存

### 5.3 采样频率

第一阶段建议固定：

- `executor_rate_hz = 100` 或 `125`

不追求 `250 Hz` 满频运行。

原因：

- `1 Mbps` 总线预算有限；
- 后续扩到 `6` 轴时仍要留出心跳、反馈 PDO、SYNC 和控制字空间；
- 让驱动器 IP 插补器承担更细粒度连续性。

### 5.4 输出策略

每次 `update()`：

1. 读取当前 `actual position / actual velocity`
2. 用 Ruckig 计算当前周期输出点
3. 将输出点写入 `CanopenRobotHwRos`
4. 现有 `write()` 路径继续负责：
   - 位置对齐门控
   - `valid/epoch` 写回
   - IP 主通道与 `0x607A` 镜像

### 5.5 与现有安全门控的关系

执行器不绕开当前安全逻辑。

它必须服从：

- `recover -> enable -> resume` 流程
- `command_sync_sequence`
- `epoch/guard`
- `health gate`

换句话说，执行器只负责“生成目标点”，不负责绕开 Coordinator / RobotHWRos 的安全行为。

---

## 6. 状态机设计

单轴执行器内部建议状态：

- `Idle`
- `WaitingArm`
- `Executing`
- `Holding`
- `Faulted`

含义：

- `Idle`：无活动 goal
- `WaitingArm`：收到 goal，但底层尚未通过接管门控
- `Executing`：Ruckig 正在生成并输出点流
- `Holding`：cancel / abort / finish 后保持当前位置
- `Faulted`：底层故障，当前 goal 失败

转换规则：

- 新 goal：`Idle/Holding -> WaitingArm`
- 接管条件满足：`WaitingArm -> Executing`
- goal 完成：`Executing -> Holding`
- cancel：`WaitingArm/Executing -> Holding`
- fault：任意执行态 -> `Faulted`

---

## 7. Ruckig 参数

单轴 MVP 使用以下参数：

- `max_velocity`
- `max_acceleration`
- `max_jerk`

建议来源：

- 先放到 `joints.yaml`
- 后续扩六轴时自然升级为每轴一组参数

第一阶段允许简化：

- 未提供目标速度时，默认终点速度为 `0`
- 未提供目标加速度时，默认终点加速度为 `0`

---

## 8. 与 MoveIt 的关系

MoveIt 不需要知道底层换了执行器。

MoveIt 只要求：

- 一个标准 `FollowJointTrajectory` action server

因此新执行器对外仍暴露：

- `/arm_position_controller/follow_joint_trajectory`

这样上层规划接口保持不变，变化只发生在 action server 背后的执行器实现。

---

## 9. 为什么不直接继续用 JTC

这份设计不是否定 `JointTrajectoryController`，而是基于当前系统约束选择更合适的分工。

`JointTrajectoryController` 更适合：

- 标准 ros_control 轨迹执行
- CSP / effort / velocity 这类典型执行语义
- MoveIt 直接驱动标准机器人控制器

当前 IP 场景更适合：

- MoveIt 继续发标准 action
- 中间执行器按总线预算生成低频点流
- 驱动器按 IP 语义完成底层插补

---

## 10. 单轴到六轴的扩展

如果单轴阶段就按向量化结构设计，扩六轴的增量成本不会线性乘 6。

单轴先行时就应预留：

- `joint_names -> axis index` 映射
- 向量化状态量
- 每轴限制参数
- 同步采样时间基

预估：

- 单轴 MVP 完成后，扩到 6 轴再增加约 `30%~60%` 工作量。

---

## 11. 验收标准

单轴版本验收通过需满足：

1. `resume` 后不会在固定 guard 帧数结束时抽向旧 setpoint。
2. 新 goal 到来时，从 `actual` 而不是旧 `desired` 起步。
3. `0x305` 和镜像 `0x607A` 连续变化，不再只响应前两次命令。
4. cancel / preempt 不会导致抽动或 fault。
5. `FollowJointTrajectory` action 有完整 feedback / result。

---

## 12. 结论

对当前系统来说，最合理的路线不是继续修补 `JointTrajectoryController` 直驱 IP，而是：

- 保持 MoveIt 接口不变；
- 在 `Eyou_Canopen_Master` 内实现一个单轴 `FollowJointTrajectory` 执行器；
- 用 Ruckig 从实际状态生成低频点流；
- 将细颗粒连续性更多交给驱动器 IP 插补器。

这条路线既保留了 MoveIt 接入能力，也更符合当前总线带宽和安全门控约束。
