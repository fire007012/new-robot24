# 计划书：IP FollowJointTrajectory 执行器（单轴先行）

日期：2026-03-25  
范围：`Eyou_Canopen_Master`

---

## 1. 目标

实现一个单轴、面向 `IP(mode=7)` 的 `FollowJointTrajectory` 执行器：

- 先替代当前 `JointTrajectoryController` 在 IP 场景下的执行职责；
- 保留 MoveIt 所需的标准 action 接口；
- 以当前实际状态为起点；
- 用 Ruckig 生成适合总线预算的点流；
- 后续平滑扩展到 6 轴。

---

## 2. 范围控制

### 2.1 本轮实现

- 单轴 `joint_1`
- `FollowJointTrajectory` action server
- Ruckig 在线轨迹生成
- preempt / cancel / basic feedback
- 与 `CanopenRobotHwRos` 集成
- 单元测试与最小集成测试

### 2.2 本轮不实现

- 完整 ros_control plugin
- 6 轴版本
- 全量 MoveIt tolerance 兼容
- 复杂轨迹误差恢复策略
- 通用化抽象包拆分

---

## 3. 实施步骤

| 阶段 | 说明 | 产出 |
|---|---|---|
| P01 | 引入 Ruckig 依赖与最小包装 | 新增 executor 头/源文件骨架 |
| P02 | 实现单轴 action server | 能接收并验证 `FollowJointTrajectoryGoal` |
| P03 | 接入当前 `actual position/velocity` 读取 | 新 goal 从实际状态起步 |
| P04 | Ruckig 周期更新与位置命令输出 | 固定频率连续下发点流 |
| P05 | preempt / cancel / hold 逻辑 | goal 切换和取消行为稳定 |
| P06 | 与 `canopen_hw_ros_node` 集成开关 | 可切换 JTC 或新 executor |
| P07 | 增加测试与文档 | 单轴回归与使用说明 |

---

## 4. Commit 计划

建议分成以下提交：

### C01

`feat(ip_exec): add single-axis follow joint trajectory executor skeleton`

内容：

- 新增：
- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp`
- 定义：
  - action server 骨架
  - 单轴状态机骨架
  - 配置结构体

最小验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
```

### C02

`feat(ip_exec): start goals from actual joint state and stream ruckig setpoints`

内容：

- goal 到来时读取当前 `actual position / velocity`
- 接入 Ruckig 单轴更新
- 以固定频率向 `CanopenRobotHwRos` 写位置命令

最小验证：

```bash
ctest -R "^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

### C03

`feat(ip_exec): support preempt cancel and hold behavior`

内容：

- preempt 旧 goal
- cancel 后保持当前位置
- fault / halt / shutdown 时中断当前 goal

最小验证：

```bash
ctest -R "^IpFollowJointTrajectoryExecutor\\.|^OperationalCoordinator\\." --output-on-failure
```

### C04

`feat(ip_exec): integrate executor into canopen_hw_ros_node`

内容：

- 在 `canopen_hw_ros_node` 增加参数开关
- 支持：
  - `use_ip_executor:=true`
  - `executor_rate_hz`
- 启用新执行器时，不再依赖 `arm_position_controller` 驱动 IP 模式

最小验证：

```bash
catkin_make -C /home/dianhua/Robot24_catkin_ws --pkg Eyou_Canopen_Master
ctest -R "^CanopenRobotHwRos\\.|^IpFollowJointTrajectoryExecutor\\." --output-on-failure
```

### C05

`test(ip_exec): cover actual-state restart and repeated goals`

内容：

- 新增回归：
  - goal 从当前实际位置重起步
  - 连续两次 goal 不会只响应前两次
  - cancel/preempt 后不会抽动

最小验证：

```bash
ctest -R "^IpFollowJointTrajectoryExecutor\\.|^CanopenRobotHwRos\\.|^PositionChannelRouting\\." --output-on-failure
```

### C06

`docs: add ip executor usage and moveit integration notes`

内容：

- 更新 `usage.md`
- 更新 `api_reference.md`
- 增加 executor 参数说明和调试方法

最小验证：

```bash
ctest -N
```

---

## 5. 配置建议

第一阶段建议参数：

- `executor_rate_hz: 100` 或 `125`
- `max_velocity`
- `max_acceleration`
- `max_jerk`
- `goal_position_tolerance`

理由：

- 避免在 `1 Mbps` / `250 Hz` 上限下继续把 ROS 采样频率拉满；
- 给反馈 PDO、心跳和控制字留出稳定余量；
- 让驱动器 IP 插补器承担更细粒度连续性。

---

## 6. 测试计划

### 6.1 单元测试

- goal 校验
- 当前位置重起步
- Ruckig 更新
- preempt / cancel
- hold 行为

### 6.2 集成测试

- `resume` 后第一条 goal 不抽动
- 连续两次 goal 均可响应
- `0x305` 连续变化
- `0x607A` 与 `0x305` 同步镜像

### 6.3 现场抓包判据

通过标准：

- 新 goal 到来后，`0x305` 以连续点流形式变化
- `0x205` 中镜像位置目标与 `0x305` 同步
- 不再出现固定 `20` 帧后跳成 `0` 的现象
- 新 goal 到来时，轨迹起点接近当前 `0x185 actual_position`

---

## 7. 风险与应对

### 风险 1

Ruckig 输出频率过高，超过总线预算。

应对：

- 第一版限制在 `100~125 Hz`
- 先验证单轴，再评估六轴总线占用

### 风险 2

执行器绕过现有安全门控。

应对：

- 不直接下发总线对象
- 继续走 `CanopenRobotHwRos -> CanopenRobotHw -> SharedState -> AxisDriver`

### 风险 3

单轴实现写死，后续扩六轴代价升高。

应对：

- 数据结构从一开始按向量化方式组织
- joint name 校验/映射层单独抽出

---

## 8. 扩六轴预估

如果单轴阶段按向量化结构设计，扩到六轴预计新增：

- joint 映射与排序
- 多轴 Ruckig 输入输出
- 每轴限制参数
- 多轴完成判定

增量工作量约为单轴版本的 `30%~60%`。

---

## 9. 结论

推荐路线：

1. 先在 `Eyou_Canopen_Master` 包内做单轴 MVP；
2. 以 action 节点形式验证执行语义；
3. 通过后再扩到六轴；
4. 最后再决定是否收编为 ros_control plugin。

这是当前 MoveIt 接入、总线预算和现场安全性之间成本最低、风险最可控的做法。
