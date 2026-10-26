# Bug 报告与修复计划：使能保护被提前骗开导致零位抽动

**项目：** Eyou_Canopen_Master  
**日期：** 2026-03-22  
**严重级别：** High（存在突发运动风险）  
**状态：** 已完成根因确认，待按分阶段计划实施

---

## 1. 问题摘要

在 `/init`、`/recover`、`/resume` 后进入 `OperationEnabled` 的过渡阶段，当前实现可能在 1~2 个周期内把错误目标（典型为 0）透传到驱动器，造成电机瞬时抽动或向零位运动。

该问题不是单点 bug，而是由以下组合触发：

1. 上层命令缓冲初始值可能为 0；
2. 保护阶段会把目标写成 `actual_position`；
3. 状态机当前解锁条件只看 `|ros_target - actual| <= threshold`；
4. `read -> cm.update -> write` 顺序下，上升沿同步写回会被控制器下一步覆写。

---

## 2. 影响范围

1. 冷启动后首轮使能。
2. `/recover` 成功后重新使能。
3. `/halt -> /resume` 后重新放行控制。
4. 多轴系统中，单轴故障后其余轴若未统一冻结，存在联动风险。

---

## 3. 现象与复现条件

典型现象：

1. 电机短促抽动后进入故障；
2. 电机直接向绝对编码器零位运动；
3. 部分场景中仅在首轮恢复后出现，后续偶现。

复现条件（高概率）：

1. CSP 模式；
2. 控制器内部 setpoint 尚未与当前位置重同步；
3. 轴反馈刚上升为 operational 的 1~2 帧内。

---

## 4. 根因分析（已确认）

### 4.1 关键时序

1. 未就绪时 `WriteToSharedState()` 用 `joint_pos_` 写入目标位置（保护行为）。
2. Lely 线程 `OnRpdoWrite()` 先读取 `AxisCommand` 并调用 `SetRosTarget()`，再推进状态机。
3. 状态机 `StepOperationEnabled()` 在同一帧内可能执行：
   - 大偏差重基准 `ros_target_ = actual_position`；
   - 紧接着满足阈值条件并解锁。
4. 下一 ROS 循环中 `cm.update()` 覆写命令缓存（可能为 0），`write()` 透传后触发抽动。

### 4.2 现有设计缺口

现有解锁仅依赖“目标接近实际”，无法区分：

1. 上层真实就绪的合法命令；
2. 保护写回/默认值导致的“伪合法命令”。

因此会出现“被保护行为骗开锁”的结构性漏洞。

---

## 5. 修复目标与非目标

### 5.1 修复目标

1. 解锁必须满足“上层已就绪 + 会话匹配 + 偏差合理”三条件。
2. 任一轴 fault 时全轴快速冻结，且统一恢复。
3. 保持分层：状态机纯逻辑，不直接写总线，不依赖 ROS。
4. 修复后可兼容非 ROS 上层（Qt/Python/裸循环）。

### 5.2 非目标

1. 不引入 ROS 特有语义到核心层。
2. 不在本阶段重构 Lely 回调框架。
3. 不一次性引入复杂协调器大改，先最小可用闭环。

---

## 6. 设计方案（定案版）

### 6.1 命令协议增强：`valid + arm_epoch`

在 `AxisCommand` 中新增：

1. `bool valid`：上层命令源是否完成重同步并声明可用；
2. `uint32_t arm_epoch`：使能会话号。

语义：

1. `epoch=0` 永远无效；
2. 状态机每次进入“重新使能窗口”时递增内部 `arm_epoch_`；
3. 仅当 `valid=true` 且 `cmd_epoch == arm_epoch_` 时，才允许进入解锁判定。

### 6.2 状态机门控顺序（优先级固定）

每周期按以下顺序判定：

1. `global_fault` 或 `forced_halt_by_fault`：冻结输出；
2. `user_halt`：冻结输出；
3. `!cmd_valid` 或 `epoch mismatch`：冻结并上锁；
4. `position_locked` 下偏差检查（通过才解锁）；
5. 解锁后执行每周期限幅（`max_delta_per_cycle`）。

### 6.3 全局故障闩锁

1. 任一轴检测 fault/heartbeat-lost -> 置 `global_fault=true`；
2. 所有轴在下一周期进入冻结路径；
3. `global_fault` 的清除仅允许在统一恢复判定成功后执行；
4. 清除闩锁后不自动恢复用户主动 halt 状态。

### 6.4 分层边界

1. 状态机仅输出 `safe_target/safe_velocity/safe_torque`。
2. `AxisLogic/AxisDriver` 负责把安全输出写入总线对象字典。
3. ROS 仅作为上层适配器，实现 `cmd_ready -> valid` 映射。

---

## 7. 分阶段修复计划（Commit 级）

### C01
`protocol: add AxisCommand.valid/arm_epoch and fault latch fields in SharedState`

产出：

1. 命令结构扩展；
2. 快照带出 `global_fault/all_axes_halted_by_fault`；
3. SharedState 测试更新。

验收：

1. `SharedState*` 相关测试通过；
2. 并发测试无退化。

### C02
`config: add max_velocity_for_clamp parsing and validation`

产出：

1. 轴配置新增 `max_velocity_for_clamp`；
2. YAML 解析与校验；
3. 样例配置更新。

验收：

1. `JointsConfig` 测试通过；
2. 配置错误路径报错清晰。

### C03
`cia402: implement epoch-ready gate and clamp with nonzero epoch policy`

产出：

1. 状态机新增 `cmd_valid/cmd_epoch/arm_epoch`；
2. `epoch=0` 无效策略；
3. 解锁逻辑改为三条件；
4. 每周期限幅。

验收：

1. 旧“自动重基准即解锁”基线移除；
2. 新增 epoch 失配不解锁测试通过。

### C04
`axis: wire command envelope and publish arm_epoch feedback`

产出：

1. `OnRpdoWrite` 读取完整命令包；
2. `ProcessRpdo` 发布 `feedback.arm_epoch`；
3. fault 置位全局闩锁。

验收：

1. `AxisLogic` 测试通过；
2. 上层可从快照读到最新 epoch。

### C05
`fault-latch: enforce unified recover and forbid auto-resume`

产出：

1. `LifecycleManager/CanopenMaster` 统一故障清除条件；
2. 清闩锁后不自动 `ResumeAll`；
3. 区分用户 halt 与故障连带 halt。

验收：

1. `Recover` 成功才清闩锁；
2. 单轴 resume 在闩锁期间被拒绝。

### C06
`ros-adapter: implement cmd_ready+epoch handshake`

产出：

1. ROS 适配器按事件重置 `cmd_ready`；
2. 写命令时填 `valid+epoch`；
3. 全局故障期间强制 `valid=false`。

验收：

1. `/init`、`/recover`、`/resume` 后无 0 指令透传；
2. 控制器恢复后可正常接管。

### C07
`tests: rewrite unlock baseline and add regression matrix`

产出：

1. 重写依赖“自动解锁”的旧用例；
2. 新增 11 个关键场景回归矩阵。

验收：

1. 关键场景全绿；
2. 失败时可定位到门控层级。

### C08
`docs: update protocol, fault-latch contract and tuning guide`

产出：

1. 协议文档（valid/epoch）；
2. 多轴故障与恢复契约；
3. 参数调优指南。

验收：

1. 文档与实现一致；
2. 运维可直接按文档操作。

---

## 8. 测试与验收矩阵（摘要）

1. 冷启动 + 控制器初始 0：不解锁，不抽动。
2. `/recover`：需重新 valid+epoch，同步后才放行。
3. `/halt -> /resume`：恢复后需重新 arming。
4. valid 从 true 变 false 再变 true：必须重新过锁。
5. epoch 不匹配：始终锁定。
6. 跳变指令：被限幅平滑。
7. 多轴任一 fault：全轴冻结。
8. 全局故障未清除：拒绝单轴恢复。
9. 故障恢复失败：闩锁保持。
10. 清闩锁后：不覆盖用户主动 halt。
11. 非 ROS 上层：仅按协议即可安全运行。

---

## 9. 风险与缓解

1. **风险：** 旧测试基线与新语义冲突。  
   **缓解：** C03/C07 同步改基线，先改测试再放开行为。

2. **风险：** epoch 链路遗漏导致永不解锁。  
   **缓解：** C04 强制在 `ProcessRpdo` 回写 `feedback.arm_epoch` 并加单测。

3. **风险：** 全局故障清除条件过宽。  
   **缓解：** 清除路径仅限统一恢复成功，且保留失败详情日志。

---

## 10. 实施结论

本计划通过 `valid+epoch` 将“命令就绪”从隐式假设变为显式协议，通过 `global_fault` 闩锁将多轴安全策略从“建议”变为“强制”。在不破坏当前分层（状态机纯逻辑、总线写在 AxisLogic/Driver）的前提下，可系统性消除“被保护行为骗开锁”的根因漏洞。
