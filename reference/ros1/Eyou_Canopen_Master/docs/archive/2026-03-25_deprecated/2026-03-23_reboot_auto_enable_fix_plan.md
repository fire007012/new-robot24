# 修复计划：电机复电后自动使能（Reboot Auto-Enable）

日期：2026-03-23  
状态：评审通过，待实施

## 1. 背景与根因

现场现象：
- 主站已在运行（`Active`）时，某轴掉线后复电，节点重新 boot 完成即自动进入使能链路。

当前根因（代码级）：
- `CiA402StateMachine` 中 `enable_requested_` 默认值为 `true`，且运行期轴对象不重建时该标志会跨 boot 保留。  
- 状态机在 `SwitchOnDisabled/ReadyToSwitchOn/SwitchedOn` 分支会直接推进 `Shutdown -> EnableOperation`。  
- 全局故障闩锁主要影响 `OperationEnabled` 下的目标放行，不足以阻断前置使能推进链路。

## 2. 修复目标

1. 运行中掉线复电后，轴默认停在未使能路径，不允许自动重使能。  
2. 首次初始化仍可按生命周期显式使能（`InitMotors` 后）。  
3. 故障恢复流程收敛为“`Recover` 清故障 + `Resume` 使能”，避免隐式重使能。  
4. 保持现有 `halt/resume` 语义与 epoch-ready 命令门控不回退。

## 3. 非目标（本轮不做）

1. 不改动 PDO 映射验证策略。  
2. 不引入新的 ROS service（如单独 `enable` service）。  
3. 不调整控制器侧指令协议（`AxisCommand.valid/arm_epoch` 保持不变）。

## 4. Commit 级路线图

### C01: `test: reproduce reboot auto-enable and explicit arm semantics`

目标：
- 先建立可回归的失败用例，覆盖“掉线后未 resume 不应自动使能”。

改动文件（计划）：
- `test/test_state_machine.cpp`
- `test/test_axis_logic.cpp`
- `test/test_manual_control.cpp`（必要时）

关键点：
- 增加状态机用例：`enable_requested=false` 时，不推进到 `EnableOperation`。  
- 增加轴逻辑用例：`ProcessHeartbeat(true)` 后默认去使能，不因后续反馈自动恢复使能。  
- 增加流程用例：`Recover` 后需 `Resume` 才进入稳定运行态（若当前测试基座可表达）。

代码量估算：
- 约 `+120 ~ +180` 行（测试代码）

---

### C02: `feat(cia402): default disable + pre-enable gate by fault latch`

目标：
- 从状态机层封堵“前置链路自动推进”。

改动文件（计划）：
- `include/canopen_hw/cia402_state_machine.hpp`
- `src/cia402_state_machine.cpp`

关键点：
- 将 `enable_requested_` 默认值由 `true` 改为 `false`。  
- 引入统一门控 `enable_chain_allowed`（建议语义：`enable_requested_ && !global_fault_ && !forced_halt_by_fault_`）。  
- 在 `SwitchOnDisabled/ReadyToSwitchOn/SwitchedOn` 分支使用该门控，未允许时保持非使能控制字路径。  
- 保持 `OperationEnabled` 内现有安全锁定与 epoch-ready 门控逻辑。

代码量估算：
- 约 `+40 ~ +70` 行，`-10 ~ -25` 行

---

### C03: `feat(lifecycle): explicit arming in InitMotors and Resume path`

目标：
- 将“使能”变为生命周期显式动作，而非构造默认副作用。

改动文件（计划）：
- `src/lifecycle_manager.cpp`
- `src/canopen_master.cpp`
- `include/canopen_hw/canopen_master.hpp`

关键点：
- 在 `InitMotors()` 成功后，对全轴显式 `RequestEnable`（可复用/抽象 `EnableAll()`）。  
- 保留 `ResumeAll()` 中的显式 `RequestEnable`。  
- 保证 `StopCommunication()/EmergencyStop()` 仍是强制去使能优先。

代码量估算：
- 约 `+35 ~ +60` 行，`-5 ~ -15` 行

---

### C04: `fix(axis): de-arm on heartbeat lost to prevent reboot auto-enable`

目标：
- 在掉线事件发生时立即撤销该轴使能请求，避免复电后沿用旧请求。

改动文件（计划）：
- `src/axis_logic.cpp`
- `include/canopen_hw/axis_logic.hpp`（若需暴露额外状态）

关键点：
- `ProcessHeartbeat(true)` 时除置 fault 标志外，显式 `request_disable()`。  
- 心跳恢复（`ProcessHeartbeat(false)`）不自动 re-arm。  
- 与 `global_fault/all_axes_halted_by_fault` 闩锁逻辑保持一致。

代码量估算：
- 约 `+20 ~ +40` 行，`-0 ~ -10` 行

---

### C05: `fix(recover): remove implicit re-enable from fault recover`

目标：
- 恢复流程只做 fault clear，不偷偷恢复使能。

改动文件（计划）：
- `src/canopen_master.cpp`
- `src/lifecycle_manager.cpp`（必要时仅调整文案）
- `docs/api_reference.md`（语义更新）

关键点：
- `RecoverFaultedAxes()` 移除 `axis->RequestEnable()`。  
- 维持 `Recover` 成功后由 `Resume` 才真正恢复使能。  
- 更新失败提示文案，避免运维误判。

代码量估算：
- 约 `+15 ~ +35` 行，`-10 ~ -25` 行

---

### C06: `docs+tests: update usage/fault-injection and finalize regression cases`

目标：
- 完成文档与回归基线，确保现场可按统一步骤验证。

改动文件（计划）：
- `docs/usage.md`
- `docs/fault_injection_checklist.md`
- `docs/api_reference.md`
- （按需）补充/修正测试文件

关键点：
- 明确“复电后不自动使能，需 `recover + resume`”。  
- 更新 F07/F10 注入预期，给出可观测日志关键字。  
- 给出最小复现与验收命令序列。

代码量估算：
- 文档 `+80 ~ +140` 行  
- 测试（如补充）`+30 ~ +80` 行

## 5. 总代码量评估

仅代码（不含文档）：
- 预计 `+230 ~ +385` 行，`-25 ~ -75` 行  
- 净增约 `+180 ~ +330` 行

含文档：
- 总净增约 `+260 ~ +470` 行

## 6. 风险与对策

1. 风险：首次上电后未显式 re-arm 导致“看起来不动”。  
对策：`InitMotors()` 内显式全轴 `RequestEnable`，并在日志打印 `arming axes`。

2. 风险：状态机门控改动引入旧用例回归。  
对策：C01 先补测试，再在 C02/C04/C05 分步落地，逐 commit 跑单测。

3. 风险：`Recover` 语义变化影响现网脚本。  
对策：在 `usage.md` 和 service 返回文案明确“recover 不等于 resume”。

## 7. 验收标准

1. 掉线复电场景：复电后不出现自动 `EnableOperation` 连续推进。  
2. `recover` 成功后，未 `resume` 前轴保持未使能/冻结态。  
3. `resume` 后才恢复运行，`arm_epoch` 与 `cmd_ready` 门控行为正常。  
4. 现有 `halt/resume`、`shutdown/init`、fault reset 回归测试通过。

## 8. 回滚策略

若 C02-C05 任一 commit 触发现场阻塞：
1. 优先回滚单个 commit（保持测试与文档提交不回滚）。  
2. 在保留 C01 回归测试的前提下最小化修正，再重提 PR。  
3. 禁止整分支回滚，避免丢失复现与防回归资产。

