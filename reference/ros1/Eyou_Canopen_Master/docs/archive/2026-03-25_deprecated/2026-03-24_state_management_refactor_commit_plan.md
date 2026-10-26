# 计划图：状态管理架构重构（基于 0324import，精确到 commit）

日期：2026-03-24  
状态：已执行；仅剩 C17（可选性能项）未落地

## 0. 执行状态（截至 2026-03-24）

| 计划ID | 状态 | 实际提交 |
|---|---|---|
| C01 | 已完成 | `e45df1f` |
| C02 | 已完成 | `9162698` |
| C03 | 已完成 | `2ad4dc0` |
| C04 | 已完成 | `840c08e` |
| C05 | 已完成 | `51b40e8` |
| C06 | 已完成 | `6ad9a3e` |
| C07 | 已完成 | `dbc50fd` |
| C08 | 已完成 | `865efcc` |
| C09（可选） | 未执行 | — |
| C10 | 已完成 | `b686a27` |
| C11 | 已完成 | `ddad23b` |
| C12 | 已完成 | `b9a93f9` |
| C13 | 已完成 | `9d55ada` |
| C14 | 已完成 | `0cc27f7` |
| C15 | 已完成 | `d9e2d63` |
| C16 | 已完成 | `当前提交` |
| C17（可选） | 未执行 | — |

### 0.1 0324import 一致性补丁（2026-03-24）

| 目标 | 状态 | 提交 |
|---|---|---|
| intent 序列失活回落到 Disable | 已完成 | `659b9d7` |
| 严格 `Configured->Standby->Armed->Running` 矩阵 | 已完成 | `8525a4b` |
| Gateway 增加 `~/enable` 且 auto 启动链路严格化 | 已完成 | `43f5564` |
| standalone 走 Coordinator 策略路径 | 已完成 | `d42270e` |

## 1. 调研结论（实际情况）

### 1.1 基线与工作区

- 子仓库基线：`4b68d1938bc6fff4dd08983c12a681d4f316e5a8`（`main`）。
- 当前工作区非干净（需避免混入本计划提交）：
  - 已修改：`config/joints.yaml`
  - 未跟踪：`docs/0324import.md`、`docs/2026-07-14_recover_bypass_resume_bug_report.md`

### 1.2 已落地能力（与 0324import 相关）

| commit | 已实现内容 | 对应文件（示例） |
|---|---|---|
| `175498d` | 使能链路需要显式 enable，且受 fault-latch 门控 | `include/src/cia402_state_machine.*` |
| `132c34b` | `InitMotors()` 后显式 arm 轴 | `src/lifecycle_manager.cpp` |
| `2b2c4ea` | 心跳丢失时去使能，防止复电自动使能 | `src/axis_logic.cpp` |
| `503de6d` | Recover 移除隐式重新使能 | `src/canopen_master.cpp` |
| `4b68d19` | Recover 后保持需显式 `resume` | `src/lifecycle_manager.cpp` |
| `95f762c` | ROS 适配层基于 `arm_epoch` 变化沿做重同步 | `src/canopen_robot_hw_ros.cpp` |

### 1.3 仍未落地（0324import 的核心差距）

1. 仍是 `LifecycleManager + CiA402StateMachine` 架构，尚无 `OperationalCoordinator / SystemOpMode / AxisIntent / ServiceGateway / CiA402Protocol`。  
2. `CiA402StateMachine` 仍混合“402 协议翻译 + 策略决策 + fault reset 流程”。  
3. `LifecycleManager` 仍持有影子状态：`halted_ / require_init_ / state_`。  
4. `SharedState` 仍为全局大锁（`std::mutex`），未拆分 per-axis 并发模型。  

### 1.4 现状测试基线

执行命令：

```bash
ctest -R "CiA402SM|LifecycleManager|AxisLogicTest|StateMachineMultiMode|RobotHwMultiMode" --output-on-failure
```

结果：`60/60` 通过（0 失败）。

## 2. 执行约束

1. 每个提交只做一个可回归增量，提交信息必须与下表完全一致。  
2. 每个提交结束必须跑该提交列出的最小验证集。  
3. 本计划默认从 `4b68d19` 开始线性执行；`C09` 为可选性能分支。  
4. 不改动 `config/joints.yaml` 与未跟踪文档，避免与业务配置混提。  

## 3. Commit 计划图

```mermaid
flowchart TD
    B0[BASE\n4b68d193] --> C01[C01\ntest(coord): add transition-matrix regression baseline]
    C01 --> C02[C02\nfeat(coord): add SystemOpMode and OperationalCoordinator skeleton]
    C02 --> C03[C03\nrefactor(node): introduce ServiceGateway and route lifecycle services]
    C03 --> C04[C04\nfeat(shared_state): add AxisIntent and intent sequence channel]
    C04 --> C05[C05\nfeat(axis): consume AxisIntent in shadow mode with parity checks]
    C05 --> C06[C06\nrefactor(cia402): extract CiA402Protocol adapter without behavior change]
    C06 --> C07[C07\nrefactor(coord): move recover/fault policy to coordinator and master]
    C07 --> C08[C08\nrefactor(lifecycle): reduce LifecycleManager to resource owner]
    C08 --> C10[C10\ndocs+tests: finalize migration matrix and update API/usage]
    C08 --> C09[C09 optional\nperf(shared_state): switch mutex to per-axis seqlock]
    C09 --> C10
```

## 4. 精确到 Commit 的执行表

| ID | 父提交 | 目标 commit message（必须一致） | 主要改动文件 | 交付目标 | 最小验证命令 |
|---|---|---|---|---|---|
| C01 | `4b68d193` | `test(coord): add transition-matrix regression baseline` | `test/test_lifecycle_manager.cpp`, `test/test_axis_logic.cpp`, `test/test_state_machine.cpp` | 固化现有 `init/halt/resume/recover/shutdown` 与 fault-latch 行为，建立迁移前红线 | `ctest -R "LifecycleManager|AxisLogicTest|CiA402SM" --output-on-failure` |
| C02 | C01 | `feat(coord): add SystemOpMode and OperationalCoordinator skeleton` | `include/canopen_hw/operational_coordinator.hpp`, `src/operational_coordinator.cpp`, `CMakeLists.txt` | 新增 Coordinator 和 `SystemOpMode`，仅旁路接入，不改现有行为 | `ctest -R "LifecycleManager|CanopenMaster" --output-on-failure` |
| C03 | C02 | `refactor(node): introduce ServiceGateway and route lifecycle services` | `include/canopen_hw/service_gateway.hpp`, `src/service_gateway.cpp`, `src/canopen_hw_ros_node.cpp` | service lambda 收敛到无状态网关，统一转发 Coordinator | `ctest -R "LifecycleManager|StartupIntegration" --output-on-failure` |
| C04 | C03 | `feat(shared_state): add AxisIntent and intent sequence channel` | `include/src/shared_state.*`, `include/canopen_hw/operational_coordinator.hpp`, `src/operational_coordinator.cpp` | SharedState 新增每轴 `AxisIntent` + `intent_sequence`，Coordinator 周期写入 | `ctest -R "SharedState|LifecycleManager" --output-on-failure` |
| C05 | C04 | `feat(axis): consume AxisIntent in shadow mode with parity checks` | `include/src/axis_logic.*`, `src/axis_driver.cpp`, `test/test_axis_logic.cpp` | 轴侧读取 intent，与旧闩锁决策做一致性校验（先 shadow，不切主路径） | `ctest -R "AxisLogicTest|CiA402SM" --output-on-failure` |
| C06 | C05 | `refactor(cia402): extract CiA402Protocol adapter without behavior change` | `include/canopen_hw/cia402_protocol.hpp`, `src/cia402_protocol.cpp`, `include/src/cia402_state_machine.*`, `test/test_state_machine.cpp` | 抽离纯协议层实现并通过适配器保持行为一致 | `ctest -R "CiA402SM|StateMachineMultiMode|Boundary" --output-on-failure` |
| C07 | C06 | `refactor(coord): move recover/fault policy to coordinator and master` | `src/operational_coordinator.cpp`, `include/src/canopen_master.*`, `src/lifecycle_manager.cpp`, `test/test_lifecycle_manager.cpp` | 故障恢复策略迁移到 Coordinator；Protocol 不再承担 recover 策略 | `ctest -R "LifecycleManager|AxisLogicTest|CanopenMaster" --output-on-failure` |
| C08 | C07 | `refactor(lifecycle): reduce LifecycleManager to resource owner` | `include/src/lifecycle_manager.*`, `src/canopen_hw_ros_node.cpp`, `test/test_lifecycle_manager.cpp` | 去除 `halted_/require_init_` 影子状态；Lifecycle 退化为资源所有权管理 | `ctest -R "LifecycleManager|StartupIntegration|RobotHw" --output-on-failure` |
| C09 (可选) | C08 | `perf(shared_state): switch mutex to per-axis seqlock` | `include/canopen_hw/seqlock.hpp`, `include/src/shared_state.*`, `test/test_shared_state_concurrent.cpp` | SharedState 从大锁切到 per-axis seqlock（仅性能优化） | `ctest -R "SharedState|SharedStateConcurrent" --output-on-failure` |
| C10 | C08 或 C09 | `docs+tests: finalize migration matrix and update API/usage` | `docs/api_reference.md`, `docs/usage.md`, `docs/command_cheatsheet.md`, `docs/README.md`, `test/*(按需)` | 输出迁移完成矩阵、服务语义、回滚指引与最终回归脚本 | `ctest -N && ctest -R "CiA402SM|LifecycleManager|AxisLogicTest" --output-on-failure` |

## 5. 每个里程碑的退出条件

### M1（C01-C03 完成）

- Service 入口已统一到 Gateway/Coordinator。  
- `recover` 不自动进入运行态，且原有 service 语义保持可用。  

### M2（C04-C06 完成）

- `AxisIntent` 通道可稳定运行，shadow 一致性检查无告警。  
- 协议层代码已从策略层分离，但行为回归不变。  

### M3（C07-C08 完成）

- 运行策略仅由 Coordinator 决策。  
- Lifecycle 不再承担业务状态机角色。  

### M4（C09 可选 + C10）

- 文档和测试矩阵与新架构一致，具备可运维回退说明。  

## 6. 回滚策略（按 commit 粒度）

1. 若 C04-C08 任一提交出现现场不稳定，按单提交回滚，不回滚 C01 测试基线。  
2. C09 为独立可选提交，可随时放弃，不影响功能闭环。  
3. 回滚后必须保留 `C01` 新增回归用例，并补一个修复提交重新进入主线。

## 7. 第二阶段（终态）计划表

> 目标：从当前过渡版（`b686a27`）迁移到 `0324import` 定义的最终版。  
> 范围：必做 6 个 commit（C11-C16）+ 可选 1 个性能 commit（C17）。

| ID | 目标 commit message | 主要内容 | 关键文件 | 提交前验证 |
|---|---|---|---|---|
| C11 | `refactor(protocol): implement pure CiA402Protocol and remove strategy state` | 将 `CiA402Protocol` 从适配器实现为纯协议翻译，策略字段不再驻留协议层 | `include/src/cia402_protocol.*`, `include/src/cia402_state_machine.*`, `test/test_state_machine.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "CiA402SM|CiA402Protocol|StateMachineMultiMode|Boundary" --output-on-failure` |
| C12 | `refactor(axis): drive control path by AxisIntent as primary source` | `AxisLogic/AxisDriver` 切到 `AxisIntent` 主驱动；旧 latch 路径降级为兼容或移除 | `include/src/axis_logic.*`, `src/axis_driver.cpp`, `test/test_axis_logic.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "AxisLogicTest|CiA402Protocol" --output-on-failure` |
| C13 | `refactor(master): keep CanopenMaster as bus lifecycle and fault-reset executor` | `CanopenMaster` 收口为通信管理和故障复位执行层，去除策略编排职责 | `include/src/canopen_master.*`, `src/canopen_master.cpp`, `test/test_canopen_master.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "MasterConfig|CanopenMaster|StartupIntegration" --output-on-failure` |
| C14 | `refactor(coord): finalize SystemOpMode transition matrix and atomic recover flow` | 完成 Coordinator 终态：唯一策略写入者、Recovering 原子流程、自动故障降级闭环 | `include/src/operational_coordinator.*`, `src/service_gateway.cpp`, `test/test_lifecycle_manager.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "LifecycleManager|AxisLogicTest|MasterConfig" --output-on-failure` |
| C15 | `refactor(lifecycle+node): reduce LifecycleManager to owner and remove shadow semantics` | `LifecycleManager` 彻底退化为 owner，`node` 完全由 `ServiceGateway+Coordinator` 驱动 | `include/src/lifecycle_manager.*`, `src/lifecycle_manager.cpp`, `src/canopen_hw_ros_node.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "LifecycleManager|StartupIntegration|RobotHw" --output-on-failure` |
| C16 | `test+docs: finalize final-state migration matrix and operational contract` | 终态测试矩阵与运维契约收口，更新 API/usage/计划状态 | `docs/*.md`, `test/*(按需)` | `cmake --build build -j$(nproc)` + `ctest -N` + `ctest -R "CiA402SM|CiA402Protocol|LifecycleManager|AxisLogicTest" --output-on-failure` |
| C17（可选） | `perf(shared_state): switch global mutex to per-axis seqlock` | `SharedState` 大锁替换为 per-axis seqlock（性能优化） | `include/src/shared_state.*`, `include/canopen_hw/seqlock.hpp`, `test/test_shared_state_concurrent.cpp` | `cmake --build build -j$(nproc)` + `ctest -R "SharedState|SharedStateConcurrent" --output-on-failure` |
