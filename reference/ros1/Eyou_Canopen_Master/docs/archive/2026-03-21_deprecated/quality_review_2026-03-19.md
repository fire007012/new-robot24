# canopen_hw 质量审查报告

- 日期：2026-03-19
- 评分：7/10
- 状态：原型阶段，架构良好，存在影响运行正确性的 bug 与异常处理缺口

## 概况

| 指标 | 数值 |
|------|------|
| 核心源文件 | 8 .cpp + 10 .hpp |
| 测试用例 | 30/30 通过 |
| 编译警告 | 0（-Wall -Wextra -Werror） |
| CI | Debug/Release 双矩阵 + clang-tidy |

---

## P0 — 必须修复

### BUG-1: main.cpp 换算参数丢失（运行时错误）

- 文件：`src/main.cpp:77-106`
- 现象：`LoadJointsYaml` 将 `AxisConversion`（counts_per_rev、rated_torque_nm 等）写入了临时对象 `temp_hw`，随后用新的 `shared_state` 和 `robot_hw` 替换。正式的 `robot_hw` 未重新加载换算参数。
- 影响：运行时所有轴使用默认 counts_per_rev=5308416，而非 YAML 配置值。对于 joint_4/5/6（实际 counts_per_rev=2621440），位置换算误差约 2 倍。
- 修复方案：构造正式 `robot_hw` 后再调用一次 `LoadJointsYaml`，或将换算参数从 config 结构体传递。

### BUG-3: joints.yaml 加载失败后仍继续启动（错误配置上线风险）

- 文件：`src/main.cpp:86-89`
- 现象：`LoadJointsYaml` 返回 false 时仅记录日志，不中止启动流程。
- 影响：系统可能在配置不完整/错误时以默认参数继续运行，导致设备行为与期望配置不一致。
- 修复方案：将 YAML 加载失败视为启动失败，直接 `return 1`；如需降级模式，必须显式开关并输出强告警。

### BUG-2: OnRpdoWrite 每周期全量 Snapshot（实时路径性能问题）

- 文件：`src/axis_driver.cpp:126`
- 现象：每个 RPDO 回调调用 `shared_state_->Snapshot()`，锁内拷贝 3 个 vector（feedback + commands + safe_commands），仅为读取单轴的 `target_position`。
- 影响：6 轴 1kHz SYNC = 每秒 6000 次全量拷贝 + 锁竞争。在实时控制路径上不可接受。
- 修复方案：给 `SharedState` 增加 `GetCommand(axis_index)` 单轴读取接口。

---

## P1 — 应该修复

### ISSUE-3: LoadJointsYaml 接口设计导致鸡生蛋问题

- 文件：`src/joints_config.cpp`、`src/main.cpp:77-95`
- 现象：`LoadJointsYaml` 需要 `CanopenRobotHw*` 来写入换算参数，但构造 `CanopenRobotHw` 需要知道轴数，而轴数要从 YAML 中读取。导致 main.cpp 中出现 temp_state/temp_hw 的笨拙模式，也是 BUG-1 的根因。
- 建议：将换算参数存入 `CanopenMasterConfig`，与 `JointConfig` 合并，构造 `CanopenRobotHw` 后从 config 中一次性应用。

### ISSUE-4: 缺少集成测试覆盖启动流程

- 现象：30 个测试全部是单元测试，没有一个测试覆盖 main.cpp 的启动序列（加载 YAML → 构造 SharedState → 构造 RobotHw → 验证换算参数）。BUG-1 因此漏过。
- 建议：增加一个集成测试，模拟完整启动流程并验证换算参数正确传递。

### ISSUE-5: CanopenMasterConfig 数据冗余

- 文件：`include/canopen_hw/canopen_master.hpp:44-70`
- 现象：`joints` vector 和 `node_ids`/`verify_pdo_mapping` 等扁平 vector 是同一份数据的两种表示，`SyncFromJoints()` 手动同步。
- 风险：新增字段时忘记同步。建议只保留 `joints`，通过访问器按需提取。

### ISSUE-11: YAML 字段类型错误会抛异常，未被捕获

- 文件：`src/joints_config.cpp:77-100`
- 现象：多处 `as<int>()/as<double>()` 转换可能抛 `YAML::Exception`，当前仅在 `YAML::LoadFile` 处有异常处理。
- 影响：当 YAML 字段类型不合法时进程可能异常退出，而不是返回可诊断错误信息。
- 修复方案：在解析阶段统一捕获 `YAML::Exception`，并通过 `error` 输出字段路径与原因后返回 false。

---

## P2 — 建议改进

### ISSUE-6: 测试文件重复

- `test/test_shared_state.cpp` 和 `test/test_shared_state_gtest.cpp` 存在重复用例（`BasicUpdateAndSnapshot` vs `UpdateAndSnapshot`，`OutOfRangeIgnored` 重复）。建议合并为一个文件。

### ISSUE-7: WaitForStateChange 虚假唤醒语义

- 文件：`src/shared_state.cpp:73-77`
- 现象：`wait_until` 返回 `no_timeout` 不代表状态真的变了。当前调用方 `WaitForAllState` 的循环结构能容忍，但接口语义容易误用。

### ISSUE-8: CanopenRobotHw null shared_state 回退值不合理

- 文件：`src/canopen_robot_hw.cpp:17`
- 现象：`shared_state` 为 null 时回退到 6 轴。null 下 Read/Write 都是空操作，分配 6 轴内存无意义。应回退到 0 或直接 assert。

### ISSUE-9: 主循环 sleep_for 无实时保证

- 文件：`src/main.cpp:119-124`
- 现象：100Hz 循环用 `sleep_for(10ms)`，实际周期受调度抖动影响。CSP 模式下后续需切换到 `clock_nanosleep` + `SCHED_FIFO`。

### ISSUE-10: PdoMapping 硬编码 4 通道

- 文件：`include/canopen_hw/pdo_mapping.hpp:28-31`
- 现象：`std::array<PdoChannelMapping, 4>` 硬编码。CiA 301 允许更多 PDO 通道，当前够用但扩展性受限。

---

## 修复优先级建议

| 顺序 | 编号 | 工作量 | 说明 |
|------|------|--------|------|
| 1 | BUG-1 | 小 | 运行时错误，必须立即修复 |
| 2 | BUG-3 | 小 | 配置加载失败后继续运行，必须立即修复 |
| 3 | BUG-2 | 小 | 实时路径性能，增加单轴读取接口 |
| 4 | ISSUE-3 | 中 | 重构 LoadJointsYaml 接口，根治 BUG-1 的设计根因 |
| 5 | ISSUE-4 | 小 | 补充集成测试 |
| 6 | ISSUE-11 | 小 | 补齐 YAML 解析异常处理，避免崩溃 |
| 7 | ISSUE-5~10 | 各小 | 按需处理 |

---

## 复核记录（2026-03-19）

- 代码复核确认：`BUG-1`、`BUG-2`、`BUG-3` 结论成立。
- 测试复核：在 `build/` 目录执行 `ctest --output-on-failure`，结果 `30/30` 通过（总耗时约 0.48s）。

---

## 修复路径（以 commit 为单位）

依赖关系：

```
C01 (BUG-1 止血) ──→ C04 (根治，替代 C01 临时方案)
C02 (BUG-3)       ──→ 独立
C03 (BUG-2)       ──→ 独立
C04 (ISSUE-3)     ──→ C06 (集成测试验证 C04，同时覆盖 C02 负路径)
C04               ──→ C07 (在 C04 基础上继续清理)
C05 (ISSUE-11)    ──→ 独立，但建议在 C04 之后做（接口可能变动）
C08~C12           ──→ 互相独立
```

### C01 — `fix: reload axis conversion on final robot_hw` (BUG-1)

- 优先级：P0 | 工作量：小
- 止血修复。在 `src/main.cpp:106` 构造正式 `robot_hw` 后，再调一次 `LoadJointsYaml` 将换算参数写入正式对象，并检查返回值，失败时 `return 1` 中止启动。
- 注意：此为临时热修，C04 落地后该调用会被 `robot_hw.ApplyConfig(master_cfg)` 替代。
- 改动文件：`src/main.cpp`（+3~4 行）

### C02 — `fix: abort startup on joints.yaml load failure` (BUG-3)

- 优先级：P0 | 工作量：小
- `src/main.cpp:86-89`：`LoadJointsYaml` 返回 false 时 `return 1` 中止启动，而非仅打日志继续运行。
- 改动文件：`src/main.cpp`（改 1 个分支）

### C03 — `perf: add GetCommand single-axis read to SharedState` (BUG-2)

- 优先级：P0 | 工作量：小
- `include/canopen_hw/shared_state.hpp`：声明 `bool GetCommand(size_t axis_index, AxisCommand* out) const`，越界返回 false，避免默认值与合法零命令混淆
- `src/shared_state.cpp`：实现，越界返回 false，合法时锁内拷贝单个 `AxisCommand` 到 `*out` 并返回 true
- `src/axis_driver.cpp:125-128`：`Snapshot()` 替换为 `GetCommand(axis_index_)`

### C04 — `refactor: move AxisConversion into CanopenMasterConfig` (ISSUE-3)

- 优先级：P1 | 工作量：中
- 根治 BUG-1 的设计根因，消除 temp_state/temp_hw 模式。
- `include/canopen_hw/canopen_master.hpp`：`JointConfig` 加 `counts_per_rev`、`rated_torque_nm`、`velocity_scale`、`torque_scale`
- `src/joints_config.cpp`：换算参数写入 `config->joints[i]` 而非 `robot_hw->ConfigureAxisConversion()`
- `include/canopen_hw/canopen_robot_hw.hpp` / `src/canopen_robot_hw.cpp`：加 `ApplyConfig(const CanopenMasterConfig&)` 批量应用换算参数
- `src/main.cpp`：删除 `temp_state`/`temp_hw`，构造后调 `robot_hw.ApplyConfig(master_cfg)`

### C05 — `fix: catch YAML field type exceptions in LoadJointsYaml` (ISSUE-11)

- 优先级：P1 | 工作量：小
- `src/joints_config.cpp:68-147`：将 joints 循环体内的 `as<>()` 调用包裹在 `try/catch(YAML::Exception)` 中，通过 `error` 输出字段路径与原因后返回 false。

### C06 — `test: add startup integration test` (ISSUE-4)

- 优先级：P1 | 工作量：小
- 新增 `test/test_startup_integration.cpp`，覆盖正路径和负路径：
  - 正路径：加载 YAML → 构造 SharedState → 构造 RobotHw → 验证换算参数正确传递（验证 C04）
  - 负路径：YAML 加载失败时启动中止、返回错误信息（验证 C02 / BUG-3）
- `CMakeLists.txt` 注册新测试。

### C07 — `refactor: remove flat vectors from CanopenMasterConfig` (ISSUE-5)

- 优先级：P1 | 工作量：小
- `include/canopen_hw/canopen_master.hpp`：删除 `node_ids`/`verify_pdo_mapping` 等扁平 vector 和 `SyncFromJoints()`
- `src/canopen_master.cpp`：`CreateAxisDrivers` 改用 `config_.joints[i].xxx`
- `src/joints_config.cpp`：删 `SyncFromJoints()` 调用，改为手动设 `axis_count`

### C08 — `test: merge duplicate shared_state test files` (ISSUE-6)

- 优先级：P2 | 工作量：小
- 合并 `test/test_shared_state.cpp` 和 `test/test_shared_state_gtest.cpp`，去重用例。

### C09 — `fix: use predicate overload in WaitForStateChange` (ISSUE-7)

- 优先级：P2 | 工作量：小
- `src/shared_state.cpp:73-77`：`wait_until` 改为带谓词重载，消除虚假唤醒语义问题。

### C10 — `fix: fallback to 0 axes when shared_state is null` (ISSUE-8)

- 优先级：P2 | 工作量：小
- `src/canopen_robot_hw.cpp:17`：`? 6` 改为 `? 0`。

### C11 — `chore: document realtime loop TODO` (ISSUE-9)

- 优先级：P2 | 工作量：小
- `src/main.cpp:119`：标注 TODO，CSP 模式切换前暂不改动 `sleep_for`。

### C12 — `refactor: make PdoMapping channel count configurable` (ISSUE-10)

- 优先级：P2 | 工作量：中（波及 PdoMappingReader、DiffPdoMapping、LoadExpectedPdoMappingFromDcf 及相关测试）
- `include/canopen_hw/pdo_mapping.hpp:28-31`：`std::array<..., 4>` 改为 `std::vector` 或模板参数。
- 需同步修改所有读取/比对/构造 `PdoMapping` 的调用点和测试用例。
