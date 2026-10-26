# canopen_hw 第一阶段 Bug 修复报告

> 日期：2026-03-19
> 范围：C01 ~ C06（全部 P0 + 部分 P1）
> 基线：quality_upgrade_plan.md 中定义的 7.0 分起点

---

## C01 — fix(pdo): remove reader lifetime race between timeout and SDO callbacks

**问题编号**：P0-1
**严重度**：P0（影响安全/正确性）

### 问题描述

`PdoMappingReader::Finish()` 存在两条并发调用路径：

1. **SDO 回调链**（Lely 事件线程）：PDO 映射读取完成或失败时调用
2. **超时线程**：等待超时后调用

原实现用 `atomic<bool> finished_` 做首次进入检测，用 `thread::get_id()` 判断是否从超时线程自身调用并选择 `detach()`。问题在于：

- 超时线程持有 `weak_ptr`，`lock()` 后获得临时 `shared_ptr`，但若 SDO 路径先完成并在 `OnBoot` 回调内调用 `pdo_reader_.reset()`，`weak_ptr::lock()` 可能成功（时序窗口内引用计数尚未归零），超时线程继续访问即将析构的成员——**Use-After-Free**
- `Finish()` 内部 `detach()` 后，`OnBoot` lambda 中 `pdo_reader_.reset()` 立即析构对象，此时超时线程栈帧仍在 `Finish()` 内——**悬空访问**

### 修复方案

| 改动 | 说明 |
|------|------|
| `finished_` 从 `atomic<bool>` 改为 `finish_mtx_` 保护的普通 `bool` | 将 Finish() 整体串行化，而非仅检测首次进入 |
| 超时线程持 `shared_from_this()` 替代 `weak_ptr` | 保证对象在超时线程执行期间不被析构 |
| 析构函数移除 `detach` 分支，改为无条件 `join()` | 超时线程持有 shared_ptr → 析构函数只在线程退出后才会被调用 → join 必定安全 |
| `Finish()` 内拷贝回调和数据到局部变量，锁外调用回调 | 防止回调内 reset() 导致死锁或重入析构 |
| `OnBoot` 回调内移除 `pdo_reader_.reset()` | 回调可能从 reader 自身线程调用，自毁会导致 UAF |

### 涉及文件

- `include/canopen_hw/pdo_mapping.hpp`
- `src/pdo_mapping.cpp`
- `src/axis_driver.cpp`
- `include/canopen_hw/axis_driver.hpp`

---

## C02 — refactor(pdo): document callback-thread contract

**问题编号**：P0-1 补充
**严重度**：文档性

### 说明

已合并到 C01 中一并完成。在 `PdoMappingReader` 类声明上方添加了完整的线程模型 block comment，在 `axis_driver.hpp` 的 `pdo_reader_` 成员旁添加了生命周期约束注释。

---

## C03 — fix(shared_state): separate ros_command and safe_command to eliminate write race

**问题编号**：P0-2
**严重度**：P0（影响正确性）

### 问题��述

`SharedState::commands_[]` 被两个线程写入同一字段：

- **ROS 线程**通过 `CanopenRobotHw::WriteToSharedState()` → `UpdateCommand()` 写入用户期望目标位置
- **Lely 线程**通过 `AxisDriver::PublishSnapshot()` → `UpdateCommand()` 写入状态机过滤后的 `safe_target`

虽然 `mutex` 防止了数据撕裂，但语义上 ROS 线程每个周期写入的 target 都会在下一个 RPDO 周期被 Lely 线程覆盖——**ROS 侧写入实际无效**。

### 修复方案

引入独立的 `AxisSafeCommand` 结构体和 `safe_commands_[]` 数组，将两路写入彻底分离：

| 写入方 | 写入目标 | 方法 |
|--------|----------|------|
| ROS 线程 | `commands_[]`（用户期望位置） | `UpdateCommand()` — 不变 |
| Lely 线程 | `safe_commands_[]`（状态机安全目标） | `UpdateSafeCommand()` — 新增 |

`PublishSnapshot()` 改为调用 `UpdateSafeCommand()`，不再触碰 `commands_[]`。`Snapshot()` 同时拷贝两组数据。

### 涉及文件

- `include/canopen_hw/shared_state.hpp`
- `src/shared_state.cpp`
- `src/axis_driver.cpp`

---

## C04 — test: migrate test_state_machine and test_unit_conversion from assert to gtest

**问题编号**：P0-3（第一批）
**严重度**：P0（测试在 Release 下失效）

### 问题描述

`test_state_machine.cpp` 和 `test_unit_conversion.cpp` 使用 `assert()` 做断言。在 Release 构建（`-DNDEBUG`）下，`assert()` 被预处理器移除，测试变为空操作——**永远"通过"但实际什么都没验证**。

### 修复方案

- 删除 `#include <cassert>` 和 `int main()`
- 添加 `#include <gtest/gtest.h>`
- 将每个逻辑段落拆为独立 `TEST()` 用例
- `assert(x == y)` → `EXPECT_EQ(x, y)`，`assert(cond)` → `EXPECT_TRUE(cond)`
- CMakeLists: 链接 `GTest::gtest_main`，改用 `gtest_discover_tests()`

### 迁移后用例

| 文件 | 用例 |
|------|------|
| `test_state_machine.cpp` | `CiA402SM.SwitchOnDisabledSendsShutdown`, `ReadyToSwitchOnJumpEnable`, `FirstOperationEnabledLocksPosition`, `RosTargetCloseUnlocks`, `FaultResetThreePhaseFlow` |
| `test_unit_conversion.cpp` | `UnitConversion.TicksToRadDefaultAxis`, `CustomAxisConversion`, `NotOperationalSkipsWrite` |

---

## C05 — test: migrate remaining 4 test files from assert to gtest

**问题编号**：P0-3（第二批）
**严重度**：P0

### 问题描述

同 C04，剩余 4 个测试文件 (`test_shared_state`, `test_canopen_robot_hw`, `test_canopen_master`, `test_joints_config`) 同样使用 `assert()`。

### 修复方案

与 C04 相同的迁移策略。完成后 `test/` 目录下不再有 `#include <cassert>`。

### 迁移后用例

| 文件 | 用例 |
|------|------|
| `test_shared_state.cpp` | `SharedState.BasicUpdateAndSnapshot`, `OutOfRangeIgnored` |
| `test_canopen_robot_hw.cpp` | `RobotHw.ReadTicksToRad`, `WriteRadToTicks` |
| `test_canopen_master.cpp` | `MasterConfig.ZeroAxisNormalized`, `NotRunningAfterConstruction` |
| `test_joints_config.cpp` | `JointsConfig.LoadValidYaml`, `InvalidNodeIdRejected` |

---

## C06 — test: add boundary and edge-case tests

**问题编号**：P1-3
**严重度**：P1

### 问题描述

原有测试仅覆盖正常路径，缺少以下边界/极值场景的验证：
- `axis_count` 为 0 或超过上限时的归一化行为
- `node_id` 非法值（0、128、300）的拒绝逻辑
- `int32_t` 极值位置下 `AbsDiff` 的截断行为
- 故障复位次数耗尽后的 `PermanentFault` 行为

### 修复方案

新增 `test/test_boundary_cases.cpp`，包含 7 个用例：

| 用例 | 验证内容 | 结果 |
|------|---------|------|
| `AxisCountZeroNormalizesToOne` | `axis_count=0` 被归一化为 1 | ✅ 通过 |
| `AxisCountExceedsMaxClamped` | `SetActiveAxisCount(100)` 后不越界 | ✅ 通过 |
| `NodeIdZeroDefaultFill` | `node_ids` 为空时自动填充 1..N | ✅ 通过 |
| `NodeIdOutOfRange` | `node_id=0` 和 `node_id=128` 被拒绝 | ✅ 通过 |
| `Int32ExtremePositionSameValue` | `INT32_MAX` 同值时正常解锁 | ✅ 通过 |
| `Int32ExtremeAbsDiffTruncation` | `AbsDiff(INT32_MAX, INT32_MIN)` 的 int32 截断 | ✅ 记录了 P2-5 已知缺陷 |
| `FaultResetMaxAttemptsReached` | 超过 max 后进入 PermanentFault | ✅ 通过 |

### 发现的已知缺陷

测试过程中确认了 P2-5 记录的 `AbsDiff` 截断问题：当 `actual_position=INT32_MIN`、`ros_target=INT32_MAX` 时，`int64` 差值为 `4294967295`，截断回 `int32_t` 变为 `-1`，导致误判为"接近"而解锁。该问题在工程实际中概率极低（需位置跨越整个 int32 范围），已在测试中标注，待后续修复。

---

## 总结

| Commit | 问题 | 级别 | 状态 |
|--------|------|:----:|:----:|
| C01 | PDO Reader 生命周期竞态 (UAF/悬空访问) | P0 | ✅ 已修复 |
| C02 | 线程契约文档缺失 | P0 补充 | ✅ 已合并到 C01 |
| C03 | SharedState 命令覆盖竞态 | P0 | ✅ 已修复 |
| C04 | assert 测试 Release 下失效 (第一批) | P0 | ✅ 已修复 |
| C05 | assert 测试 Release 下失效 (第二批) | P0 | ✅ 已修复 |
| C06 | 缺少边界/极值测试 | P1 | ✅ 已补充 |

### 测试统计变化

| 指标 | 修复前 | 修复后 |
|------|:------:|:------:|
| 测试用例总数 | 12 | 29 |
| GTest 用例 | 4 | 29 |
| assert 型用例 | 8 | 0 |
| Release 下有效 | 4 / 12 | 29 / 29 |

所有 3 个 P0 问题已修复，P1-3 边界测试已补充。剩余 C07 ~ C13 待后续阶段执行。
