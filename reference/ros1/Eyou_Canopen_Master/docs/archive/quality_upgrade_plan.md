# canopen_hw 质量提升总计划：7 分 → 9 分

> 合并自 `quality_review_report.md` + `quality_9_score_plan.md`
> 日期：2026-03-19

---

## 一、现状总结

### 1.1 项目概览

| 项目 | 详情 |
|------|------|
| **用途** | 基于 Lely CANopen 的 6 轴工业机器人关节驱动层，CiA 402 状态机 + ROS 硬件接口 |
| **语言/标准** | C++17 |
| **构建系统** | CMake 3.10+ |
| **依赖** | Lely CANopen, yaml-cpp, Google Test, pthreads |
| **代码规模** | ~2620 行 (源码 1586 + 头文件 591 + 测试 443) |
| **测试** | 12 用例全部通过 (8 assert + 4 GTest，含并发压力) |
| **编译** | 零错误；开启严格警告后有 3 个 `-Wshadow`（Lely 基类遮蔽） |

### 1.2 现有优点

- 架构模块化清晰：状态机 / 驱动 / 主站 / 硬件层 / 共享状态职责分离
- 线程安全基础扎实：mutex + 快照模式 + atomic 防重入
- 安全防护完善：三阶段故障复位、无扰切换、心跳检测、有序关机
- 代码风格统一，关键决策处有中文注释

### 1.3 当前评分

| 维度 | 分数 | 9 分差距 |
|------|:----:|----------|
| 架构设计 | 8 | 数据流职责有交叉 |
| 代码质量 | 7.5 | 存在逻辑竞态 |
| 线程安全 | 7 | PDO Reader 生命周期隐患 |
| 测试覆盖 | 6.5 | assert 型、缺边界用例 |
| 可维护性 | 7 | 无日志、配置冗余 |
| 构建/工程 | 7.5 | 无 CI、无警告标志 |
| 生产就绪度 | 5.5 | 骨架阶段、配置不完整 |
| **总体** | **7.0** | — |

### 1.4 9 分定义

同时满足以下四条：

1. **安全性**：无已知高风险并发/生命周期缺陷（UAF、data race）
2. **可验证性**：关键逻辑有稳定自动化回归，Debug/Release 均有效
3. **可运维性**：关键故障可观测、可定位、可复现
4. **工程闭环**：CI 质量门阻断回归，变更有明确验收标准

---

## 二、全量问题清单（按严重度排序）

### P0 — 必须修复（影响安全/正确性）

| # | 问题 | 来源 | 位置 |
|---|------|------|------|
| P0-1 | `PdoMappingReader::Finish()` 生命周期竞态：超时线程 detach 后对象可能已析构 | 两份文档共识 | `pdo_mapping.cpp`, `axis_driver.cpp` |
| P0-2 | `PublishSnapshot` 命令覆盖竞态：Lely 线程写 `safe_target` 到 `AxisCommand`，ROS 线程也在写同一字段，逻辑上后者被覆盖 | 审查报告独有 | `axis_driver.cpp:281-292`, `shared_state.hpp` |
| P0-3 | `assert()` 测试在 Release (`-DNDEBUG`) 下被移除，6 个测试可执��文件空转 | 两份文档共识 | `test/test_state_machine.cpp` 等 6 个文件 |

### P1 — 应当修复（影响健壮性/可维护性）

| # | 问题 | 来源 | 位置 |
|---|------|------|------|
| P1-1 | `CanopenMaster` 库内无 `axis_count <= kAxisCount` 约束，复用时可越界 | 9 分计划 | `canopen_master.cpp` 构造函数 |
| P1-2 | `LoadJointsYaml` 中 `node_id -> axis_index` 映射逻辑脆弱，跳号时行为不可预期 | 审查报告独有 | `joints_config.cpp:103-109` |
| P1-3 | 缺少边界测试：`int32` 极值、`axis_count` 上下界、`node_id` 非法值、PDO 超时只触发一次 | 9 分计划 | `test/` 目录 |
| P1-4 | 日志使用裸 `std::cerr/cout`，无级别/时间戳/结构化，故障定位困难 | 两份文档共识 | 全项目多处 |
| P1-5 | 缺少 CI 质量门：无 Debug/Release 双构建、无 clang-tidy、无 `-Werror` | 两份文档共识 | `CMakeLists.txt`, `.github/` |
| P1-6 | `WaitForAllState` busy-wait + `sleep_for(10ms)` 轮询 | 审查报告独有 | `canopen_master.cpp:153-173` |
| P1-7 | `CanopenMasterConfig` 与 `MasterConfig`/`JointCanopenConfig` 配置结构冗余 | 审查报告独��� | `canopen_master.hpp`, `joints_config.hpp`, `main.cpp` |

### P2 — 建议改进（影响可读性/长期维护）

| # | 问题 | 来源 | 位置 |
|---|------|------|------|
| P2-1 | 编译器 `-Wshadow` 警告未消除（Lely 头文件未标 SYSTEM） | 审查报告独有 | `CMakeLists.txt` |
| P2-2 | `kCtrl_DisableOperation` 与 `kCtrl_SwitchOn` 同值 (0x0007) 无说明注释 | 审查报告独有 | `cia402_defs.hpp:31,35` |
| P2-3 | `kAxisCount` 编译期硬编码 6 与运行时 `active_axis_count_` 概念不一致 | 审查报告独有 | `shared_state.hpp:41` |
| P2-4 | `joints.yaml` 只配了 1 轴，TODO 提示 2-6 轴未补齐 | 审查报告独有 | `config/joints.yaml` |
| P2-5 | `AbsDiff` 返回 `int32_t` 存在理论截断（工程概率极低，不作缺陷） | 9 分计划 | `cia402_state_machine.cpp:9-12` |
| P2-6 | 缺少上线前 Soak 测试与故障注入验证 | 9 分计划 | — |

---

## 三、Commit 级修复计划（共 13 个 Commits）

### C01 `fix(pdo): remove reader lifetime race between timeout and SDO callbacks`

**严重度**: P0-1 | **阶段**: A

**问题根因**:
`PdoMappingReader::Finish()` 可从两条路径并发进入：SDO 回调链（Lely 事件线程）和超时线程。当超时线程调用 `Finish()` 时检测到 `thread::id` 相同而 `detach()`，但 `detach()` 返回后 `AxisDriver::OnBoot` 的 lambda 执行 `pdo_reader_.reset()`��此时超时线程可能仍在访问已析构的成员。

**执行方案**:

1. `pdo_mapping.hpp` — 在 `PdoMappingReader` 中新增 `std::mutex finish_mtx_`，将 `finished_` 从 `atomic<bool>` 改为受 `finish_mtx_` 保护的普通 `bool`
2. `pdo_mapping.cpp: Finish()` — 用 `std::lock_guard<std::mutex>` 包裹整个 `Finish` 逻辑体，确保同一时刻只有一个路径执行完成流程
3. `pdo_mapping.cpp: Start()` — 超时线程持有 `shared_from_this()` 的拷贝（而非 `weak_ptr`），确保对象在超时线程执行期间不会被析构；超时线程内调用 `Finish()` 后不再访问任何成员
4. `pdo_mapping.cpp: ~PdoMappingReader()` — 析构函数中将 `timeout_stop_` 置 true 并 `notify_all()`，然后在 `finish_mtx_` 锁外 `join()` 超时线程（此时线程必已退出或即将退出）
5. `axis_driver.cpp: OnBoot()` — `pdo_reader_.reset()` 改为在 lambda 外部、`OnBoot` 末尾执行，避免在回调链内部析构 reader

**涉及文件**:
- `include/canopen_hw/pdo_mapping.hpp`
- `src/pdo_mapping.cpp`
- `src/axis_driver.cpp`

**验收标准**:
- `TSAN` 构建下跑全量测试无 data race 报告
- 手动模拟超时路径（timeout = 1ms）连续运行 1000 次无崩溃

---

### C02 `refactor(pdo): document callback-thread contract and serialize finish path`

**严重度**: P0-1 补充 | **阶段**: A

**问题根因**:
线程模型约定只存在于代码隐含逻辑中，后续维护者容易引入新的并发路径。

**执行方案**:

1. `pdo_mapping.hpp` — 在 `PdoMappingReader` 类声明上方添加 block comment，说明：
   - `Start()` 只能在 Lely 事件线程调用
   - `ScheduleNext()` / SDO 回调在 Lely 事件线程执行
   - 超时线程是唯一的非 Lely 线程入口
   - `Finish()` 可被上述两条路径调用，内部用 `finish_mtx_` 串行化
   - 调用者（`AxisDriver`）不得在回调内部 `reset()` reader
2. `axis_driver.hpp` — 在 `pdo_reader_` 成员旁添加注释：生命周期由 `shared_ptr` + `Finish` 串行化保证，禁止在 `OnBoot` 回调内部同步析构

**涉及文件**:
- `include/canopen_hw/pdo_mapping.hpp`
- `include/canopen_hw/axis_driver.hpp`

**验收标准**:
- 文档注释可通过 code review 确认完整性

---

### C03 `fix(shared_state): separate ros_command and safe_command to eliminate write race`

**严重度**: P0-2 | **阶段**: A

**问题根因**:
`SharedState` 中 `commands_[i]` 被两个线程写入同一字段：
- ROS 线程通过 `CanopenRobotHw::WriteToSharedState()` 写入用户期望位置
- Lely 线程通过 `AxisDriver::PublishSnapshot()` 写入状态机过滤后的 `safe_target`

虽然 mutex 防止了数据撕裂，但 ROS 线程刚写入的 target 在下个 RPDO 周期���会被 Lely 覆盖，语义上 ROS 写入无效。

**执行方案**:

1. `shared_state.hpp` — 新增 `AxisSafeCommand` 结构体（包含 `int32_t safe_target_position`），在 `SharedState` 中增加 `std::array<AxisSafeCommand, kAxisCount> safe_commands_`；`SharedSnapshot` 中相应增加 `safe_commands` 字段
2. `shared_state.hpp` — 新增方法 `UpdateSafeCommand(size_t axis_index, const AxisSafeCommand&)`
3. `shared_state.cpp` — 实现 `UpdateSafeCommand()`，与现有方法共用 `mtx_`；`Snapshot()` 中同步拷贝 `safe_commands_`
4. `axis_driver.cpp: PublishSnapshot()` — 改为调用 `UpdateSafeCommand()` 写入 safe_target，不再写 `UpdateCommand()`
5. `axis_driver.cpp: OnRpdoWrite()` — 从 `snap.commands[axis_index_]` 读取 ROS 侧目标（此值不再被 Lely 覆盖）；下发位置改为从 `snap.safe_commands[axis_index_]` 读取，或直接从本地 `state_machine_.safe_target()` 获取
6. `canopen_robot_hw.cpp: WriteToSharedState()` — 保持原有逻辑，仅写 `commands_[]`（ROS 期望位置）

**涉及文件**:
- `include/canopen_hw/shared_state.hpp`
- `src/shared_state.cpp`
- `src/axis_driver.cpp`
- `src/canopen_robot_hw.cpp`（确认无需改动）

**验收标准**:
- 新增单元测试：ROS 线程写入 command 后，Lely 线程写入 safe_command，再次 Snapshot 确认 command 未被覆盖
- 现有 `test_canopen_robot_hw` 和 `test_shared_state_concurrent` 通过

---

### C04 `test: migrate test_state_machine and test_unit_conversion from assert to gtest`

**严重度**: P0-3 (第一批) | **阶段**: B

**问题根因**:
`assert()` 在 `-DNDEBUG` (Release) 下被预处理器移除，测试变为空操作。

**执行方案**:

1. `test/test_state_machine.cpp`:
   - 删除 `#include <cassert>` 和 `int main()`
   - 添加 `#include <gtest/gtest.h>`
   - 将每个逻辑段落拆为独立 `TEST()` 用例：
     - `TEST(CiA402SM, SwitchOnDisabledSendsShutdown)`
     - `TEST(CiA402SM, ReadyToSwitchOnJumpEnable)`
     - `TEST(CiA402SM, FirstOperationEnabledLocksPosition)`
     - `TEST(CiA402SM, RosTargetCloseUnlocks)`
     - `TEST(CiA402SM, FaultResetThreePhaseFlow)`
   - 所有 `assert(x == y)` 改为 `EXPECT_EQ(x, y)`，`assert(cond)` 改为 `EXPECT_TRUE(cond)`
2. `test/test_unit_conversion.cpp`:
   - 同样迁移为 GTest，按换算方向拆分：`TEST(UnitConversion, TicksToRad)`、`TEST(UnitConversion, RadToTicks)` 等
3. `CMakeLists.txt`:
   - 两个目标改为链接 `GTest::gtest_main`
   - 改用 `gtest_discover_tests()` 替代 `add_test()`

**涉及文件**:
- `test/test_state_machine.cpp`
- `test/test_unit_conversion.cpp`
- `CMakeLists.txt`

**验收标准**:
- `cmake --build . && ctest` 在 Debug 和 Release (`-DNDEBUG`) 下��通过

---

### C05 `test: migrate test_shared_state, test_canopen_robot_hw, test_canopen_master, test_joints_config to gtest`

**严重度**: P0-3 (第二批) | **阶段**: B

**执行方案**:

1. 对以下 4 个文件逐一执行与 C04 相同的迁移：
   - `test/test_shared_state.cpp` → `TEST(SharedState, BasicUpdateAndSnapshot)`, `TEST(SharedState, OutOfRangeIgnored)`
   - `test/test_canopen_robot_hw.cpp` → `TEST(RobotHw, ReadTicksToRad)`, `TEST(RobotHw, WriteRadToTicks)`, `TEST(RobotHw, NotOperationalSkipsWrite)`
   - `test/test_canopen_master.cpp` → `TEST(MasterConfig, ZeroAxisNormalized)`, `TEST(MasterConfig, EmptyNodeIdsAutoFill)`
   - `test/test_joints_config.cpp` → `TEST(JointsConfig, LoadValidYaml)`, `TEST(JointsConfig, InvalidNodeIdRejected)`
2. `CMakeLists.txt` — 4 个目标全部链接 `GTest::gtest_main`，使用 `gtest_discover_tests()`

**涉及文件**:
- `test/test_shared_state.cpp`
- `test/test_canopen_robot_hw.cpp`
- `test/test_canopen_master.cpp`
- `test/test_joints_config.cpp`
- `CMakeLists.txt`

**验收标准**:
- 全量 `ctest` 在 Debug 和 Release 下通过
- 项目中不再有 `#include <cassert>` 的测试文件

---

### C06 `test: add boundary and edge-case tests for axis_count, node_id, pdo timeout`

**严重度**: P1-3 | **阶段**: B

**执行方案**:

1. 新增 `test/test_boundary_cases.cpp`，包含以下用例：
   - `TEST(Boundary, AxisCountZeroNormalizesToOne)` — 构造 `CanopenMaster(axis_count=0)` 验证归一化
   - `TEST(Boundary, AxisCountExceedsMaxClamped)` — `SetActiveAxisCount(100)` 后 Snapshot 验证限制在 6
   - `TEST(Boundary, NodeIdZeroDefaultFill)` — `node_ids` 全空时验证默认 1..N
   - `TEST(Boundary, NodeIdOutOfRange)` — `LoadJointsYaml` 传入 `node_id: 0` 和 `node_id: 128` 验证拒绝
   - `TEST(Boundary, Int32ExtremePosition)` — 状态机 `Update()` 传入 `INT32_MAX` 和 `INT32_MIN`，验证 `AbsDiff` 不溢出、`safe_target` 合理
   - `TEST(Boundary, FaultResetMaxAttemptsReached)` — 连续触发超过 `max_fault_resets` 次，验证进入 `PermanentFault`
2. `CMakeLists.txt` — 新增 `test_boundary_cases` 目标，链接 `canopen_core GTest::gtest_main`

**涉及文件**:
- `test/test_boundary_cases.cpp`（新增）
- `CMakeLists.txt`

**验收标准**:
- 全部新增用例通过
- 行覆盖率提升至 >= 80%

---

### C07 `fix(master): enforce axis_count upper bound and fail-fast config validation`

**严重度**: P1-1 + P1-2 | **阶段**: C

**执行方案**:

1. `canopen_master.cpp` 构造函数 — 在 `axis_count` 归一化逻辑后增加上限裁剪：
   ```cpp
   if (config_.axis_count > SharedState::kAxisCount) {
     std::cerr << "axis_count " << config_.axis_count
               << " exceeds max " << SharedState::kAxisCount
               << ", clamping" << std::endl;
     config_.axis_count = SharedState::kAxisCount;
   }
   ```
2. `canopen_master.cpp` 构造函数 — 遍历 `node_ids`，对每个值校验 `1 <= id <= 127`，非法值打印警告并替换为默认 `i+1`
3. `joints_config.cpp:103-109` — 统一映射规则为"YAML 中第 N 个 joint 映射到 axis_index = N（从 0 开始）"，不再用 `node_id - 1` 做索引；`node_id` 仅用于总线通信，不参与数组索引
4. `joints_config.cpp` — 新增 `counts_per_rev <= 0`、`rated_torque_nm <= 0` 的参数合法性检查，非法时设置 error 并返回 false

**涉及文件**:
- `src/canopen_master.cpp`
- `src/joints_config.cpp`

**验收标准**:
- `axis_count = 100` 构造后不越界
- `node_id = 300` 的 YAML 被拒绝
- `counts_per_rev = -1` 的 YAML 被拒绝
- 现有测试不受影响

---

### C08 `refactor(config): merge redundant config structs and eliminate manual copy in main`

**严重度**: P1-7 | **阶段**: C

**执行方案**:

1. `joints_config.hpp` — 删除 `MasterConfig` 和 `CanopenRuntimeConfig`，将 `JointCanopenConfig` 中的字段整合进 `CanopenMasterConfig`：
   - `CanopenMasterConfig` 新增内嵌类型 `struct JointConfig`（合并原 `JointCanopenConfig` 和 `AxisConversion` 的字段）
   - `CanopenMasterConfig` 新增 `std::vector<JointConfig> joints`
2. `joints_config.hpp` — `LoadJointsYaml` 签名改为：
   ```cpp
   bool LoadJointsYaml(const std::string& path,
                       CanopenMasterConfig* config,
                       CanopenRobotHw* robot_hw,
                       std::string* error);
   ```
3. `joints_config.cpp` — 解析结果直接填入 `CanopenMasterConfig`
4. `main.cpp` — 删除 80-110 行的手动对拷逻辑，改为一次 `LoadJointsYaml` 调用完成所有配置
5. 同步更新所有测试中的调用方式

**涉及文件**:
- `include/canopen_hw/joints_config.hpp`
- `include/canopen_hw/canopen_master.hpp`
- `src/joints_config.cpp`
- `src/main.cpp`
- `test/test_joints_config.cpp`

**验收标准**:
- 项目中不再有 `MasterConfig` 和 `CanopenRuntimeConfig` 类型
- `main.cpp` 配置流程从 ~50 行缩减到 ~15 行
- 全量测试通过

---

### C09 `feat(log): replace iostream with structured logging and add runtime health counters`

**严重度**: P1-4 | **阶段**: D

**执行方案**:

1. `CMakeLists.txt` — 添加 `find_package(spdlog)` 或 header-only 引入（若无包管理，使用 `FetchContent` 拉取 spdlog）
2. 新增 `include/canopen_hw/logging.hpp`：
   - 定义 `canopen_hw::Logger()` 返回全局 spdlog logger
   - 统一 format 规范：`[canopen_hw] [axis={} node={}] message`
3. 全局替换 `std::cerr <<` / `std::cout <<`：
   - `axis_driver.cpp` — EMCY、heartbeat、PDO 验证日志改为 `SPDLOG_WARN` / `SPDLOG_ERROR`
   - `canopen_master.cpp` — 启动/停止/关机日志改为 `SPDLOG_INFO`
   - `pdo_mapping.cpp` — SDO 读取失败改为 `SPDLOG_ERROR`
   - `main.cpp` — 启动信息改为 `SPDLOG_INFO`，错误改为 `SPDLOG_ERROR`
4. 新增 `include/canopen_hw/health_counters.hpp`：
   - 结构体 `HealthCounters`，包含：`pdo_verify_ok`, `pdo_verify_fail`, `pdo_verify_timeout`, `heartbeat_lost`, `heartbeat_recovered`, `fault_reset_attempts`, `emcy_count`
   - 在 `AxisDriver` 中维护实例，通过 `SharedState` 或独立接口暴露给上层
5. `axis_driver.cpp` — 在相应回调中递增计数器

**涉及文件**:
- `CMakeLists.txt`
- `include/canopen_hw/logging.hpp`（新增）
- `include/canopen_hw/health_counters.hpp`（新增）
- `src/axis_driver.cpp`
- `src/canopen_master.cpp`
- `src/pdo_mapping.cpp`
- `src/main.cpp`

**验收标准**:
- 项目中不再有 `std::cerr <<` 或 `std::cout <<`（除测试外）
- 日志输出带时间戳、级别、轴号
- 任一故障可在 5 分钟内定位到轴号与失败阶段

---

### C10 `refactor(master): replace busy-wait in WaitForAllState with condition_variable`

**严重度**: P1-6 | **阶段**: D

**执行方案**:

1. `shared_state.hpp` — 新增 `std::condition_variable state_cv_` 成员
2. `shared_state.cpp: UpdateFeedback()` — 在更新 feedback 后调用 `state_cv_.notify_all()`
3. `canopen_master.hpp` — `WaitForAllState` 签名不变
4. `canopen_master.cpp: WaitForAllState()` — 将 `sleep_for(10ms)` 循环改为：
   ```cpp
   std::unique_lock<std::mutex> lk(shared_state_->mutex());
   while (std::chrono::steady_clock::now() < deadline) {
     if (AllMatchUnlocked(target_state)) return true;
     shared_state_->wait_for(lk, 10ms);
   }
   return false;
   ```
   注意：需要 `SharedState` 暴露 `wait_for()` 方法或公开 cv，在已有 `mtx_` 上等待
5. 备选简化方案：如果暴露 cv 不理想，可在 `SharedState` 新增 `WaitForStateChange(deadline)` 方法，封装 `cv.wait_until()`

**涉及文件**:
- `include/canopen_hw/shared_state.hpp`
- `src/shared_state.cpp`
- `src/canopen_master.cpp`

**验收标准**:
- `GracefulShutdown()` 功能不变
- 无 busy-wait，CPU 使用率在关机期间显著降低

---

### C11 `build: enable -Wall -Wextra -Werror, mark Lely includes as SYSTEM`

**严重度**: P2-1 + P1-5 (部分) | **阶段**: E

**执行方案**:

1. `CMakeLists.txt` — 修改 `target_compile_options`：
   ```cmake
   target_compile_options(canopen_core PRIVATE -Wall -Wextra -Werror ${LELY_CFLAGS_OTHER})
   ```
2. `CMakeLists.txt` — 将 Lely 头文件标记为 SYSTEM 以抑制第三方警告：
   ```cmake
   target_include_directories(canopen_core
     PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
     SYSTEM PUBLIC ${LELY_INCLUDE_DIRS}
   )
   ```
3. `cia402_defs.hpp:31,35` — 在 `kCtrl_SwitchOn` 和 `kCtrl_DisableOperation` 旁添加注释：
   ```cpp
   // CiA 402 协议规定 SwitchOn 与 DisableOperation 共用 0x0007，
   // 两者语义由当前状态机状态决定。
   constexpr uint16_t kCtrl_SwitchOn = 0x0007;
   // ...
   constexpr uint16_t kCtrl_DisableOperation = 0x0007;  // 同 kCtrl_SwitchOn，见 CiA 402 Table 39
   ```
4. 修复编译过程中因 `-Werror` 暴露的所有新警告（如有）

**涉及文件**:
- `CMakeLists.txt`
- `include/canopen_hw/cia402_defs.hpp`
- 可能涉及被 `-Wextra` 暴露的其他源文件

**验收标准**:
- 全项目零警告编译通过（`-Wall -Wextra -Werror`）
- Lely 头文件的第三方警告不再出现

---

### C12 `ci: add GitHub Actions workflow with debug/release build, ctest, and clang-tidy`

**严重度**: P1-5 | **阶段**: E

**执行方案**:

1. 新增 `.github/workflows/ci.yml`：
   ```yaml
   jobs:
     build:
       strategy:
         matrix:
           build_type: [Debug, Release]
       steps:
         - uses: actions/checkout@v4
         - name: Install deps
           run: apt-get install -y liblely-dev libyaml-cpp-dev libgtest-dev clang-tidy
         - name: Configure
           run: cmake -B build -DCMAKE_BUILD_TYPE=${{ matrix.build_type }}
                 -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
         - name: Build
           run: cmake --build build -j$(nproc)
         - name: Test
           run: cd build && ctest --output-on-failure
         - name: Clang-Tidy
           run: clang-tidy src/*.cpp -- -Iinclude $(pkg-config --cflags ...)
   ```
2. 在 ci.yml 中添加覆盖率收集步骤（`gcov` / `lcov`），输出覆盖率报告
3. 配置分支保护规则：PR 未通过 CI 不可合并

**涉及文件**:
- `.github/workflows/ci.yml`（新增）

**验收标准**:
- Push 后自动触发 Debug + Release 双构建 + 测试
- clang-tidy 零错误
- PR 页面显示 CI 状态

---

### C13 `docs: add soak test plan, fault injection checklist, and complete joints.yaml`

**严重度**: P2-4 + P2-6 | **阶段**: F

**执行方案**:

1. `config/joints.yaml` — 补全 6 轴配置：
   ```yaml
   joints:
     - name: joint_1
       canopen: { node_id: 1, verify_pdo_mapping: true }
       counts_per_rev: 5308416
       rated_torque_nm: 1.0
       # ...
     - name: joint_2
       canopen: { node_id: 2, verify_pdo_mapping: true }
       # ... (按实际硬件参数填写)
     # joint_3 ~ joint_6 同上
   ```
2. 新增 `docs/soak_test_plan.md`：
   - 8 小时 Soak 测试流程：启动条件、监控指标、通过标准
   - 24 小时 Soak 测试流程（可选）
3. 新增 `docs/fault_injection_checklist.md`：
   - 故障场景清单：CAN 断线、节点重启、PDO 不匹配、超时、EMCY、心跳丢失
   - 每个场景的预期行为和验收标准
4. 新增 `docs/release_readiness.md`：
   - 发布评审检查项：问题清零表 + 风险接受清单

**涉及文件**:
- `config/joints.yaml`
- `docs/soak_test_plan.md`（新增）
- `docs/fault_injection_checklist.md`（新增）
- `docs/release_readiness.md`（新增）

**验收标准**:
- `joints.yaml` 包含 6 轴完整配置
- 文档可支撑实际 Soak 执行

---

## 四、执行总览与里程碑

```
Week 1 (P0 + P1 核心)
│
├── C01  fix(pdo): reader 生命周期竞态        ██████████  P0
├── C02  refactor(pdo): 线程契约文档           ████        P0
├── C03  fix(shared_state): 命令覆盖竞态       ██████████  P0
├── C04  test: 迁移 GTest (第一批)             ██████      P0
├── C05  test: 迁移 GTest (第二批)             ██████      P0
├── C06  test: 边界测试                        ████        P1
└── C07  fix(master): 上限约束 + 配置校验       ██████      P1

Week 2 (P1 改进 + P2 打磨)
│
├── C08  refactor(config): 合并配置结构         ██████      P1
├── C09  feat(log): 结构化日志 + 健康计数器     ████████    P1
├── C10  refactor(master): cv 替代 busy-wait ████        P1
├── C11  build: -Werror + SYSTEM include     ████        P2
├── C12  ci: GitHub Actions 工作流            ██████      P1
└── C13  docs: Soak 计划 + 配置补全            ████        P2
```

**完成后预期评分**: 9.0 / 10
