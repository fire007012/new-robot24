# Bug 修复报告（已知问题）

日期：2026-03-19  
分支：`main`  
范围：代码审查中确认的 4 个已知问题修复

---

## 1. 修复概览

| 编号 | 问题 | 级别 | Commit |
|---|---|---|---|
| BUG-01 | SDO 读写固定 4 字节，存在截断/语义不一致风险 | 高 | `8235a68` |
| BUG-02 | `fault_reset_attempts` 诊断计数未更新 | 中 | `1b01807` |
| BUG-03 | `AxisLogic::PublishSnapshot()` 读取 `feedback_cache_` 存在并发边界不一致 | 中 | `7576e74` |
| BUG-04 | 构建强依赖在线拉取 spdlog，离线场景易失败 | 低 | `c62ea31` |

---

## 2. 详细修复说明

### BUG-01：SDO 宽度处理修复（`8235a68`）

#### 现象
- `AxisDriver::AsyncSdoRead()` 固定按 `uint32_t` 读取并返回 4 字节。
- `AxisDriver::AsyncSdoWrite()` 将输入打包到 `uint32_t`，`>4` 字节无保护，`<4` 字节语义不明确。
- 与 `SdoAccessor` 的“字节数组接口”不一致，容易在 1/2/3 字节对象读写时出错。

#### 根因
- 驱动层默认了 SDO 都是 32bit，对 CANopen 对象字节宽度缺乏显式约束与参数。

#### 修复内容
- `AxisDriver::AsyncSdoRead()` 新增 `expected_size` 参数（1..4，默认 4）。
- 按 `expected_size` 选择 `SubmitRead<uint8_t/uint16_t/uint32_t>`。
- `AxisDriver::AsyncSdoWrite()` 新增有效载荷校验：
  - 空 payload -> 返回错误。
  - `>4` 字节 -> 返回错误。
  - 1/2/3/4 字节按对应路径编码写入。
- `SdoAccessor` 的 `AsyncRead/Read` 对应增加 `expected_size`（默认保持 4，兼容旧调用）。

#### 影响文件
- `include/canopen_hw/axis_driver.hpp`
- `src/axis_driver.cpp`
- `include/canopen_hw/sdo_accessor.hpp`
- `src/sdo_accessor.cpp`

---

### BUG-02：故障复位计数镜像修复（`1b01807`）

#### 现象
- 诊断里读取 `HealthCounters::fault_reset_attempts`，但运行时未更新，长期为 0。

#### 根因
- `CiA402StateMachine` 内部有 `fault_reset_count_`，未同步到 `HealthCounters`。

#### 修复内容
- 在 `AxisLogic::ProcessRpdo()` 中将状态机的 `fault_reset_count()` 镜像到 `health_.fault_reset_attempts`。
- 在 `AxisLogic::ResetFault()` 中显式清零 `health_.fault_reset_attempts`。
- 新增回归测试：`AxisLogicTest.FaultResetAttemptsMirroredToHealthCounters`。

#### 影响文件
- `src/axis_logic.cpp`
- `test/test_axis_logic.cpp`

---

### BUG-03：快照发布并发边界修复（`7576e74`）

#### 现象
- `PublishSnapshot()` 对 `feedback_cache_` 的读取不是完整锁内快照，线程安全边界不一致。

#### 根因
- `feedback_cache_` 与 `state_machine_` 的读取快照策略不统一。

#### 修复内容
- 在 `PublishSnapshot()` 内，持锁同时复制：
  - `AxisFeedback feedback_snapshot`
  - `AxisSafeCommand safe_cmd`
- 锁外仅执行 `SharedState::UpdateFeedback/UpdateSafeCommand`，避免无锁读取共享缓存。

#### 影响文件
- `src/axis_logic.cpp`

---

### BUG-04：构建离线韧性修复（`c62ea31`）

#### 现象
- 默认只通过 `FetchContent` 拉取 spdlog，网络受限环境配置失败。

#### 根因
- 缺少“优先系统包”的依赖解析路径和显式开关。

#### 修复内容
- 新增 CMake 选项：`CANOPEN_FETCH_SPDLOG`（默认 `ON`）。
- 构建流程改为：
  1. `find_package(spdlog QUIET CONFIG)` 优先系统包。
  2. 未找到且 `CANOPEN_FETCH_SPDLOG=ON` 时再走 `FetchContent`。
  3. 未找到且开关关闭时给出明确 `FATAL_ERROR`。

#### 影响文件
- `CMakeLists.txt`

---

## 3. 验证记录

所有修复均执行构建与测试验证。

### 验证命令
```bash
cmake --build build -j8
ctest --output-on-failure
```

### 结果
- BUG-01 提交后：构建通过，`87/87` 测试通过。
- BUG-02 提交后：构建通过，`88/88` 测试通过（新增 1 个测试）。
- BUG-03 提交后：构建通过，`88/88` 测试通过。
- BUG-04 提交后：构建通过，`88/88` 测试通过。

---

## 4. 提交记录（按修复顺序）

1. `8235a68` `sdo: support 1-4 byte payload sizes safely`  
2. `1b01807` `diag: mirror fault reset attempts into health counters`  
3. `7576e74` `axis_logic: publish feedback snapshots under lock`  
4. `c62ea31` `build: prefer system spdlog and gate online fetch`

---

## 5. 备注

- 本报告仅覆盖上述 4 个已知问题及对应提交。
- 仓库中仍有其他未提交改动（与本次 4 个修复提交独立），需单独评审与处理。
