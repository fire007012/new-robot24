# Bug 修复计划表（精确到 Commit）

日期：2026-03-19  
适用分支：当前 `main` 工作分支  
输入依据：`docs/bug_report.md` + 本轮补充审查项（`axis_count` 越界、`node_id` 回绕）

---

## 0. 执行原则

1. 每个 commit 只解决一类问题，避免混改。  
2. 每个 commit 必须带可执行验证命令。  
3. 先修阻断运行与安全风险，再做清理项。  
4. 任何行为变化必须同步补单测或集成测试。  

---

## 1. Commit 计划总表

| 顺序 | Commit 标题（建议） | 修复项 | 主要改动文件 | 验证命令 | 通过标准 |
|---|---|---|---|---|---|
| C1 | `fix(shared_state): honor configured axis count in all_operational recompute` | B2（固定 6 轴遍历导致系统永远不可运行） | `include/canopen_hw/shared_state.hpp` `src/shared_state.cpp` `src/main.cpp` `test/test_shared_state_gtest.cpp` | `cmake --build build -j && ctest --test-dir build -R SharedStateGTest --output-on-failure` | 1 轴配置下 `all_operational` 可变为 `true`；未配置轴不再影响判定 |
| C2 | `fix(axis_driver): send safe_target directly to prevent lock bypass` | B3（位置锁定被应用线程命令绕过） | `src/axis_driver.cpp` `test/test_state_machine.cpp` `test/test_canopen_robot_hw.cpp` | `cmake --build build -j && ctest --test-dir build -R "test_state_machine\|test_canopen_robot_hw" --output-on-failure` | 位置锁定阶段总线下发值始终来自 `safe_target` |
| C3 | `fix(config): validate axis_count and node_id bounds` | 补充高风险：`axis_count>6` 越界、`node_id` 回绕 | `src/main.cpp` `src/joints_config.cpp` `include/canopen_hw/joints_config.hpp` `test/test_joints_config.cpp` | `cmake --build build -j && ctest --test-dir build -R test_joints_config --output-on-failure` | 非法 `node_id` 和超轴数配置会明确报错并拒绝启动 |
| C4 | `fix(pdo): replace detached timeout thread with event-loop timer` | R1（`detach` 超时线程生命周期风险） | `include/canopen_hw/pdo_mapping.hpp` `src/pdo_mapping.cpp` `test/*(按实现补充)` | `cmake --build build -j && ctest --test-dir build --output-on-failure` | 无裸 `detach` 线程；超时仍能稳定触发并回调一次 |
| C5 | `refactor(master): restrict CreateAxisDrivers visibility and clarify API contracts` | D3（初始化期 API 不应 public）+ D2（悬空公开接口收口） | `include/canopen_hw/canopen_master.hpp` `include/canopen_hw/axis_driver.hpp` `src/axis_driver.cpp` | `cmake --build build -j && ctest --test-dir build --output-on-failure` | 对外 API 不再暴露易误用入口；编译与现有行为不回归 |
| C6 | `refactor(config): remove or wire runtime-only bitrate/sync settings` | D4（`bitrate/sync_period_us` 解析后丢弃） | `include/canopen_hw/joints_config.hpp` `src/joints_config.cpp` `src/main.cpp` `docs/yaml_config_guide.md` `docs/usage.md` | `cmake --build build -j && ctest --test-dir build -R test_joints_config --output-on-failure` | 文档与代码一致，不再出现“可配但无效”字段 |
| C7 | `cleanup(shared_state): remove dead SetAllOperational path and update tests` | D1（死代码清理） | `include/canopen_hw/shared_state.hpp` `src/shared_state.cpp` `test/test_shared_state.cpp` `test/test_canopen_robot_hw.cpp` | `cmake --build build -j && ctest --test-dir build -R "test_shared_state\|test_canopen_robot_hw\|SharedStateGTest" --output-on-failure` | 生产路径仅保留一个 `all_operational` 来源，测试全部通过 |
| C8 | `cleanup(cia402): resolve controlword naming ambiguity` | B1（`kCtrl_EnableVoltage`/`kCtrl_QuickStop` 命名混淆） | `include/canopen_hw/cia402_defs.hpp` `src/cia402_state_machine.cpp` `docs/debug_notes.md` | `cmake --build build -j && ctest --test-dir build -R test_state_machine --output-on-failure` | 常量命名与语义一致，无歧义引用 |
| C9 | `test(ci): include missing canopen_master test target and gate` | 测试覆盖缺口（`test_canopen_master.cpp` 未纳入 CTest） | `CMakeLists.txt` `test/test_canopen_master.cpp` | `cmake --build build -j && ctest --test-dir build -N && ctest --test-dir build --output-on-failure` | `ctest -N` 能看到 `test_canopen_master`，并可在 CI 中执行 |

---

## 2. 每个 Commit 的验收细则

### C1 验收点
- 新增“按已配置轴数汇总”的状态模型，默认值安全。
- 1 轴场景下，轴 1 `operational=true` 时系统整体可运行。
- 6 轴场景行为不变。

### C2 验收点
- `OnRpdoWrite()` 不再通过二次 `Snapshot` 读取目标后再发。
- 发给驱动的目标位置与 `safe_target` 一致。
- 并发压力下不出现锁定期跳变。

### C3 验收点
- `joints.yaml` 的 `node_id` 非法值（`<=0`、`>127`）明确报错。
- `axis_count > SharedState::kAxisCount` 阻断启动。
- 错误信息可直接定位到配置项。

### C4 验收点
- `PdoMappingReader` 生命周期与 Lely 事件循环一致。
- 超时、成功、失败三路径都只回调一次。
- 退出流程中不残留后台线程。

### C5 验收点
- `CreateAxisDrivers` 改为 `private` 后外部不可误调。
- `SetRosTargetPosition` 若继续保留，需调整可见性和注释；若删除，调用链不受影响。

### C6 验收点
- 二选一：彻底接通 `bitrate/sync_period_us`，或从 YAML/文档删除。
- `docs/yaml_config_guide.md` 与实际实现一致。

### C7 验收点
- 移除/收口 `SetAllOperational`，不允许绕过汇总逻辑直接赋值。
- 对应测试改为通过 `RecomputeAllOperational()` 驱动状态。

### C8 验收点
- 控制字常量一处定义、全局统一引用。
- `QuickStopActive` 分支注释清晰表达“退出 quick-stop”的动作意图。

### C9 验收点
- `ctest -N` 列表包含 `test_canopen_master`。
- 若测试依赖硬件，应改为 mock 或 fake，确保 CI 可跑。

---

## 3. 推荐执行顺序（含风险控制）

1. 先执行：`C1 -> C2 -> C3`（立即消除运行阻断与越界风险）。  
2. 中段执行：`C4 -> C6`（收敛生命周期风险和配置一致性）。  
3. 收尾执行：`C5 -> C7 -> C8 -> C9`（接口清理与覆盖补齐）。  

每完成一个 commit，执行：

```bash
cmake -S /home/dianhua/robot_test -B /home/dianhua/robot_test/build
cmake --build /home/dianhua/robot_test/build -j
ctest --test-dir /home/dianhua/robot_test/build --output-on-failure
```

---

## 4. 交付标准（完成本计划后）

1. `docs/bug_report.md` 中 `B2/B3/R1` 关闭。  
2. 本轮补充风险 `axis_count` 越界、`node_id` 回绕关闭。  
3. `ctest` 用例集完整且可在无硬件 CI 上稳定通过。  
4. 配置文档与实际行为无冲突字段。  

