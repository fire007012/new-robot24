# Bug 修复报告（阶段性）

日期：2026-03-19  
范围：按 `docs/bugfix_commit_plan.md` 已完成的修复提交

---

## 1. 执行摘要

本轮共完成 9 个修复/治理 commit，覆盖：
- 运行阻断问题（`all_operational` 固定 6 轴导致无法运行）
- 并发/一致性问题（位置锁定被旁路）
- 配置安全问题（`node_id` 回绕、轴数越界）
- 线程生命周期风险（PDO 校验超时 `detach`）
- API 误用风险（初始化接口和内部接口对外暴露）
- 配置语义误导（`bitrate/sync_period_us` 可配但无效）
- 死代码清理与测试补齐

当前基线测试结果：`ctest -VV` 通过 `12/12`。

---

## 2. 提交明细（按时间）

### 2.1 计划文档提交

1. `84f7854`  
标题：`docs: add commit-level bugfix execution plan`  
内容：
- 新增 [bugfix_commit_plan.md](/home/dianhua/robot_test/docs/bugfix_commit_plan.md)，定义 C1~C9 的修复顺序、文件范围、验证命令和通过标准。

---

### 2.2 代码修复与测试接入

1. `be1e03e`  
标题：`fix(shared_state): recompute all_operational using configured axis count`  
对应问题：B2  
主要改动：
- `SharedState` 增加 `active_axis_count` 概念；`RecomputeAllOperational()` 只统计已配置轴。
- `main` 在加载配置后设置有效轴数。
- 新增 GTest 用例验证“单轴配置时其余空槽不影响 all_operational”。  
关键文件：
- [shared_state.hpp](/home/dianhua/robot_test/include/canopen_hw/shared_state.hpp)
- [shared_state.cpp](/home/dianhua/robot_test/src/shared_state.cpp)
- [main.cpp](/home/dianhua/robot_test/src/main.cpp)
- [test_shared_state_gtest.cpp](/home/dianhua/robot_test/test/test_shared_state_gtest.cpp)

2. `17a2d2d`  
标题：`fix(axis_driver): send safe target directly in RPDO callback`  
对应问题：B3  
主要改动：
- `OnRpdoWrite()` 下发目标位置时不再二次 `Snapshot()` 取 `SharedState`，改为直接发送状态机 `safe_target`，防止锁定阶段被应用线程命令覆盖。  
关键文件：
- [axis_driver.cpp](/home/dianhua/robot_test/src/axis_driver.cpp)

3. `dbb1e66`  
标题：`fix(config): validate node_id range and reject axis count overflow`  
对应问题：补充高风险（`node_id` 回绕、轴数越界）  
主要改动：
- `LoadJointsYaml()` 对显式 `node_id` 强制校验 `1..127`，非法直接返回错误。
- `main` 对 `axis_count` 做上限检查，超过 `SharedState::kAxisCount` 直接拒绝启动。
- 测试新增非法 `node_id` 场景。  
关键文件：
- [joints_config.cpp](/home/dianhua/robot_test/src/joints_config.cpp)
- [main.cpp](/home/dianhua/robot_test/src/main.cpp)
- [test_joints_config.cpp](/home/dianhua/robot_test/test/test_joints_config.cpp)

4. `2b734a8`  
标题：`fix(pdo): replace detached timeout thread with managed lifecycle`  
对应问题：R1  
主要改动：
- 移除裸 `detach` 超时线程。
- 引入受控超时线程生命周期（停止信号 + 条件变量 + join/detach 保护）。
- `Finish()` 与析构阶段统一回收线程。  
关键文件：
- [pdo_mapping.hpp](/home/dianhua/robot_test/include/canopen_hw/pdo_mapping.hpp)
- [pdo_mapping.cpp](/home/dianhua/robot_test/src/pdo_mapping.cpp)

5. `cb3a4a6`  
标题：`refactor(api): make init-only and internal driver methods private`  
对应问题：D2/D3  
主要改动：
- `CanopenMaster::CreateAxisDrivers` 改为 `private`。
- `AxisDriver::SetRosTargetPosition` 改为 `private`，避免外部绕过数据面。  
关键文件：
- [canopen_master.hpp](/home/dianhua/robot_test/include/canopen_hw/canopen_master.hpp)
- [axis_driver.hpp](/home/dianhua/robot_test/include/canopen_hw/axis_driver.hpp)

6. `ba9c048`  
标题：`refactor(config): remove ineffective bitrate and sync_period_us runtime fields`  
对应问题：D4  
主要改动：
- 移除运行时 `bitrate/sync_period_us` 字段解析与日志输出（避免“可配但无效”误导）。
- 同步精简 `config/joints.yaml` 示例。
- 更新文档说明运行时仅 `interface/master_node_id` 生效。  
关键文件：
- [joints_config.hpp](/home/dianhua/robot_test/include/canopen_hw/joints_config.hpp)
- [joints_config.cpp](/home/dianhua/robot_test/src/joints_config.cpp)
- [main.cpp](/home/dianhua/robot_test/src/main.cpp)
- [joints.yaml](/home/dianhua/robot_test/config/joints.yaml)
- [yaml_config_guide.md](/home/dianhua/robot_test/docs/yaml_config_guide.md)
- [roadmap_to_production.md](/home/dianhua/robot_test/docs/roadmap_to_production.md)

7. `f9ac58c`  
标题：`refactor(shared_state): remove SetAllOperational and use recompute path in tests`  
对应问题：D1  
主要改动：
- 删除 `SetAllOperational`，统一通过 `RecomputeAllOperational()` 计算系统可运行状态。
- 更新受影响测试，按真实生产路径构造状态。  
关键文件：
- [shared_state.hpp](/home/dianhua/robot_test/include/canopen_hw/shared_state.hpp)
- [shared_state.cpp](/home/dianhua/robot_test/src/shared_state.cpp)
- [test_shared_state.cpp](/home/dianhua/robot_test/test/test_shared_state.cpp)
- [test_shared_state_gtest.cpp](/home/dianhua/robot_test/test/test_shared_state_gtest.cpp)
- [test_canopen_robot_hw.cpp](/home/dianhua/robot_test/test/test_canopen_robot_hw.cpp)
- [test_unit_conversion.cpp](/home/dianhua/robot_test/test/test_unit_conversion.cpp)

8. `c7b43d6`  
标题：`cleanup(cia402): remove ambiguous kCtrl_QuickStop alias`  
对应问题：B1  
主要改动：
- 删除与 `kCtrl_EnableVoltage` 同值的别名 `kCtrl_QuickStop`，消除控制字语义歧义。  
关键文件：
- [cia402_defs.hpp](/home/dianhua/robot_test/include/canopen_hw/cia402_defs.hpp)

9. `747516e`  
标题：`test(cmake): include canopen_master test in ctest without hardware dependency`  
对应问题：测试覆盖缺口  
主要改动：
- 将 `test_canopen_master` 接入 `CMakeLists.txt` 与 CTest。
- 测试改为无硬件依赖，仅验证构造期配置归一化逻辑，确保可进 CI。  
关键文件：
- [CMakeLists.txt](/home/dianhua/robot_test/CMakeLists.txt)
- [test_canopen_master.cpp](/home/dianhua/robot_test/test/test_canopen_master.cpp)

---

## 3. 验证记录

每个代码 commit 均按以下顺序执行并通过后才提交：

```bash
cmake -S /home/dianhua/robot_test -B /home/dianhua/robot_test/build
cmake --build /home/dianhua/robot_test/build -j
cd /home/dianhua/robot_test/build && ctest -VV
```

最终状态：
- CTest 总数：12
- 通过：12
- 失败：0

---

## 4. 结果与剩余项

已关闭（本轮完成）：
- B1 / B2 / B3 / D1 / D2 / D3 / D4 / R1
- 补充风险：`axis_count` 越界、`node_id` 回绕
- 测试接入缺口：`test_canopen_master` 未进 CTest

仍建议后续跟进：
- R2（`boot_retry_count_` 并发保护）可做防御性增强（`atomic` 或统一锁保护）。
- `docs/bug_report.md` 可更新状态栏（已修/待修），便于审计闭环。

