# 9分质量提升执行清单

日期：2026-03-19  
目标：将当前项目质量从约 7/10 提升到 9/10（可稳定上线、可持续回归）

---

## 1. 评分口径（9分定义）

达到 9/10 需同时满足：

1. 安全性：无已知高风险并发/生命周期缺陷（UAF、data race）。
2. 可验证性：关键逻辑具备稳定自动化回归，测试在 Debug/Release 都有效。
3. 可运维性：关键故障可观测、可定位、可复现。
4. 工程闭环：CI 质量门阻断回归，变更有明确验收标准。

---

## 2. 问题复核结论（2026-03-19）

基于代码复核，对关键问题的判定如下：

1. Bug 1（`PdoMappingReader` UAF/竞争）：部分属实。  
   说明：并非常规路径必现，更接近“超时线程与事件线程交叉时的低概率生命周期窗口”。  
   严重度：中（低概率，但触发后影响严重）。

2. Bug 2（`assert` 在 `NDEBUG` 下失效）：完全属实。  
   说明：Release 下测试可能空转。  
   严重度：中（质量门风险）。

3. Bug 3（`CanopenMaster` 缺少 `axis_count <= 6` 库内约束）：属实。  
   说明：当前 `main` 入口有上限保护，但库复用场景仍有越界风险。  
   严重度：低到中（取决于复用方式）。

4. Bug 4（`AbsDiff` 溢出）：不作为当前缺陷。  
   说明：差值计算已在 `int64_t` 上完成；仅存在极端值回写 `int32_t` 的理论截断边界，工程概率极低。  
   严重度：低（可作为健壮性优化项）。

---

## 3. 分阶段实施

### 阶段A（P0，必须先完成）

1. 修复 PDO 验证异步生命周期问题（`PdoMappingReader`）。
2. 消除超时线程与 SDO 回调并发写状态的竞争窗口。
3. 增加线程模型说明文档（谁可以调用 `Finish()`、何时释放 reader）。

验收标准：
- `TSAN` 无 data race 报告。
- 启动/超时/失败/停止路径无崩溃，无悬挂线程。

### 阶段B（P0，测试可信度）

1. 将 `test/*.cpp` 中 `assert` 迁移为 `gtest EXPECT/ASSERT`。
2. 补关键边界测试：
   - `int32` 极值路径（作为健壮性保护，不作为当前高风险缺陷）。
   - `axis_count` 上下界。
   - `node_id` 非法输入。
   - PDO 验证超时与失败回调只触发一次。

验收标准：
- Release 构建下测试仍有效。
- 关键模块覆盖率：行覆盖 >= 85%，分支覆盖 >= 70%。

### 阶段C（P1，库级防御与契约）

1. 在 `CanopenMaster` 内部强制 `axis_count <= SharedState::kAxisCount`。
2. 参数合法性 fail-fast（`node_id`、阈值、重试次数）。
3. 将关键约束写入头文件注释与运行日志。

验收标准：
- 非法配置不会越界或静默降级，错误信息可定位到配置项。

### 阶段D（P1，可观测性）

1. 统一关键日志字段：`axis/node/state/ec/retry/latency_ms`。
2. 增加运行计数器：
   - PDO verify success/fail/timeout
   - heartbeat lost/recover
   - fault reset attempts
3. 输出简要健康快照（周期性或事件触发）。

验收标准：
- 任一故障可在 5 分钟内定位到轴号与失败阶段。

### 阶段E（P1，CI质量门）

1. 新增 CI：`Debug + Release` 双构建。
2. CI 门禁至少包含：`ctest`、`clang-tidy`、`-Wall -Wextra -Werror`。
3. 覆盖率阈值和变更回归阈值（低于阈值阻断合并）。

验收标准：
- PR 未通过质量门不可合并。

### 阶段F（P2，上线前验证）

1. Soak：8-24 小时稳定运行（建议先 8 小时，再 24 小时）。
2. 故障注入：断线、节点重启、PDO 不匹配、超时、EMCY。
3. 发布评审：问题清零表 + 风险接受清单。

验收标准：
- 无崩溃/死锁。
- 故障场景满足恢复时限。

---

## 4. 建议里程碑（两周）

1. Week 1：完成 A + B + C。
2. Week 2：完成 D + E，执行 F 的 8 小时 Soak。
3. Week 3（可选）：完成 24 小时 Soak 和发布评审。

---

## 5. 按 Commit 拆分（对应 A-F 阶段）

总计建议：10 个 commits。

1. 阶段A（2 commits）
   - C1: `fix(pdo): remove reader lifetime race between timeout and SDO callbacks`
   - C2: `refactor(pdo): serialize finish path and document callback-thread contract`

2. 阶段B（3 commits）
   - C3: `test: migrate test_state_machine and test_unit_conversion from assert to gtest`
   - C4: `test: migrate test_canopen_master and test_joints_config from assert to gtest`
   - C5: `test: add boundary cases for axis_count/node_id/pdo-timeout-once semantics`

3. 阶段C（1 commit）
   - C6: `fix(master): enforce axis_count upper bound and fail-fast config validation`

4. 阶段D（1 commit）
   - C7: `feat(observability): add structured logs and runtime health counters`

5. 阶段E（2 commits）
   - C8: `ci: add debug/release build and ctest gating workflow`
   - C9: `ci: add clang-tidy and warning-as-error quality gates`

6. 阶段F（1 commit）
   - C10: `docs(verification): add soak/fault-injection report and release readiness checklist`

说明：
- Bug 4（`AbsDiff` 极端值保护）不单独占用 commit，可并入 C5（测试）或 C6（防御性修正）。
- 若阶段D范围扩大（如导出 metrics endpoint），可将 C7 拆为 2 个 commits。

---

## 6. 任务映射（代码位置）

1. 并发与生命周期：
   - `include/canopen_hw/pdo_mapping.hpp`
   - `src/pdo_mapping.cpp`
   - `src/axis_driver.cpp`
2. 测试体系：
   - `test/test_state_machine.cpp`
   - `test/test_canopen_master.cpp`
   - `test/test_joints_config.cpp`
   - `test/test_unit_conversion.cpp`
3. 配置与边界防御：
   - `src/canopen_master.cpp`
   - `src/main.cpp`
   - `src/joints_config.cpp`
4. 工程质量门：
   - `.github/workflows/*`（新增）
   - `CMakeLists.txt`

---

## 7. 当前状态（2026-03-19）

1. 已具备可运行基础与测试框架（本地 `ctest` 可通过）。
2. 尚未达 9 分的关键缺口：
   - 异步生命周期窗口风险未彻底消除。
   - `assert` 型测试仍较多。
   - 库内上限防御与 CI 质量门仍缺失。
