# 生产化路线图（合并版）

日期：2026-03-18  
目标：从“可联调”推进到“可稳定上线（8/10）”，并满足较好环境下连续 3 小时稳定运行。

说明：
- 本文档合并了“代码修复路线”与“测试/验收路线”。
- `docs/test_plan.md` 是执行细则；本文是 commit 级主计划与里程碑。

---

## 阶段总览

| 阶段 | commit | 核心目标 | 是否上线门槛 |
|------|--------|----------|--------------|
| 一：安全底线 | 1-4 | 并发安全、状态一致性、指令链路、心跳感知 | 必须完成 |
| 二：故障恢复 | 5-7 | EMCY/QuickStop/Boot 失败恢复闭环 | 强烈建议 |
| 三：配置可控 | 8-9 | 配置项真实生效，消除悬空参数 | 必须完成 |
| 四：CI 保障 | 10-13 | 自动回归、可诊断测试输出、并发压测 | 必须完成 |
| 五：验收收口 | 14 | 3 小时 Soak 与发布准入结论 | 必须完成 |

---

## 阶段一：安全底线（上线硬门槛）

### commit 1：修复 PDO 校验标志线程安全
问题：`pdo_verified_`/`pdo_verification_done_` 跨线程读写，普通 `bool` 存在数据竞争。  
改动：
- `include/canopen_hw/axis_driver.hpp`：改为 `std::atomic<bool>`
- `src/axis_driver.cpp`：读写改为 `load/store`  
验证：构建通过；`test_canopen_robot_hw` 通过；TSAN 无报告（后续补跑）。

### commit 2：修复 RecomputeAllOperational 双锁 TOCTOU
问题：读取 `feedback_` 与写 `all_operational_` 分两次加锁，中间存在竞争窗口。  
改动：
- `src/shared_state.cpp`：单锁内完成计算与写入  
验证：`test_shared_state` 通过。

### commit 3：闭合指令下发链路
问题：`SharedState::commands` 被写入但未下发到总线，控制链路不闭环。  
改动：
- `include/canopen_hw/axis_driver.hpp`：新增目标位置下发接口
- `src/axis_driver.cpp`：在 `OnRpdoWrite` 末尾读取 `commands[axis_index_]`，写 `0x607A`  
验证：`candump` 可观察目标相关 PDO 变化。

### commit 4：心跳超时上报
问题：`OnHeartbeat` 空实现，掉线后系统无感知。  
改动：
- `include/canopen_hw/shared_state.hpp`：新增 `heartbeat_lost`
- `src/axis_driver.cpp`：超时置位故障，恢复清位，并发布快照  
验证：断线时 `all_operational=false`，恢复后状态回升。

---

## 阶段二：故障自恢复完整化

### commit 5：EMCY 记录与上报
改动：
- `AxisFeedback` 增加 `last_emcy_eec`
- `OnEmcy` 记录 eec/er，打印结构化日志并发布快照  
验证：触发 EMCY 后日志与快照一致。

### commit 6：QuickStop 自动恢复路径
改动：
- `QuickStopActive` 分支改为发送 `EnableVoltage(0x0002)` 推动回迁
- 补状态机测试用例  
验证：QuickStop 后可回到可使能路径。

### commit 7：OnBoot 失败重试
改动：
- 新增重试计数与上限
- 失败时执行 `RESET_NODE`，超限后永久失败  
验证：模拟失败场景可见重试与最终决策。

---

## 阶段三：配置可控

### commit 8：接通每轴状态机参数
改动：
- 解析并接通 `position_lock_threshold/max_fault_resets/fault_reset_hold_cycles`
- 注入到 `AxisDriver`/状态机  
验证：修改 YAML 后行为可观测变化。

### commit 9：接通 joints.yaml 顶层 canopen
改动：
- 解析并接通 `interface/master_node_id`
- `main.cpp` 不再硬编码 `canable0/127`  
验证：修改配置可切换接口与主站 ID。

---

## 阶段四：CI 回归保障

### commit 10：接入 CTest
改动：
- `CMakeLists.txt` 增加 `enable_testing()` + `add_test()`  
验证：`ctest --test-dir build --output-on-failure` 统一执行。

### commit 11：升级测试可诊断性（Catch2）
改动：
- 引入 Catch2，替换裸 `assert`  
验证：失败输出包含文件、行号与断言上下文。

### commit 12：并发压力测试
改动：
- 新增 `test_shared_state_concurrent.cpp`，多线程高频读写  
验证：TSAN 下无 data race 报告。

### commit 13：仓库卫生与忽略规则
改动：
- `.gitignore` 补 `build/` `build_cmake/` 等  
验证：新产物不再误入版本库。

---

## 阶段五：验收收口

### commit 14：3 小时 Soak 与发布准入
改动：
- 新增 soak 执行脚本与报告模板（参照 `docs/test_plan.md`）
- 固化发布前检查清单  
验证（必须全部满足）：
- 连续 3 小时：0 崩溃、0 卡死、0 不可恢复故障
- 故障注入后可在约定时限恢复
- CI 全绿
- 测试报告完整可追溯

---

## 当前执行策略

1. 优先按阶段顺序提交，避免跨阶段混改。  
2. 每个 commit 要求：
- 代码改动最小可验证
- 本地编译通过
- 至少执行受影响测试  
3. 阶段门槛：
- 阶段一未完成，不进入线上联调
- 阶段四未完成，不允许长期维护分支扩展
- 阶段五未通过，不打发布标签
