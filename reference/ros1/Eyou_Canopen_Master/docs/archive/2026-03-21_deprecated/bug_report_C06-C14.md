# Bug 报告与修复计划：C06-C14 引入的问题

- 日期：2026-03-19
- 审查范围：C06-C14 全部改动
- 严重程度分级：P0（必须修复）、P1（应该修复）、P2（建议改进）

---

## P0 — 必须修复

### BUG-4: SdoAccessor 同步方法 promise 生命周期 UAF

- 引入 commit：C06
- 文件：`src/sdo_accessor.cpp:90-104`（Read）、`106-121`（Write）
- 现象：`promise` 是栈变量，lambda 以 `&promise` 引用捕获。当前错误路径（master==null、node_id not found）回调是同步执行的，不会触发问题。但接入真实 Lely 后，`AsyncRead` 的回调在 Lely 事件线程异步执行。如果 `future.wait_for(timeout)` 超时，函数返回，`promise` 被销毁，随后 Lely 线程回调写入已销毁的 `promise`，导致 use-after-free。
- 影响：接入真实硬件后，SDO 超时场景必崩。
- 根因：同步包装异步时未考虑超时后回调仍会执行的情况。
- 修复方案：将 `promise` 改为 `shared_ptr<promise>`，lambda 按值捕获 `shared_ptr`。超时后 `promise` 仍存活直到回调完成。

```cpp
// 修复前（危险）：
std::promise<SdoResult> promise;
auto future = promise.get_future();
AsyncRead(node_id, index, subindex,
          [&promise](const SdoResult& result) {
            promise.set_value(result);
          });

// 修复后（安全）：
auto promise = std::make_shared<std::promise<SdoResult>>();
auto future = promise->get_future();
AsyncRead(node_id, index, subindex,
          [promise](const SdoResult& result) {
            promise->set_value(result);
          });
```

---

### BUG-5: CSV/CST 模式下位置锁定解锁条件语义错误

- 引入 commit：C11
- 文件：`src/cia402_state_machine.cpp:242-255`（StepOperationEnabled）
- 现象：`StepOperationEnabled` 的解锁条件是 `AbsDiff(ros_target_, actual_position) <= position_lock_threshold_`。这对 CSP 模式正确——上层必须先把目标位置设到接近当前位置才放行。但对 CSV/CST 模式，用户可能不设置 `ros_target_`（默认 0），如果电机实际位置不在 0 附近，位置锁定永远不会释放，速度/力矩命令永远被归零，轴永远不会变成 operational。
- 影响：CSV/CST 模式在电机非零位时无法进入运行态。
- 根因：多模式扩展时机械地复用了 CSP 的位置锁定逻辑，没有按模式区分解锁策略。
- 修复方案：在 `StepOperationEnabled` 中按 `target_mode_` 区分：
  - CSP：保持现有位置锁定逻辑
  - CSV/CST：跳过位置锁定，首次进入 OperationEnabled 后直接 `position_locked_ = false`（速度/力矩目标默认 0 本身就是安全的）

```cpp
void CiA402StateMachine::StepOperationEnabled(int32_t actual_position) {
  if (!was_operation_enabled_) {
    position_locked_ = true;
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
  }

  // CSV/CST 模式不需要位置锁定：速度/力矩默认 0 本身安全。
  if (target_mode_ == kMode_CSV || target_mode_ == kMode_CST) {
    if (!was_operation_enabled_) {
      // 首帧保持归零（上面已设），下一帧解锁。
      return;
    }
    position_locked_ = false;
    safe_target_ = actual_position;  // CSP 目标跟随实际，防止意外切回 CSP 时跳变。
    safe_target_velocity_ = ros_target_velocity_;
    safe_target_torque_ = ros_target_torque_;
    is_operational_ = true;
    return;
  }

  // CSP 模式：保持原有位置锁定逻辑。
  if (position_locked_) {
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
    if (AbsDiff(ros_target_, actual_position) <=
        static_cast<int64_t>(position_lock_threshold_)) {
      position_locked_ = false;
      safe_target_ = ros_target_;
      safe_target_velocity_ = ros_target_velocity_;
      safe_target_torque_ = ros_target_torque_;
    }
  } else {
    safe_target_ = ros_target_;
    safe_target_velocity_ = ros_target_velocity_;
    safe_target_torque_ = ros_target_torque_;
  }
  is_operational_ = !position_locked_;
}
```

---

## P1 — 应该修复

### BUG-6: AsyncSdoRead 固定使用 SubmitRead<uint32_t>

- 引入 commit：C06
- 文件：`src/axis_driver.cpp:136-156`
- 现象：`AsyncSdoRead` 始终调用 `SubmitRead<uint32_t>`，对于 1 字节（如 0x6061 mode_display）或 2 字节对象，部分驱动器可能拒绝 4 字节 expedited 读取并返回 SDO abort。
- 影响：对严格校验传输大小的驱动器，读取小于 4 字节的对象会失败。
- 修复方案：`AsyncSdoRead` 增加 `size_hint` 参数（默认 4），按大小选择 `SubmitRead<uint8_t>` / `<uint16_t>` / `<uint32_t>`。或改用 Lely 的 segmented/block SDO 接口。

### BUG-7: NmToTorquePermille 溢出未 clamp

- 引入 commit：C13
- 文件：`src/canopen_robot_hw.cpp:165-171`
- 现象：`std::llround(nm / scale / rated * 1000.0)` 结果直接 `static_cast<int16_t>`。极端输入（如 rated_torque_nm 很小、nm 很大）会导致 `llround` 结果超出 int16_t 范围（-32768~32767）��截断后值无意义。
- 影响：极端参数下力矩命令值错误。正常使用场景不太可能触发。
- 修复方案：`llround` 后 clamp 到 `[-32767, 32767]` 再 cast。

### BUG-8: RadPerSecToTicksPerSec 同样存在溢出风险

- 引入 commit：C13
- 文件：`src/canopen_robot_hw.cpp:149-156`
- 现象：与 BUG-7 同理，`llround` 结果直接 `static_cast<int32_t>`，极端输入可能溢出。
- 修复方案：clamp 到 `INT32_MIN/INT32_MAX`。

---

## P2 — 建议改进

### ISSUE-12: as_u16() 双重 static_cast 可读性差

- 引入 commit：C06（修 -Werror=conversion 时引入）
- 文件：`src/sdo_accessor.cpp:16-17`
- 现象：`static_cast<uint16_t>(static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8))` 嵌套 4 层 cast，难以阅读。
- 修复方案：用 `uint32_t` 中间变量运算，最终 cast 回 `uint16_t`。

### ISSUE-13: DiagnosticsCollector 未测试与真实 CanopenMaster 的集成

- 引入 commit：C09
- 现象：测试仅覆盖 null master 和 SharedState 直接注入。`Collect()` 通过 `CanopenMaster` 拉取数据的路径未被测试覆盖。
- 修复方案：补充集成测试，构造 mock CanopenMaster 验证完整 Collect 路径。

### ISSUE-14: test_multi_mode 中 CSVModeEnablesCorrectly 断言不够精确

- 引入 commit：C14
- 文件：`test/test_multi_mode.cpp:30-33`
- 现象：`DriveToOperational(kMode_CSV)` 后断言 `is_operational() == true`，但注释说"还在锁定阶段第一帧"。实际上由于 BUG-5 的存在，这个测试能通过是因为 `ros_target_` 被设为 1000 且 `position_lock_threshold_` 设为 50000，恰好满足解锁条件。测试没有暴露 BUG-5。
- 修复方案：BUG-5 修复后，重写此测试，验证 CSV 模式下不依赖位置锁定即可进入 operational。

---

## 修复计划

依赖关系：

```
F01 (BUG-4) ──→ 独立
F02 (BUG-5) ──→ F05 (测试重写依赖 F02)
F03 (BUG-6) ──→ 独立
F04 (BUG-7+8) ──→ 独立
F05 (ISSUE-14) ──→ 依赖 F02
F06 (ISSUE-12) ──→ 独立
```

### F01 — `fix: use shared_ptr<promise> in SdoAccessor sync methods` (BUG-4)

- 优先级：P0 | 工作量：小
- 改动文件：`src/sdo_accessor.cpp`（Read 和 Write 两个方法，各改 3 行）
- 将 `std::promise<SdoResult> promise` 改为 `auto promise = std::make_shared<std::promise<SdoResult>>()`
- lambda 按值捕获 `promise`（shared_ptr 拷贝）
- `future` 从 `promise->get_future()` 获取
- 测试：现有 test_sdo_accessor 的同步测试仍通过；真实硬件场景下超时不再 UAF

### F02 — `fix: skip position lock for CSV/CST modes in state machine` (BUG-5)

- 优先级：P0 | 工作量：小
- 改动文件：`src/cia402_state_machine.cpp`（StepOperationEnabled 方法，约 15 行重构）
- 按 `target_mode_` 区分：CSV/CST 首帧归零、次帧起直接解锁；CSP 保持原有位置锁定
- 测试：需同步更新 test_multi_mode 中的相关用例

### F03 — `fix: support variable SDO object sizes in AsyncSdoRead` (BUG-6)

- 优先级：P1 | 工作量：中
- 改动文件：
  - `include/canopen_hw/axis_driver.hpp`（AsyncSdoRead 签名加 `uint8_t size_hint = 4`）
  - `src/axis_driver.cpp`（按 size_hint 分发 SubmitRead 模板）
  - `include/canopen_hw/sdo_accessor.hpp`（AsyncRead 签名加 size_hint）
  - `src/sdo_accessor.cpp`（透传 size_hint）

### F04 — `fix: clamp torque/velocity conversion to prevent overflow` (BUG-7, BUG-8)

- 优先级：P1 | 工作量：小
- 改动文件：`src/canopen_robot_hw.cpp`（NmToTorquePermille +2 行 clamp, RadPerSecToTicksPerSec +2 行 clamp）
- 使用 `std::clamp` 限制 `llround` 结果到目标类型范围

### F05 — `test: rewrite CSV/CST tests after position lock fix` (ISSUE-14)

- 优先级：P1 | 工作量：小
- 依赖：F02
- 改动文件：`test/test_multi_mode.cpp`
- 重写 `CSVModeEnablesCorrectly`：验证电机在非零位时 CSV 模式仍能进入 operational
- 新增：CST 模式非零位解锁测试
- 新增：CSV���CSP 模式切换时位置锁定重新生效测试

### F06 — `refactor: simplify as_u16 cast` (ISSUE-12)

- 优先级：P2 | 工作量：小
- 改动文件：`src/sdo_accessor.cpp`（as_u16 方法，改 3 行）

---

## 修复优先级总表

| 顺序 | 编号 | Bug | 严重程度 | 工作量 |
|------|------|-----|----------|--------|
| 1 | F01 | BUG-4 promise UAF | P0 | 小 |
| 2 | F02 | BUG-5 CSV/CST 位置锁定 | P0 | 小 |
| 3 | F04 | BUG-7+8 换算溢出 | P1 | 小 |
| 4 | F03 | BUG-6 SDO 对象大小 | P1 | 中 |
| 5 | F05 | 测试重写 | P1 | 小 |
| 6 | F06 | cast 可读性 | P2 | 小 |

建议 F01 和 F02 在下一次 commit 中立即修复，它们是本轮开发引入的最严重问题。
