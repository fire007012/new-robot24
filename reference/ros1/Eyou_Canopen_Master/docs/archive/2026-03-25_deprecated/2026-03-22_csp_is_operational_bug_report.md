# Bug 报告：CSP 模式下 Recover 后无法进入 is_operational

**项目：** Eyou_Canopen_Master（Lely Core + ROS）
**日期：** 2026-03-22
**总线：** `canable0`
**从站：** Node ID = 5
**模式：** CSP（Cyclic Synchronous Position，mode=8）
**严重程度：** 高（Recover 后驱动器永久无法使能）

---

## 1. 问题现象

执行以下操作序列后，驱动器无法进入可运行状态：

```bash
rosservice call /canopen_hw_node/halt   "{}"   # success: True
rosservice call /canopen_hw_node/recover "{}"  # success: True
```

`Recover` 返回成功，但此后驱动器的 `mode_display` 从 `0x08`（CSP）归零，
`is_operational` 永远为 `false`，轴无法受控。

CAN 总线抓包（`candump` 节选）：

```
# Recover 调用后，主站开始发送：
(1774110525.332335) canable0 185#5002E902000008   ← 节点第一帧 TPDO，mode_display=8 ✓
(1774110525.332560) canable0 285#00000000F9FF
(1774110525.342328) canable0 205#00000000000000   ← 主站 RPDO：controlword=0, mode=0 ✗
(1774110525.342412) canable0 185#5002E902000000   ← 节点 mode_display 变 0，被损坏
```

---

## 2. 报文解码

| 报文 | COB-ID | 字段 | 值 | 含义 |
|------|--------|------|----|------|
| 节点 TPDO1（首帧） | `185` | statusword | `0x0250` | SwitchOnDisabled |
| | | actual_position | 745 counts | — |
| | | mode_display | `0x08` | CSP ✓ |
| 主站 RPDO1 | `205` | controlword | `0x0000` | DisableVoltage ✗ |
| | | target_position | `0` | — |
| | | mode_of_operation | `0x00` | 未设置 ✗ |
| 节点 TPDO1（次帧） | `185` | mode_display | `0x00` | 模式被清除 ✗ |

---

## 3. 根因分析

### 3.1 直接触发路径

```
Recover()
  └─ master_->Stop()   → axis_drivers_ 全部销毁，ev_thread_ join
  └─ master_->Start()  → 新 AxisDriver 创建，tpdo_mapped 全部归零
                          ev_loop 启动，master_->Reset()
                             │
                             ├─ Lely 开始发 SYNC（每 10 ms）
                             ├─ SYNC-triggered RPDO 以 tpdo_mapped 快照发送
                             │   → tpdo_mapped 全零 → 205#00000000000000
                             │
                             ├─ 节点 boot 完成，进入 Operational
                             ├─ 节点发第一帧 TPDO（185#...08）
                             ├─ OnRpdoWrite 触发，写入 mode=8 + WriteEvent
                             │
                             └─ 但此时下一个 SYNC 的 PDO 快照已提交
                                → 仍然发出 205#...00（mode=0）← 损坏点
```

**关键**：Lely 的 SYNC-triggered RPDO 在 SYNC 处理时读取 `tpdo_mapped` 快照，
`OnRpdoWrite` 的 `WriteEvent` 要等到**再下一个 SYNC** 才能覆盖。
因此在 `OnRpdoWrite` 首次触发之前，至少有一帧全零的 RPDO 被发出。

### 3.2 被损坏后的死锁链路

驱动器收到 `mode_of_operation=0` 后将 `mode_display` 清零。
此时旧状态机代码（修复前）的 `ReadyToSwitchOn` 分支：

```cpp
// 修复前代码
if (enable_requested_ && mode_display == target_mode_) {
    controlword_ = kCtrl_EnableOperation;
} else {
    controlword_ = kCtrl_Shutdown;   // ← mode_display=0≠8，永远走这里
}
```

`EnableOperation`（`0x000F`）永远不会被发出，驱动器卡在
`ReadyToSwitchOn`，`is_operational` 永远为 `false`。

### 3.3 为何 Halt→Recover 能复现，但首次启动不能

首次启动时，从节点 boot 完成到第一个 SYNC 之间有足够时间让
`OnBoot` 回调完成设置。`Recover` 会销毁并重建 Lely 事件循环，
SYNC 在重建后立即开始计时，`OnBoot` 还未来得及预写 `tpdo_mapped`
就已经发出了全零 RPDO。

---

## 4. 修复方案

### 修复 1：`OnBoot` 成功路径预写 `tpdo_mapped`（`src/axis_driver.cpp`）

在节点 boot 完成（进入 Operational）时，**立即**写入正确的初始值，
确保下一个 SYNC 发出的是正确 mode 和 controlword，
不再依赖 `OnRpdoWrite` 赶上 SYNC 窗口。

```cpp
// OnBoot，es == 0 成功路径中新增：
{
    std::error_code ec;
    tpdo_mapped[0x6060][0].Write(kMode_CSP, ec);      // mode=8
    if (!ec) tpdo_mapped[0x6060][0].WriteEvent(ec);

    ec.clear();
    tpdo_mapped[0x6040][0].Write(kCtrl_Shutdown, ec); // controlword=0x0006
    if (!ec) tpdo_mapped[0x6040][0].WriteEvent(ec);
}
```

### 修复 2：去掉 `mode_display` 门控（`src/cia402_state_machine.cpp`）

`ReadyToSwitchOn` 分支不应用 `mode_display` 来门控使能序列：
mode 是主站主动写出的，用回读值阻塞自己的使能命令是错误设计，
会在任何 mode 短暂抖动时造成死锁。

```cpp
// 修复后：只看 enable_requested_
case CiA402State::ReadyToSwitchOn:
    controlword_ = enable_requested_ ? kCtrl_EnableOperation : kCtrl_Shutdown;
    ...
```

mode 的正确性应在进入 `OperationEnabled` 后由上层检查，而非在此门控。

---

## 5. 修复后预期的 CAN 报文

```
OnBoot 触发
    ↓
205#06000000000008   ← controlword=Shutdown, mode=CSP（不再全零）
185#5002E902000008   ← statusword=SwitchOnDisabled, mode_display=8 ✓
205#06000000000008
185#2100E902000008   ← statusword=ReadyToSwitchOn
205#0F000000000008   ← controlword=EnableOperation
185#2700E902000008   ← statusword=OperationEnabled
                        → is_operational = true ✓
```

---

## 6. 测试验证

新增回归测试（`test/test_state_machine.cpp`）：

```cpp
TEST(CiA402SM, ReadyToSwitchOnEnablesEvenIfModeDisplayIsZero) {
    CiA402StateMachine sm;
    sm.set_target_mode(kMode_CSP);
    sm.Update(0x0040, kMode_CSP, 100);
    sm.Update(0x0021, /*mode_display=*/0, 100);  // mode 被损坏
    EXPECT_EQ(sm.state(), CiA402State::ReadyToSwitchOn);
    EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);  // 仍然发使能 ✓
}
```

编译结果：`[100%] Built target canopen_hw_node`，无警告无错误。
测试结果：`7 tests from CiA402SM — PASSED`。

---

## 7. 分析结论补充（评审修正）

以下两点是初始分析中的描述偏差，已在代码层面核实：

| 描述 | 初始分析 | 正确结论 |
|------|----------|----------|
| 自动重基准后解锁时机 | "需要下一帧才能解锁" | 重基准与解锁在**同一 `Update()` 帧内**完成（两个顺序 `if`，非 `else if`） |
| 未调 `set_ros_target` 是否永久卡死 | "会永久卡死" | 修复门控后，`WriteToSharedState` 持续刷新目标，不构成永久卡死条件 |

---

## 8. 受影响文件

| 文件 | 改动类型 |
|------|----------|
| `src/axis_driver.cpp` | 新增 `OnBoot` 预写 `tpdo_mapped` 逻辑 |
| `src/cia402_state_machine.cpp` | 去掉 `ReadyToSwitchOn` 的 `mode_display` 门控；`(void)mode_display` |
| `test/test_state_machine.cpp` | 新增回归测试 `ReadyToSwitchOnEnablesEvenIfModeDisplayIsZero` |
