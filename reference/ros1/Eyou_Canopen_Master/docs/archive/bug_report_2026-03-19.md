# Bug 报告与悬空问题清单

日期：2026-03-19
范围：全量代码审查（按数据流向逐文件排查）

---

## 优先级总览

| 级别 | 编号 | 问题 | 文件 |
|------|------|------|------|
| 必须修 | B2 | `RecomputeAllOperational` 遍历全 6 轴，未配置轴导致系统永远不可运行 | `shared_state.cpp` |
| 必须修 | B3 | 位置锁定可被应用线程命令绕过 | `axis_driver.cpp` |
| 建议修 | B1 | `kCtrl_EnableVoltage` 与 `kCtrl_QuickStop` 同值，命名混淆 | `cia402_defs.hpp` |
| 建议修 | D4 | `bitrate`/`sync_period_us` 解析后被丢弃，配置无效 | `joints_config.cpp` |
| 建议修 | D3 | `CreateAxisDrivers` 应为 private | `canopen_master.hpp` |
| 清理 | D1 | `SetAllOperational` 生产路径死代码 | `shared_state.hpp/cpp` |
| 清理 | D2 | `SetRosTargetPosition` 悬空公开接口 | `axis_driver.hpp` |
| 风险 | R1 | 超时线程用裸 `detach`，进程退出时行为未定义 | `pdo_mapping.cpp` |
| 风险 | R2 | `boot_retry_count_` 无锁保护 | `axis_driver.cpp` |

---

## B1 — `kCtrl_EnableVoltage` 与 `kCtrl_QuickStop` 同值

文件：[include/canopen_hw/cia402_defs.hpp](../include/canopen_hw/cia402_defs.hpp#L34-L35)

```cpp
constexpr uint16_t kCtrl_EnableVoltage = 0x0002;
constexpr uint16_t kCtrl_QuickStop     = 0x0002;  // 同值
```

两个语义不同的常量值相同。`QuickStopActive` 状态下发的是 `kCtrl_EnableVoltage`
（[src/cia402_state_machine.cpp:125](../src/cia402_state_machine.cpp#L125)），
实际意图是"退出 QuickStop 回到 SwitchOnDisabled"，但用了 `EnableVoltage` 的名字，
维护者容易误解为两者可互换。建议删除 `kCtrl_QuickStop`，统一使用 `kCtrl_EnableVoltage`，
并在调用处加注释说明意图。

---

## B2 — `RecomputeAllOperational` 遍历全 6 轴导致系统永远不可运行

文件：[src/shared_state.cpp:14-24](../src/shared_state.cpp#L14)

```cpp
for (const auto& axis_feedback : feedback_) {  // 固定遍历 6 个槽
    if (!axis_feedback.is_operational || axis_feedback.is_fault) {
        all_ok = false;
        break;
    }
}
```

`feedback_` 固定 6 个槽，未配置的轴 `is_operational` 默认 `false`。
当 joints.yaml 只配置 1 轴时，剩余 5 个空槽会使 `all_operational` 永远为 `false`，
`CanopenRobotHw::WriteToSharedState()` 永远不会下发命令。

这是当前配置下系统完全无法运行的直接原因。

修复方向：`RecomputeAllOperational` 只遍历已配置的轴数，或在 `SharedState`
中记录实际轴数并以此为边界。

---

## B3 — 位置锁定可被应用线程命令绕过

文件：[src/axis_driver.cpp:162-166](../src/axis_driver.cpp#L162)

```cpp
// PublishSnapshot() 已将 safe_target 写入 commands[axis_index_]
PublishSnapshot();

// 紧接着再次 Snapshot 读回并发送
const SharedSnapshot snap = shared_state_->Snapshot();
(void)SendTargetPosition(snap.commands[axis_index_].target_position);
```

`PublishSnapshot()` 把状态机过滤后的 `safe_target` 写入 `commands[axis_index_]`，
但随即 `Snapshot()` 读回时，如果应用线程在这两步之间调用了 `WriteToSharedState()`，
读回的就是应用线程的原始命令而非 `safe_target`，位置锁定阶段因此失效。

修复方向：直接用 `state_machine_.safe_target()` 调用 `SendTargetPosition`，
不经过 `SharedState` 的二次读取。

---

## D1 — `SetAllOperational` 生产路径死代码

文件：[include/canopen_hw/shared_state.hpp:52](../include/canopen_hw/shared_state.hpp#L52)，
[src/shared_state.cpp:35](../src/shared_state.cpp#L35)

`SetAllOperational(bool)` 有声明和实现，但生产路径中只有 `RecomputeAllOperational()`
在更新 `all_operational_`，`SetAllOperational` 仅在测试文件中被调用，是生产路径死代码。

建议：若无计划在生产路径使用，改为 `private` 或删除，避免外部误用绕过
`RecomputeAllOperational` 的汇总逻辑。

---

## D2 — `SetRosTargetPosition` 悬空公开接口

文件：[include/canopen_hw/axis_driver.hpp:33](../include/canopen_hw/axis_driver.hpp#L33)

注释说"ROS 线程每周期调用"，但 `CanopenMaster` 和 `main.cpp` 均未调用它。
目标位置实际通过 `OnRpdoWrite` 内部从 `SharedState` 读取，该公开接口是悬空的。

建议：接入 ROS 前改为 `private` 或删除，避免外部绕过 `SharedState` 直接写入。

---

## D3 — `CreateAxisDrivers` 应为 private

文件：[include/canopen_hw/canopen_master.hpp:59](../include/canopen_hw/canopen_master.hpp#L59)

注释明确写"只能在初始化阶段调用，禁止在运行循环中调用"，但方法是 `public`。
`Start()` 内部已调用，外部再调用会导致驱动重复创建。

建议：改为 `private`。

---

## D4 — `bitrate` / `sync_period_us` 解析后被丢弃

文件：[src/joints_config.cpp:51,61](../src/joints_config.cpp#L51)

`LoadJointsYaml` 从 YAML 读取 `bitrate` 和 `sync_period_us` 存入 `MasterConfig`，
但 `CanopenMasterConfig` 没有这两个字段，`main.cpp` 也未将其传给 Lely。
CAN 波特率和 SYNC 周期实际由 DCF 文件决定，YAML 中的值没有任何效果。

建议：要么在 `CanopenMasterConfig` 补充这两个字段并传给 Lely，
要么从 YAML 解析中删除，避免用户误以为配置生效。

---

## R1 — 超时线程用裸 `detach`

文件：[src/pdo_mapping.cpp:212-218](../src/pdo_mapping.cpp#L212)

```cpp
std::thread([weak, timeout]() {
    std::this_thread::sleep_for(timeout);
    if (auto self = weak.lock()) {
        self->Finish(false, "PDO verify timeout");
    }
}).detach();
```

detach 的线程在进程退出时若仍在 `sleep_for`，行为未定义。

建议：改用 Lely 的 `io::Timer` 实现超时，与事件循环统一管理，避免裸线程。

---

## R2 — `boot_retry_count_` 无锁保护

文件：[src/axis_driver.cpp:215](../src/axis_driver.cpp#L215)

`boot_retry_count_` 在 `OnBoot` 回调中读写，未被 `mtx_` 覆盖。
`OnBoot` 通常只在 Lely 线程调用，但若 Lely 内部并发触发多次回调，计数会产生竞争。

建议：将 `boot_retry_count_` 纳入 `mtx_` 保护范围，或改为 `std::atomic<int>`。
