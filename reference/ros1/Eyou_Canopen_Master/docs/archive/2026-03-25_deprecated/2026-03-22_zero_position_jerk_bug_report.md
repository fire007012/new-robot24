# Bug 报告：上电或 Recover 后电机突然抽向位置 0

**项目：** Eyou_Canopen_Master（Lely Core + ROS）
**日期：** 2026-03-22
**模式：** CSP（Cyclic Synchronous Position）
**严重程度：** 高（存在安全隐患和电机损坏风险）

---

## 1. 问题现象

驱动器在 `/init` 或 `/recover` 完成、进入 `is_operational` 的瞬间，电机会突然向绝对编码器零位（0 ticks）运动，表现为：

- 轻微情况：电机嗡嗡声、小幅抽动（当前位置接近零位时）
- 严重情况：电机快速跑向零位（当前位置远离零位时），可能造成机械碰撞或驱动器过载

---

## 2. 根因分析

### 2.1 数据流链路

```
CanopenRobotHw 构造函数
  joint_cmd_(axis_count_, 0.0)           ← 初始化为 0.0 rad（0 ticks）

WriteToSharedState()（每帧）
  cmd.target_position = RadToTicks(joint_cmd_[i]) = 0

OnRpdoWrite → SetRosTarget(0)
  ros_target_ = 0                         ← 状态机目标始终为 0

StepOperationEnabled（position_locked_ = true 阶段）
  |ros_target_(=0) - actual_position| 大 → 重基准：ros_target_ = actual_position
  → 解锁：position_locked_ = false，safe_target_ = actual_position
  → is_operational_ = true（安全）

下一帧 WriteToSharedState
  cmd.target_position = RadToTicks(0.0) = 0   ← ROS 循环再次写入 0
  position_locked_ = false → safe_target_ = ros_target_ = 0
  → 驱动器收到目标位置 0 → 电机运动！
```

### 2.2 位置锁定机制为何没能拦住

`StepOperationEnabled` 中的重基准逻辑在 `position_locked_ = true` 阶段有效，但一旦解锁（`is_operational_ = true`），`safe_target_` 直接跟随 `ros_target_`，不再有任何滤波。下一帧 `WriteToSharedState` 将 `ros_target_` 重新写成 0，驱动器立刻收到错误目标。

### 2.3 ros_control 位置控制器无法弥补

`joint_position_controller` 在 `starting()` 中读取 `joint_.getPosition()` 作为初始指令。但 `starting()` 在驱动器就绪之前就已调用，此时反馈为 0，因此控制器的内部 setpoint = 0。此后每次 `update()` 都将 `joint_cmd_[i]` 覆写为 0，直到上层显式发送位置指令为止。

### 2.4 多轴场景的额外问题

原代码用全局 `all_operational_`（所有轴的与门）决定是否使用 `joint_cmd_`。若 4 轴中 3 轴就绪、1 轴未就绪，`all_operational_` 为 false，但 3 个已就绪轴没有任何保护，同样收到 0 目标。

---

## 3. 修复方案演进

### 3.1 第一版方案（已否决：`WriteToSharedState` 中条件赋值）

```cpp
// 错误方案：未就绪时将 joint_cmd_[i] 赋值为 joint_pos_[i]
if (!all_operational_) {
    joint_cmd_[i] = joint_pos_[i];
}
cmd.target_position = RadToTicks(i, joint_cmd_[i]);
```

**漏洞一：** 若驱动器上电时已处于 OperationEnabled，`all_operational_` 可能在第一次
`OnRpdoWrite` 时就变为 `true`，此时 ROS 循环尚未执行过一次 `read()`，`joint_pos_[i]`
仍为 0，`joint_cmd_[i]` 也为 0，问题依旧。

**漏洞二：** `update()` 在 `read()` 之后执行，控制器每帧用自身 setpoint（0）覆写
`joint_cmd_[i]`，任何在 `read()` 中做的同步都会被覆盖，保护只持续一帧。

### 3.2 第二版方案（已否决：`ReadFromSharedState` 上升沿同步）

```cpp
if (!axis_operational_[i] && now_operational) {
    joint_cmd_[i] = joint_pos_[i];  // 上升沿同步
}
```

**漏洞：** `read()` 在上升沿帧将 `joint_cmd_[i]` 同步到实际位置，但 `update()` 紧接着
用控制器的 0 覆写，`write()` 拿到的还是 0。上升沿同步对 `WriteToSharedState` 没有实际保护效果。

### 3.3 最终方案（已实现）

双管齐下，各自解决不同层面的问题：

**① `WriteToSharedState` 中逐轴保护（核心修复）**

```cpp
if (axis_operational_[i]) {
    cmd.target_position = RadToTicks(i, joint_cmd_[i]);  // 就绪：透传控制器指令
} else {
    cmd.target_position = RadToTicks(i, joint_pos_[i]);  // 未就绪：锁定到实际位置
}
```

关键点：
- 使用 per-axis `axis_operational_[i]` 而非全局 `all_operational_`，修复多轴场景漏洞
- 绕过控制器的 `joint_cmd_[i]` 覆写，直接以 `joint_pos_[i]` 作为目标
- 整个非就绪阶段（可持续数秒）都有效，不只保护一帧

**② `ReadFromSharedState` 中上升沿同步（辅助，为后续使用）**

```cpp
if (!axis_operational_[i] && now_operational) {
    joint_cmd_[i] = joint_pos_[i];
    if (snap.feedback[i].actual_position == 0) {
        CANOPEN_LOG_WARN("joint {}: operational rising edge but "
                         "actual_position=0, TPDO may not have arrived yet.", i);
    }
}
axis_operational_[i] = now_operational;
```

作用：确保 `joint_cmd_[i]` 在控制器接管的第一帧持有正确初始值；同时通过 WARN 日志
排除极少数 TPDO 未到达时 `actual_position` 仍为 0 的情况（该情况在正常时序下不会发生，
因为 `is_operational` 只在 `OnRpdoWrite` 中设置，而 `OnRpdoWrite` 必然在 TPDO 到达后触发）。

---

## 4. 已知遗留限制

上升沿帧（frame N）之后，若 `joint_position_controller` 仍持有 setpoint = 0，在
`axis_operational_[i]` 变为 `true` 后，`WriteToSharedState` 将开始透传 0 给驱动器。

**影响：** 操作员必须在 `/init` 成功后、发送运动指令前，显式发送一次当前位置作为目标，
或通过 controller_manager 重启控制器（触发 `starting()` 以正确位置初始化 setpoint）。

**根本原因：** `joint_position_controller` 的内部 setpoint 在硬件接口层不可见，无法从
`CanopenRobotHw` 侧强制修正。该问题属于 ros_control 集成层问题，超出硬件接口层的修复范围。

---

## 5. 修复覆盖的场景

| 场景 | 修复前 | 修复后 |
|------|--------|--------|
| 冷启动，驱动器从 SwitchOnDisabled 走完使能链路 | 解锁后立即跳向 0 | 非就绪阶段全程跟随实际位置，位置锁无跳变 |
| `/recover` 后驱动器重新使能 | 同上 | 同上 |
| 驱动器已在 OperationEnabled，`/init` 后立即就绪 | `all_operational_` 第一帧为 true，`joint_pos_` 尚未刷新，发送 0 | `axis_operational_[i]` 尚未置位，`WriteToSharedState` 使用 `joint_pos_[i]` |
| 4 轴中 1 轴未就绪 | `all_operational_=false` 但其余 3 轴无保护，收到 0 | per-axis 判断，已就绪轴独立保护 |

---

## 6. 修改文件

| 文件 | 修改内容 |
|------|----------|
| `include/canopen_hw/canopen_robot_hw.hpp` | 新增 `axis_operational_` 成员（`std::vector<bool>`） |
| `src/canopen_robot_hw.cpp` | 构造函数初始化 `axis_operational_`；`ReadFromSharedState` 增加上升沿同步与 WARN 日志；`WriteToSharedState` 改用 per-axis 保护，未就绪时使用 `joint_pos_[i]` |
