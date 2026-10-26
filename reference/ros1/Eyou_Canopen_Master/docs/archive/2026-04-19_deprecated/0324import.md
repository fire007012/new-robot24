# 状态管理架构重构方案

状态说明：

- 本文档保留为 2026-03-24 的架构设计草案，不作为当前实现的唯一验收基线。
- 当前已接受并回归验证的安全行为，以 `docs/2026-03-25_现行安全行为规范.md` 为准。
- 若本文档与现行实现语义冲突，应优先遵循现行安全行为规范，再决定是否继续推进架构收敛。

---

## 一、设计原则

| 编号 | 原则 | 现状违反点 |
|------|------|-----------|
| P1 | **单一权威状态源** | LifecycleManager 和 CiA402StateMachine 各持有独立状态 |
| P2 | **电平触发优于边沿闩锁** | `enable_requested_` 一旦置位永远生效，直到主动清除 |
| P3 | **上行报告，下行指令** | CiA402StateMachine 既做协议翻译又做策略决策 |
| P4 | **状态转换的原子性** | Recover 的多步操作在 CAN 线程可见窗口内不是原子的 |
| P5 | **关注点分离** | "CAN 通信生命周期"和"电机操作意图"混在 LifecycleManager 中 |

---

## 二、新状态模型

### 2.1 系统操作模式（SystemOpMode）

取代当前散落在 LifecycleManager（`halted_`、`require_init_`、`state_`）和 CiA402StateMachine（`enable_requested_`、`halt_requested_`、`global_fault_`）中的多个 bool 组合。

```
                    ┌──────────────────────────────────────────────┐
                    │              SystemOpMode                    │
                    │         （全局唯一权威状态）                    │
                    └──────────────────────────────────────────────┘

                              ┌──────────┐
                              │ Inactive │  CAN 栈未启动
                              └────┬─────┘
                                   │ Configure()
                              ┌────▼─────┐
                              │Configured│  CAN 栈就绪，电机未初始化
                              └────┬─────┘
                                   │ InitMotors()
                              ┌────▼─────┐
                              │ Standby  │  电机在 SwitchOnDisabled，等待使能
                              └────┬─────┘
                                   │ Enable() 
                              ┌────▼─────┐
                              │  Armed   │  OperationEnabled + Halt，冻结状态
                              └────┬─────┘
                                   │ Release()
                              ┌────▼─────┐
                              │ Running  │  OperationEnabled，命令流通
                              └────┬─────┘
                                   │ Halt()
                                   └──────────► Armed
                                   
       任意已使能状态 ──故障──► Faulted ──Recover()──► Armed
       任意状态 ──Shutdown()──► Configured ──(需重新Init)──► Standby
```

```cpp
enum class SystemOpMode : uint8_t {
  Inactive,       // CAN 栈未启动
  Configured,     // CAN 栈就绪，电机未初始化（对应旧 require_init_ = true）
  Standby,        // 电机已初始化但未使能（SwitchOnDisabled/ReadyToSwitchOn）
  Armed,          // OperationEnabled + Halt（对应旧 halted_ = true）
  Running,        // OperationEnabled，命令流通
  Faulted,        // 一个或多个轴故障
  Recovering,     // 故障恢复进行中（瞬态）
  ShuttingDown,   // 关停进行中（瞬态）
};
```

**关键约束**：`SystemOpMode` 存储在一个 `std::atomic<SystemOpMode>` 中，由 `OperationalCoordinator`（新类）独占写入，所有其他组件只读。

### 2.2 与旧状态的映射

| 旧状态组合 | 新 SystemOpMode |
|------------|-----------------|
| `state_ = Configured, require_init_ = true` | `Configured` |
| `state_ = Active, halted_ = false, 未全部 OperationEnabled` | `Standby` |
| `state_ = Active, halted_ = true` | `Armed` |
| `state_ = Active, halted_ = false, 全部 OperationEnabled` | `Running` |
| 任何轴 `is_fault = true` | `Faulted` |

### 2.3 每轴指令意图（AxisIntent）

取代当前的 `enable_requested_` + `halt_requested_` 闩锁组合。

```cpp
// 由 OperationalCoordinator 每帧计算并下发，电平语义（非闩锁）。
// 如果 Coordinator 不再下发 Enable，状态机自动回落到安全状态。
enum class AxisIntent : uint8_t {
  Disable,    // 要求轴在 SwitchOnDisabled
  Enable,     // 要求轴推进到 OperationEnabled
  Halt,       // 要求轴在 OperationEnabled + Halt（冻结）
  Run,        // 要求轴在 OperationEnabled，透传命令
};
```

---

## 三、新类架构

### 3.1 职责重新划分

```
旧架构：
  LifecycleManager ──► CanopenMaster ──► AxisDriver ──► CiA402StateMachine
  (生命周期+操作意图)   (总线管理+轴管理)  (Lely绑定+状态机) (402协议+策略决策)

新架构：
  ServiceGateway ──► OperationalCoordinator ──► CanopenMaster ──► AxisDriver ──► CiA402Protocol
  (ROS service)      (操作意图+策略决策)         (总线管理)        (Lely绑定)     (纯402协议)
```

| 类 | 职责 | 线程 |
|----|------|------|
| `ServiceGateway` | ROS service 的薄包装，验证参数后转发给 Coordinator | ROS callback |
| `OperationalCoordinator` | **唯一策略决策者**：管理 SystemOpMode，决定每轴的 AxisIntent，处理故障策略 | 可被任意线程调用（内部加锁） |
| `CanopenMaster` | CAN 总线生命周期（启停 Lely 栈），轴驱动创建/销毁 | 多线程 |
| `AxisDriver` | Lely BasicDriver 子类，RPDO/TPDO 读写 | CAN event loop |
| `CiA402Protocol` | **纯协议翻译器**：(AxisIntent, statusword) → controlword，无策略逻辑 | CAN event loop |

### 3.2 OperationalCoordinator（核心新类）

```cpp
class OperationalCoordinator {
 public:
  explicit OperationalCoordinator(SharedState* shared_state, std::size_t axis_count);

  // ========== 状态查询（任意线程可调用） ==========
  SystemOpMode mode() const { return mode_.load(std::memory_order_acquire); }

  // ========== 状态转换请求（ROS service 线程调用） ==========
  // 所有方法返回 (success, message)，内部保证原子性。
  struct Result { bool ok; std::string message; };

  Result RequestInit();        // Configured → Standby
  Result RequestEnable();      // Standby → Armed（首次使能进入冻结态）
  Result RequestRelease();     // Armed → Running
  Result RequestHalt();        // Running → Armed
  Result RequestRecover();     // Faulted → Armed
  Result RequestShutdown();    // Any → Configured

  // ========== 每帧调用（CAN 线程或主循环线程） ==========
  // 根据 mode_ 和轴反馈，计算每轴的 AxisIntent 并写入 SharedState。
  void ComputeIntents();

  // 根据轴反馈更新 mode_（如检测到故障自动转 Faulted）。
  void UpdateFromFeedback();

 private:
  std::atomic<SystemOpMode> mode_{SystemOpMode::Inactive};
  mutable std::mutex transition_mtx_;  // 保护状态转换的原子性

  SharedState* shared_state_;
  std::size_t axis_count_;

  // 内部：在 transition_mtx_ 保护下执行转换
  Result DoTransition(SystemOpMode from, SystemOpMode to,
                      std::function<bool()> action);
};
```

### 3.3 CiA402Protocol（取代 CiA402StateMachine）

从当前的 CiA402StateMachine 中剥离所有策略逻辑，只保留 402 协议状态解码和控制字生成。

```cpp
class CiA402Protocol {
 public:
  // 纯函数式接口：输入当前状态和意图，输出控制字和安全目标。
  struct Input {
    uint16_t statusword;
    int8_t mode_display;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
    AxisIntent intent;          // 来自 Coordinator
    int8_t target_mode;
    int32_t ros_target_position;
    int32_t ros_target_velocity;
    int16_t ros_target_torque;
    bool cmd_valid;             // 来自 ROS 层的命令有效性
  };

  struct Output {
    uint16_t controlword;
    int32_t safe_target_position;
    int32_t safe_target_velocity;
    int16_t safe_target_torque;
    int8_t safe_mode;
    CiA402State decoded_state;
    bool is_fault;
    bool is_operational;        // 处于 OperationEnabled 且非 halt
    bool arm_epoch_advanced;    // 本帧发生了 epoch 推进
    uint32_t arm_epoch;
  };

  Output Process(const Input& input);

 private:
  CiA402State state_ = CiA402State::NotReadyToSwitchOn;
  bool was_operation_enabled_ = false;
  uint32_t arm_epoch_ = 0;
  int32_t position_lock_target_ = 0;
  bool position_locked_ = true;
  int32_t position_lock_threshold_ = 100;

  // 策略参数
  int32_t max_delta_per_cycle_ = 0;  // 速率限幅
};
```

关键变化：

- **没有 `enable_requested_`**：由 `AxisIntent` 每帧提供，不再是闩锁。
- **没有 `halt_requested_`**：`AxisIntent::Halt` 就是 halt。
- **没有 `global_fault_`**：Coordinator 在检测到任何轴故障时直接将所有轴的 intent 设为 `Halt` 或 `Disable`。
- **没有 `forced_halt_by_fault_`**：Coordinator 负责在故障清除后决定回到 `Armed` 而非 `Running`。
- **没有 fault reset 逻辑**：故障复位由 Coordinator 通过 SDO 发送 controlword 0x0080 实现，不在 Protocol 层。

### 3.4 AxisIntent 的电平语义实现

```cpp
void OperationalCoordinator::ComputeIntents() {
  const SystemOpMode current_mode = mode_.load(std::memory_order_acquire);
  
  for (std::size_t i = 0; i < axis_count_; ++i) {
    AxisIntent intent;
    
    switch (current_mode) {
      case SystemOpMode::Inactive:
      case SystemOpMode::Configured:
      case SystemOpMode::ShuttingDown:
        intent = AxisIntent::Disable;
        break;

      case SystemOpMode::Standby:
        intent = AxisIntent::Disable;  // 等待 Enable 请求
        break;

      case SystemOpMode::Armed:
      case SystemOpMode::Recovering:
        intent = AxisIntent::Halt;     // 使能但冻结
        break;

      case SystemOpMode::Running:
        intent = AxisIntent::Run;      // 使能且透传命令
        break;

      case SystemOpMode::Faulted:
        intent = AxisIntent::Halt;     // 故障时冻结（而非去使能，保持抱闸）
        break;
    }

    shared_state_->SetAxisIntent(i, intent);
  }
}
```

**安全保证**：如果 Coordinator 崩溃或停止调用 `ComputeIntents()`，SharedState 中的 intent 不会被更新。CiA402Protocol 应检测 intent 的"新鲜度"（时间戳或序列号），超时后自动回落到 `AxisIntent::Disable`。

```cpp
// SharedState 中的 intent 带时间戳
struct StampedIntent {
  AxisIntent intent{AxisIntent::Disable};
  uint64_t sequence{0};  // 每次 ComputeIntents() 递增
};

// CiA402Protocol 中的超时检查
Output CiA402Protocol::Process(const Input& input) {
  AxisIntent effective_intent = input.intent;
  
  // 防御：如果 intent 序列号长时间不变（Coordinator 可能已死），
  // 回落到 Disable。
  if (intent_sequence_unchanged_count_ > kMaxStaleFrames) {
    effective_intent = AxisIntent::Disable;
  }
  // ...
}
```

---

## 四、CiA402Protocol 核心逻辑

### 4.1 控制字生成（纯协议翻译）

```cpp
CiA402Protocol::Output CiA402Protocol::Process(const Input& input) {
  Output out{};
  const CiA402State prev_state = state_;
  state_ = DecodeState(input.statusword);
  out.decoded_state = state_;
  out.is_fault = (state_ == CiA402State::Fault ||
                  state_ == CiA402State::FaultReactionActive);
  out.arm_epoch = arm_epoch_;
  out.arm_epoch_advanced = false;

  // AxisIntent 直接映射到使能链决策——不再依赖闩锁。
  const bool want_enable = (input.intent == AxisIntent::Enable ||
                            input.intent == AxisIntent::Halt ||
                            input.intent == AxisIntent::Run);
  const bool want_halt = (input.intent == AxisIntent::Halt);
  const bool want_run = (input.intent == AxisIntent::Run);

  switch (state_) {
    case CiA402State::NotReadyToSwitchOn:
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;

    case CiA402State::SwitchOnDisabled:
      out.controlword = want_enable ? kCtrl_Shutdown : kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;

    case CiA402State::ReadyToSwitchOn:
      out.controlword = want_enable ? kCtrl_EnableOperation : kCtrl_Shutdown;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;

    case CiA402State::SwitchedOn:
      out.controlword = want_enable ? kCtrl_EnableOperation : kCtrl_Shutdown;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;

    case CiA402State::OperationEnabled:
      ProcessOperationEnabled(input, want_enable, want_halt, want_run, &out);
      break;

    case CiA402State::Fault:
    case CiA402State::FaultReactionActive:
      // 协议层不做故障复位——由 Coordinator 通过独立路径发送 0x0080。
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;

    case CiA402State::QuickStopActive:
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      break;
  }

  out.safe_mode = input.target_mode;
  was_operation_enabled_ = (state_ == CiA402State::OperationEnabled);
  return out;
}

void CiA402Protocol::ProcessOperationEnabled(
    const Input& input, bool want_enable, bool want_halt, bool want_run,
    Output* out) {
  
  // 首次进入 OperationEnabled
  if (!was_operation_enabled_) {
    AdvanceArmEpoch();
    out->arm_epoch_advanced = true;
    out->arm_epoch = arm_epoch_;
    position_locked_ = true;
    LockPosition(input.actual_position, out);
    out->controlword = kCtrl_EnableOperation;
    ApplyHaltBit(input.target_mode, out);
    out->is_operational = false;  // 首帧不算 operational
    return;
  }

  // 不再想使能 → 退出
  if (!want_enable) {
    out->controlword = kCtrl_DisableOperation;
    out->is_operational = false;
    LockPosition(input.actual_position, out);
    return;
  }

  // Halt 冻结模式
  if (want_halt) {
    out->controlword = kCtrl_EnableOperation;
    ApplyHaltBit(input.target_mode, out);
    out->controlword |= kCtrl_Bit_Halt;
    out->is_operational = false;  // halt 时不算 operational
    LockPosition(input.actual_position, out);

    // halt→run 转换沿检测
    prev_was_halt_ = true;
    return;
  }

  // Run 模式
  if (want_run) {
    // halt→run 转换沿：重新推进 epoch
    if (prev_was_halt_) {
      AdvanceArmEpoch();
      out->arm_epoch_advanced = true;
      out->arm_epoch = arm_epoch_;
      position_locked_ = true;
      LockPosition(input.actual_position, out);
      prev_was_halt_ = false;
      // 本帧仍不算 operational，等 position lock 解除
      out->controlword = kCtrl_EnableOperation;
      ApplyHaltBit(input.target_mode, out);
      out->is_operational = false;
      return;
    }

    out->controlword = kCtrl_EnableOperation;
    ApplyHaltBit(input.target_mode, out);

    // Position lock 逻辑
    StepPositionLock(input, out);
    return;
  }
}
```

### 4.2 位置锁与命令透传

```cpp
void CiA402Protocol::StepPositionLock(const Input& input, Output* out) {
  if (position_locked_) {
    // 检查 ROS 命令是否已收敛到实际位置附近
    if (input.cmd_valid &&
        AbsDiff(input.ros_target_position, input.actual_position)
            <= position_lock_threshold_) {
      position_locked_ = false;
    }
  }

  if (position_locked_) {
    out->safe_target_position = input.actual_position;
    out->safe_target_velocity = 0;
    out->safe_target_torque = 0;
    out->is_operational = false;
  } else {
    // 速率限幅后透传
    int32_t clamped_pos = input.ros_target_position;
    if (max_delta_per_cycle_ > 0) {
      const int64_t delta = static_cast<int64_t>(clamped_pos) -
                            static_cast<int64_t>(input.actual_position);
      if (AbsDiff(clamped_pos, input.actual_position) > max_delta_per_cycle_) {
        clamped_pos = input.actual_position +
                      static_cast<int32_t>(
                          std::clamp(delta,
                                    static_cast<int64_t>(-max_delta_per_cycle_),
                                    static_cast<int64_t>(max_delta_per_cycle_)));
      }
    }
    out->safe_target_position = clamped_pos;
    out->safe_target_velocity = input.ros_target_velocity;
    out->safe_target_torque = input.ros_target_torque;
    out->is_operational = true;
  }
}

void CiA402Protocol::LockPosition(int32_t actual_position, Output* out) {
  position_locked_ = true;
  out->safe_target_position = actual_position;
  out->safe_target_velocity = 0;
  out->safe_target_torque = 0;
}
```

---

## 五、OperationalCoordinator 状态转换实现

### 5.1 原子转换框架

```cpp
OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowed_from,
    SystemOpMode to,
    std::function<bool(std::string*)> action) {
  
  std::lock_guard<std::mutex> lk(transition_mtx_);
  
  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  
  bool from_ok = false;
  for (auto allowed : allowed_from) {
    if (current == allowed) { from_ok = true; break; }
  }
  if (!from_ok) {
    return {false, "invalid transition from " + ModeName(current)};
  }

  std::string detail;
  if (action && !action(&detail)) {
    return {false, detail.empty() ? "action failed" : detail};
  }

  mode_.store(to, std::memory_order_release);
  return {true, detail.empty() ? ("→ " + ModeName(to)) : detail};
}
```

### 5.2 各转换实现

```cpp
Result OperationalCoordinator::RequestInit() {
  return DoTransition(
      {SystemOpMode::Configured},
      SystemOpMode::Standby,
      [this](std::string* detail) {
        // 启动 CAN 栈，boot 所有节点，等待 SwitchOnDisabled
        return master_->Start(detail);
      });
}

Result OperationalCoordinator::RequestEnable() {
  return DoTransition(
      {SystemOpMode::Standby},
      SystemOpMode::Armed,  // 首次使能进入 Armed（冻结），不直接 Running
      [this](std::string* detail) {
        // AxisIntent 通过下一帧 ComputeIntents() 自动变为 Halt
        // 等待所有轴到达 OperationEnabled
        return WaitAllOperationEnabled(detail, std::chrono::seconds(5));
      });
}

Result OperationalCoordinator::RequestRelease() {
  return DoTransition(
      {SystemOpMode::Armed},
      SystemOpMode::Running,
      nullptr);  // 无额外动作，ComputeIntents 下帧自动输出 Run
}

Result OperationalCoordinator::RequestHalt() {
  return DoTransition(
      {SystemOpMode::Running},
      SystemOpMode::Armed,
      nullptr);  // ComputeIntents 下帧自动输出 Halt
}

Result OperationalCoordinator::RequestRecover() {
  // 关键：Faulted → Recovering → Armed
  // 全程在 transition_mtx_ 保护下执行，CAN 线程看到的 intent 始终一致。
  return DoTransition(
      {SystemOpMode::Faulted},
      SystemOpMode::Armed,  // 最终目标
      [this](std::string* detail) {
        // 1. 先设为 Recovering，ComputeIntents 输出 Halt
        mode_.store(SystemOpMode::Recovering, std::memory_order_release);
        
        // 2. 对所有故障轴发送 fault reset（SDO 写 0x0080）
        if (!master_->ResetAllFaults(detail)) {
          return false;
        }
        
        // 3. 等待所有轴离开 Fault 状态
        if (!WaitAllNotFault(detail, std::chrono::seconds(3))) {
          return false;
        }
        
        // 4. 清除全局故障标志
        shared_state_->ClearGlobalFault();
        
        // 5. 等待所有轴到达 OperationEnabled
        //   （ComputeIntents 已在 Recovering 时输出 Halt，
        //    状态机会自动推进使能链并在 OperationEnabled+Halt 停住）
        if (!WaitAllOperationEnabled(detail, std::chrono::seconds(5))) {
          return false;
        }
        
        // 转换到 Armed（由 DoTransition 最终设置）
        return true;
      });
}

Result OperationalCoordinator::RequestShutdown() {
  return DoTransition(
      {SystemOpMode::Standby, SystemOpMode::Armed, SystemOpMode::Running,
       SystemOpMode::Faulted},
      SystemOpMode::Configured,
      [this](std::string* detail) {
        mode_.store(SystemOpMode::ShuttingDown, std::memory_order_release);
        // ComputeIntents 在 ShuttingDown 时输出 Disable
        // 等待所有轴去使能后停止 CAN 栈
        master_->GracefulShutdown(detail);
        master_->Stop();
        return true;
      });
}
```

### 5.3 故障自动检测

```cpp
void OperationalCoordinator::UpdateFromFeedback() {
  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  
  // 只在运行态检测故障（Recovering/ShuttingDown 中不自动转换）
  if (current != SystemOpMode::Armed &&
      current != SystemOpMode::Running) {
    return;
  }
  
  const SharedSnapshot snap = shared_state_->Snapshot();
  bool any_fault = false;
  for (std::size_t i = 0; i < axis_count_; ++i) {
    if (snap.feedback[i].is_fault || snap.feedback[i].heartbeat_lost) {
      any_fault = true;
      break;
    }
  }
  
  if (any_fault) {
    // 原子转换到 Faulted
    // 不需要 transition_mtx_——这是唯一的"自动下降"转换，
    // 且 CAS 保证不会与手动转换冲突。
    SystemOpMode expected = current;
    mode_.compare_exchange_strong(expected, SystemOpMode::Faulted,
                                  std::memory_order_acq_rel);
  }
}
```

---

## 六、数据流重新设计

### 6.1 新 SharedState 扩展

```cpp
class SharedState {
 public:
  // ========== 已有（保留） ==========
  void UpdateFeedback(std::size_t axis, const AxisFeedback& fb);
  void UpdateCommand(std::size_t axis, const AxisCommand& cmd);
  SharedSnapshot Snapshot() const;

  // ========== 新增 ==========
  // Coordinator → CiA402Protocol：每轴操作意图
  void SetAxisIntent(std::size_t axis, AxisIntent intent);
  AxisIntent GetAxisIntent(std::size_t axis) const;

  // Coordinator → 全局：intent 序列号（用于超时检测）
  void AdvanceIntentSequence();
  uint64_t intent_sequence() const;

  // Protocol → Coordinator：轴报告（替代旧的 is_operational/is_fault 散落存储）
  struct AxisReport {
    CiA402State state;
    bool is_fault;
    bool is_operational;
    bool heartbeat_lost;
    uint32_t arm_epoch;
  };
  void UpdateAxisReport(std::size_t axis, const AxisReport& report);

 private:
  // 全部使用 per-axis seqlock 替代大锁
  struct alignas(64) PerAxisData {
    SeqLock lock;
    AxisFeedback feedback;
    AxisCommand command;
    AxisIntent intent{AxisIntent::Disable};
    AxisReport report;
  };
  std::array<PerAxisData, kMaxAxisCount> axes_;
  std::atomic<uint64_t> intent_sequence_{0};
};
```

### 6.2 帧内数据流（CAN 线程）

```
每个 RPDO 周期：
  ┌─────────────────────────────────────────────────────────────┐
  │ 1. RPDO 到达 → AxisDriver::OnRpdoWrite()                   │
  │    ├─ 读取 statusword, actual_position, mode_display, ...  │
  │    ├─ 从 SharedState 读取 AxisIntent                        │
  │    ├─ 从 SharedState 读取 AxisCommand（ROS 目标）            │
  │    └─ 调用 CiA402Protocol::Process(input) → output         │
  │                                                             │
  │ 2. 写出 TPDO                                                │
  │    ├─ controlword = output.controlword                      │
  │    ├─ target_position = output.safe_target_position         │
  │    ├─ mode_of_operation = output.safe_mode                  │
  │    └─ target_velocity, target_torque                        │
  │                                                             │
  │ 3. 更新 SharedState                                         │
  │    ├─ UpdateFeedback(axis, {actual_pos, vel, torque, ...})  │
  │    └─ UpdateAxisReport(axis, {state, is_fault, arm_epoch})  │
  └─────────────────────────────────────────────────────────────┘
```

### 6.3 帧内数据流（ROS 主循环线程）

```
每个主循环帧 (100Hz)：
  ┌─────────────────────────────────────────────────────────────┐
  │ 1. Coordinator.UpdateFromFeedback()                         │
  │    └─ 检查 SharedState 中的 AxisReport → 自动故障检测        │
  │                                                             │
  │ 2. Coordinator.ComputeIntents()                             │
  │    └─ 根据 SystemOpMode 计算每轴 AxisIntent → SharedState   │
  │                                                             │
  │ 3. robot_hw_ros.read()                                      │
  │    ├─ 从 SharedState 读 feedback → 更新 pos_/vel_/eff_      │
  │    ├─ 检测 arm_epoch 变化 → 重同步 pos_cmd_                  │
  │    └─ 管理 cmd_ready_guard_                                 │
  │                                                             │
  │ 4. cm.update()                                              │
  │    └─ ROS 控制器运行，修改 pos_cmd_/vel_cmd_                 │
  │                                                             │
  │ 5. robot_hw_ros.write()                                     │
  │    ├─ 如果 cmd_ready_: 透传 pos_cmd_ → SharedState.Command  │
  │    └─ 否则: 锁定 pos_ → SharedState.Command                 │
  └─────────────────────────────────────────────────────────────┘
```

---

## 七、ServiceGateway（取代旧 canopen_hw_ros_node.cpp 中的 lambda）

```cpp
class ServiceGateway {
 public:
  ServiceGateway(ros::NodeHandle& pnh, OperationalCoordinator* coordinator);

 private:
  OperationalCoordinator* coord_;

  // 所有 service handler 只做参数校验和转发，不持有任何状态。
  bool OnInit(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto r = coord_->RequestInit();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }

  bool OnEnable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto r = coord_->RequestEnable();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }

  bool OnRelease(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    // 旧 resume 改名为 release，语义更清晰
    auto r = coord_->RequestRelease();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }

  bool OnHalt(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto r = coord_->RequestHalt();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }

  bool OnRecover(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto r = coord_->RequestRecover();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }

  bool OnShutdown(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    auto r = coord_->RequestShutdown();
    res.success = r.ok;
    res.message = r.message;
    return true;
  }
};
```

**注意**：ServiceGateway 完全无状态。不再有 `halted_`、`require_init_` 等影子变量。用户可以安全地从任何线程调用 Coordinator。

---

## 八、消除的故障模式

| 旧 Bug | 旧根因 | 新架构如何消除 |
|--------|--------|----------------|
| Recover 绕开 Resume 直接使能 | `enable_requested_` 闩锁未被 Recover 清除 | 不再有闩锁。Recover 终态是 `Armed`，ComputeIntents 输出 `Halt`。必须 RequestRelease 才能变为 `Running` |
| Resume 时 pos_cmd 不重同步 | halt→run 不触发 AdvanceArmEpoch | CiA402Protocol 在 `prev_was_halt_ → Run` 转换沿显式推进 epoch |
| Init 后 ROS 控制器覆盖重同步值 | read 中重同步后 cm.update 立即覆盖 | write 中 guard 期间强制输出实际位置，不透传控制器输出 |
| 心跳丢失后重连状态不一致 | LifecycleManager 不感知底层事件 | UpdateFromFeedback 自动检测 heartbeat_lost 并转 Faulted；重连后需用户手动 Recover → Release |
| 状态转换非原子 | Recover 的多步操作之间 CAN 线程可见中间状态 | DoTransition 全程在 transition_mtx_ 保护下执行 |
| 新增路径忘记设 halt | 需要在每个新路径中记得调 HaltAll | AxisIntent 是电平语义——Coordinator 不输出 Run 就不可能 Run |

---

## 九、安全性证明（AxisIntent 电平语义的不变量）

**定理**：在新架构下，电机只有在 `SystemOpMode == Running` 时才可能接收到非冻结的目标位置。

**证明**：

1. 只有 `ComputeIntents()` 向 SharedState 写入 AxisIntent。
2. `ComputeIntents()` 的 switch 语句中，只有 `case Running` 输出 `AxisIntent::Run`。
3. `CiA402Protocol::Process()` 中，只有 `AxisIntent::Run` 且 `position_locked_ == false` 时才透传 ROS 目标。
4. `SystemOpMode` 只能通过 `DoTransition()`（transition_mtx_ 保护）或 `UpdateFromFeedback()`（CAS 只允许降级到 Faulted）修改。
5. `RequestRelease()` 是到达 `Running` 的唯一入口，且只允许从 `Armed` 转换。
6. `Armed` 只能从 `Standby`（首次使能）或 `Faulted`（Recover）到达。

因此：用户必须显式调用 `Init → Enable → Release` 才能让电机运行。任何故障、Halt、Shutdown 都会退出 `Running`，且重新进入 `Running` 必须再次通过 `Release`。█

---

## 十、迁移策略

### 阶段 1：引入 OperationalCoordinator（不改 CiA402StateMachine）

- 新建 `OperationalCoordinator` 类。
- 在 LifecycleManager 中实例化 Coordinator，将现有 service 处理逻辑转发到 Coordinator。
- Coordinator 内部仍调用旧的 `CanopenMaster::EnableAll()`/`HaltAll()` 等方法。
- 删除 LifecycleManager 中的 `halted_`、`require_init_` 等成员，改为查询 `Coordinator.mode()`。

**验证点**：所有现有 service 调用序列的行为不变。

### 阶段 2：SharedState 添加 AxisIntent

- 在 SharedState 中添加 per-axis AxisIntent 字段。
- OperationalCoordinator 实现 `ComputeIntents()`，在主循环中调用。
- CiA402StateMachine 中添加 intent 读取，但暂时同时保留旧的 `enable_requested_`/`halt_requested_`（双写验证）。
- 添加断言：intent 和旧闩锁的决策结果必须一致。不一致时 WARN 但以 intent 为准。

**验证点**：运行 1000 次 init/halt/resume/recover/shutdown 随机序列，无断言失败。

### 阶段 3：剥离 CiA402Protocol

- 将 CiA402StateMachine 重构为 CiA402Protocol（纯协议翻译器）。
- 删除 `enable_requested_`、`halt_requested_`、`global_fault_`、`forced_halt_by_fault_`。
- 删除 fault reset 状态机（移到 Coordinator）。
- 所有策略决策由 Coordinator 通过 AxisIntent 驱动。

**验证点**：CiA402Protocol 可以独立于 Coordinator 进行单元测试——给定 (statusword, intent) 输入，验证 controlword 输出。

### 阶段 4：SharedState 无锁化

- 将 SharedState 的大锁改为 per-axis SeqLock。
- 验证 CAN 线程和 ROS 线程的延迟特性。

**验证点**：CAN 线程 RPDO 处理延迟的 99.9 分位不超过 50μs。

---

## 十一、测试矩阵

```
┌────────────────────────────────────┬───────────────────────────────────┐
│ 测试场景                           │ 预期 SystemOpMode 转换            │
├────────────────────────────────────┼───────────────────────────────────┤
│ 正常启动                           │ Inactive→Configured→Standby→     │
│                                    │ Armed→Running                    │
│ 正常停止                           │ Running→Armed→ShuttingDown→      │
│                                    │ Configured                       │
│ 运行中单轴故障                      │ Running→Faulted                  │
│ 故障恢复                           │ Faulted→Recovering→Armed         │
│ 恢复后继续运行                      │ Armed→Running                    │
│ Halt后Resume                      │ Running→Armed→Running            │
│ 故障恢复后直接Release              │ Armed→Running (合法)              │
│ 故障中调Release                    │ 拒绝 (Faulted≠Armed)            │
│ 未Init调Enable                    │ 拒绝 (Configured≠Standby)        │
│ 心跳丢失                           │ Armed/Running→Faulted            │
│ 心跳恢复后自动重连                  │ 停留在 Faulted (需手动Recover)    │
│ Recover失败（故障未清除）           │ 停留在 Faulted                   │
│ Coordinator 线程死亡               │ intent 序列号超时→自动 Disable    │
│ 快速连续 Halt/Release              │ 最终状态取决于最后一次调用         │
│ 并发 Recover + Release             │ transition_mtx_ 串行化           │
└────────────────────────────────────┴───────────────────────────────────┘
```

## 十二、文件级改动清单与代码量估算

### 12.1 新增文件

| 文件 | 职责 | 预估行数 | 复杂度 |
|------|------|----------|--------|
| `operational_coordinator.hpp` | SystemOpMode 枚举、OperationalCoordinator 声明 | ~120 | 中 |
| `operational_coordinator.cpp` | 状态转换逻辑、ComputeIntents、UpdateFromFeedback | ~350 | **高** |
| `cia402_protocol.hpp` | CiA402Protocol 声明（Input/Output 结构体） | ~100 | 低 |
| `cia402_protocol.cpp` | 纯协议翻译 Process()、位置锁、速率限幅 | ~280 | 中 |
| `service_gateway.hpp` | ServiceGateway 声明 | ~50 | 低 |
| `service_gateway.cpp` | ROS service 注册与转发 | ~130 | 低 |
| `seqlock.hpp` | Per-axis SeqLock 实现（阶段4） | ~80 | 中 |
| `test/operational_coordinator_test.cpp` | Coordinator 状态转换单元测试 | ~400 | 中 |
| `test/cia402_protocol_test.cpp` | 协议翻译器单元测试 | ~500 | 中 |
| `test/integration_scenario_test.cpp` | 完整流程集成测试 | ~300 | 高 |

**新增合计：~2310 行**

---

### 12.2 重度修改文件

| 文件 | 改动内容 | 当前行数 | 预估改动行数 | 净增减 |
|------|----------|----------|--------------|--------|
| `lifecycle_manager.hpp/cpp` | 剥离 `halted_`/`require_init_`/`state_`，内部转为持有 `OperationalCoordinator*`。所有 service 逻辑移到 ServiceGateway。最终退化为薄壳或删除 | ~300 | -200 / +30 | **-170** |
| `cia402_state_machine.hpp/cpp` | 阶段2：添加 intent 读取并双写验证。阶段3：重命名为 `cia402_protocol`，删除全部策略字段（`enable_requested_`、`halt_requested_`、`global_fault_`、`forced_halt_by_fault_`、fault reset FSM） | ~600 | -350 / +180 | **-170** |
| `axis_logic.hpp/cpp` | 删除 `RequestEnable()`/`RequestDisable()`/`RequestHalt()`/`RequestResume()` 等闩锁操作。ProcessRpdo 改为从 SharedState 读取 AxisIntent 传入 CiA402Protocol | ~230 | -80 / +40 | **-40** |
| `canopen_master.hpp/cpp` | 删除 `EnableAll()`/`HaltAll()`/`ResumeAll()`/`RecoverFaultedAxes()` 等策略方法。保留 `Start()`/`Stop()`/`GracefulShutdown()` 等纯通信管理。新增 `ResetAllFaults()` SDO 方法 | ~490 | -120 / +30 | **-90** |
| `axis_driver.hpp/cpp` | 删除 `RequestEnable()`/`RequestDisable()`/`RequestHalt()`/`RequestResume()`/`ResetFault()` 代理方法。OnRpdoWrite 改为读取 AxisIntent 并调用 CiA402Protocol::Process | ~625 | -60 / +30 | **-30** |
| `canopen_robot_hw_ros.hpp/cpp` | 小改：guard 期间 write 覆盖为实际位置（已在修复3中完成），epoch 检测逻辑不变 | ~120 | -5 / +15 | **+10** |
| `canopen_hw_ros_node.cpp` | 删除所有 service lambda，替换为 ServiceGateway 实例化。主循环添加 `Coordinator.UpdateFromFeedback()` 和 `ComputeIntents()` 调用 | ~295 | -100 / +30 | **-70** |
| `shared_state.hpp/cpp` | 添加 AxisIntent、AxisReport、intent_sequence。阶段4：大锁→per-axis SeqLock | ~250 | -40 / +120 | **+80** |

**现有文件改动合计：~-480 行（净删减）**

---

### 12.3 总量汇总

| 类别 | 行数 |
|------|------|
| 新增文件（含测试） | +2310 |
| 现有文件净变化 | -480 |
| **总净增** | **+1830** |
| 涉及改动的总行数（增+删） | ~2810 |
| 需要 review 的核心逻辑行数 | ~730（Coordinator + Protocol） |

---

## 十三、各阶段工作量与风险

### 阶段 1：引入 OperationalCoordinator（不改状态机）

| 维度 | 评估 |
|------|------|
| **改动量** | 新增 ~470 行（Coordinator + ServiceGateway），修改 ~300 行（LifecycleManager 退化、node.cpp service 迁移） |
| **工期** | 2-3 天 |
| **风险** | **低**。行为完全等价——Coordinator 内部仍调用旧的 `EnableAll()`/`HaltAll()`，只是把分散在 LifecycleManager 中的 if-else 整合到 Coordinator 的 DoTransition 中 |
| **回退代价** | 极低。Coordinator 是纯新增类，回退只需还原 node.cpp 和 LifecycleManager |
| **验证方法** | 对比现有 service 调用序列的日志输出，逐一确认行为一致 |

**阶段1完成后的收益**：

- 所有 `halted_`/`require_init_` 消除，统一查询 `Coordinator.mode()`
- Recover 绕过 Resume 的 Bug **在架构层面不可能再发生**——因为 Recover 终态是 `Armed`，而 `Armed` → `Running` 必须经过 `RequestRelease()`
- 新增状态转换路径时只需在 DoTransition 的 allowed_from 列表中添加，不可能遗漏同步逻辑

```
这一步是投入产出比最高的，建议优先落地。
```

### 阶段 2：SharedState 添加 AxisIntent + 双写验证

| 维度 | 评估 |
|------|------|
| **改动量** | SharedState +80 行，CiA402StateMachine +60 行（读取 intent 并与旧闩锁交叉验证），Coordinator +40 行（ComputeIntents），node.cpp +5 行 |
| **工期** | 1-2 天 |
| **风险** | **中低**。双写验证期间新旧逻辑同时运行，不一致时以旧逻辑为准，仅 WARN。不影响运行安全 |
| **回退代价** | 低。SharedState 中的 intent 字段是纯新增，不读就不影响 |
| **验证方法** | 跑 1000 次随机状态转换序列，统计 WARN 数量。期望为 0 |

**阶段2完成后的收益**：

- 确认 AxisIntent 电平语义在所有已知场景下与旧闩锁决策一致
- 为阶段3的剥离提供信心

### 阶段 3：剥离 CiA402Protocol

| 维度 | 评估 |
|------|------|
| **改动量** | 新增 CiA402Protocol ~380 行，删除 CiA402StateMachine 策略代码 ~350 行，修改 AxisLogic ~80 行，修改 AxisDriver ~60 行 |
| **工期** | 3-4 天 |
| **风险** | **中高**。这是核心控制逻辑的重写。虽然有阶段2的双写验证数据做支撑，但 CiA402Protocol 作为全新实现，可能在边界条件上与旧状态机有微妙差异 |
| **回退代价** | 中。需要还原 CiA402StateMachine + AxisLogic + AxisDriver 的修改 |
| **验证方法** | CiA402Protocol 单元测试覆盖所有状态组合 × AxisIntent 组合（约 7×4=28 种基本组合，含边界条件约 80 个 test case） |

**关键风险点与应对**：

```
风险1：fault reset 从状态机移到 Coordinator 后，SDO 的异步时序可能导致
       复位命令重复发送或遗漏。
应对：Coordinator 中维护 per-axis fault_reset_in_progress 标志，
      SDO 完成回调中清除，超时后重试。

风险2：位置锁阈值判断从 CiA402StateMachine 中搬出后，如果 CiA402Protocol
       的判断时机与旧代码不同，可能导致使能瞬间的微小跳变。
应对：位置锁逻辑原样迁移，仅删除外部策略依赖。用实际驱动器测试
      使能瞬间的位置跟踪误差，确认 < 1 个编码器脉冲。

风险3：CiA402Protocol 的 Process() 是纯函数（无副作用），但 AxisDriver
       中的 TPDO 写入可能因 Lely 的 WriteEvent 失败而静默丢帧。
应对：Protocol 不负责 TPDO 写入——AxisDriver 处理写入失败并上报健康计数器。
      这一点与旧架构一致，不引入新风险。
```

### 阶段 4：SharedState 无锁化

| 维度 | 评估 |
|------|------|
| **改动量** | SeqLock +80 行，SharedState 改写 ~160 行 |
| **工期** | 2 天 |
| **风险** | **中**。SeqLock 在弱内存序架构（ARM）上需要正确的 memory barrier 对 |
| **回退代价** | 低。SeqLock 和 mutex 是可互换的同步原语 |
| **验证方法** | ThreadSanitizer + 延迟基准测试 |

这一阶段是性能优化，对功能正确性无影响。**如果当前的 mutex 延迟可接受，可以无限期推迟。**

---

## 十四、阶段 1 的具体实现骨架

这是最应优先落地的部分，给出可直接开始编码的骨架：

### 14.1 operational_coordinator.hpp

```cpp
#pragma once

#include <atomic>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

enum class SystemOpMode : uint8_t {
  Inactive,
  Configured,
  Standby,
  Armed,
  Running,
  Faulted,
  Recovering,
  ShuttingDown,
};

const char* SystemOpModeName(SystemOpMode mode);

class OperationalCoordinator {
 public:
  struct Result {
    bool ok;
    std::string message;
  };

  OperationalCoordinator(CanopenMaster* master, SharedState* shared_state,
                         std::size_t axis_count);

  SystemOpMode mode() const {
    return mode_.load(std::memory_order_acquire);
  }

  // ========== 状态转换 ==========
  Result RequestInit();
  Result RequestEnable();
  Result RequestRelease();
  Result RequestHalt();
  Result RequestRecover();
  Result RequestShutdown();

  // ========== 每帧调用 ==========
  void UpdateFromFeedback();

  // ========== 生命周期辅助 ==========
  // Configure 阶段设置 master 但不启动
  void SetConfigured();

 private:
  Result DoTransition(std::initializer_list<SystemOpMode> allowed_from,
                      SystemOpMode to,
                      std::function<bool(std::string*)> action = nullptr);

  std::atomic<SystemOpMode> mode_{SystemOpMode::Inactive};
  mutable std::mutex transition_mtx_;

  CanopenMaster* master_;
  SharedState* shared_state_;
  std::size_t axis_count_;
};

}  // namespace canopen_hw
```

### 14.2 operational_coordinator.cpp（阶段 1 版本——仍调用旧 API）

```cpp
#include "canopen_hw/operational_coordinator.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

const char* SystemOpModeName(SystemOpMode mode) {
  switch (mode) {
    case SystemOpMode::Inactive:     return "Inactive";
    case SystemOpMode::Configured:   return "Configured";
    case SystemOpMode::Standby:      return "Standby";
    case SystemOpMode::Armed:        return "Armed";
    case SystemOpMode::Running:      return "Running";
    case SystemOpMode::Faulted:      return "Faulted";
    case SystemOpMode::Recovering:   return "Recovering";
    case SystemOpMode::ShuttingDown: return "ShuttingDown";
  }
  return "Unknown";
}

OperationalCoordinator::OperationalCoordinator(
    CanopenMaster* master, SharedState* shared_state, std::size_t axis_count)
    : master_(master), shared_state_(shared_state), axis_count_(axis_count) {}

void OperationalCoordinator::SetConfigured() {
  mode_.store(SystemOpMode::Configured, std::memory_order_release);
}

OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowed_from,
    SystemOpMode to,
    std::function<bool(std::string*)> action) {

  std::lock_guard<std::mutex> lk(transition_mtx_);
  const SystemOpMode current = mode_.load(std::memory_order_acquire);

  bool from_ok = false;
  for (auto allowed : allowed_from) {
    if (current == allowed) { from_ok = true; break; }
  }

  if (!from_ok) {
    // 特殊处理：已经在目标状态
    if (current == to) {
      return {true, std::string("already ") + SystemOpModeName(to)};
    }
    return {false, std::string("cannot transition from ") +
                   SystemOpModeName(current) + " to " +
                   SystemOpModeName(to)};
  }

  std::string detail;
  if (action && !action(&detail)) {
    return {false, detail.empty() ? "action failed" : detail};
  }

  mode_.store(to, std::memory_order_release);
  CANOPEN_LOG_INFO("SystemOpMode: {} → {}",
                   SystemOpModeName(current), SystemOpModeName(to));
  return {true, detail.empty() ? SystemOpModeName(to) : detail};
}

// --- 阶段 1：仍通过旧 CanopenMaster API 实现 ---

Result OperationalCoordinator::RequestInit() {
  return DoTransition(
      {SystemOpMode::Configured},
      SystemOpMode::Standby,
      [this](std::string* detail) {
        if (!master_->Start()) {
          if (detail) *detail = "CAN master start failed";
          return false;
        }
        if (!master_->EnableAll()) {
          if (detail) *detail = "EnableAll failed";
          return false;
        }
        return true;
      });
}

Result OperationalCoordinator::RequestEnable() {
  // Standby → Armed：使能所有轴但立即 halt（冻结）
  return DoTransition(
      {SystemOpMode::Standby},
      SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!master_->EnableAll()) {
          if (detail) *detail = "EnableAll failed";
          return false;
        }
        if (!master_->HaltAll()) {
          CANOPEN_LOG_WARN("HaltAll failed during enable→armed");
        }
        return true;
      });
}

Result OperationalCoordinator::RequestRelease() {
  return DoTransition(
      {SystemOpMode::Armed},
      SystemOpMode::Running,
      [this](std::string* detail) {
        if (!master_->ResumeAll()) {
          if (detail) *detail = "ResumeAll failed";
          return false;
        }
        return true;
      });
}

Result OperationalCoordinator::RequestHalt() {
  return DoTransition(
      {SystemOpMode::Running},
      SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!master_->HaltAll()) {
          if (detail) *detail = "HaltAll failed";
          return false;
        }
        return true;
      });
}

Result OperationalCoordinator::RequestRecover() {
  return DoTransition(
      {SystemOpMode::Faulted},
      SystemOpMode::Armed,
      [this](std::string* detail) {
        // 阶段1：用旧 API
        // 1. 先确保所有轴 halt
        master_->HaltAll();

        // 2. 清全局故障
        if (shared_state_) {
          shared_state_->ClearGlobalFault();
          shared_state_->ClearAllAxesHaltedByFault();
        }

        // 3. 复位故障轴
        std::string recover_detail;
        if (!master_->RecoverFaultedAxes(&recover_detail)) {
          if (detail) *detail = recover_detail;
          // 恢复失败，回到 Faulted
          mode_.store(SystemOpMode::Faulted, std::memory_order_release);
          return false;
        }

        // 4. 确保 halt 状态（落入 Armed）
        master_->HaltAll();

        if (detail) *detail = "recovered → armed";
        return true;
      });
}

Result OperationalCoordinator::RequestShutdown() {
  return DoTransition(
      {SystemOpMode::Standby, SystemOpMode::Armed, SystemOpMode::Running,
       SystemOpMode::Faulted},
      SystemOpMode::Configured,
      [this](std::string* detail) {
        mode_.store(SystemOpMode::ShuttingDown, std::memory_order_release);
        std::string shutdown_detail;
        master_->GracefulShutdown(&shutdown_detail);
        master_->Stop();
        if (detail && !shutdown_detail.empty()) {
          *detail = shutdown_detail;
        }
        return true;
      });
}

void OperationalCoordinator::UpdateFromFeedback() {
  const SystemOpMode current = mode_.load(std::memory_order_acquire);

  // 只在 Armed/Running 时自动检测故障
  if (current != SystemOpMode::Armed && current != SystemOpMode::Running) {
    return;
  }

  if (!shared_state_) return;

  const SharedSnapshot snap = shared_state_->Snapshot();
  bool any_fault = false;
  for (std::size_t i = 0; i < axis_count_ && i < snap.feedback.size(); ++i) {
    if (snap.feedback[i].is_fault || snap.feedback[i].heartbeat_lost) {
      any_fault = true;
      break;
    }
  }

  if (any_fault) {
    SystemOpMode expected = current;
    if (mode_.compare_exchange_strong(expected, SystemOpMode::Faulted,
                                      std::memory_order_acq_rel)) {
      CANOPEN_LOG_WARN("SystemOpMode: {} → Faulted (auto-detected)",
                       SystemOpModeName(current));
    }
  }
}

}  // namespace canopen_hw
```

### 14.3 canopen_hw_ros_node.cpp 改动（阶段 1）

```cpp
// 变化概要：
// 1. 删除所有 service lambda 中的 lifecycle.xxx() 调用
// 2. 替换为 coordinator.RequestXxx()
// 3. 主循环中添加 coordinator.UpdateFromFeedback()

// ---- 初始化部分 ----
canopen_hw::OperationalCoordinator coordinator(
    lifecycle.master(), lifecycle.shared_state(), master_cfg.joints.size());
coordinator.SetConfigured();

// ---- service 注册（以 recover 为例）----
auto recover_srv = pnh.advertiseService<std_srvs::Trigger::Request,
                                        std_srvs::Trigger::Response>(
    "recover", [&](std_srvs::Trigger::Request&,
                   std_srvs::Trigger::Response& res) {
      std::lock_guard<std::mutex> lk(loop_mtx);
      auto r = coordinator.RequestRecover();
      res.success = r.ok;
      res.message = r.message;
      return true;
    });

// ---- 主循环 ----
while (ros::ok()) {
  const ros::Time now = ros::Time::now();
  const ros::Duration period = now - last_time;
  last_time = now;

  {
    std::lock_guard<std::mutex> lk(loop_mtx);
    coordinator.UpdateFromFeedback();   // <-- 新增
    robot_hw_ros.read(now, period);
    cm.update(now, period);
    robot_hw_ros.write(now, period);
    diag_updater.update();
  }
  rate.sleep();
}
```

### 14.4 阶段 1 删除的旧代码（LifecycleManager 侧）

```diff
// lifecycle_manager.hpp
- bool halted_ = false;
- bool require_init_ = true;
- LifecycleState state_ = LifecycleState::Configured;
+ // 这些状态全部迁移到 OperationalCoordinator

// lifecycle_manager.cpp  
- bool LifecycleManager::InitMotors() { ... }    // 逻辑迁入 Coordinator::RequestInit
- bool LifecycleManager::Halt() { ... }           // 逻辑迁入 Coordinator::RequestHalt  
- bool LifecycleManager::Resume() { ... }         // 逻辑迁入 Coordinator::RequestRelease
- bool LifecycleManager::Recover() { ... }        // 逻辑迁入 Coordinator::RequestRecover
- bool LifecycleManager::StopCommunication() { ... } // 逻辑迁入 Coordinator::RequestShutdown

// LifecycleManager 退化为：
// - 持有 CanopenMaster 和 SharedState 的生命周期所有权
// - 提供 Configure() 初始化 CAN 配置
// - 提供 Shutdown() 析构期清理
// - 不再持有任何操作状态
```

---

## 十五、总工期与里程碑

| 阶段 | 工期 | 累计 | 里程碑验证 |
|------|------|------|-----------|
| **阶段 1** | 2-3 天 | 2-3 天 | 所有 service 调用序列行为等价；Recover 后必须 Release 才能运行 |
| **阶段 2** | 1-2 天 | 3-5 天 | 双写验证 0 次 WARN |
| **阶段 3** | 3-4 天 | 6-9 天 | CiA402Protocol 单元测试 80+ case 全过；实际驱动器使能跳变 < 1 脉冲 |
| **阶段 4** | 2 天 | 8-11 天 | TSan 无报警；RPDO 延迟 P99.9 < 50μs |
| **测试收尾** | 2 天 | 10-13 天 | 集成测试矩阵全覆盖 |

**如果只做阶段 1**，2-3 天即可消除当前两个 Bug 的架构根因，且改动风险极低。这是我强烈推荐的最小可行改动。

阶段 2-4 是渐进式改善，可以根据实际维护压力决定是否推进。阶段 3 是改动最大、风险最高的一步——如果当前系统在阶段 1 后运行稳定且无新 Bug，可以暂缓。

---

## 十六、与当前三个 Bug 修复的关系

| 修复 | 是否仍然需要 | 与重构的关系 |
|------|-------------|-------------|
| 修复 1（Recover 中加 HaltAll） | **阶段 1 落地后不需要** | Coordinator.RequestRecover() 终态是 Armed，内含 HaltAll 调用 |
| 修复 2（halt_released 时 AdvanceArmEpoch） | **仍需要**，在阶段 3 时迁移到 CiA402Protocol | Protocol 中 `prev_was_halt_ → Run` 转换沿显式推进 epoch |
| 修复 3（guard 期间 write 覆盖为实际位置） | **仍需要**，与架构重构正交 | 这是 ROS 适配层的防御，不受 Coordinator 影响 |

**推荐做法**：先合入修复 2 和修复 3（它们是独立的、低风险的一行到几行改动），然后用阶段 1 从架构层面替代修复 1。
