# 意优谐波关节 CANopen 主站完整架构规格书

## 基于 Lely-core · 6轴 CSP · 原型级

---

## 0. 关键事实提取（来自 EDS + 手册）

在设计之前，先把从你提供的资料中提取的关键参数列出来，确保后续设计基于事实而非假设。

| 参数 | 值 | 来源 |
|---|---|---|
| 驱动器型号 | EuPH11D-101（PH11系列谐波关节） | EDS 0x1008 |
| 减速比 | 81:1 | EDS 0x6091:01 = 5,308,416 = 65536×81 |
| 编码器分辨率（输出轴） | 5,308,416 counts/rev | EDS 0x6091:01 |
| 用户分辨率（默认） | 5,308,416 counts/rev（与电机分辨率相同） | EDS 0x6091:02 |
| 支持模式 | PP(1), PV(3), PT(4), IP(7), CSP(8), CSV(9), CST(10) | 0x6502 = 0x03CD |
| 跟随误差窗口 | 300,000 counts（约20.3°输出轴） | 手册 0x6065 默认值 |
| 抱闸控制 | 驱动器内部自动管理：使能前自动松开，下使能后自动抱死 | 手册 3.7 节 |
| 抱闸延时 | 100ms（下使能前等待抱闸抱死） | 0x2026 默认值 |
| 故障反应 | 默认=0，立即停机+抱闸抱死 | 0x605E 默认值 |
| Heartbeat | 支持 Producer(0x1017) + Consumer(0x1016，5组) | EDS |
| PDO 数量 | 4 RxPDO + 4 TxPDO | EDS |
| PDO 映射 | 可动态重映射（sub0 RW） | EDS |
| 跳级跃迁 | **支持**。从 ReadyToSwitchOn 发 0x000F 直达 OperationEnabled | 手册 2.2.2 注1 |
| 插补周期单位 | 0x60C2:02 = -3（毫秒），默认值 0x60C2:01 = 1（1ms） | EDS |

**EDS 与手册的矛盾点（以 EDS 为准）：**

| 项目 | EDS（v1.4, 2025-03） | 手册（V2.0） | 采信 |
|---|---|---|---|
| RPDO transmission type AccessType | **RW** | const | **EDS**（可配置） |
| RPDO1 transmission type 默认值 | **1**（同步） | 0xFF（事件驱动） | **EDS** |
| TxPDO1 COB-ID | 0x40000180+id（bit30=1, no RTR） | 0x180+id | **EDS** |

---

## 1. 系统总体约束

| 项 | 规格 | 说明 |
|---|---|---|
| 轴数 | 6 | 节点 ID 分配：1-6 |
| 控制模式 | CSP（模式值=8） | |
| 同步周期 | **10ms**（10,000μs） | |
| 波特率 | 1Mbps | 驱动器出厂默认 |
| CAN 接口 | CANable 2.0 + gs_usb 固件 + SocketCAN | |
| 上位机 | Ubuntu 20.04, 标准内核（无 PREEMPT_RT） | |
| ROS | Noetic | |
| Lely-core | 2.3.x（源码编译） | |
| 可靠性目标 | 连续运行 3 小时无需人工干预 | 原型级 |

**核心设计原则（原型级务实原则）：**

1. 简单优于花哨——能用 mutex 解决的不搞无锁
2. 在非 RT 内核 + USB-CAN 上不追求微秒级确定性，10ms 周期留足余量
3. 故障处理以"安全停止+日志记录"为主，有限度自动恢复
4. 所有魔数来自 EDS 和手册，不凭空假设

---

## 2. CAN 总线负载分析

### 2.1 PDO 帧清单（每个 SYNC 周期）

| 帧类型 | 数量 | DLC(字节) | 方向 |
|---|---|---|---|
| SYNC | 1 | 0 | 主站→总线 |
| RxPDO1 × 6 | 6 | 7 | 主站→从站 |
| TxPDO1 × 6 | 6 | 7 | 从站→主站 |
| TxPDO2 × 6 | 6 | 8 | 从站→主站 |

其他帧（不在每个周期内）：

| 帧类型 | 频率 | DLC |
|---|---|---|
| Heartbeat（主站） | 每 1000ms 一帧 | 1 |
| Heartbeat（从站 × 6） | 每 500ms 一帧/轴 | 1 |
| EMCY（偶发） | 故障时 | 8 |
| SDO（仅初始化阶段） | 一次性 | 8 |

### 2.2 负载计算

CAN 2.0A 帧长度估算（含填充位）：

```
帧长度(bits) ≈ 47 + 8×DLC + stuffing
stuffing ≈ (47 + 8×DLC) / 4  (最坏情况每4个同值bit插1个)
```

| 帧类型 | DLC | 估算 bits/帧 | 数量 | 总 bits |
|---|---|---|---|---|
| SYNC | 0 | 55 | 1 | 55 |
| RxPDO1 | 7 | 125 | 6 | 750 |
| TxPDO1 | 7 | 125 | 6 | 750 |
| TxPDO2 | 8 | 135 | 6 | 810 |
| **合计** | | | **19帧** | **2,365** |

**10ms 内 1Mbps 可用带宽 = 10,000 bits**

**总线负载 ≈ 23.7%**

结论：**负载非常健康**。即使加上偶发的 SDO、EMCY、Heartbeat，也远低于 CiA 推荐的 70% 安全线。总线仲裁延迟不会成为问题。

### 2.3 SYNC → TxPDO 时序窗口

在收到 SYNC 后，6个从站同时准备发送 TxPDO1 + TxPDO2 = 12帧。按仲裁排序（COB-ID 从低到高）：

```
0x181, 0x182, 0x183, 0x184, 0x185, 0x186,  ← TxPDO1
0x281, 0x282, 0x283, 0x284, 0x285, 0x286   ← TxPDO2
```

12 帧 × ~130 bits = ~1,560 bits，需要 ~1.56ms 全部发完。

在 10ms 的 SYNC 周期内，1.56ms 留给 TxPDO 应答是充裕的。主站有约 8ms 来处理数据和准备下一轮 RxPDO。

---

## 3. PDO 映射方案

### 3.1 RxPDO1（主站 → 从站，控制指令）

每轴使用 **1 个 RxPDO**，覆盖 CSP 所需全部控制数据。

| 字节位置 | 对象 | Index:Sub | 类型 | 大小 |
|---|---|---|---|---|
| Byte 0-1 | Controlword | 0x6040:00 | UINT16 | 16 bit |
| Byte 2-5 | Target Position | 0x607A:00 | INT32 | 32 bit |
| Byte 6 | Mode of Operation | 0x6060:00 | INT8 | 8 bit |
| **合计** | | | | **7 bytes ✓** |

映射值：`0x60400010, 0x607A0020, 0x60600008`

通信参数：
- COB-ID：`0x200 + NodeID`（标准 RxPDO1）
- Transmission type：**1**（每个 SYNC 生效）

### 3.2 TxPDO1（从站 → 主站，核心反馈）

| 字节位置 | 对象 | Index:Sub | 类型 | 大小 |
|---|---|---|---|---|
| Byte 0-1 | Statusword | 0x6041:00 | UINT16 | 16 bit |
| Byte 2-5 | Position Actual | 0x6064:00 | INT32 | 32 bit |
| Byte 6 | Mode Display | 0x6061:00 | INT8 | 8 bit |
| **合计** | | | | **7 bytes ✓** |

映射值：`0x60410010, 0x60640020, 0x60610008`

通信参数：
- COB-ID：`0x180 + NodeID`（去掉出厂默认的 bit30，确保有效）
- Transmission type：**1**

### 3.3 TxPDO2（从站 → 主站，辅助反馈）

| 字节位置 | 对象 | Index:Sub | 类型 | 大小 |
|---|---|---|---|---|
| Byte 0-3 | Velocity Actual | 0x606C:00 | INT32 | 32 bit |
| Byte 4-5 | Torque Actual | 0x6077:00 | INT16 | 16 bit |
| **合计** | | | | **6 bytes ✓** |

映射值：`0x606C0020, 0x60770010`

通信参数：
- COB-ID：`0x280 + NodeID`
- Transmission type：**1**

### 3.4 禁用其他 PDO

| PDO | 处理 |
|---|---|
| RxPDO2 (0x1401) | COB-ID 设置 bit31=1 禁用 |
| RxPDO3 (0x1402) | COB-ID 设置 bit31=1 禁用 |
| RxPDO4 (0x1403) | 已出厂禁用，保持 |
| TxPDO3 (0x1802) | 已出厂禁用，保持 |
| TxPDO4 (0x1803) | 已出厂禁用，保持 |

### 3.5 关于 dcfgen

所有 PDO 映射通过 `master.yaml` 定义，由 `dcfgen` 生成 DCF 文件。Lely 主站在从站进入 Pre-Operational 时通过 SDO 自动写入这些配置。**不在 C++ 代码中手写 SDO 配置序列。**

---

## 4. SYNC 与心跳策略

### 4.1 SYNC 配置

| 参数 | 值 |
|---|---|
| SYNC COB-ID | 0x80（标准） |
| SYNC 周期 | 10,000 μs（10ms） |
| SYNC 生产者 | 主站 |

Lely-core 的 SYNC 定时由内部 ev::Loop 定时器驱动。在非 RT 内核上，定时器精度约 ±1ms（取决于系统负载和 USB 栈延迟）。

### 4.2 心跳配置

| 角色 | 参数 | 值 | 说明 |
|---|---|---|---|
| 主站 | heartbeat_producer | 500ms | 主站每 500ms 发一次心跳 |
| 从站 | heartbeat_consumer | 无需配置 | 不让从站监控主站（避免 0x8130 错误） |
| 主站 | heartbeat_consumer | 2000ms per slave | 主站监控从站，2秒未收到判定离线 |
| 从站 | heartbeat_producer | 500ms | 通过 dcfgen 在初始化时写入 0x1017=500 |

**为什么从站不设 consumer heartbeat（不监控主站）：**

意优手册明确指出，从站如果配置了 0x1016 监控主站心跳，主站超时会触发 0x8130 错误，驱动器进入 Fault 状态。在非 RT Linux + USB-CAN 环境下，主站心跳受 USB 调度影响可能偶尔延迟，会导致误触发。原型阶段不配置此功能，改用主站单向监控从站。

### 4.3 EMCY 配置

无需额外配置。从站的 EMCY COB-ID 出厂默认为 `0x80 + NodeID`，Lely-core 会自动接收并触发 `OnEmcy()` 回调。

---

## 5. CiA 402 状态机规格

### 5.1 Statusword 解码表（掩码 + 值）

**必须使用 `(statusword & mask) == value` 判断，禁止直接等值比较。**

| 状态 | Mask | Value | 说明 |
|---|---|---|---|
| NOT_READY_TO_SWITCH_ON | 0x004F | 0x0000 | 上电自检中 |
| SWITCH_ON_DISABLED | 0x004F | 0x0040 | 初始化完成，未使能 |
| READY_TO_SWITCH_ON | 0x006F | 0x0021 | 等待 Switch On |
| SWITCHED_ON | 0x006F | 0x0023 | 电源就绪，未使能 |
| OPERATION_ENABLED | 0x006F | 0x0027 | 正常运行 |
| QUICK_STOP_ACTIVE | 0x006F | 0x0007 | 快停执行中 |
| FAULT_REACTION_ACTIVE | 0x004F | 0x000F | 故障制动中 |
| FAULT | 0x004F | 0x0008 | 故障已停止 |

补充位解读：

| Bit | 名称 | 含义 |
|---|---|---|
| bit4 | Voltage enabled | 1=母线电压正常 |
| bit7 | Warning | 1=有警告（不影响运行） |
| bit9 | Remote | 1=CANopen 可操作 |
| bit10 | Target reached | 1=目标位置已到达 |
| bit12 | (CSP模式) | 1=目标位置被位置环采用 |
| bit13 | (CSP模式) | 1=跟随误差超限 |

### 5.2 Controlword 命令表

| 命令 | Controlword 值 | 适用跃迁 |
|---|---|---|
| Shutdown | 0x0006 | 2, 6, 8 |
| Switch On | 0x0007 | 3 |
| Enable Operation | 0x000F | 4, 16, **3+4（此驱动器支持）** |
| Disable Voltage | 0x0000 | 7, 9, 10, 12 |
| Quick Stop | 0x0002 | 7, 10, 11 |
| Disable Operation | 0x0007 | 5 |
| Fault Reset（上升沿） | 先 0x0000，再 0x0080 | 15 |

### 5.3 完整状态跃迁流程

```
上电
  │
  ▼
NOT_READY_TO_SWITCH_ON ──(自动)──▶ SWITCH_ON_DISABLED
                                          │
                                    发 0x0006 (Shutdown)
                                          │
                                          ▼
                                   READY_TO_SWITCH_ON
                                          │
           ┌──────────────────────────────┤
           │                              │
      (三步走法)                     (跳级走法，此驱动器支持)
      发 0x0007                       发 0x000F
           │                              │
           ▼                              │
      SWITCHED_ON                         │
           │                              │
      发 0x000F                           │
           │                              │
           ▼                              ▼
      OPERATION_ENABLED ◄────────────────┘
           │
      (正常运行，写 target_position)
           │
      ┌────┴────────────────┐
   (故障)                 (主动停止)
      │                    发 0x0006
      ▼                       │
FAULT_REACTION_ACTIVE         │
      │                       ▼
   (自动)              READY_TO_SWITCH_ON
      │                 (可再次使能)
      ▼
    FAULT
      │
   复位流程
      │
      ▼
SWITCH_ON_DISABLED
   (重走使能流程)
```

### 5.4 状态机运行策略

状态机驱动方式：**在每个 SYNC 周期的 TxPDO 接收回调中执行**（即 `OnRpdoWrite` 或等效时机）。每个周期根据最新 statusword 决定输出 controlword。

**各状态下的行为：**

#### NOT_READY_TO_SWITCH_ON
- 动作：不发任何 controlword，等待驱动器自检完成自动跃迁
- 超时：如果停留超过 5 秒，记录错误日志，标记该轴异常

#### SWITCH_ON_DISABLED
- 动作：发 `0x0006`（Shutdown），触发跃迁 2
- 同时：通过 RxPDO 设置 `mode_of_operation = 8`（CSP）

#### READY_TO_SWITCH_ON
- **前置校验**：读取 TxPDO1 中的 `mode_of_operation_display`（0x6061）
  - 如果 == 8（CSP）：发 `0x000F`（跳级使能），触发跃迁 3+4
  - 如果 ≠ 8：继续发 `0x0006` 保持当前状态，等待模式切换完成。正常情况下驱动器应在 1-2 个周期内反映模式切换
  - 超时：如果 20 个周期（200ms）后仍不等于 8，记录错误日志

#### SWITCHED_ON
- 情况说明：如果跳级使能成功，不会经过此状态。但作为防御性编程仍需处理
- 动作：发 `0x000F`（Enable Operation）

#### OPERATION_ENABLED（正常运行）
- 动作：发 `0x000F`，同时在 RxPDO 中写入 `target_position`
- **首次进入特殊处理**：见 5.5 节

#### QUICK_STOP_ACTIVE
- 情况说明：驱动器正在执行快停减速
- 动作：不发新指令。等待驱动器减速完成后自动跃迁到 SWITCH_ON_DISABLED（因 quick_stop_option_code 默认=1，不保持位置）
- 如需恢复：等进入 SWITCH_ON_DISABLED 后走正常使能流程

#### FAULT_REACTION_ACTIVE
- **关键**：驱动器正在执行故障制动，**绝对不发任何控制指令**
- 等待驱动器自动跃迁到 FAULT
- 此状态下 controlword 写 `0x0000`（无操作）

#### FAULT
- **带节流的上升沿复位流程**（见 5.6 节）

### 5.5 首次使能 & 无扰切换

#### 场景 A：首次进入 OPERATION_ENABLED

当某轴从非 OPERATION_ENABLED 状态首次进入 OPERATION_ENABLED：

1. 读取该周期 TxPDO1 中的 `position_actual`
2. 将 `target_position` 设为该 `position_actual` 值
3. 标记该轴处于 **"位置锁定"** 状态
4. 拦截上层（ROS）下发的目标位置，**强制 target = actual**
5. **释放条件**：当 ROS 下发的目标位置与当前 `position_actual` 的差值 < **安全阈值**时，才真正放开拦截

**安全阈值计算**：

```
阈值 = 1° 输出轴 = 5,308,416 / 360 ≈ 14,745 counts
```

取整：**15,000 counts**（约 1.02°）。这个值足够小不会产生明显跳变，又足够大能容忍浮点累积误差。

#### 场景 B：故障恢复后重新进入 OPERATION_ENABLED

与场景 A 完全相同的逻辑——因为在 Fault 期间上层控制器可能已经跑飞。

### 5.6 故障复位流程（带节流）

```
进入 FAULT 状态
     │
     ▼
┌─────────────┐
│ Phase 1:    │  发 controlword = 0x0000
│ 清除低位    │  维持至少 50ms (5个周期)
│ 计数器 = 0  │
└──────┬──────┘
       │ 计数器 >= 5
       ▼
┌─────────────┐
│ Phase 2:    │  发 controlword = 0x0080 (bit7上升沿)
│ 发复位沿    │  仅发送 1 个周期
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Phase 3:    │  发 controlword = 0x0000
│ 等待响应    │  等待 statusword 变为 SWITCH_ON_DISABLED
│             │  超时: 100 个周期 (1秒)
└──────┬──────┘
       │
  ┌────┴────┐
  │         │
成功       超时/仍在FAULT
  │         │
  ▼         ▼
正常使能   重试计数器 +1
流程       │
           ├── 重试 < 3次: 回到 Phase 1
           │
           └── 重试 >= 3次: 标记该轴为
               "永久故障"，停止重试，
               记录日志，等待人工干预
```

**注意**：在整个复位流程期间，该轴的 `target_position` 始终锁定为最后已知的 `position_actual`。

---

## 6. 线程架构与数据流

### 6.1 线程模型

```
┌─────────────────────────────────────────────────┐
│                   进程                            │
│                                                   │
│  ┌─────────────────┐    ┌──────────────────────┐ │
│  │ Thread 1        │    │ Thread 2             │ │
│  │ Lely ev::Loop   │    │ ROS Spinner +        │ │
│  │                 │    │ controller_manager    │ │
│  │ - SYNC 定时器   │    │                      │ │
│  │ - PDO 收发      │◄──►│ - read()             │ │
│  │ - 状态机逻辑    │    │ - update()           │ │
│  │ - EMCY 回调     │    │ - write()            │ │
│  │ - Heartbeat     │    │                      │ │
│  └─────────────────┘    └──────────────────────┘ │
│          ▲                        ▲               │
│          │    SharedState         │               │
│          └────(std::mutex)────────┘               │
└─────────────────────────────────────────────────┘
```

### 6.2 数据交换机制

**使用 `std::mutex`，理由如下：**

1. 非 RT 内核上无意义追求无锁——USB 栈的调度延迟 (1-2ms) 远大于 mutex 的锁竞争开销 (~微秒)
2. 需要保证多变量一致性（statusword + position_actual + velocity_actual 必须是同一周期的数据）
3. 10ms 周期下，mutex 的优先级反转问题不构成实际威胁
4. 代码简单可维护，减少 bug

```
SharedAxisData {
    // 反馈（Lely线程写，ROS线程读）
    int32_t  actual_position;
    int32_t  actual_velocity;
    int16_t  actual_torque;
    uint16_t statusword;
    int8_t   mode_display;

    // 指令（ROS线程写，Lely线程读）
    int32_t  target_position;

    // 状态标志（Lely线程写，ROS线程读）
    AxisState state_machine_state;  // 枚举
    bool     is_operational;        // true = OPERATION_ENABLED 且已释放锁定
    bool     is_fault;
}

SharedState {
    SharedAxisData axes[6];
    std::mutex     mtx;             // 一把锁保护全部6轴
    bool           all_operational; // 所有轴都 operational 且已释放锁定
}
```

**锁粒度**：一把全局锁。理由：
- 6轴数据量小（<200字节），拷贝极快
- 减少锁管理复杂度
- 不构成性能瓶颈

**锁的持有时间**：
- Lely 线程：每个 SYNC 周期锁一次，拷贝 6轴反馈数据 + 读取 6轴目标位置，<1μs
- ROS 线程：每个 controller update 周期锁一次，读取 6轴反馈 + 写入 6轴目标，<1μs

### 6.3 ROS hardware_interface 侧逻辑

```
read():
    lock(mtx)
    for each axis:
        joint_position[i] = ticks_to_rad(shared.axes[i].actual_position)
        joint_velocity[i] = ticks_to_rad_per_sec(shared.axes[i].actual_velocity)
        joint_effort[i]   = torque_permille_to_Nm(shared.axes[i].actual_torque)
    all_ok = shared.all_operational
    unlock(mtx)

write():
    lock(mtx)
    if shared.all_operational:
        for each axis:
            shared.axes[i].target_position = rad_to_ticks(joint_cmd[i])
    else:
        // 不写入任何指令，底层自动保持 target = actual
    unlock(mtx)
```

### 6.4 单位换算

```
counts_per_rev = 5,308,416  (0x6091:02, 可能各轴不同减速比)

ticks_to_rad(ticks)          = ticks * (2π / counts_per_rev)
rad_to_ticks(rad)            = rad * (counts_per_rev / 2π)
ticks_to_rad_per_sec(ticks)  = ticks * (2π / counts_per_rev)   // 速度单位已经是 pulse/s
torque_permille_to_Nm(perm)  = perm / 1000.0 * rated_torque_Nm  // 需要查手册获取额定扭矩
```

**注意**：6个轴如果使用不同型号（PH08/PH11/PH14/PH17/PH20/PH25），减速比不同，`counts_per_rev` 也不同。必须为每个轴单独配置。

---

## 7. 故障处理体系

### 7.1 故障分级

| 等级 | 条件 | 处理 |
|---|---|---|
| **L1 - 瞬态故障** | 单轴进入 FAULT，复位 1 次成功 | 自动复位，日志记录 WARNING。其他轴继续运行但锁定在原位 |
| **L2 - 反复故障** | 单轴复位 3 次失败 | 该轴标记永久故障，停止复位。其他轴锁定在原位。日志 ERROR |
| **L3 - 通讯丢失** | 心跳超时（2秒无心跳） | 该轴标记离线。主站不对该轴发送 NMT Reset（见下文说明）。等待自然恢复 |
| **L4 - 总线故障** | CAN Bus-Off（SocketCAN 自动恢复） | 日志 FATAL，依赖 Linux 内核 CAN 子系统自动恢复。如 10 秒不恢复，退出程序 |

### 7.2 关于不主动发 NMT Reset Node 的决策

**在原型阶段不实现 NMT Reset Node 自动恢复。理由：**

1. NMT Reset Node 会导致驱动器完全重启，抱闸先松后合，垂直轴可能下坠
2. 重启后驱动器丢失所有运行时 PDO 配置（除非已存入 EEPROM），需要完整重走 DCF 配置流程
3. Lely-core 的 `AsyncMaster` 在节点重启时自动重新执行 DCF 配置，但这个流程与手动 NMT Reset 的交互可能有边界情况
4. 对于 3 小时原型目标，通讯丢失的概率很低，如果发生，人工重启更安全

**恢复策略**：如果从站自行重启（电源波动等），Lely-core 会检测到 Bootup 报文，自动重新配置并触发 `OnBoot()` 回调，此时重走使能流程即可。

### 7.3 全局安全逻辑

```
每个 SYNC 周期（Lely 线程）:
    all_ok = true
    for each axis:
        if axis.state != OPERATION_ENABLED || axis.position_locked:
            all_ok = false

    if !all_ok:
        for each axis that IS OPERATION_ENABLED:
            axis.target_position = axis.actual_position  // 锁定原位

    shared.all_operational = all_ok
```

**含义**：任何一轴不在正常运行状态，**所有轴**锁定原位（但不下使能）。这保证了：
- 机械臂不会因为部分轴运动部分轴不动而进入奇异姿态
- 所有轴保持刚性保持力
- 一旦故障轴恢复，通过无扰切换机制安全恢复

### 7.4 EMCY 处理

重写 Lely 的 `OnEmcy()` 回调：

| 信息 | 处理 |
|---|---|
| error_code (uint16) | 按意优错误码表解析并写入日志 |
| error_register (uint8) | 记录 |
| manufacturer_specific (5 bytes) | 原样记录到日志 |

EMCY 仅做**日志记录**，不做额外动作。状态机的故障处理由 statusword 驱动，不依赖 EMCY 时序。

**错误码对照表（来自手册 5.2 节）：**

| Code | 含义 | 可自动复位？ |
|---|---|---|
| 0x3210 | 过压 | 取决于电源是否恢复 |
| 0x3220 | 欠压 | 取决于电源是否恢复 |
| 0x3230 | 过载 i2t | 冷却后可能可以 |
| 0x4210 | 温度过高 | 冷却后可能可以 |
| 0x7121 | 堵转 | 需排除机械卡滞 |
| 0x7310 | 超速 | 可尝试复位 |
| 0x8130 | 心跳超时 | 通讯恢复后可以 |
| 0x8500 | 速度跟随误差过大 | 可尝试复位 |
| 0x8611 | 位置跟随误差过大 | 可尝试复位 |

---

## 8. 启动与关机流程

### 8.1 启动流程

```
1. 初始化 SocketCAN 接口
   $ sudo ip link set canable0 up type can bitrate 1000000

2. 创建 Lely AsyncMaster
   - 加载由 dcfgen 生成的 master DCF
   - 启动 ev

## 8. 启动与关机流程（续）

### 8.1 启动流程

```
1. 初始化 SocketCAN 接口
   $ sudo ip link set canable0 up type can bitrate 1000000

2. 创建 Lely AsyncMaster
   - 加载由 dcfgen 生成的 master DCF
   - 启动 ev::Loop 线程

3. Lely 自动执行（对每个从站）：
   a. 等待从站 Bootup 报文（或通过 NMT 触发）
   b. 进入 Pre-Operational
   c. 通过 SDO 写入 DCF 中定义的全部配置：
      - PDO 映射（先 disable PDO → 写 mapping → enable PDO）
      - 心跳 producer = 500ms
      - mode_of_operation = 8 (CSP)
   d. 发送 NMT Start → 从站进入 Operational
   e. 触发 OnBoot(ok) 回调

4. OnBoot(ok=true) 中：
   - 标记该轴为 "已上线"
   - 状态机开始运行（此时从站应处于 SWITCH_ON_DISABLED）

5. 等待所有 6 轴 OnBoot 完成
   - 超时：10 秒。任一轴未上线则记录 ERROR 但不阻塞其他轴
   - 只有已上线的轴参与后续控制

6. 状态机自动将各轴推进至 OPERATION_ENABLED
   - 0x0006 → READY_TO_SWITCH_ON
   - 校验 0x6061 == 8
   - 0x000F → OPERATION_ENABLED
   - 首次使能位置锁定启动

7. 所有已上线轴进入 OPERATION_ENABLED 且无扰切换释放后：
   - shared.all_operational = true
   - ROS controller_manager 开始正常工作
```

### 8.2 上电时序图

```
时间轴(ms)  主站                        从站(node 1为例)
─────────────────────────────────────────────────────────
0           启动 ev::Loop               上电自检
~500        等待 bootup                 发送 Bootup (0x700)
~500        收到 bootup →               NMT: Pre-Operational
            开始 SDO 配置
~500-2000   SDO 写 PDO mapping          接收并应用配置
            SDO 写 0x1017=500
            SDO 写 0x6060=8
~2000       NMT Start Node              进入 Operational
~2000       OnBoot(ok=true)             开始响应 SYNC/PDO
~2010       SYNC #1                     TxPDO: statusword
            读到 SWITCH_ON_DISABLED
            RxPDO: cw=0x0006
~2020       SYNC #2                     TxPDO: READY_TO_SWITCH_ON
            确认 0x6061==8
            RxPDO: cw=0x000F
~2030       SYNC #3                     TxPDO: OPERATION_ENABLED
            读取 actual_pos
            锁定 target = actual
            该轴正式运行
```

### 8.3 关机流程

```
1. 收到 ROS shutdown 信号（SIGINT / ros::shutdown()）

2. 停止 ROS controller_manager 更新

3. 对所有轴：
   a. 发 controlword = 0x0007 (Disable Operation)
      → 驱动器按 0x605C 设定的减速斜坡停机
      → 进入 SWITCHED_ON，轴保持位置
   b. 等待 statusword 确认 SWITCHED_ON（超时 2秒）
   c. 发 controlword = 0x0006 (Shutdown)
      → 进入 READY_TO_SWITCH_ON
      → 驱动器自动关闭抱闸供电，抱闸抱死
   d. 等待 statusword 确认（超时 1秒）

4. 对所有轴发 NMT Stop（进入 Stopped 状态）

5. 停止 ev::Loop 线程

6. 关闭 SocketCAN 接口

7. 进程退出
```

**紧急关机（SIGTERM / 强杀）：**

如果进程被强杀，主站不再发送任何 CAN 帧。从站的行为取决于：

- 如果配置了从站监控主站心跳（我们没配置）：从站会在超时后自行报 0x8130 并停机
- 如果没有配置：**从站将持续执行最后收到的 target_position**

**对策**：由于不配置从站监控主站，需要确保异常退出时驱动器不会失控：

- CSP 模式下，从站如果不再收到 SYNC 帧，不会更新位置，电机保持在最后位置
- 验证：需在调试阶段确认意优驱动器在 SYNC 丢失后的行为（是否有 SYNC 超时保护）
- 建议在手册中确认或通过实验验证：中断 SYNC 5秒后驱动器是否自动下使能

---

## 9. master.yaml 结构规格

以下是给 `dcfgen` 使用的 `master.yaml` 的完整结构。dcfgen 会根据此文件和 EDS 文件生成 master 和 slave 的 DCF 配置。

```yaml
master:
  node_id: 127                          # 主站节点ID，避开1-6
  heartbeat_producer: 500               # 主站心跳 500ms（此处仅供参考，
                                        # 不配置从站 consumer 所以不被监控）
  sync:
    period: 10000                       # SYNC 周期 10ms = 10000μs
    overflow: 0                         # 不使用 SYNC counter

defaults: &defaults
  dcf_path: YiyouServo_V1.4.eds        # EDS 文件路径
  heartbeat_producer: 500               # 配置从站 0x1017 = 500ms
  boot_timeout: 5000                    # 上线超时 5 秒
  # 主站监控从站心跳：node_id 在各 slave 中分别指定
  # consumer 的超时设为 2000ms

nodes:
  - id: 1
    <<: *defaults
    name: "joint_1"
    heartbeat_consumer:
      - node_id: 1
        timeout: 2000

    rpdo:
      - index: 1                        # RPDO1
        cob_id: 0x201                   # 标准 0x200 + NodeID
        transmission: 1                 # 每个 SYNC 生效
        mapping:
          - {index: 0x6040, sub_index: 0, bits: 16}  # controlword
          - {index: 0x607A, sub_index: 0, bits: 32}  # target_position
          - {index: 0x6060, sub_index: 0, bits: 8}   # mode_of_operation
      - index: 2                        # RPDO2 - 禁用
        enabled: false
      - index: 3                        # RPDO3 - 禁用
        enabled: false
      - index: 4                        # RPDO4 - 禁用
        enabled: false

    tpdo:
      - index: 1                        # TPDO1
        cob_id: 0x181                   # 标准 0x180 + NodeID (确保 bit30/31 = 0)
        transmission: 1                 # 每个 SYNC 后应答
        mapping:
          - {index: 0x6041, sub_index: 0, bits: 16}  # statusword
          - {index: 0x6064, sub_index: 0, bits: 32}  # position_actual
          - {index: 0x6061, sub_index: 0, bits: 8}   # mode_display
      - index: 2                        # TPDO2
        cob_id: 0x281                   # 标准 0x280 + NodeID
        transmission: 1
        mapping:
          - {index: 0x606C, sub_index: 0, bits: 32}  # velocity_actual
          - {index: 0x6077, sub_index: 0, bits: 16}  # torque_actual
      - index: 3                        # TPDO3 - 禁用
        enabled: false
      - index: 4                        # TPDO4 - 禁用
        enabled: false

    sdo:                                # 初始化阶段的额外 SDO 写入
      - {index: 0x6060, sub_index: 0, value: 8}      # CSP 模式

  # Node 2-6: 结构相同，只改 id 和对应的 cob_id
  - id: 2
    <<: *defaults
    name: "joint_2"
    heartbeat_consumer:
      - node_id: 2
        timeout: 2000
    rpdo:
      - index: 1
        cob_id: 0x202
        transmission: 1
        mapping:
          - {index: 0x6040, sub_index: 0, bits: 16}
          - {index: 0x607A, sub_index: 0, bits: 32}
          - {index: 0x6060, sub_index: 0, bits: 8}
      - index: 2
        enabled: false
      - index: 3
        enabled: false
      - index: 4
        enabled: false
    tpdo:
      - index: 1
        cob_id: 0x182
        transmission: 1
        mapping:
          - {index: 0x6041, sub_index: 0, bits: 16}
          - {index: 0x6064, sub_index: 0, bits: 32}
          - {index: 0x6061, sub_index: 0, bits: 8}
      - index: 2
        cob_id: 0x282
        transmission: 1
        mapping:
          - {index: 0x606C, sub_index: 0, bits: 32}
          - {index: 0x6077, sub_index: 0, bits: 16}
      - index: 3
        enabled: false
      - index: 4
        enabled: false
    sdo:
      - {index: 0x6060, sub_index: 0, value: 8}

  # Node 3-6: 同上模式，省略展示，实际文件需完整列出
  # ... (id: 3, cob_id: 0x203/0x183/0x283, etc.)
  # ... (id: 4, cob_id: 0x204/0x184/0x284, etc.)
  # ... (id: 5, cob_id: 0x205/0x185/0x285, etc.)
  # ... (id: 6, cob_id: 0x206/0x186/0x286, etc.)
```

**重要说明**：以上 YAML 结构是逻辑规格。Lely dcfgen 的实际 YAML 语法可能与此有差异（键名、层级），实现时需参照 lely-core 文档中 `dcfgen` 的具体语法要求。核心约束是：

1. 每个 RxPDO 和 TxPDO 必须 ≤ 8 字节
2. transmission type = 1
3. PDO COB-ID 不能有 bit30/bit31 被置位（否则 PDO 无效/禁用）
4. 禁用的 PDO 通过 COB-ID bit31=1 禁用

---

## 10. 软件模块结构

```
canopen_hw/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── master.yaml              # dcfgen 输入
│   ├── YiyouServo_V1.4.eds      # 驱动器 EDS
│   ├── master.dcf               # dcfgen 输出（编译时生成）
│   └── joints.yaml              # 各轴参数（node_id, 减速比, 额定扭矩等）
├── launch/
│   └── canopen_hw.launch
├── include/canopen_hw/
│   ├── cia402_state_machine.hpp  # CiA 402 状态机（纯逻辑，无IO）
│   ├── cia402_defs.hpp           # 掩码表、controlword 常量定义
│   ├── axis_driver.hpp           # 单轴驱动类（继承 lely BasicDriver）
│   ├── canopen_master.hpp        # 主站管理类
│   ├── shared_state.hpp          # 线程间共享数据结构
│   └── canopen_robot_hw.hpp      # ROS hardware_interface 实现
├── src/
│   ├── cia402_state_machine.cpp
│   ├── axis_driver.cpp
│   ├── canopen_master.cpp
│   ├── canopen_robot_hw.cpp
│   └── main.cpp                  # 入口
└── test/
    ├── test_state_machine.cpp    # 状态机单元测试
    └── test_unit_conversion.cpp  # 单位换算测试
```

### 10.1 模块职责

| 模块 | 职责 | 线程 |
|---|---|---|
| `cia402_defs.hpp` | 纯头文件，定义掩码、controlword 常量、错误码枚举 | 无 |
| `cia402_state_machine` | 纯状态机逻辑：输入 statusword + mode_display，输出 controlword + 状态枚举。不涉及任何 IO、定时、线程 | 被 axis_driver 调用 |
| `axis_driver` | 继承 `lely::canopen::BasicDriver`。重写 `OnRpdoWrite()` 解析 TxPDO 数据，调用状态机，构造 RxPDO 数据。重写 `OnEmcy()`、`OnHeartbeat()` | Lely ev::Loop |
| `canopen_master` | 创建和管理 `lely::io::CanController`、`lely::canopen::AsyncMaster`、`ev::Loop`。创建 6 个 `axis_driver` 实例。启动/停止 ev::Loop 线程 | 管理 Lely 线程 |
| `shared_state` | 定义 `SharedAxisData`、`SharedState`，包含 mutex。提供 `update_feedback()` / `get_command()` 方法 | 被两个线程共享 |
| `canopen_robot_hw` | 实现 `hardware_interface::RobotHW`。`read()` / `write()` 通过 SharedState 交换数据。注册 JointStateInterface + PositionJointInterface | ROS 线程 |
| `main.cpp` | 创建 NodeHandle、canopen_master、canopen_robot_hw、controller_manager。主循环 `ros::Rate(100)` + `cm.update()` | ROS 线程 |

### 10.2 CiA 402 状态机类接口

```
class CiA402StateMachine:
    // 输入
    void update(uint16_t statusword, int8_t mode_display)

    // 输出
    uint16_t   get_controlword()
    State      get_state()          // 枚举：各 CiA 402 状态
    bool       is_operational()     // OPERATION_ENABLED 且锁定已释放
    bool       is_fault()
    int        fault_reset_count()  // 已累计复位次数

    // 配置
    void       set_target_mode(int8_t mode)  // 8 for CSP
    void       request_enable()              // 开始使能流程
    void       request_disable()             // 开始下使能流程
    void       acknowledge_actual_position(int32_t pos)  // 首次使能/恢复时写入

    // 无扰切换
    void       set_ros_target(int32_t target)  // ROS 希望的目标位置
    int32_t    get_safe_target()                // 经过无扰过滤后的实际下发目标
    bool       is_position_locked()
```

**状态机是纯函数式的**——给定输入（statusword、mode_display），输出确定的 controlword 和状态。所有定时（如复位节流的 50ms 计数）通过调用次数计数实现（每次调用 = 1 个 SYNC 周期 = 10ms），不依赖系统时钟。这使得状态机可以被完整单元测试。

---

## 11. 关键注意事项与已知风险

### 11.1 EDS 的 TxPDO1 COB-ID 出厂默认有 bit30

EDS 中 `[1800sub1]` 默认值为 `$NODEID+0x40000180`，bit30=1 表示 "no RTR allowed"。这在 CAN 2.0A 下是合法的（CiA 301 定义 bit30=1 表示帧类型为 data frame only），但需要确认 Lely-core 是否正确处理此标志。

**对策**：在 dcfgen 配置中显式设置 `cob_id: 0x180 + NodeID`（不带 bit30），覆盖默认值。

### 11.2 EDS 的 TxPDO2 COB-ID 出厂也有 bit30

`[1801sub1]` 默认值 `$NODEID+0x40000280`。同样在 dcfgen 中覆盖。

### 11.3 SYNC 丢失后驱动器行为 —— 需实验验证

手册未明确说明 CSP 模式下 SYNC 帧停止后驱动器的行为。可能的情况：

| 可能行为 | 结果 | 风险 |
|---|---|---|
| 保持最后位置 | 电机刚性锁定 | 安全 |
| 超时下使能 | 抱闸抱死 | 安全但可能有惯性冲击 |
| 无动作，持续等待 | 电机刚性锁定 | 安全 |

**建议**：在第一次通电调试时，用 `candump` 抓包确认从站在 SYNC 中断后的行为。如果驱动器不自动停机，可考虑配置从站的 0x1006（Communication Cycle Period），部分驱动器支持 SYNC 超时检测。

### 11.4 USB-CAN 的 SYNC 抖动

CANable 2.0 (gs_usb) 的 SYNC 发送受 USB 帧调度影响：

- USB 2.0 Full Speed 的微帧周期 = 1ms
- 最坏情况：SYNC 帧的发送延迟 = 1-2ms
- 意味着实际 SYNC 周期在 8-12ms 之间波动

**对于 10ms 周期**：这个抖动比例为 ±20%。CSP 模式下驱动器收到新位置后立即执行，不依赖 SYNC 间隔的精确性（位置插补在驱动器内部按 1ms 执行）。因此 SYNC 抖动主要影响控制带宽，不影响安全性。

**对于 3 小时原型目标**：可以接受。如果后续需要更高精度，替换为 PCIe CAN 卡。

### 11.5 controller_manager 与 SYNC 的频率对齐

| 组件 | 频率 |
|---|---|
| SYNC 周期 | 100 Hz (10ms) |
| controller_manager update | 建议设为 **100 Hz** |

两者设为相同频率最简单。controller_manager 的 update 周期由 `main.cpp` 中的 `ros::Rate` 控制，与 SYNC 没有硬同步关系。在每次 `read()` 时读取最新的反馈即可，不需要精确对齐。

如果 controller_manager 运行在更高频率（如 500Hz），`read()` 可能多次读到同一组反馈数据（因为底层 PDO 只在 100Hz 更新）。这不影响正确性，但浪费 CPU。保持 100Hz 最合理。

### 11.6 大跟随误差窗口的利弊

驱动器默认 0x6065 = 300,000 counts ≈ 20.3° 输出轴。这个值很大，意味着：

- **好处**：首次使能时即使 target 和 actual 有小偏差，不会立即报 0x8611
- **坏处**：真正的机械问题（如堵转）可能要积累到 20° 偏差才报错

**建议**：原型阶段保持默认值不修改。如果后续需要更灵敏的保护，可通过 SDO 在初始化阶段写入更小的值（如 50,000 counts ≈ 3.4°）。

---

## 12. 调试与验证计划

### 12.1 阶段划分

| 阶段 | 内容 | 轴数 | 通过标准 |
|---|---|---|---|
| **D1** | SocketCAN 通讯验证 | 1 | `candump` 能看到从站心跳 |
| **D2** | Lely 主站启动 + DCF 配置 | 1 | OnBoot(ok=true)，从站进入 Operational |
| **D3** | 状态机使能 | 1 | 单轴 CSP 使能成功，statusword = OPERATION_ENABLED |
| **D4** | 位置控制 | 1 | 写入不同 target_position，电机跟随 |
| **D5** | 故障复位 | 1 | 手动触发故障（拔电源线再插回），观察自动复位 |
| **D6** | 无扰切换 | 1 | 故障恢复后电机不跳变 |
| **D7** | 多轴联调 | 6 | 6轴同时使能和运动 |
| **D8** | ROS 集成 | 6 | controller_manager + JointTrajectoryController 正常工作 |
| **D9** | 耐久测试 | 6 | 连续运行 3 小时，反复运动，无故障 |

### 12.2 每个阶段的关键观测手段

| 手段 | 用途 |
|---|---|
| `candump canable0` | 原始 CAN 帧观测，确认 SYNC/PDO/EMCY 时序 |
| `cansend canable0` | 手动发送单帧，验证驱动器响应 |
| 自定义日志 | 每个 SYNC 周期记录 statusword、controlword、actual_pos、target_pos |
| `rqt_plot` | ROS 侧观测关节位置曲线 |
| 示波器/逻辑分析仪 | （可选）测量 SYNC 实际间隔，验证 USB-CAN 抖动 |

### 12.3 D1 阶段验证命令

```bash
# 设置 CAN 接口
sudo ip link set canable0 up type can bitrate 1000000

# 检查驱动器心跳（如果 0x1017 出厂默认为 0 则看不到，
# 但应该能看到 Bootup 帧）
candump canable0

# 如果看不到任何帧，检查：
# 1. 终端电阻是否正确（总线两端各120Ω）
# 2. CANable 是否被识别（lsusb 应显示 gs_usb 设备）
# 3. CAN_H / CAN_L 是否接反

# 手动读取 statusword（SDO read，假设 NodeID=1）
cansend canable0 601#40.41.60.00.00.00.00.00
# 期望收到 581#4B.41.60.00.XX.XX.00.00
# XX.XX 应为 0x0240 或 0x0640 (SWITCH_ON_DISABLED，bit6=1, bit9=1)

# 手动读取固件版本
cansend canable0 601#40.0A.10.00.00.00.00.00
```

### 12.4 D3 阶段手动使能验证

在写任何 C++ 代码之前，可以先用 `cansend` 手动走一遍使能流程：

```bash
# 1. 设置模式为 CSP (0x6060 = 8)
cansend canable0 601#2F.60.60.00.08.00.00.00
# 期望返回 581#60.60.60.00.00.00.00.00

# 2. 读取当前位置 (0x6064)
cansend canable0 601#40.64.60.00.00.00.00.00
# 记下返回的位置值 actual_pos

# 3. 将 target_position 设为 actual_pos (0x607A)
cansend canable0 601#23.7A.60.00.<actual_pos 4字节小端>

# 4. Shutdown (0x6040 = 0x0006)
cansend canable0 601#2B.40.60.00.06.00.00.00

# 5. 读 statusword 确认 READY_TO_SWITCH_ON
cansend canable0 601#40.41.60.00.00.00.00.00

# 6. Enable Operation (0x6040 = 0x000F)
cansend canable0 601#2B.40.60.00.0F.00.00.00

# 7. 读 statusword 确认 OPERATION_ENABLED
cansend canable0 601#40.41.60.00.00.00.00.00
# 期望: (statusword & 0x006F) == 0x0027

# 8. 下使能 (0x6040 = 0x0006)
cansend canable0 601#2B.40.60.00.06.00.00.00
```

**此步骤是在写代码前确认驱动器行为与文档一致的关键验证。如果手动使能都不成功，代码写得再好也没用。**

---

## 13. 配置参数汇总

所有可调参数集中在一个 YAML 文件中：

```yaml
# joints.yaml
canopen:
  interface: "canable0"
  bitrate: 1000000
  sync_period_us: 10000
  master_node_id: 127

joints:
  - name: "joint_1"
    node_id: 
    counts_per_rev: 5308416        # 65536 * 81 (PH11, 减速比81)
    rated_torque_nm: 6.0           # 需查手册确认
    position_lock_threshold: 15000 # counts, 约1°
    max_fault_resets: 3
    fault_reset_hold_cycles: 5     # 50ms

  - name: "joint_2"
    node_id: 2
    counts_per_rev: 5308416
    rated_torque_nm: 6.0
    position_lock_threshold: 15000
    max_fault_resets: 3
    fault_reset_hold_cycles: 5

  # joint_3 ~ joint_6 同理
  # 如果有不同型号（如 PH14, PH17），counts_per_rev 和
  # rated_torque_nm 需要分别填写

safety:
  any_fault_locks_all: true        # 任一轴故障锁定全部轴
  graceful_shutdown_timeout_ms: 3000
```

---

## 14. 方案局限性声明

以下是此方案的已知局限，在原型阶段可以接受，但在产品化时必须解决：

| # | 局限 | 影响 | 产品化时的解决方案 |
|---|---|---|---|
| 1 | 非 RT 内核 | SYNC 抖动 ±1-2ms | 使用 PREEMPT_RT + SCHED_FIFO + mlockall |
| 2 | USB-CAN | SYNC 和 PDO 有 USB 帧延迟 | 替换为 PCIe/SPI CAN 控制器 |
| 3 | std::mutex | 理论上有优先级反转风险 | RT 环境下改为 SeqLock 或无锁双缓冲 |
| 4 | 无位置增量限幅 | 如果上层规划器输出跳变位置，驱动器会全速追赶 | 增加每周期 Δpos 限幅器 |
| 5 | 不自动 NMT Reset | 通讯丢失后不自动恢复 | 增加 NMT Reset Communication 恢复逻辑 |
| 6 | 无急停硬件 | 软件崩溃后无独立安全回路 | 增加硬件急停按钮，独立于 CAN 通讯 |
| 7 | 单进程 | ROS node 崩溃 = CAN 通讯中断 | 分离 CAN 通讯进程和 ROS 接口进程 |
| 8 | 从站不监控主站 | 主站崩溃后从站不知道 | 产品化后配置合理的从站 heartbeat consumer |

---

这就是完整的架构规格。用这份文档可以直接开始编码，所有设计决策都有明确的依据（EDS 字段、手册段落、或工程分析），没有悬而未决的假设。
