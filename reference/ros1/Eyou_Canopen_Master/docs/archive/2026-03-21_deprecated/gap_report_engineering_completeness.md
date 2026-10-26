# Bug 报告与修复计划：工程完整度差距

- 日期：2026-03-19
- 审查视角：从 7.5 分到 8.5+ 分需要补什么
- 当前状态：功能验证阶段，核心逻辑扎实，但离生产级有明确差距

---

## GAP-1: CSV/CST 模式 PDO 下行链路为空（功能未完成）

- 严重程度：P0
- 文件：`src/axis_driver.cpp:229-231`
- 现象：`OnRpdoWrite` 中读取了 `safe_target_velocity` 和 `safe_target_torque`，但只调了 `SendTargetPosition`，速度和力矩用 `(void)` 丢弃，标注 TODO。
- 影响：声称支持三种运动模式（CSP/CSV/CST），但 CSV 和 CST 的下行链路是断的。状态机、SharedState、RobotHw 全链路都已适配，唯独最后一步——往总线写 0x60FF（target velocity）和 0x6071（target torque）——没做。等于上层设了速度/力矩命令，到驱动器那里全是零。
- 根因：C12 开发时为了避免在没有硬件验证的情况下写错 PDO 映射，刻意留了 TODO。但这导致整个多模式功能链路不完整。

### 修复计划

#### F07 — `feat: add SendTargetVelocity and SendTargetTorque PDO writes`

- 工作量：小
- 改动文件：
  - `include/canopen_hw/axis_driver.hpp`：声明 `SendTargetVelocity(int32_t)` 和 `SendTargetTorque(int16_t)`
  - `src/axis_driver.cpp`：
    - 实现 `SendTargetVelocity`：写 `tpdo_mapped[0x60FF][0]`
    - 实现 `SendTargetTorque`：写 `tpdo_mapped[0x6071][0]`
    - `OnRpdoWrite` 中删除 `(void)` 行，替换为实际调用
  - 注意：`SendTargetVelocity` 的 PDO 对象 0x60FF 是 int32_t，`SendTargetTorque` 的 0x6071 是 int16_t，与 CiA 402 规范一致

#### F08 — `feat: send mode_of_operation via PDO 0x6060`

- 工作量：小
- 依赖：F07
- 改动文件：
  - `include/canopen_hw/axis_driver.hpp`：声明 `SendModeOfOperation(int8_t)`
  - `src/axis_driver.cpp`：
    - 实现 `SendModeOfOperation`：写 `tpdo_mapped[0x6060][0]`
    - `OnRpdoWrite` 中在发送目标值之前先发送模式字节
  - 说明：当前 controlword 由状态机管理，但 mode_of_operation（0x6060）没有显式写入。部分驱动器依赖主站在 PDO 中持续发送 0x6060 才能维持模式，不发的话可能回退到默认模式。

---

## GAP-2: 主循环 sleep_for 无实时保证（实时性缺失）

- 严重程度：P1
- 文件：`src/main.cpp:84-88`
- 现象：100Hz 控制循环用 `std::this_thread::sleep_for(10ms)`，依赖 CFS 调度器。实际周期抖动可达数毫秒，在 CSP 模式下伺服驱动器期望严格等间隔的目标位置更新，抖动会导致跟踪误差甚至跟随报警。
- 影响：CSP 模式下位置跟踪精度受限，高速运动场景可能触发驱动器 following error。
- 根因：原型阶段优先功能验证，实时调度基础设施未搭建。

### 修复计划

#### F09 — `feat: add RealtimeLoop utility with clock_nanosleep`

- 工作量：中
- 改动文件：
  - `include/canopen_hw/realtime_loop.hpp`（新增）：
    - `RealtimeLoop` 类，封装周期性循环
    - 构造参数：周期（ns）、是否启用 SCHED_FIFO、FIFO 优先级
    - `Run(std::function<bool()> tick)` 方法：每周期调用 tick，tick 返回 false 时退出
    - 内部用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` 做绝对时间等待
    - 可选 `SetScheduler()` 设置 SCHED_FIFO（需 root 或 CAP_SYS_NICE）
    - 提供 `Stats()` 接口返回最大/平均/最近抖动
  - `src/realtime_loop.cpp`（新增）：实现
  - `src/main.cpp`：将 while+sleep_for 替换为 `RealtimeLoop::Run`
  - `CMakeLists.txt`：注册新源文件
- 注意：
  - `clock_nanosleep` 是 POSIX 接口，Linux 专用，需要 `#ifdef __linux__` 守卫，非 Linux 回退到 `sleep_for`
  - SCHED_FIFO 设置失败不应阻止启动，降级为普通调度并打 warning
  - 不改变现有 10ms 周期，���改等待机制

#### F10 — `test: add RealtimeLoop unit test`

- 工作量：小
- 依赖：F09
- 改动文件：
  - `test/test_realtime_loop.cpp`（新增）：
    - 验证周期回调被正确调用指定次数
    - 验证 tick 返回 false 时循环退出
    - 验证 Stats 返回合理值（不验证精确抖动，CI 环境无实时保证）

---

## GAP-3: AxisDriver 无独立单元测试（可测试性瓶颈）

- 严重程度：P1
- 文件：`include/canopen_hw/axis_driver.hpp`
- 现象：AxisDriver 是系统最复杂的类（380 行，含状态机驱动、PDO 读写、SDO 转发、EMCY/心跳处理、PDO 校验），但因为继承 `lely::canopen::BasicDriver`，构造需要真实的 `BasicMaster`，无法在没有 Lely 事件循环的环境下实例化。69 个测试中没有一个直接测试 AxisDriver。
- 影响：核心总线交互路径（OnRpdoWrite、OnEmcy、OnHeartbeat、OnBoot）只能通过集成测试或硬件测试覆盖。状态机逻辑虽然有独立测试，但 AxisDriver 中的"胶水逻辑"（读 PDO → 调状态机 → 写 PDO → 更新 SharedState）没有覆盖。
- 根因：AxisDriver 直接继承 Lely 具体类，没有抽象层隔离总线操作。

### 修复计划

#### F11 — `refactor: extract BusIO interface from AxisDriver`

- 工作量：中偏大
- 改动文件：
  - `include/canopen_hw/bus_io.hpp`（新增）：
    ```
    class BusIO {
     public:
      virtual ~BusIO() = default;
      virtual bool WriteControlword(uint16_t cw) = 0;
      virtual bool WriteTargetPosition(int32_t pos) = 0;
      virtual bool WriteTargetVelocity(int32_t vel) = 0;
      virtual bool WriteTargetTorque(int16_t torque) = 0;
      virtual bool WriteModeOfOperation(int8_t mode) = 0;
    };
    ```
  - `src/axis_driver.cpp`：AxisDriver 实现 BusIO 接口，Send 方法委托给 tpdo_mapped
  - `include/canopen_hw/axis_logic.hpp`（新增）：
    - `AxisLogic` 类，持有 `CiA402StateMachine`、`HealthCounters`、`AxisFeedback` 缓存
    - 构造参数：`BusIO*`、`SharedState*`、`axis_index`
    - `ProcessRpdo(statusword, actual_position, actual_velocity, actual_torque, mode_display)` 方法：原 InjectFeedback + 安全目标写入逻辑
    - `ProcessEmcy(eec, er)` / `ProcessHeartbeat(occurred)` 方法
  - `src/axis_driver.cpp`：OnRpdoWrite/OnEmcy/OnHeartbeat 委托给 AxisLogic
  - 注意：这是一个较大的重构，AxisDriver 从"什么都做"变成"Lely 适配器 + AxisLogic 委托"

#### F12 — `test: add AxisLogic unit tests with mock BusIO`

- 工作量：中
- 依赖：F11
- 改动文件：
  - `test/test_axis_logic.cpp`（新增）：
    - MockBusIO 记录所有写入调用
    - 测试 RPDO 处理：反馈 → 状态机推进 → 安全目标写入 → SharedState 更新
    - 测试 EMCY 处理：健康计数递增、反馈缓存更新
    - 测试心跳丢失/恢复：operational 状态切换
    - 测试 PDO 写入：验证 CSV 模式下 WriteTargetVelocity 被调用、CST 模式下 WriteTargetTorque 被调用

---

## GAP-4: 缺少根目录 README 和 API 文档（文档缺失）

- 严重程度：P2
- 现象：`docs/` 下有 7 个文档但组织松散，根目录没有 README。新人拿到代码不知道：项目是什么、怎么编译、怎么运行、架构是什么样的、API 入口在哪。
- 影响：上手成本高，代码审查效率低。

### 修复计划

#### F13 — `docs: add root README with build/run/architecture overview`

- 工作量：小
- 改动文件：
  - `README.md`（新增）：
    - 项目简介（一句话：基于 Lely 的 CANopen 伺服驱动层）
    - 编译：cmake 命令
    - 运行：命令行参数、joints.yaml 配置示例
    - 架构图：ASCII 图，标注 main → LifecycleManager → CanopenMaster → AxisDriver → SharedState → CanopenRobotHw 的数据流
    - 目录结构说明
    - 链接到 docs/ 下的详细文档

#### F14 — `docs: add API quick reference`

- 工作量：小
- 依赖：F13
- 改动文件：
  - `docs/api_reference.md`（新增）：
    - LifecycleManager：Init/Halt/Recover/Shutdown
    - CanopenRobotHw：Read/Write/SetJoint* ���列
    - SdoAccessor：Read/Write/WriteU8/U16/U32
    - DiagnosticsCollector：Collect
    - 每个接口一行签名 + 一句说明，不写长篇大论

---

## 修复优先级总表

| 顺序 | 编号 | GAP | 内容 | 严重程度 | 工作量 |
|------|------|-----|------|----------|--------|
| 1 | F07 | GAP-1 | PDO 写入 velocity/torque | P0 | 小 |
| 2 | F08 | GAP-1 | PDO 写入 mode_of_operation | P0 | 小 |
| 3 | F09 | GAP-2 | RealtimeLoop 封装 | P1 | 中 |
| 4 | F10 | GAP-2 | RealtimeLoop 测试 | P1 | 小 |
| 5 | F11 | GAP-3 | 抽取 BusIO + AxisLogic | P1 | 中偏大 |
| 6 | F12 | GAP-3 | AxisLogic 单元测试 | P1 | 中 |
| 7 | F13 | GAP-4 | 根目录 README | P2 | 小 |
| 8 | F14 | GAP-4 | API 快速参考 | P2 | 小 |

依赖关系：

```
F07 ──→ F08（模式字节应在目标值之前发送）
F09 ──→ F10（测试依赖实现）
F11 ──→ F12（测试依赖接口抽取）
F13 ──→ F14（API 文档引用 README 中的架构图）
四组之间互相独立，可并行推进。
```

建议执行顺序：先 F07+F08（堵上功能缺口，工作量最小收益最大），然后 F09+F10（实时性基础设施），F13+F14（文档，随时可做），F11+F12 放最后（重构工作量最大，但对正确性提升也最大）。
