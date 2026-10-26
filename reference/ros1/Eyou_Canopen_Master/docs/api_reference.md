# API Quick Reference

当前 API 的安全行为基线见：`docs/2026-03-25_现行安全行为规范.md`。
若与 `docs/archive/2026-04-19_deprecated/0324import.md` 存在状态语义差异，以现行安全行为规范为准。

## OperationalCoordinator

`SystemOpMode`（当前实现）：

- `Inactive`
- `Configured`
- `Standby`
- `Armed`
- `Running`
- `Faulted`
- `Recovering`
- `ShuttingDown`

| 方法 | 说明 |
|------|------|
| `SetConfigured()` | Configure 成功后设置起始模式 |
| `RequestInit()` | `Configured -> Armed`（启动主站并自动上电到冻结态；成功时推进一次 `command_sync_sequence`） |
| `RequestEnable()` | `Standby -> Armed`（进入使能冻结态；若存在 fault / heartbeat_lost / global fault 则拒绝） |
| `RequestDisable()` | `Running/Armed/Standby -> Standby`（去使能但保持通信在线） |
| `RequestHalt()` | `Running -> Armed` |
| `RequestRelease()` | `Armed -> Running`（对应 `~/resume`；健康门控同 `RequestEnable()`） |
| `RequestRecover()` | `Faulted -> Standby`（等待全部轴 `fault`/`heartbeat_lost` 清除后才成功；超时保持 `Faulted`） |
| `RequestShutdown()` | 通信停机并回到 `Configured`（成功时推进一次 `command_sync_sequence`） |
| `UpdateFromFeedback()` | 在 `Armed/Running` 下自动检测 fault/heartbeat 丢失并降级到 `Faulted` |
| `ComputeIntents()` | 按模式下发每轴 `AxisIntent`（控制主通道） |

## ServiceGateway

| Service | 内部转发 |
|------|------|
| `~/init` | `OperationalCoordinator::RequestInit()` |
| `~/enable` | `OperationalCoordinator::RequestEnable()` |
| `~/disable` | `OperationalCoordinator::RequestDisable()` |
| `~/halt` | `OperationalCoordinator::RequestHalt()` |
| `~/resume` | `OperationalCoordinator::RequestRelease()` |
| `~/recover` | `OperationalCoordinator::RequestRecover()` |
| `~/shutdown` | `OperationalCoordinator::RequestShutdown()` |

## LifecycleManager

当前定位：资源拥有者（`SharedState/CanopenMaster/CanopenRobotHw` 生命周期管理），
运行策略逐步迁移到 `OperationalCoordinator`。

| 方法 | 说明 |
|------|------|
| `Init(dcf_path, joints_path)` | 加载配置并启动主站，进入 Active |
| `Init(CanopenMasterConfig)` | 同上，直接传配置结构体 |
| `Configure(config)` | 仅构造对象，进入 Configured |
| `InitMotors()` | 启动通信与驱动流程（Configured -> Active） |
| `Halt()` | 轻量停转：置 Halt bit，保持 Active |
| `Resume()` | 清 Halt bit 恢复运动（global fault latch 期间会被拒绝） |
| `Recover()` | 仅对 fault 轴执行复位（不重启通信、不自动使能；需后续 `Resume()`） |
| `StopCommunication()` | 执行 402 降级 + 停通信，状态回 Configured |
| `Shutdown()` | 完全关闭，释放所有资源 |
| `robot_hw()` | 返回 CanopenRobotHw 指针 |

## CanopenMaster

| 方法 | 说明 |
|------|------|
| `Start()` | 启动主站与事件循环；启动前会校验 `master_dcf_path` 非空且文件存在，失败时返回 false |
| `GracefulShutdown(detail)` | 402 降级 + NMT Stop；超时返回 false 并给 detail |
| `HaltAll()` / `ResumeAll()` | 全轴置/清 Halt bit（保持 Operation Enabled） |
| `RecoverFaultedAxes(detail)` | 兼容入口，内部转发到 `ResetAllFaults` |
| `ResetAllFaults(detail)` | 故障复位执行器（发送 0x0080 复位脉冲并等待 fault 清除） |
| `EnableAxis/DisableAxis/ResetAxisFault` | 单轴手动控制接口 |
| `GetAxisFeedback(i, out)` | 读取单轴反馈快照 |

## CanopenRobotHw

| 方法 | 说明 |
|------|------|
| `ReadFromSharedState()` | 从 SharedState 读取全轴反馈快照 |
| `WriteToSharedState()` | 将目标命令写入 SharedState |
| `ApplyConfig(config)` | 应用轴配置（单位换算参数等） |
| `SetJointCommand(axis, rad)` | 设置单轴目标位置（弧度） |
| `SetJointVelocityCommand(axis, rad_s)` | 设置单轴目标速度（rad/s） |
| `SetJointTorqueCommand(axis, nm)` | 设置单轴目标力矩（Nm） |
| `SetJointMode(axis, mode)` | 设置单轴运动模式（CSP/CSV/CST） |
| `SetCommandReady(axis, ready)` | 写入命令有效标志（AxisCommand.valid） |
| `SetCommandEpoch(axis, epoch)` | 写入命令会话号（AxisCommand.arm_epoch） |
| `arm_epoch(axis)` | 读取反馈会话号（AxisFeedback.arm_epoch） |
| `command_sync_sequence()` | 读取命令重同步序列；用于显式判定“旧命令源必须失效并重对齐” |
| `all_axes_halted_by_fault()` | 读取全轴故障连带停机标志 |

## CanopenRobotHwRos

| 方法 | 说明 |
|------|------|
| `read()` | 观测 `command_sync_sequence`、`arm_epoch` 变化沿或 fault-halt 上升沿；触发时强制 `pos_cmd = pos`、`cmd_ready = false` 并重置 guard |
| `write()` | 将 `valid/arm_epoch` 写回 SharedState；guard 未结束或 fault-halt 期间保持 `valid = false` |

## IpFollowJointTrajectoryExecutor

位置：
- `include/canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp`
- `src/controllers/ip_follow_joint_trajectory_executor_*.cpp`

关键语义：
- 一个 action goal 覆盖全轴（标准 `FollowJointTrajectory` 语义）。
- 关节顺序可与配置不同，内部做 `goal joint -> config axis` 映射。
- 动态 DOF：`Ruckig<0>`，轴数来自 `joints.yaml`。

| 方法 / 配置 | 说明 |
|------|------|
| `Config.joint_names/joint_indices` | 执行器覆盖的关节与轴索引（由节点从 `master_cfg.joints` 聚合） |
| `Config.max_velocities/max_accelerations/max_jerks` | 每轴 Ruckig 约束 |
| `Config.goal_tolerances` | 每轴终点容差 |
| `ValidateGoal()` | 校验关节集合、轨迹点尺寸、时间单调性 |
| `startGoal()` | 从当前实际状态起步，初始化 Ruckig |
| `step()` | 周期推进；多段轨迹切换；终点保持与完成判定 |
| `cancelGoal()` | 清理活动目标并重置执行状态 |
| `update()` | 节点循环入口：读取实际状态 -> 执行一步 -> 回写外部位置命令 |

## AxisLogic

| 方法 | 说明 |
|------|------|
| `ProcessRpdo(sw, pos, vel, torque, mode)` | RPDO 周期处理：反馈→状态机→写总线 |
| `ProcessEmcy(eec, er)` | EMCY 处理，递增健康计数 |
| `ProcessHeartbeat(lost)` | 心跳丢失/恢复处理 |
| `Configure(threshold, max_resets, hold)` | 配置状态机参数 |
| `SetRosTarget(pos)` | 设置目标位置 |
| `SetRosTargetVelocity(vel)` | 设置目标速度 |
| `SetRosTargetTorque(torque)` | 设置目标力矩 |
| `SetTargetMode(mode)` | 设置运动模式 |
| `SetExternalCommand(cmd)` | 写入上层命令包（target/mode/valid/epoch） |
| `SetGlobalFault(fault)` | 注入全局故障闩锁输入 |
| `RequestEnable()` / `RequestDisable()` | 使能/去使能 |
| `RequestHalt()` / `RequestResume()` | 置/清 Halt bit |
| `ResetFault()` | 复位故障 |

## SharedState 协议字段

`AxisCommand`：

| 字段 | 说明 |
|------|------|
| `target_position/velocity/torque` | 上层期望目标（工程量已换算为设备单位） |
| `mode_of_operation` | 目标模式（CSP/CSV/CST） |
| `valid` | 上层命令源是否完成重同步并声明可用；ROS 适配层在重同步窗口内会主动拉低 |
| `arm_epoch` | 目标所属使能会话号（`0` 永远无效；与反馈侧会话号不一致时不会透传） |

`SharedSnapshot`：

| 字段 | 说明 |
|------|------|
| `global_fault` | 任一轴故障后置位的全局闩锁 |
| `all_axes_halted_by_fault` | 全轴因故障被连带冻结标志 |
| `intents` | 每轴 `AxisIntent`（当前 shadow 使用） |
| `intent_sequence` | intent 更新序列号 |
| `command_sync_sequence` | 命令重同步序列；成功的 `RequestInit()` / `RequestRecover()` / `RequestShutdown()` 会递增 |

## BusIO (接口)

| 方法 | CiA 402 对象 |
|------|-------------|
| `WriteControlword(cw)` | 0x6040 |
| `WriteTargetPosition(pos)` | 0x607A |
| `WriteTargetVelocity(vel)` | 0x60FF |
| `WriteTargetTorque(torque)` | 0x6071 |
| `WriteModeOfOperation(mode)` | 0x6060 |

## SdoAccessor

| 方法 | 说明 |
|------|------|
| `Read(index, subindex)` | 同步 SDO 读（返回 SdoResult） |
| `Write(index, subindex, data)` | 同步 SDO 写 |
| `WriteU8/U16/U32(index, sub, val)` | 类型化同步写 |

## DiagnosticsCollector

| 方法 | 说明 |
|------|------|
| `Collect()` | 收集全轴诊断信息（健康计数、状态等） |

## RealtimeLoop

| 方法 | 说明 |
|------|------|
| `Run(tick)` | 执行周期循环，tick 返回 false 时退出 |
| `stats()` | 返回 LoopStats（max/avg/last jitter, iterations） |

## 运动模式常量

| 常量 | 值 | 说明 |
|------|----|------|
| `kMode_IP` | 7 | Interpolated Position |
| `kMode_CSP` | 8 | Cyclic Synchronous Position |
| `kMode_CSV` | 9 | Cyclic Synchronous Velocity |
| `kMode_CST` | 10 | Cyclic Synchronous Torque |
