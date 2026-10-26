# 工作报告：C06-C14 功能开发

- 日期：2026-03-19
- 范围：第二阶段（SDO 接口）、第三阶段（诊断收集）、第四阶段（多运动模式）
- 测试：69/69 通过，编译 0 warning

---

## C06 — SDO 读写接口（双层：异步底层 + 同步便捷）

目标：提供通用 SDO 读写能力，供上层诊断、参数配置等场景使用。

做了什么：
- 新增 `SdoAccessor` 类（`sdo_accessor.hpp` / `sdo_accessor.cpp`），提供两层接口：
  - 异步层：`AsyncRead` / `AsyncWrite`，回调在 Lely 事件线程执行
  - 同步层：`Read` / `Write` / `WriteU8` / `WriteU16` / `WriteU32`，内部用 `promise/future` 包装异步调用
- 新增 `SdoResult` 结构体，包含小端序便捷转换（`as_u8/u16/u32/i8/i16/i32`）
- `AxisDriver` 新增 `AsyncSdoRead` / `AsyncSdoWrite`，封装 Lely 的 `SubmitRead<uint32_t>` / `SubmitWrite`
- `CanopenMaster` 新增 `FindDriverByNodeId`，按 node_id 查找 AxisDriver 供 SdoAccessor 路由
- `CMakeLists.txt` 注册 `sdo_accessor.cpp`

改动文件：
- `include/canopen_hw/sdo_accessor.hpp`（新增）
- `src/sdo_accessor.cpp`（新增）
- `include/canopen_hw/axis_driver.hpp`（+include, +SDO 回调类型, +2 方法声明）
- `src/axis_driver.cpp`（+AsyncSdoRead/Write 实现）
- `include/canopen_hw/canopen_master.hpp`（+FindDriverByNodeId 声明）
- `src/canopen_master.cpp`（+FindDriverByNodeId 实现）
- `CMakeLists.txt`（+1 源文件）

---

## C07 — SDO 测试

目标：覆盖 SdoResult 类型转换正确性和 SdoAccessor 错误路径。

做了什么：
- 新增 `test/test_sdo_accessor.cpp`，12 个测试用例：
  - `SdoResult` 小端转换：u8、u16、u32、i8（-1）、i16（-32768）、i32（-1）
  - 空数据返回零、短数据 fallback
  - `SdoAccessor` null master 下的异步/同步读写均立即返回错误

改动文件：
- `test/test_sdo_accessor.cpp`（新增）
- `CMakeLists.txt`（+测试注册）

---

## C08 — DiagnosticsCollector 类

目标：提供结构化的整机诊断快照，与 ROS 解耦，便于后续适配 `diagnostic_msgs`。

做了什么：
- 新增 `DiagnosticsCollector` 类（`diagnostics_collector.hpp` / `diagnostics_collector.cpp`）
- 定义 `AxisDiagnostics`（单轴：名称、node_id、CiA402 状态、故障/心跳标志、EMCY 码、健康计数）
- 定义 `SystemDiagnostics`（整机：master 运行状态、轴数、全轴 operational 标志、轴列表）
- `Collect()` 方法从 `CanopenMaster` 拉取反馈和健康计数，组装快照

改动文件：
- `include/canopen_hw/diagnostics_collector.hpp`���新增）
- `src/diagnostics_collector.cpp`（新增）
- `CMakeLists.txt`（+1 源文件）

---

## C09 — 诊断测试

目标：验证 DiagnosticsCollector 的基本行为。

做了什么：
- 新增 `test/test_diagnostics_collector.cpp`，3 个测试用例：
  - null master 返回空诊断
  - 通过 SharedState 直接注入反馈验证 fault/operational 状态传递
  - AxisDiagnostics 默认值验证

改动文件：
- `test/test_diagnostics_collector.cpp`（新增）
- `CMakeLists.txt`（+测试注册）

---

## C10 — 模式常量扩展

目标：为 CSV（Cyclic Synchronous Velocity）和 CST（Cyclic Synchronous Torque）模式提供常量定义。

做了什么：
- `cia402_defs.hpp` 新增 `kMode_CSV = 9`、`kMode_CST = 10`
- 注释从 "Mode of operation for CSP" 改为 "Mode of operation"

改动文件：
- `include/canopen_hw/cia402_defs.hpp`（改 3 行）

---

## C11 — AxisCommand + 状态机适配

目标：让命令结构和状态机支持速度/力矩/模式字段，非运行态安全归零。

做了什么：
- `AxisCommand` 新增 `target_velocity`（int32_t��、`target_torque`（int16_t）、`mode_of_operation`（int8_t，默认 CSP）
- `AxisSafeCommand` 新增 `safe_target_velocity`、`safe_target_torque`、`safe_mode_of_operation`
- `CiA402StateMachine` 新增：
  - setter：`set_ros_target_velocity`、`set_ros_target_torque`
  - getter：`safe_target_velocity`、`safe_target_torque`、`safe_mode_of_operation`
  - 成员变量：`ros_target_velocity_`、`ros_target_torque_`、`safe_target_velocity_`、`safe_target_torque_`
- 状态机 `Update()` 中所有非 OperationEnabled 分支和退出运行态守卫均将 `safe_target_velocity_` / `safe_target_torque_` 置零
- `StepOperationEnabled` 在解锁后透传速度/力矩目标，锁定阶段归零

改动文件：
- `include/canopen_hw/shared_state.hpp`（AxisCommand +3 字段, AxisSafeCommand +3 字段）
- `include/canopen_hw/cia402_state_machine.hpp`（+6 方法, +4 成员变量）
- `src/cia402_state_machine.cpp`（所有状态分支 +2 行归零, StepOperationEnabled 扩展）

---

## C12 — AxisDriver PDO 写入适配

目标：让 AxisDriver 在 RPDO 回调中读取全部命令字段并传给状态机，写回全部安全目标。

做了什么：
- `OnRpdoWrite` 重构：从 `SharedState::GetCommand` 读取完整 `AxisCommand`，一次性设置 `ros_target`、`ros_target_velocity`、`ros_target_torque`、`target_mode`
- 安全目标回写扩展：读取 `safe_target_velocity` / `safe_target_torque`，留 TODO 标记 PDO 总线写入（0x60FF / 0x6071）
- `PublishSnapshot` 扩展：`AxisSafeCommand` 写入全部 4 个安全字段

改动文件：
- `src/axis_driver.cpp`（OnRpdoWrite 重构 ~20 行, PublishSnapshot +3 行）

---

## C13 — CanopenRobotHw 命令接口扩展

目标：让 ROS 硬件层可以设置速度/力矩/模式命令。

做了什么：
- 新增 3 个 setter：`SetJointVelocityCommand`、`SetJointTorqueCommand`、`SetJointMode`
- 新增 2 个反向换算：`RadPerSecToTicksPerSec`、`NmToTorquePermille`
- 构造函数初始化 `joint_vel_cmd_`、`joint_torque_cmd_`、`joint_mode_`
- `WriteToSharedState` 扩展：写入 `target_velocity`、`target_torque`、`mode_of_operation`

改动文件：
- `include/canopen_hw/canopen_robot_hw.hpp`（+4 方法声明, +3 成员变量, +2 私有方法声明）
- `src/canopen_robot_hw.cpp`（+4 方法实现, 构造函数 +3 初始化, WriteToSharedState +3 行）

---

## C14 — 多模式测试

目标：验证 CSV/CST 模式下状态机和 RobotHw 的完整行为。

做了什么：
- 新增 `test/test_multi_mode.cpp`，11 个测试用例：
  - 状态机：CSV 使能、CSV 速度透传、CST 力矩透传、故障归零、禁用归零、模式反映
  - RobotHw：速度命令写入 SharedState、力矩命令写入、默认模式 CSP、非 operational 阻断写入、无��轴忽略

改动文件：
- `test/test_multi_mode.cpp`（新增）
- `CMakeLists.txt`（+测试注册）

---

## 统计

| 指标 | 数值 |
|------|------|
| 新增源文件 | 4（sdo_accessor.hpp/cpp, diagnostics_collector.hpp/cpp） |
| 新增测试文件 | 3（test_sdo_accessor, test_diagnostics_collector, test_multi_mode） |
| 修改源文件 | 8 |
| 测试用例 | 43 → 69（+26） |
| 编译警告 | 0 |
