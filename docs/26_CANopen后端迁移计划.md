# Eyou_Canopen_Master 向 ros2_canopen 迁移计划

初版日期：2026-10-24，软件进度更新至 2026-10-25。本文是 [ROS1 迁移 ROS2 实施计划](25_ROS1迁移ROS2实施计划.md) 的 CANopen 专项执行清单，依据当前本地源码制定；当前实现、离线结果和现场待验项目分别记录，设备验收不由离线测试代替。

> 2026-10-25 进度：配置、模式/故障门控、设备维护、身份/PDO 校验和双后端协调的软件实现及空前缀离线验收已完成。记录见 [迁移执行记录](27_CANopen迁移执行记录.md)，操作单见 [软件交付与实机验收](28_CANopen软件交付与实机验收.md)。当前仅剩 M0 现场登记、真实设备验收及通过后的正式切换收尾。

## 1. 目标与版本

以 `ros2_canopen` 承担 ROS2 CANopen 主站、Lely 通信、CiA402 驱动和 `ros2_control` 硬件接口；把 `Eyou_Canopen_Master` 的设备配置、专用对象操作和安全语义迁入项目配置包及必要的最小扩展。最终由一个 `controller_manager` 同时管理 CANopen 和 `can_driver` 硬件组件，整车运行协调层负责双后端操作。旧包保留为 ROS1 行为基线和回退来源，不在 ROS2 正式链路中另起主站。

| 输入 | 离线验证的基础版本 | 用途 |
| --- | --- | --- |
| `Eyou_Canopen_Master` | `c890355` | 六轴配置、设备策略、ROS1 合同和回归用例 |
| `ros2_canopen` | `fef50e5`，`humble`，项目仓库 `fire007012/ros2_canopen` | ROS2 通用栈及待验证的硬件插件 |
| `can_driver` | `0e6ca8d` | 双后端轨迹与整车集成验证 |

2026-10-25 发布锁定：`ros2_canopen` 为 `78242f5c81809bac173051c28fe8154719f10d30`（`origin/humble`），`can_driver` 为 `0ba4e613e16fc233677db30ab7a07df1830e55ad`（`origin/main`）。上表保留离线验证时的基础提交，源码校验清单对应其上的已发布修改。

目标环境为 Ubuntu 22.04 / ROS2 Humble。旧 ROS1 工作空间与新 ROS2 构建目录分离；同一物理 CAN 总线在任何时刻只能有一个主站控制设备。

## 2. 功能归属与差异

| 旧包能力与依据 | ROS2 落点 | 完成判据 |
| --- | --- | --- |
| Lely 主站、NMT、SDO、PDO、SYNC、心跳；[`canopen_master.cpp`](../Eyou_Canopen_Master/src/canopen_master.cpp) | 上游 `canopen_core`、`canopen_base_driver`、`canopen_402_driver` | 六轴启动、对象读写、反馈、心跳和故障行为通过实机验证 |
| 全量、肩轴、摆臂三组 [`master*.yaml`](../Eyou_Canopen_Master/config/master.yaml) 和 [`joints*.yaml`](../Eyou_Canopen_Master/config/joints.yaml) | 新建 `robot_canopen_config` 保存 bus 配置、EDS、生成规则和关节映射 | 从仓库文件可重新生成所需 DCF/bin，输出不依赖开发机绝对路径 |
| ROS1 `RobotHW` 与单后端节点；[`CMakeLists.txt`](../Eyou_Canopen_Master/CMakeLists.txt) | `canopen_ros2_control` 的 `SystemInterface`，与 `can_driver` 共用管理器 | 状态/命令接口与控制器匹配，六轴 JTC 不需要两套主站 |
| 启动身份比对、PDO 映射校验、单轴诊断和 SDO 排队；[`axis_driver.cpp`](../Eyou_Canopen_Master/src/axis_driver.cpp) | 上游设备状态/诊断接口加项目校验与报告 | 型号/固件不符、PDO 不符、SDO 超时和 EMCY 有可定位的失败记录 |
| 肩轴 IP `0x60C1:01` 与 `0x607A` 镜像；[`position_channel_routing.hpp`](../Eyou_Canopen_Master/include/canopen_hw/position_channel_routing.hpp) | 上游 IP 模式加设备专用目标通道适配 | 5 ms 命令、halt/resume 和重连边沿无旧目标跳变 |
| 零点 `0x607C`、厂商限位 `0x2003`、范围 `0x607D`、保存 `0x1010:01`，含按 URDF 自动写限位；[`zero_soft_limit_executor.cpp`](../Eyou_Canopen_Master/src/zero_soft_limit_executor.cpp)、[`canopen_aux_services.cpp`](../Eyou_Canopen_Master/src/canopen_aux_services.cpp) | 项目设备扩展与受控维护接口，复用上游 SDO 通道 | 写入、读回、重启保持及失败恢复一致；软件 offset 不重复叠加 |
| Armed/Running、故障收敛、命令会话隔离；[`operational_coordinator.hpp`](../Eyou_Canopen_Master/include/canopen_hw/operational_coordinator.hpp)、[`axis_logic.cpp`](../Eyou_Canopen_Master/src/axis_logic.cpp) | 项目运行协调层和硬件命令门控 | 停机、故障和重新使能后不重放旧命令，反馈确认后才开放运动 |
| 可选 Ruckig IP `FollowJointTrajectory` executor | 先用统一 JTC；仅在确认独立入口仍被使用时迁移 | 不出现重复 Action 入口或双重插补 |

旧 ROS1 服务名和消息类型不是 ROS2 的二进制兼容目标；需要保留的业务语义由新的 ROS2 接口承接，并同步迁移 `Eyou_ROS1_Master`、`flipper_control` 等调用方。

## 3. 实施阶段

### M0：冻结基线与设备资料

1. 保存旧 ROS1 可运行提交、启动参数、六轴设备型号/固件/Node-ID、零点及限位持久化值、典型轨迹和故障日志。记录实际接线与 `canable0` 总线参数；配置文件不能替代现场确认。
2. 核对 [`YiyouServo_V1.4.eds`](../Eyou_Canopen_Master/config/YiyouServo_V1.4.eds)、[`YiyouServo_V1.4.dcfgen.eds`](../Eyou_Canopen_Master/config/YiyouServo_V1.4.dcfgen.eds) 与三组旧 DCF 的版本、对象定义和生成关系；给源文件及生成产物留校验值。
3. 从旧测试提取行为断言，至少覆盖 [`test_position_channel_routing.cpp`](../Eyou_Canopen_Master/test/test_position_channel_routing.cpp)、[`test_zero_soft_limit_executor.cpp`](../Eyou_Canopen_Master/test/test_zero_soft_limit_executor.cpp)、[`test_multi_mode.cpp`](../Eyou_Canopen_Master/test/test_multi_mode.cpp)、[`test_operational_coordinator.cpp`](../Eyou_Canopen_Master/test/test_operational_coordinator.cpp)。

**出口**：设备清单、旧配置和回归样本齐全；有可恢复的 ROS1 基线。设备持久化参数未记录前，不执行写零点或写限位实验。

### M1：构建上游并选定硬件插件

1. 在独立 Humble 工作空间构建锁定提交，运行上游可用的单元测试和虚拟 CAN 示例；记录依赖版本及失败项。先验证插件可加载，再接真实总线。
2. 对照 [`canopen_ros2_control.xml`](../ros2_canopen/canopen_ros2_control/canopen_ros2_control.xml) 实测 `RobotSystem` 和 `Cia402System` 的接口。`RobotSystem` 按配置将关节位置/速度接口映射到模式，适合作为优先候选；`Cia402System` 暴露显式模式命令接口。选择依据是控制器切换及安全语义的测试结果，而非插件名称。
3. 为模式切换建立最小回归：切到设备当前模式、仅停用控制器、CSP↔CSV、IP 位置控制、失败后状态恢复。锁定基线中的 [`perform_switch()`](../ros2_canopen/canopen_ros2_control/include/canopen_ros2_control/cia402_data.hpp) 在没有待启用接口时直接返回，且 [`set_operation_mode()`](../ros2_canopen/canopen_402_driver/include/canopen_402_driver/node_interfaces/node_canopen_402_driver_impl.hpp) 对已处于目标模式返回 `false`；先确认这些路径的实际结果，再决定最小补丁或项目层适配。
4. 核实插件内部 `DeviceContainer` 的生命周期、驱动类型约束和故障传播方式。[`RobotSystem`](../ros2_canopen/canopen_ros2_control/src/robot_system.cpp) 当前按 `ros2_canopen::Cia402Driver` 字符串识别驱动；若需要派生设备驱动，须先确认插件如何接纳它。正式硬件插件与单轴 service 调试不得同时打开同一 CAN 接口。

**出口**：选定一个硬件插件及接口清单；影响本项目的插件缺口有可复现测试和处理方案。虚拟总线通过不代表设备通过。

### M2：转换六轴配置

1. 新建 `robot_canopen_config`，从旧 `master.yaml` 迁出主站 Node-ID `127`、SYNC `5000 us`、心跳 `500 ms`，以及节点 `1` 至 `4` 的摆臂和节点 `6`、`5` 的肩轴。旧 `200 Hz` 控制环是对照值，不直接认定新链路能稳定达到。
2. 为四摆臂配置 CSV（模式 9）的 `0x60FF`，为两肩轴配置 IP（模式 7）的 `0x60C1:01`；逐项比对 `0x6040/0x6041`、`0x6060/0x6061`、`0x6064/0x606C`、`0x607A` 和 `0x6077` 的 PDO、COB-ID、传输类型及启动 SDO。旧配置 `verify_pdo_mapping=false`，所以生成成功不能代替设备读回核验。
3. 将摆臂 `19857408 counts/rev`、肩轴 `6619136 counts/rev` 转为上游所需的 SI↔设备比例，逐轴验证方向、零位和实际角度；不要把 counts/rev 直接填入比例字段。
4. 按上游 [`bus.yml` 示例](../ros2_canopen/canopen_tests/config/robot_control/bus.yml) 和生成工具生成配置，比较旧/新对象映射与启动写入；在 CI 或干净环境重复生成，禁止依赖本机绝对路径。

**出口**：六轴源配置、生成规则、映射比对表和可重复生成的产物；不通过配置工具的警告或设备对象读回差异不得进入运动测试。

### M3：迁移设备专用行为

1. 先验证上游 IP、CSP、CSV 的目标对象、模式反馈和单位。肩轴 IP 必须覆盖启动锁定、halt/release、模式往返和掉线恢复；若设备仍依赖双通道目标镜像，以最小设备扩展实现并附回归测试。
2. 通过当前唯一主站的 SDO 接口实现设置零点、软件限位和参数保存；保留数值范围检查、读回、持久化确认与失败恢复。旧配置的 `auto_write_soft_limits_from_urdf=true` 需单独验证启动写入时机、URDF 单位及越界失败处理。写设备零点与软件位置 offset 的先后关系须明确。
3. 将旧 `arm_epoch`、位置锁定、速度归零、心跳丢失后撤销运行意图等安全要求转成可测试的命令门控。上游硬件 `activate`、CiA402 Operation Enabled 和整车 Running 分别核对，不互相代称。
4. 对照旧启动身份检查、PDO 校验、诊断计数及 SDO 单飞行为，确认上游已有机制是否足够；缺少的项目诊断单独实现并提供故障注入测试。
5. 仅当统一 JTC 无法覆盖实际业务且确认旧独立 Action 仍被调用时，再迁移 Ruckig IP executor；其目标、取消、超时及故障处理单独验收。

**出口**：设备专用行为测试通过；任何运动恢复都由新命令触发，不自动重放故障前目标。

### M4：整车集成与交付

1. 将新的 CANopen 硬件组件与 `can_driver` 插件放入同一 `controller_manager`；迁移统一协调层的 init/enable/halt/resume/recover/shutdown 及失败收敛。`flipper_control` 的模式切换需与控制器资源切换串行，避免多个写入者。
2. 先做单轴低速运动，再做六轴 CANopen、多轴故障与同一 JTC 跨 CANopen/私有 CAN 后端的轨迹。覆盖取消、控制器停用、断线、EMCY、重启及一侧失败时另一侧停止。
3. 将旧 ROS1 启动、服务和上层依赖逐个替换；只有本节验收全部通过，才从 ROS2 工作空间移除 `Eyou_Canopen_Master`。旧提交、配置和实测记录继续保留供回退。

**出口**：第 4 节验收矩阵通过，ROS2 正式启动路径不依赖 ROS1 包或双主站。

## 4. 验收矩阵

| 范围 | 必须观察到的结果 |
| --- | --- |
| 配置与启动 | 六轴身份、EDS/DCF、PDO/SDO、SYNC 和心跳与现场设备一致；新环境可重建配置和加载插件 |
| 启动与诊断 | 错误设备身份、PDO 不符、SDO 超时、EMCY 和心跳丢失均能定位到轴与原因；不把未校验的设备标记为就绪 |
| 模式与运动 | 肩轴 IP、摆臂 CSV/CSP 及往返切换有真实模式反馈；位置/速度单位、方向和限位与旧基线一致 |
| 安全边沿 | 首次使能、halt/resume、控制器停用、掉线/重连、故障恢复不产生位置跳变或旧命令重放 |
| 维护操作 | 零点、软限位、读回和保存后的重启结果一致；错误参数不写设备，部分失败可识别并恢复 |
| 整车控制 | 一个管理器控制两硬件组件；统一六轴轨迹、取消及任一后端故障时的停止收敛通过 |
| 性能与诊断 | 记录控制周期、抖动、总线负载、掉线与 EMCY；以旧 200 Hz / 5 ms 配置为对照，由实测确定交付阈值 |

每项保留配置版本、设备/固件信息、日志或抓包、测试步骤及结果。代码层单测、虚拟 CAN、真实单轴和整车实测分别留记录，不相互替代。

## 5. 回退与提交顺序

回退前先停止 ROS2 控制进程并确认设备不再接收命令，再恢复 ROS1 checkout 和原设备配置；若迁移测试改写过设备零点、软限位或持久化参数，应按 M0 记录恢复并读回。只切换软件分支不足以保证设备回到原状态。

建议按以下独立变更提交：版本与基线记录 → 可生成的项目配置 → 虚拟 CAN 与插件选型测试 → 必要的上游最小补丁/设备扩展 → 单轴实机验证 → 双后端协调与整车验收 → 移除 ROS2 构建中的旧包。每一步都应能指出对应的验收记录和回退提交。
