# ROS1 迁移 ROS2 实施计划

编写日期：2026-10-24。本文依据当前检出的源码、配置及启动文件制定，是后续实施计划，不表示迁移已经完成。

## 1. 迁移目标与关键决策

将当前 ROS Noetic / Catkin 工程迁移为 ROS2 工程，保留整车实机、仿真、机械臂规划、摆臂控制、底盘、夹爪及上位机功能。

**CANopen 是本次迁移的前置重点：由现有 `Eyou_Canopen_Master` 自研后端切换到 ROS-Industrial 维护的 `ros2_canopen`，优先复用其 CiA402 驱动和 `canopen_ros2_control` 接口。** 不把“将旧 CANopen 包改成 ament 后继续使用”作为正式交付路线。

本文将用户所说的“官方支持的 CANopen 包”具体对应到 [`ros-industrial/ros2_canopen`](https://github.com/ros-industrial/ros2_canopen)。它是 ROS-Industrial 维护的 ROS2 CANopen 框架；具体电机兼容性、模式能力和故障行为仍须经过本项目设备验证。

建议执行以下决策：

1. 采用 **Ubuntu 22.04 + ROS2 Humble + MoveIt 2 + ros2_control + Gazebo Fortress** 作为迁移基线；Ubuntu 和 ROS2 版本按项目要求固定，阶段 0 验证并锁定配套依赖版本。
2. 保留一个正式 `controller_manager`，在同一 Resource Manager 中加载两个硬件组件：`can_driver` 的 ROS2 硬件插件和上游 CANopen 硬件插件。
3. 保留统一整车运行态及生命周期操作入口，但将旧统一层从“组合两个 ROS1 RobotHW”改为“双硬件组件的运行协调与兼容接口”。
4. 先完成 CANopen IP 肩轴、CSV/CSP 摆臂与跨后端机械臂的技术验证，再投入整车功能迁移。
5. ROS1 保留为可回退基线；ROS2 在独立工作空间构建，最终正式控制链不依赖 ROS1 桥。

Ubuntu 22.04 与 ROS2 Humble 是本项目确定的目标环境。仿真配套建议使用 Gazebo Fortress，依据见 [Gazebo 官方 ROS 配套说明](https://github.com/gazebosim/docs/blob/master/common/ros_installation.md) 和 [gz_ros2_control Humble 兼容矩阵](https://github.com/ros-controls/gz_ros2_control/blob/humble/README.md)。设备 SDK、插件和系统库仍需在此环境验证；不能直接套用其他 ROS2 发行版的 API、配置和安装命令。

## 2. 当前项目基线与阅读结论

### 2.1 现有正式架构

主要依据：

- [`robot_bringup/launch/full_system.launch`](../robot_bringup/launch/full_system.launch)
- [`robot_bringup/launch/full_system_hardware.launch`](../robot_bringup/launch/full_system_hardware.launch)
- [`robot_bringup/launch/full_system_simulate.launch`](../robot_bringup/launch/full_system_simulate.launch)
- [`Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp`](../Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp)
- [`Eyou_ROS1_Master/src/hybrid_robot_hw.cpp`](../Eyou_ROS1_Master/src/hybrid_robot_hw.cpp)
- [`Eyou_ROS1_Master/config/controllers_jtc.yaml`](../Eyou_ROS1_Master/config/controllers_jtc.yaml)
- [`20_电机生命周期管理约定_2026-04-15.md`](20_电机生命周期管理约定_2026-04-15.md)

```text
robot_bringup
  ├─ Eyou_ROS1_Master / hybrid_motor_hw_node
  │    ├─ CanDriverHW                    → 私有 CAN 协议电机
  │    ├─ CanopenRobotHwRos              → 自研 CANopen 后端
  │    ├─ HybridOperationalCoordinator   → 统一状态迁移、回滚
  │    └─ 一个 controller_manager        → 机械臂、摆臂、底盘、夹爪
  ├─ mobility_control / flipper_control / arm_control
  ├─ MoveIt / MoveIt Servo / RViz
  └─ 视觉、CO₂ 等外围节点
```

现有主循环按 `read → controller_manager.update → write` 执行，顶层硬件启动默认 `loop_hz=200.0`；迁移时需实测周期、抖动和总线容量，不能将默认值直接视为设备能力证明。

### 2.2 不能沿用的旧方案和描述

- [`07_开发步骤.md`](07_开发步骤.md) 描述的是早期双控制器管理器、轨迹按 4+2 拆分、`arm_joint_1` 等命名方案。当前统一硬件层已经改变了这些前提，不能据此重建迁移架构。
- 当前机械臂关节名为 `shoulder_yaw_joint`、`shoulder_pitch_joint`、`elbow_pitch_joint`、`wrist_pitch_joint`、`wrist_yaw_joint`、`wrist_roll_joint`。
- 摆臂关节名为 `left_front_arm_joint`、`right_front_arm_joint`、`left_rear_arm_joint`、`right_rear_arm_joint`。
- 当前夹爪节点和主仓库默认控制器采用速度控制：北向 `Bool / Float64`，南向 `std_msgs/Float64`；但子模块关节配置为 csp，存在接口冲突，详见第 2.4 节。部分 README 仍描述位置或轨迹控制，不能直接作为迁移合同。
- 当前摆臂 CSV 控制器是自定义插件，包含超时归零、非有限数值检查、停用归零；不能直接替换为普通速度转发控制器后丢失这些行为。
- 当前仿真包含 Gazebo Classic 控制和传感器插件，底盘链路还涉及 skid-steer 插件；需迁移整个仿真执行链。

### 2.3 子模块恢复与源码核查基线

这些目录此前为空是因为子模块尚未初始化。此前按父仓库 gitlink 锁定提交拉取；之后手眼标定已替换为本地 `easy_handeye2`，RealSense 已切换到 ROS2 `ros2-master` 分支提交 `7a87097`，ArUco 已切换到 ROS2 `humble-devel` 分支提交 `2872a00`。Orbbec 已从旧 ROS1 路径替换为 `OrbbecSDK_ROS2`，仅在父仓库索引中记录远端 `v2-main` 提交 `c153462`，仍未下载或核查源码。`can_driver` 已切换到 ROS2 `main` 分支提交 `0e6ca8d`，`ros2_canopen` 已从其本地 `humble` 分支锁定提交 `fef50e5` 并推送到项目仓库，`Eyou_Canopen_Master` 仍保持原锁定提交。以下是当前本地工作区与索引的计划集成版本，统一提交后才形成可复现的父仓库基线；`.gitmodules` 中的分支配置不覆盖 gitlink 锁定的提交。

| 子模块 | 当前本地指针（短哈希） | 本次核查重点 |
| --- | --- | --- |
| `Eyou_Canopen_Master` | `c890355` | 六轴配置、Lely 主站、IP 通道、归零与软限位 |
| `can_driver` | `0ba4e61`（ROS2 `main`；原 ROS1 提交 `cfe7174`） | Humble 构建、SDK 安装及双后端离线验收通过；实机待验 |
| `ros2_canopen` | `78242f5c`（项目仓库 `humble`） | RobotSystem 软件路径和离线回归通过；设备功能实机待验 |
| [`realsense-ros`](https://github.com/realsenseai/realsense-ros) | `7a87097`（`4.59.1`；原 ROS1 提交 `2e1ecd5`） | 已切换为 ROS2 `realsense2_camera 4.59.1`，Humble 构建与实机集成待验证 |
| [`OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2) | `c153462`（gitlink 已记录，源码未拉取；原 ROS1 提交 `86ab046`） | 已选定 ROS2 仓库，尚未初始化；Humble、Astra+ 型号与 SDK 兼容性待核查 |
| [`aruco_ros`](https://github.com/pal-robotics/aruco_ros) | `2872a00`（`5.0.6`；原 ROS1 提交 `3680aea`） | 已切换为 ROS2 `aruco / aruco_msgs / aruco_ros 5.0.6`，Humble 构建与标定集成待验证 |
| `easy_handeye2` | `29bd50a`（本地新增 gitlink） | ROS2 `easy_handeye2 / easy_handeye2_msgs 0.5.0`，替换原 `easy_handeye 0.4.3` |

已获取两个自研驱动、RealSense、ArUco 和新的 ROS2 手眼标定子模块配置及实现，原“两驱动源码缺失”的限制不再作为计划前提。Orbbec 目前只锁定 ROS2 仓库与远端提交；具体 SDK 版本、包依赖、支持设备和源码改动仍待后续初始化后核查，本轮不自动下载。尚待现场确认的是实际接线、运行时覆盖参数、具体电机型号/固件、零点持久化状态及实时性能；配置文件反映仓库基线，不等于现场状态已验证。

### 2.4 从子模块确认的实际轴分配

依据 [`joints.yaml`](../Eyou_Canopen_Master/config/joints.yaml)、[`master.yaml`](../Eyou_Canopen_Master/config/master.yaml) 和 [`can_driver.yaml`](../can_driver/config/can_driver.yaml)：

| 后端 / 接口 | 关节 | 节点或电机 ID | 当前配置模式 |
| --- | --- | --- | --- |
| CANopen / `canable0` | `left_front_arm_joint` | 1 | CSV（9） |
| CANopen / `canable0` | `right_front_arm_joint` | 2 | CSV（9） |
| CANopen / `canable0` | `left_rear_arm_joint` | 3 | CSV（9） |
| CANopen / `canable0` | `right_rear_arm_joint` | 4 | CSV（9） |
| CANopen / `canable0` | `shoulder_yaw_joint` | 6 | IP（7） |
| CANopen / `canable0` | `shoulder_pitch_joint` | 5 | IP（7） |
| PP / `canable1` | `elbow_pitch_joint` | `0x13` | csp，方向修正 -1 |
| PP / `canable1` | `wrist_pitch_joint` | `0x14` | csp |
| PP / `canable1` | `wrist_yaw_joint` | `0x15` | csp，方向修正 -1 |
| PP / `canable1` | `wrist_roll_joint` | `0x16` | csp |
| PP / `canable1` | `left_gripper_finger_joint` | `0x17` | csp，直线位移换算 |
| DM / `canable1` | `left_front_wheel_joint` | `0x07`，反馈 master ID `0x08` | velocity |
| DM / `canable1` | `right_front_wheel_joint` | `0x05`，反馈 master ID `0x06` | velocity，方向修正 -1 |

这里的私有协议 `PP` 是后端协议标识，不能与 CiA402 的 Profile Position 模式混为一谈。

**夹爪需要先统一接口合同**：主仓库默认控制器与上层节点走速度语义，但子模块默认关节配置为 csp，且 URDF 为 prismatic。源码 `CanDriverHW::init` 按控制模式注册接口，`AxisCommandSemantics.h` 仅将 velocity 认定为速度语义，csp 则注册位置接口。因此在默认配置组合下，速度控制器请求与硬件注册接口不匹配。迁移前须决定使用设备速度模式，还是通过受限速度积分生成 csp 位置参考，并同步更新硬件、控制器、超时和限位策略；仅改消息类型无法解决。ROS 标准单位应为 m、m/s，不能直接沿用上层注释中的 rad/s。仓库给出的位移换算系数为 `1.7807297172304439e-6 m/count`，需对照实测行程验证。

## 3. 建议的 ROS2 目标架构

```text
HMI / TCP-JSON / 遥控 / 任务调用
                │
                ▼
arm_control / mobility_control / flipper_control / MoveIt 2 / Servo
                │
                ▼
一个 controller_manager + Resource Manager
  ├─ arm_position_controller：统一六轴 FollowJointTrajectory
  ├─ flipper_csp_controller / flipper_csv_forward_controller：互斥
  ├─ wheel_controller
  ├─ gripper_controller
  └─ joint_state_broadcaster
                │
        ┌───────┴──────────────────┐
        ▼                          ▼
can_driver ROS2 SystemInterface    ros2_canopen 上游硬件插件
        │                          │
私有 CAN 协议电机                   CiA402 驱动 / CANopen 主站

统一运行协调模块（迁移现有 facade 的职责）
  ├─ 双后端 init/enable/halt/resume/recover/shutdown 编排
  ├─ 模式切换、失败收敛、运行许可、关节状态归一化
  └─ 保持上层只有一个正式操作入口
```

关键约束：

- 两个 SystemInterface 由同一个 Resource Manager 管理，不再把上游 CANopen 插件包进旧 `HybridRobotHW`。
- 一个轨迹控制器跨两个硬件组件领取六轴接口，作为替代原统一 RobotHW 的关键技术验证；首选方案仍不需要轨迹拆分器。
- 上游 CANopen 插件若在内部创建 DeviceContainer，就不再对同一总线另起独立 CANopen 主站。
- CAN 总线上的每组电机只能有一个命令写入者；旧 ROS1 驱动与新 ROS2 驱动不能同时控制相同设备。
- 普通 LifecycleNode、ros2_control 硬件组件状态、CANopen NMT 状态、CiA402 状态、整车业务运行态分别记录，不能互相等同。
- 协调模块属于现有统一层职责的迁移，不额外增加一份整车生命周期状态机。建议以库或组件形式组织，部署形式在技术验证后决定。

## 4. CANopen 替换专项计划（最高优先级）

两套驱动的逐阶段迁移、功能归属和验收清单见 [CANopen 后端迁移计划](26_CANopen后端迁移计划.md)。

2026-10-25 更新：CANopen 专项的软件实现及离线验收已完成，`ros2_canopen` 发布到 `78242f5c`，配套 `can_driver` 发布到 `0ba4e61`；主仓库同步锁定这两个版本。现场登记与实机验收见 [操作单](28_CANopen软件交付与实机验收.md)，完整证据见 [执行记录](27_CANopen迁移执行记录.md)。

### 4.1 替换范围与保留范围

| 内容 | ROS2 迁移策略 |
| --- | --- |
| 自研 CANopen 主站、协议通信、CiA402 通用驱动 | 切换到 `ros2_canopen` 上游能力 |
| 自研 CANopen ROS1 RobotHW 适配 | 采用 `canopen_ros2_control` 的合适插件 |
| EDS、设备对象字典、厂商手册 | 作为设备资料保留，逐项核对 |
| 当前 DCF、关节 YAML | 提取配置语义，转换到上游配置体系并重新生成产物 |
| 零点、方向、减速比、限位、分组及故障策略 | 保留项目需求，验证上游支持后放到正确层 |
| 双后端事务一致性、整车生命周期接口 | 在统一协调层适配和验证 |
| 电机特有功能 | 优先配置或最小设备扩展，避免重新实现通用协议栈 |

上游提供配置包及硬件接口机制，参考 [配置说明](https://ros-industrial.github.io/ros2_canopen/manual/humble/user-guide/configuration.html) 和 [机器人系统接入教程](https://ros-industrial.github.io/ros2_canopen/manual/humble/user-guide/how-to-create-a-robot-system.html)。实施以 Humble 文档和 `humble` 分支为依据，并锁定具体提交；不能将其他发行版的配置示例直接复制到本项目。

### 4.2 先锁定上游版本和接入模式

- [ ] 验证 Humble 下 `canopen_402_driver`、`canopen_ros2_control`、`canopen_interfaces`、`lely_core_libraries` 的可安装或可构建版本。
- [x] 从上游 `humble` 分支锁定 `fef50e5` 并推送到项目仓库，主仓库通过 gitlink 固定提交；后续升级需单独验证。
- [ ] 阅读该版本的 plugin XML、硬件插件源码、配置生成宏和测试示例，确认真实插件名与参数。
- [ ] 确认插件是否内置设备容器，以及是否支持所需的普通/受管理驱动组合。
- [ ] 优先选择 ros2_control 接入；独立 service 接入用于单轴排查和功能验证，不能与正式硬件插件同时抢占设备。

特别注意：[Humble 硬件接口文档](https://ros-industrial.github.io/ros2_canopen/manual/humble/user-guide/operation/ros2-control-interface.html) 列出了 `Cia402RobotSystem`，并将该接口标为开发中；文档还说明该接入方式使用非 lifecycle 驱动。需要以锁定的 [humble 分支源码和插件声明](https://github.com/ros-industrial/ros2_canopen/tree/humble/canopen_ros2_control) 核实实际能力、插件名、关节绑定参数及初始化方式。硬件/控制器激活与电机就绪之间的关系必须实测，不能套用其他分支的 `RobotSystem` 实现，也不能将“上游维护”视为本项目设备已验证。

### 4.3 设备配置迁移清单

每个实际轴建立一行配置映射表，至少包括：

| 项目 | 要核查的内容 | 验收依据 |
| --- | --- | --- |
| 身份与寻址 | 关节名、物理电机、Node-ID、CAN 接口、固件版本 | 接线和实际反馈一致 |
| EDS/DCF | 厂商文件、对象类型、读写属性、版本 | 配置生成和设备启动成功 |
| 位置换算 | 编码器计数、减速比、方向、零位 | 已知角度与 ROS 弧度一致 |
| 速度换算 | 对象单位、比例系数、方向 | 已知转速与 rad/s 一致 |
| 工作模式 | 机械臂实际模式、摆臂 CSP/CSV、支持情况 | 模式反馈与命令一致 |
| 总线通信 | PDO 映射、COB-ID、传输类型、SYNC、心跳 | 抓包、反馈刷新率、总线负载 |
| 限位和归零 | 软限位、硬限位、零点持久化、重启行为 | 越界与重启用例 |
| 异常处理 | EMCY、心跳丢失、驱动故障、恢复 | 运行态与实际电机状态一致 |

将现有 `master.dcf / joints.yaml` 的内容拆分为上游 `bus.yml`、设备 EDS/DCF、启动 SDO 配置、关节映射及项目限位配置。最终目录建议采用新包 `robot_canopen_config`，集中保存源配置和生成规则；生成后的主站文件必须能通过构建重现，不依赖开发机绝对路径。

不要直接套用上游示例的 Node-ID、EDS、SYNC 周期和模式参数。总线周期需要同时匹配设备要求、控制循环及实测负载。

### 4.4 从旧 CANopen 实现提取的专项任务

源码和配置已确认以下行为，切换上游栈时逐项建立对照用例：

1. **IP 肩轴不是 CSP 肩轴**：肩部两轴默认模式为 7。旧 [`position_channel_routing.hpp`](../Eyou_Canopen_Master/include/canopen_hw/position_channel_routing.hpp) 在 IP 下写 `0x60C1:01`，同时镜像 `0x607A`，用于避免 halt/release 边沿目标跳变。上游 Humble [CiA402 驱动文档](https://ros-industrial.github.io/ros2_canopen/manual/humble/user-guide/cia402-driver.html) 提供固定时间线性 IP；是否满足本设备及边沿语义需实测，不能因存在 IP 接口就视为等价。若决定改为 CSP，须另行验证设备支持与轨迹质量，不作为默认替换。
2. **换算方向要反查**：四摆臂 `counts_per_rev=19857408`，肩部为 `6619136`。旧配置是 counts/rev，上游采用 SI→device 和 device→SI 的比例配置，需按实际单位构造互逆换算，禁止直接把 counts/rev 填进比例字段。
3. **5ms 时序与心跳**：主站 Node-ID 为 127，SYNC 为 `5000µs`，运行环为 `200Hz`，IP 插补周期为 `5ms`；主从心跳配置为 `500ms`，心跳倍率 3。新配置以这些值为对照，并实测总线负载和超时，不直接认定上游默认值等效。
4. **PDO 分模式迁移**：旧映射包含控制字/状态字、模式命令/反馈、实际位置/速度/力矩；IP 数据通过 `0x60C1:01`，摆臂速度通过 `0x60FF`。肩轴原速度 RPDO 被禁用，不能直接复用摆臂 PDO 配置。当前 `verify_pdo_mapping=false`，需额外核对实际设备映射和生成结果。
5. **设备专用限位和持久化**：旧 [`zero_soft_limit_executor.cpp`](../Eyou_Canopen_Master/src/zero_soft_limit_executor.cpp) 涉及 `0x607C` 零位、`0x2003` 厂商软限位使能（魔数 `0x4C494D54`）、`0x607D` 范围及 `0x1010:01` 参数保存。不能假定上游通用限位参数会完成这些写入。保留必要设备扩展、读回校验及失败恢复，明确软件 offset 和设备零点不能重复叠加。
6. **配置生成链复用资料**：仓库同时带有 `YiyouServo_V1.4.eds` 和 `YiyouServo_V1.4.dcfgen.eds`，并有整车、肩部两轴、四摆臂三组配置。核对两版 EDS 差异后迁移生成规则；原 DCF/bin 仅作比对和回退，不直接当作上游生成产物。
7. **独立 IP executor 与统一 JTC 分开处理**：旧包还包含可选的 Ruckig IP FollowJointTrajectory executor；主仓库正式统一链路使用 JTC。先迁移正式链路；只有确定独立 IP executor 仍被使用时，才迁移其 Action 和轨迹生成，避免与 JTC 重复插补、重复占用 Action 名称。
8. **命令会话隔离**：旧 `axis_logic.cpp` 带 `arm_epoch/cmd_arm_epoch` 及缓存清理。上游接入后需通过同等会话门控或其他机制确保 halt、故障、重新使能后拒绝旧会话命令，不能只验证 topic 超时。

### 4.5 最小技术验证顺序

1. **虚拟总线与构建验证**：通过上游模拟从站/测试验证配置生成、插件加载及状态接口；只能证明软件链路，不能代替电机验证。
2. **真实单轴通信**：完成 boot、NMT、SDO 读取、PDO 反馈、心跳和故障读取；此阶段不要求整车运行。
3. **肩部 IP 单轴与摆臂 CSV 单轴低速运动**：分别验证两类目标通道、单位、方向、初始参考值、停止行为及掉线处理，不能只用一种模式代表全部六轴。
4. **摆臂 CSP/CSV 验证**：验证位置与速度接口切换是否真正触发预期 CiA402 模式，覆盖切换失败和重启。
5. **多轴 CANopen 验证**：核查所有目标轴、SYNC 时序、总线负载、单轴故障对同组及其他轴的影响。
6. **跨后端控制验证**：同一 JTC 领取 `can_driver` 和 CANopen 两侧关节，执行六轴轨迹并检查时间一致性、取消和失败停止。

**阶段门槛：只有以上验证成立，才将 `ros2_canopen` 集成方案确认为整车正式基线。** 若缺少能力，先定位到配置、设备扩展或上游缺陷并安排最小补丁；不默认回到完整自研 CANopen 栈。

### 4.6 必须补齐的语义差异

现有整车接口有 `init → enable → resume`，其中 Armed 和 Running 的区别需要明确保留。上游硬件组件的 `activate` 可能包含电机初始化，不能直接把硬件 active 宣布为整车 Running。

| 整车操作 | 要完成的迁移工作 |
| --- | --- |
| `init` | 确认设备通信与状态收敛，定义是否涉及电机上电/使能 |
| `enable` | 双后端推进到允许准备运动的状态，确认真实反馈 |
| `resume` | 开放运动命令；初始位置取当前值，速度清零，丢弃旧命令 |
| `halt` | 按设备定义冻结/停止，确认位置持位或零速度的实际效果 |
| `disable` | 关闭运行许可并执行设备对应的退出使能动作 |
| `recover` | 错误复位并重新验证就绪，禁止自动重放故障前目标 |
| `shutdown` | 停止控制并释放资源，验证两侧结果 |

上述为需实现和测试的项目语义，不是对上游同名服务行为的假定。

协调器必须读取两个后端真实结果；一侧成功而另一侧失败时，执行回滚或停止收敛，并记录失败轴、后端和原因。不能只将几个异步服务调用串起来后就宣称完成原有事务语义。

CSP ↔ CSV 的推荐切换事务：关闭运动许可 → 停止当前命令/轨迹 → 停用冲突控制器 → 请求设备模式并等反馈 → 初始化新模式参考值 → 激活目标控制器 → 重新确认运行许可。若上游插件自身负责模式切换，该事务应与插件的切换机制衔接，避免两个模块同时写模式。

## 5. 各包迁移任务表

| 当前包或目录 | 计划动作 | 优先级 |
| --- | --- | --- |
| `Eyou_Canopen_Master` | 按已确认六轴、IP/CSV/PDO 和厂商对象清单转换；用上游 `ros2_canopen` 替换通用后端，新建 `robot_canopen_config` | P0 |
| `can_driver` | ROS2 包和 `hardware_interface::SystemInterface` 已提交；验证 Humble 构建、SDK 依赖、接口语义及双后端集成 | P0 |
| `Eyou_ROS1_Master` | 建议迁移为小写包 `hybrid_hardware`；保留协调器、运行态和服务语义，移除旧 RobotHW 聚合 | P0 |
| 各包自定义 msg/srv/action | 使用 rosidl 生成；建议统一运行态接口抽入 `robot_interfaces`，其他接口按包保留或拆分 | P0 |
| `car_urdf` | ament 安装资源；拆为基础模型、硬件 ros2_control、仿真配置，复用 mesh 和关节定义 | P1 |
| `flipper_control` | 迁移管理节点、服务、参考生成器及自定义 CSV 插件；适配状态与控制器切换接口 | P1 |
| `mobility_control` | 迁移 C++ 正式节点，保留限幅、加速度/jerk、方向修正和超时；Python 旧实现核对后再决定保留 | P1 |
| `arm_control` | 迁移 Action、夹爪、TF 桥、腕部服务；替换 `moveit_commander` | P1 |
| `car_moveit_config` | 基于现有模型/SRDF 重建 MoveIt 2 启动及控制器配置，先 OMPL 再验证其他规划器 | P1 |
| `car_control` | 盘点仍被引用的 Servo、遥控、仿真里程计节点；迁移有效功能，避免复制重复节点 | P1 |
| `robot_bringup` | 重写 launch、参数、事件和就绪检查，提供 mock/仿真/实机入口 | P1 |
| 项目内 `ros1_bridge` | 建议更名为 `host_bridge`；保留 TCP/JSON、RTSP 协议，迁移 ROS 输出适配层 | P2 |
| `tcp_bridge` | 盘点与 host bridge 的实际使用关系，迁移被使用的接口、Boost 网络逻辑和消息 | P2 |
| `vision_pkg` | 迁移 ROS API、image_transport、cv_bridge；验证 OpenCV、模型文件、GStreamer 和视频编码 | P2 |
| `vision_opencv` | 优先使用目标发行版 ROS2 上游包，避免携带 ROS1 cv_bridge 源码进入正式构建 | P2 |
| 相机子模块 | RealSense 已锁定 ROS2 `4.59.1`；Orbbec 已记录 ROS2 gitlink，先确认 Astra+ 支持，再验证 Humble、SDK、话题、USB 与标定数据 | P2 |
| `Eye_in_hand` / `aruco_ros` / `easy_handeye2` | ArUco 已锁定 ROS2 `5.0.6`，标定依赖已选定 easy_handeye2；迁移采样客户端、MoveIt 调用和 launch，验证 Humble 与外参发布 | P2 |
| `senseair_co2` / `thermal_camera` | 迁移 rospy 和打包；保留采集协议，核对 Python/SDK 兼容性 | P2 |

P0 为技术路线前置项，P1 为控制主链路，P2 为外围功能；若某项外围能力是当前整车交付必需，应提前排期。

### 5.1 ROS1 上游依赖替换专项清单

**初始化子模块本身不会将 ROS1 包升级为 ROS2 包；目前 `can_driver` 已锁定 ROS2 提交，手眼标定子模块已替换为 ROS2 `easy_handeye2`，RealSense、ArUco 分别切换到 ROS2 `4.59.1`、`5.0.6`，Orbbec 改为锁定 ROS2 仓库提交。** 当前主链路的大多数包仍使用 Catkin/roscpp/rospy；以下上游依赖仍须完成目标环境验证和调用端适配，不能只把顶层工程改成 ament。

| 当前依赖及源码证据 | Humble 迁移要求 | 必须回归的调用端 |
| --- | --- | --- |
| `realsense2_camera 4.59.1`：已锁定 ROS2/ament 源码 `7a87097`，原 ROS1 提交为 `2e1ecd5` | 在 Humble 构建并核对 librealsense/固件与 D405、后置相机兼容性；原 ROS1 提交留作回退 | `vision_pkg`、`Eye_in_hand` 的 ROS1 相机 launch，RGB/depth/CameraInfo、对齐深度、optical frame |
| `OrbbecSDK_ROS2`：已记录 `v2-main` 提交 `c153462`，尚未下载；原 ROS1 提交为 `86ab046` | 先确认 Astra+ 是否在该提交的支持列表中，再核查 Humble、SDK 与 udev；来源选定不代表设备可用 | `vision_pkg/launch/vision.launch` 的旧 `orbbec_camera/astra_adv.launch`、相机枚举及后置视觉 |
| `aruco_ros 5.0.6`：已锁定 ROS2/ament 源码 `2872a00`，原 ROS1 提交为 `3680aea` | 在 Humble 构建并验证 marker ID、尺寸、图像订阅、位姿与 TF；旧 README 的 ROS1 命令不能作为新启动依据 | `Eye_in_hand/launch/aruco_detect.launch` 与采样脚本 |
| `easy_handeye2 0.5.0`：rclpy、ament、easy_handeye2_msgs | 已替换子模块并锁定 `29bd50a`；Humble 构建和端到端集成仍待验证，自动运动模块不视为已迁移 | 自动采样、MoveIt 2 控制、标定文件转换与静态 TF 发布 |
| `vision_opencv/cv_bridge`、`image_geometry` 的 ROS1 源码 | 使用 Humble 对应上游包；避免同名 ROS1 包遮蔽 ROS2 安装版本 | `vision_pkg` C++/Python 图像转换及 OpenCV ABI |
| MoveIt 1、ros_control、Classic Gazebo 插件 | 分别迁移到 Humble 对应 MoveIt 2、ros2_control 与 Fortress 插件 | 规划、执行、Servo、控制器配置和仿真传感器 |

实施时生成依赖锁定表，至少记录“当前来源/提交 → 目标来源/发行版/提交或 deb 版本 → SDK/固件 → 接口差异 → 验证状态”。尚未完成选型的依赖标为待验证，不填猜测版本。相机/标定的选型在阶段 0 提前验证，不能拖到最后才发现没有可用的 Humble 组合。

除已切换的 `can_driver`、`realsense-ros`、`aruco_ros`、已替换的 `easy_handeye2` 和仅锁定远端提交的 `OrbbecSDK_ROS2` 外，其余来源替换在后续 ROS2 迁移中逐项完成。旧 ROS1 标定、ArUco 和相机 launch 仍需适配，不能因子模块切换就视为可运行。最终对选定源码树执行包发现和依赖检查，确认 ROS1 的 Catkin、nodelet、`message_generation` 等未混入 ROS2 正式构建。

### 5.2 easy_handeye2 替换决定与后续接入任务

- 新子模块路径：`easy_handeye2/`；来源：`git@github.com:marcoesposito1988/easy_handeye2.git`；锁定提交：`29bd50a7939e853d768d203d2abc61eb6e50facc`。旧路径 `easy_handeye/` 从父仓库索引移除，不在源码树同时保留两套标定包。
- 本地源码已确认 ROS2 包 `easy_handeye2` 和接口包 `easy_handeye2_msgs` 均为 0.5.0。包声明和 README 不等同于已通过 Humble 构建，阶段 0 仍需验证 launch API、依赖及 rqt 功能。
- `Eye_in_hand` 目前仍是 ROS1 包，`rospy`、ROS1 XML launch、`moveit_commander` 和旧消息导入不能通过名称替换直接接入 ROS2。保留这些文件作为迁移参考；后续整体迁移为 ROS2，并将依赖更新为 `easy_handeye2 / easy_handeye2_msgs`。
- ROS2 标定入口为 `calibrate.launch.py`，发布入口为 `publish.launch.py`；`namespace_prefix / eye_on_hand` 参数需分别迁移为 `name / calibration_type=eye_in_hand`，四个 frame 参数逐项核对。
- 客户端应使用 `easy_handeye2_msgs/srv`。当前服务前缀在源码中固定为 `/easy_handeye2/calibration/`，涵盖 `take_sample / compute_calibration / save_calibration`；需要异步调用、检查返回值、确认 TF 和采样时间一致性。多标定实例须通过明确 remap/适配隔离服务，不能只修改 `name` 就假定隔离完成。
- 保存路径由 ROS1 的 `~/.ros/easy_handeye/*.yaml` 改为 `~/.ros2/easy_handeye2/calibrations/<name>.calib`，样本目录为 `~/.ros2/easy_handeye2/samples`。旧文件不能仅改扩展名，应转换并核对字段、单位及父子 frame，或重新标定。
- 上游 `handeye_robot.py` 仍导入 `moveit_commander`，README 将自动运动标为进行中；首版采用项目 MoveIt 2 执行接口负责走点，easy_handeye2 负责采样、求解、保存和发布。不能将克隆完成视为自动采样功能完成。
- 验收：Humble 干净构建 → 人工采样求解保存 → 重启读取并发布 TF → 项目自动采样 → 失败/缺失 TF/重复 TF 发布检查。标定 launch 会启动 dummy TF，必须避免与真实外参发布者同时发布同一变换。

### 5.3 RealSense ROS2 子模块接入任务

- 子模块来源：`https://github.com/realsenseai/realsense-ros.git`；分支：`ros2-master`；当前锁定 tag `4.59.1`，提交 `7a87097c499e3b0b69518d54a8e7a9ac1924e87c`。旧 ROS1 提交 `2e1ecd5` 仅作回退基线；分支后续更新不自动改变父仓库的 gitlink。
- 源码已包含 ROS2 `realsense2_camera`、`realsense2_camera_msgs` 和 `realsense2_description`，包版本为 `4.59.1`；README 列出 Humble，但尚未在项目目标环境完成构建、SDK 和设备验证。
- 将 `vision_pkg` 与 `Eye_in_hand` 中对 ROS1 `rs_camera.launch` 的 include 改为 ROS2 `rs_launch.py` 的启动与参数配置；分别核对 D405 和后置相机的设备选择、命名空间、彩色/深度流、对齐深度及 CameraInfo 话题，不直接沿用旧参数名。
- 锁定与此版本匹配的 librealsense 和相机固件，验证 USB 带宽、帧率、时间戳、QoS、optical frame/TF 以及标定输入；以目标设备实测结果确认兼容性。

### 5.4 ArUco ROS2 子模块接入任务

- 子模块来源：`https://github.com/pal-robotics/aruco_ros.git`；分支：`humble-devel`；当前锁定 tag `5.0.6`，提交 `2872a002ce29362896ed55e28e00abb763335235`。旧 ROS1 提交 `3680aea` 仅作回退基线；分支后续更新不自动改变父仓库的 gitlink。
- 源码已包含 ROS2 `aruco`、`aruco_msgs` 和 `aruco_ros`，均为 `5.0.6` 且使用 ament；仓库 README 仍含 ROS1 启动示例，Humble 构建与项目标定流程尚待验证。
- 将 `Eye_in_hand/launch/aruco_detect.launch` 的 ROS1 XML 节点改为 ROS2 启动配置，按当前 `single` 节点和 `single.launch.py` 核对 `marker_id=582`、`marker_size=0.05`、相机与 marker frame、图像/CameraInfo remap。`Eye_in_hand/config/calibration_params.yaml` 标注 `6x6_250` 字典却配置 ID `582`，超出该字典的 0–249 范围；迁移前须核对实际打印的 marker，统一字典、ID、实测尺寸和检测器配置。上游示例的相机名称、frame 和 marker 尺寸不适用于本项目，`corner_refinement` 在当前源码中标为弃用。
- 验证 `aruco_single/pose`、marker TF 的时间戳、坐标系、方向和检测稳定性，并与 RealSense D405 和 easy_handeye2 的采样链路联调。

### 5.5 Orbbec ROS2 候选版本验证与后置相机接入

- 子模块来源：`git@github.com:orbbec/OrbbecSDK_ROS2.git`；分支：`v2-main`；父仓库当前 gitlink 为 `c153462518ad674650bafa4464fda72c27ab797a`。旧 `OrbbecSDK_ROS1` 提交 `86ab046` 留作 ROS1 回退依据。当前只验证了远端提交存在，尚未下载源码，不能据此认定 Astra+、Humble 或 SDK 兼容。
- `vision_pkg/launch/vision.launch` 默认后置相机类型是 `realsense`；选择 `astra` 时包含旧 ROS1 `orbbec_camera/launch/astra_adv.launch`。先确认现场是否必须交付 Astra+，记录设备型号、USB ID、固件和序列号，再核对该 ROS2 提交的支持设备列表与 Ubuntu 22.04 / Humble 构建依赖。旧 `vision_pkg/README.md` 对 ROS1 驱动版本有 Astra+ 限制，不能将旧版本结论直接套到此 ROS2 仓库。
- 确认设备支持后，在独立 Humble 工作空间初始化子模块并验证 SDK、udev、设备枚举、单相机启动和固定设备选择；再迁移 ROS2 launch。若该提交不支持现场 Astra+，记录阻塞原因，选择有设备支持证据的 ROS2 驱动版本或调整明确的交付范围，不把更换仓库地址视为兼容性解决。
- 后置视觉消费者当前使用 `/behind_camera/color/image_raw`、`/behind_camera/depth/image_raw` 和 `/behind_camera/color/camera_info`。核对新驱动的实际话题、深度单位、CameraInfo、frame、时间戳和 QoS，通过启动配置或适配层保持消费者合同；分别回归 Astra+ 和 RealSense 后置模式，避免两个驱动同时占用设备或重复发布同名话题。

## 6. 通用迁移规范与本项目易错点

### 6.1 构建、接口与资源

- Catkin 改为 `ament_cmake` / `ament_python`，使用 colcon；顶层 Catkin `CMakeLists.txt` 不复制到新工作空间作为构建入口。
- `message_generation` 改为 `rosidl_default_generators`，正确声明消息、服务、Action 依赖及接口包组。
- C++ 节点改为 rclcpp，Python 改为 rclpy；消息类型名、头文件、时间类型和日志接口同步调整。
- 脚本、launch、YAML、URDF、mesh、EDS、模型等通过 install 安装；路径通过 ament 索引解析。
- ROS2 正式包统一使用小写命名；保留旧服务路径可通过兼容命名实现，但消息包类型变化必须更新客户端。
- 对混有接口与 Python 节点的包明确打包方式，避免生成接口与 Python 模块相互覆盖。

### 6.2 控制器配置

| ROS1 当前配置 | ROS2 计划 |
| --- | --- |
| `joint_state_controller/JointStateController` | `joint_state_broadcaster/JointStateBroadcaster` |
| `position_controllers/JointTrajectoryController` | `joint_trajectory_controller/JointTrajectoryController`，显式配置 position 命令及所需状态接口 |
| `arm_position_passthrough_controller` | 验证 ROS2 位置转发控制器，并与 JTC 互斥 |
| 自定义摆臂 CSV 插件 | 迁移为 ROS2 ControllerInterface，保留 0.25s 超时等保护语义 |
| `wheel_controller` | 使用 ROS2 diff_drive_controller，重写参数和命令类型适配 |
| 夹爪 Float64 速度控制 | 北向语义保留；南向按选定控制器适配数组/话题及看门狗 |

ROS2 参数文件使用节点名和 `ros__parameters` 层级，控制器参数独立作用域；不把 ROS1 YAML 原样加载到新 manager。

Humble 的 diff_drive_controller 支持两种输入：`use_stamped_vel=true` 时订阅 `~/cmd_vel`（TwistStamped），`false` 时订阅 `~/cmd_vel_unstamped`（Twist）；左右轮参数为数组 `left_wheel_names / right_wheel_names`，参考 [Humble 官方控制器文档](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)。迁移首版建议保留北向 `/cmd_vel` 的 Twist 输入，显式设置 `use_stamped_vel=false`，将 `mobility_control` 南向输出改为 `/wheel_controller/cmd_vel_unstamped`。若后续采用带时间戳输入，应同时修改该参数、输出类型和话题，并验证超时行为。

现有 `mobility_control/config/base_cmd.yaml` 中轮距为 0.438，而默认实机控制器配置为 0.36。必须核对实测几何与用途，形成单一参数来源；不能迁移时任选一个值。

### 6.3 can_driver 已有分层的复用与拆分

根据 [`CMakeLists.txt`](../can_driver/CMakeLists.txt)、[`CanDriverRuntime.h`](../can_driver/include/can_driver/CanDriverRuntime.h) 和 [`can_driver_system.hpp`](../can_driver/include/can_driver/can_driver_system.hpp)，当前已使用 ament 构建 `can_driver_core`、`can_driver_system` 和 `can_driver_node`，并导出 `hardware_interface::SystemInterface` 插件。源码迁移已提交，但尚未据此认定目标环境构建和实机集成通过。

- 在 Humble 中编译并验证 `can_driver_core`、SocketCAN/UDP transport、PP/DM/MT/ECB 协议及 `can_driver_system` 插件；核对控制器接口和生命周期行为。
- 验证 ROS2 节点的服务、Action、参数 YAML 及 `ros2_control` 路径与上层调用方的接口语义，补齐跨后端集成测试。
- 保留启动位置同步、故障 Stop、使能门控、重连首周期 hold、清空旧命令及 PP 零点文件行为；软件零点与写电机零点必须区分。
- 默认履带使用 DM 协议。配置注释明确编码器为 16384 counts/rev，但现有协议内部按自身 SI 粒度换算，不能把该数值直接填入 `position_scale/velocity_scale`。
- 当前 ROS2 构建仍要求 INNFOS `linux_x86_64/libActuatorController.so`，即使默认配置不启用 ECB 也可能阻塞构建。后续按交付范围决定是否将 ECB/厂商 SDK 做成可选能力，并核查 Ubuntu 22.04 的 ABI、架构和安装路径。

### 6.4 MoveIt 2、Action 与并发

- 保留六轴组、末端、碰撞矩阵和 named targets 的语义；核对 SRDF、joint_limits、kinematics 和执行容差。
- 先使 MoveIt 2 通过 `/arm_position_controller/follow_joint_trajectory` 完成真实执行，再接入统一目标 Action。
- `arm_goal_executor_node.py` 与腕部服务依赖 `MoveGroupCommander`，需要改用 MoveIt 2 支持的接口。Humble 首版采用 C++ `MoveGroupInterface` 实现执行核心；保留 Python 客户端时通过项目 Action 调用执行核心。`moveit_py` 不作为默认前置依赖，只有确认所锁定 Humble 版本的发布可用性、API 和取消能力后，才评估替代。参见 [MoveIt 官方 Python 库说明](https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html)。
- ROS1 actionlib 迁移到 ROS2 Action 后，覆盖 goal 接受、取消、替换目标、反馈、异常结果和执行停止；保留 `ExecuteArmGoal.action` 中现有目标类型和错误码含义。
- 控制器就绪检查由 ROS1 `running` 更新为 ROS2 状态/接口检查，不能仅检查服务是否存在。
- Servo 输出控制器与离散轨迹控制器共享关节时必须仲裁；统一目标 Action、腕部服务、HMI 也不能同时争用机械臂执行。
- 使用合适 executor 和 callback group；不要在互相等待的服务/Action 回调中同步阻塞，造成执行取消或模式切换死锁。

### 6.5 时钟、QoS 与实时循环

- 实机超时和反馈新鲜度使用适合的单调时间策略；仿真统一 `use_sim_time`，明确暂停和时间跳变行为。
- 控制循环内不做阻塞 SDO、网络服务等待、大量日志或无界锁等待；通信任务与实时读写通过有界缓存衔接。
- 控制命令禁止使用持久化 QoS 重放历史命令；命令需结合接收时间和设备反馈决定是否有效。
- 图像、关节状态、TF、运行态分别配置兼容 QoS；静态 TF 使用 tf2 对应机制，不能简单套用统一 QoS。
- 清点 latched topic、ROS1 参数服务器、dynamic_reconfigure、nodelet 等用法，分别改为 ROS2 对应机制。

### 6.6 Gazebo 与上位机

- 仿真目标调整为 Gazebo Fortress，使用与 Humble/Fortress 配套的 `gz_ros2_control` 和 `ros_gz`，替换 Classic 的控制、相机、深度、IMU 等插件。历史示例中可能出现 `ign_ros2_control` / `ros_ign` 命名，必须以锁定版本的实际包名、插件名和库名为准，避免混用。配套依据见 [Humble 兼容矩阵](https://github.com/ros-controls/gz_ros2_control/blob/humble/README.md)。
- 重写模型生成、世界启动、初始姿态、传感器桥接和 `/clock`；验证 mimic 夹爪及后轮关系。
- 底盘首选统一到 ros2_control 差速链路；若保留 Gazebo 差速系统，应记录原因并确保只有一条底盘写入链、一个 `odom → base_link` 发布者。
- `gazebo_model_odom_node.py` 若依赖 Classic 模型状态需替换；仿真里程计不能与控制器重复发布同一 TF。
- 当前项目的 `ros1_bridge` 是 TCP/JSON + RTSP 业务包，**不是 ROS 官方 ROS1↔ROS2 bridge**。更名并迁移时应保持已有 HMI 协议、错误码、重连和视频控制行为。
- 若确需过渡桥接，只用于受控数据接口并单独验证；不能假定 Noetic 和 Humble 可在同一原生系统直接组合，也不能假定 Action 可透明桥接。平台限制参见 [上游 ros1_bridge](https://github.com/ros2/ros1_bridge)。

## 7. 分阶段实施与验收

以下工作量为单名熟悉项目的开发者的初步有效工作日估计；不含等待设备、供应商资料和长期现场测试。阶段 0 完成后重新估算。

| 阶段 | 工作及交付物 | 进入下一阶段的条件 | 估算 |
| --- | --- | --- | --- |
| 0：基线与依赖盘点 | 核对本地子模块指针与现场参数，锁定 ROS1 基线；设备/接口清单、Humble 核心依赖与相机支持决策 | EDS、轴归属、模式及回退资料齐全；必需相机的支持证据与验证路径明确，核心依赖可构建 | 2–4 天 |
| 1：CANopen 专项验证 | 配置包、IP 肩轴、CSV/CSP 摆臂、厂商对象、多轴及异常验证记录 | 第 4 节关键能力确认；无法支持的项目有明确修复方案 | 5–10 天 |
| 2：ROS2 基础与 mock | 接口包、模型、ament 构建、mock 硬件、统一控制器配置 | 无硬件可完成关节反馈、JTC、摆臂、底盘控制验证 | 3–5 天 |
| 3：双后端与统一运行态 | can_driver 插件、上游 CANopen 插件、协调层、模式及失败收敛 | 跨后端轨迹、故障和生命周期用例通过 | 7–12 天 |
| 4：上层与 MoveIt 2 | 三控制包、MoveIt 2、Servo、遥控与 Action | 目标执行、取消、模式互斥、断流停止通过 | 5–8 天 |
| 5：完整 Gazebo 仿真 | Fortress 模型、传感器、桥接、无 GUI 验收入口 | 仿真主链路与北向接口可完整运行 | 4–7 天 |
| 6：外围与整车交付 | HMI/视频、RealSense/Orbbec 后置相机、视觉、ArUco/手眼标定、传感器、部署和测试报告 | 必需相机模式与标定链路通过第 8 节验收，约定交付范围全部回归，ROS1 可回退 | 5–10 天 |

合计约 **31–56 个有效工作日**。CANopen 专项是关键路径，单轴验证结果可能显著改变工期。最小主链路里程碑到阶段 4；完整迁移还必须完成阶段 5、6 中约定的范围。

### 阶段 0 详细清单

- [x] 核对已恢复子模块，将手眼标定替换为 ROS2 easy_handeye2，并将 RealSense、ArUco 分别切换到 ROS2 `4.59.1`、`5.0.6`；本地 gitlink 见第 2.3 节。
- [ ] 验证 RealSense `4.59.1` 在 Humble 中构建，核对 D405、后置相机、librealsense/固件和 ROS2 话题；`vision_pkg` 与 `Eye_in_hand` 调用端在阶段 6 迁移。
- [ ] 验证 ArUco `5.0.6` 在 Humble 中构建，解决 `6x6_250` 与 marker ID `582` 的冲突，记录图像、pose 和 TF 接口；标定入口在阶段 6 迁移。
- [ ] 验证 easy_handeye2 在 Humble 中构建，以及 rqt/launch、采样和发布接口；旧 `Eye_in_hand` 客户端在阶段 6 迁移。
- [x] 将旧 Orbbec ROS1 子模块替换为 `OrbbecSDK_ROS2`，锁定 `v2-main` 提交 `c153462`；本轮不下载源码。
- [ ] 确认现场是否必须交付 Astra+，记录具体型号与固件；按第 5.5 节核查所锁定提交的设备支持。需要该模式时再初始化源码并做 Humble 单相机验证，完整视觉接入放在阶段 6。
- [ ] 现场复核子模块配置和运行时覆盖参数，记录已有本地补丁。
- [ ] 记录所有电机型号、固件、关节归属、CAN 接口、Node-ID、模式、方向、减速比和零点。
- [ ] 导出当前 ROS1 参数、控制器列表、服务/话题/Action 合同及典型运行日志。
- [ ] 保存短时 joint_states、运行态、里程计与规划执行样本，作为数值对照基准。
- [ ] 验证 Lely、相机 SDK、热成像库、OpenCV/GStreamer、模型文件在目标平台的可用性。
- [ ] 明确整车必需的外围能力、设备测试条件及验收阈值。

### 推荐提交顺序

1. `docs: lock migration baseline and device inventory`
2. `canopen: add upstream version pin and robot configuration package`
3. `canopen: verify shoulder IP and flipper CSP/CSV integration`
4. `build: add ROS2 interfaces and robot description`
5. `control: add mock hardware and ROS2 controller configuration`
6. `hardware: port private CAN backend to SystemInterface`
7. `hardware: integrate dual backends and unified coordinator`
8. `control: port mobility, flipper and gripper nodes`
9. `moveit: add MoveIt 2 execution, Servo and arm goal action`
10. `simulation: migrate complete system to Gazebo Fortress`
11. `integration: port host bridge, perception and sensors`
12. `bringup: add acceptance checks and deployment documentation`

每步提交应携带对应验证记录；上游依赖锁定与项目配置分别管理，便于后续更新。

## 8. 验证矩阵与完成标准

| 范围 | 必测场景 | 判定标准 |
| --- | --- | --- |
| 构建与依赖 | 干净目标环境安装、构建、测试 | 无 Catkin 混入，安装目录可独立解析资源 |
| CANopen 启动 | 正常启动、缺失从站、错误 Node-ID/EDS | 错误明确，禁止虚假就绪及意外运动 |
| 数值与时序 | 已知角度/速度、零位、正反向、SYNC/控制周期 | 换算一致；刷新率、抖动、总线负载符合阶段 0 约定 |
| CANopen 模式 | 肩部 IP 及 halt/resume 边沿，摆臂 CSP/CSV 往返切换、失败、重启 | 反馈确认模式，切换无旧命令和参考跳变 |
| 生命周期 | init/enable/resume/halt/disable/recover/shutdown | 两侧实际状态与统一反馈一致，默认启动不自动放行运动 |
| 双后端事务 | 一侧失败、掉线、超时、部分使能 | 另一侧进入约定停止状态；失败原因可追踪 |
| 控制器资源 | JTC/透传、CSP/CSV 冲突和切换 | 冲突资源不能同时领取，失败不遗留运动输出 |
| 六轴机械臂 | 跨后端轨迹、named/joints/Pose、取消/抢占 | 结果码及停止行为正确，精度不劣于约定基准 |
| 摆臂 | 四轴联动、点动、命令丢失、NaN/Inf | 方向和限位正确，控制器级超时归零保留 |
| 底盘 | 直行、转向、正反向、断流、里程计 | 超时停止；几何参数统一，里程计及 TF 单源 |
| 夹爪 | Bool 开合、速度、保持超时、mimic | 先解决 csp/速度接口冲突及 m/s 单位，再验证超时停止和节点退出行为 |
| 仿真 | 无 GUI 启动、模型、控制、传感器、暂停 | 主链路可重复启动，时钟和 TF 一致 |
| HMI/视频 | 旧协议、重连、模式按钮、视频启停 | 上位机操作可用，断连不会重放旧运动命令 |
| 后置相机 | RealSense 模式、按交付范围要求的 Astra+ 模式、缺失/多设备场景 | 只启动所选驱动；设备绑定稳定，彩色/深度/CameraInfo 话题及 frame、时间戳、单位符合消费者合同 |
| 手眼标定 | D405 → ArUco → easy_handeye2 的采样、保存、重启发布 | marker 字典/ID/尺寸一致，pose 与 TF 时间和坐标正确，无重复外参发布 |
| 外围功能 | 相机帧、图像、深度、视觉结果、外参、CO₂、热成像 | 约定话题、单位、帧和数据频率正确 |

将现有 `Eyou_ROS1_Master/tests/test_hybrid_*.cpp` 的协调器、服务网关、启动及模式路由用例迁移到 ament 测试；保留 `flipper_control` 参考生成器和 `arm_control` Action 合同测试的价值，按 ROS2 并发及取消语义补充用例。子模块还包含 CANopen 轴逻辑、零点软限位、PDO 和运行协调测试，以及 `can_driver/tests` 的 PP/DM 协议、限位、设备恢复和生命周期测试；优先复用其行为断言，补充旧会话拒绝、IP 双通道和厂商对象读写回归。

阶段性检查命令如下；这是后续 ROS2 环境的检查模板，不代表当前仓库已能运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot24_ros2_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose

# 在对应 mock / 仿真 / 实机 bringup 启动后检查
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic info /joint_states --verbose
ros2 action list -t
```

建议最终提供 `full_system_mock.launch.py`、`full_system_simulate.launch.py`、`full_system.launch.py` 和 `acceptance_smoke.launch.py`。就绪检查应检查反馈新鲜度、硬件状态、控制器状态及 Action 可用性，不能仅靠固定 sleep。

## 9. 工作空间隔离、部署与回退

- ROS1 基线和 ROS2 迁移使用不同 checkout/分支及独立 build/install 目录，避免当前源码树中两套构建系统互相发现。
- ROS2 工作空间只纳入选定的 ROS2 包；不把整个现有 ROS1 树未经筛选交给 colcon。
- vendor 目录按实际内容使用 `COLCON_IGNORE` 或直接不纳入，不能依赖 ROS1 的 `CATKIN_IGNORE` 完成 ROS2 隔离。
- 保存 ROS1 可运行提交、设备配置、EDS/DCF、零点、固件信息和启动方式；ROS2 产生的设备持久化参数变更也要记录。
- 首次部署先 mock，再台架单轴，再多轴，最后整车。实机默认保留 `auto_init=false / auto_enable=false / auto_release=false` 的现有意图。
- 回退时先关闭 ROS2 控制进程并确认总线和设备状态，再恢复 ROS1 程序及必要的设备配置。只切换软件分支不能保证回退已完成。
- 最终交付包含锁定依赖清单、配置生成方法、部署说明、接口映射、测试结果、已知限制和回退操作。

## 10. 第一轮实施的明确交付物

第一轮优先完成下列内容，之后再全面改造上层节点：

1. 在已恢复的驱动源码及第 2.4 节轴分配基础上，现场确认设备、固件、模式及接口清单。
2. 锁定 Humble 配套依赖与 `ros2_canopen` 的具体版本，建立 `robot_canopen_config`。
3. 分别完成真实肩部 IP 单轴和摆臂 CSV 单轴的反馈、低速运动、停止及故障恢复，覆盖零点、软限位和持久化。
4. 完成摆臂 CSP/CSV 切换验证，并确定与整车生命周期的适配方案。
5. 完成一个 controller_manager 下跨两个硬件组件的机械臂轨迹验证。
6. 根据验证结果更新接口、架构和工期，再推进整车迁移。

这六项通过后，才能认为“从自研 CANopen 后端切换到上游维护栈”的核心路线已经验证；只有全部约定功能通过第 8 节验收，才算项目 ROS2 迁移完成。
