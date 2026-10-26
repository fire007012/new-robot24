# ROS 适配层对齐 ros_canopen 接口计划

日期：2026-03-19
目标：使 ROS 适配层对外接口与 ros_canopen（canopen_motor_node）基本一致

---

## 现状

已完成：
- `hardware_interface::RobotHW` 继承，`read()/write()` 周期回调
- `JointStateInterface` 注册（pos/vel/eff 反馈）
- `PositionJointInterface` 注册（CSP 位置命令）
- `controller_manager` 驱动主循环（`canopen_hw_ros_node.cpp`）
- launch 文件参数加载
- 底层已支持 CSP/CSV/CST 三模式（`SetJointVelocityCommand`、`SetJointTorqueCommand`）

---

## Commit 路线图

### R01：注册 VelocityJointInterface 和 EffortJointInterface

涉及文件：
- `include/canopen_hw/canopen_robot_hw_ros.hpp`
- `src/canopen_robot_hw_ros.cpp`

内容：
1. hpp 新增 `VelocityJointInterface`、`EffortJointInterface` 成员及 `vel_cmd_`、`eff_cmd_` 数组
2. 构造函数中注册 velocity / effort handle，调用 `registerInterface`
3. `write()` 中将 `vel_cmd_` → `SetJointVelocityCommand`，`eff_cmd_` → `SetJointTorqueCommand`

验收：`WITH_ROS=ON` 编译通过，无 warning

---

### R02：write() 按运动模式分发命令

涉及文件：
- `include/canopen_hw/canopen_robot_hw_ros.hpp`
- `src/canopen_robot_hw_ros.cpp`

内容：
1. 新增 `active_mode_` 数组（每轴当前模式，默认 CSP）
2. `write()` 根据 `active_mode_[i]` 决定写 position / velocity / torque
3. 暴露 `SetMode(axis_index, mode)` 方法供后续 service 调用

验收：CSP 模式行为不变，CSV/CST 模式命令正确下发

---

### R03：重构 canopen_hw_ros_node 使用 LifecycleManager

涉及文件：
- `src/canopen_hw_ros_node.cpp`

内容：
1. 将当前手动构造 SharedState/CanopenMaster/CanopenRobotHw 替换为 `LifecycleManager`
2. 主循环通过 `lifecycle.robot_hw()` 驱动 read/write
3. 保持功能等价，为后续 service 包装做准备

验收：`roslaunch` 启动行为与重构前一致

---

### R04：添加 init/halt/recover/shutdown ROS Service

涉及文件：
- `src/canopen_hw_ros_node.cpp`
- `package.xml`（添加 `std_srvs` 依赖）

内容：
1. 注册 `~init`（`std_srvs/Trigger`）→ `LifecycleManager::Init`
2. 注册 `~halt`（`std_srvs/Trigger`）→ `LifecycleManager::Halt`
3. 注册 `~recover`（`std_srvs/Trigger`）→ `LifecycleManager::Recover`
4. 注册 `~shutdown`（`std_srvs/Trigger`）→ `LifecycleManager::Shutdown`
5. service 回调中返回 success/message

验收：`rosservice call /canopen_hw_node/halt` 停止运动，`/recover` 恢复

---

### R05��添加 set_mode ROS Service

涉及文件：
- `srv/SetMode.srv`（新建）
- `src/canopen_hw_ros_node.cpp`
- `CMakeLists.txt`（添加 message_generation）
- `package.xml`（添加 message_generation / message_runtime）

内容：
1. 定义 `SetMode.srv`：`uint8 axis_index` + `int8 mode` → `bool success` + `string message`
2. 注册 `~set_mode` service，回调调用 `CanopenRobotHwRos::SetMode` + `CanopenRobotHw::SetJointMode`

验收：`rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 9}"` 切换到 CSV

---

### R06：接入 diagnostic_updater

涉及文件：
- `src/canopen_hw_ros_node.cpp`
- `CMakeLists.txt`（WITH_ROS 分支添加 `diagnostic_updater`）
- `package.xml`（添加 `diagnostic_updater` 依赖）

内容：
1. 创建 `diagnostic_updater::Updater`，设置 hardware_id
2. 为每轴注册诊断任务，读取 `HealthCounters`：heartbeat_lost_count、emcy_count、fault_reset_count
3. 根据 is_operational / is_fault 设置 OK / WARN / ERROR 级别
4. 主循环中调用 `updater.update()`

验收：`rostopic echo /diagnostics` 可见每轴健康状态

---

### R07：完�� launch 与 controller 配置

涉及文件：
- `launch/canopen_hw.launch`
- `config/ros/controllers.yaml`

内容：
1. launch 中添加 `rosparam load controllers.yaml`
2. 添加 `controller_manager/spawner` 节点，加载 `joint_state_controller`
3. controllers.yaml 补充 position / velocity / effort controller 定义
4. 添加 `robot_state_publisher` 节点（可选，需 URDF）

验收：`roslaunch canopen_hw canopen_hw.launch` 一键启动，`/joint_states` 有数据

---

### R08：更新文档

涉及文件：
- `docs/usage.md`
- `README.md`

内容：
1. usage.md 补充 ROS 模式启动命令、service 列表、controller 切换说明
2. README.md 更新代码结构（新增 srv/）和 ROS 接口说明

验收：文档与实际代码一致

---

## 依赖关系

```
R01 → R02 → R03 → R04 ─→ R05 → R07 → R08
                    │              ↑
                    └──→ R06 ──────┘
```

R01-R02 是基础（多模式 interface），R03 是重构前置，R04-R06 可部分并行，R07-R08 收尾。

---

## 不做的事项

| ros_canopen 功能 | 处理方式 | 原因 |
|------------------|---------|------|
| EDS 自动发现 / 动态 node_id | 不做，joints.yaml 静态配置 | 6 轴固定拓扑 |
| canopen_chain_node 多层管理 | 不做，LifecycleManager 直接管理 | 架构更���单 |
| layer-based 错误传播 | 不做，HealthCounters + SharedState | 已有等效机制 |

---

## 最终验收标准

1. `roslaunch canopen_hw canopen_hw.launch` 正常启动
2. `joint_state_controller` 发布 `/joint_states`
3. `rosservice list` 可见 `~init`、`~halt`、`~recover`、`~shutdown`、`~set_mode`
4. `rosservice call /canopen_hw_node/halt` 停止运动
5. `rosservice call /canopen_hw_node/recover` 恢复运动
6. `rosservice call /canopen_hw_node/set_mode` 切换 CSP/CSV/CST
7. `/diagnostics` 可见每轴 heartbeat、EMCY、fault 状态
8. position_controller / velocity_controller / effort_controller 均可加载并驱动
