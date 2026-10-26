# Bug Report: Controller Manager 调用阻塞与位置命令无效

- 日期: 2026-03-21
- 项目: Eyou_Canopen_Master
- 环境: ROS Noetic (`rosversion: 1.17.2`), Ubuntu (x86_64), SocketCAN `canable0`, 从站 NodeID=5
- 目标现象: CSP 位置命令已发布，但电机无动作

## 1. 问题摘要

当前问题不是单一故障，而是多阶段叠加：

1. 启动早期存在 CAN 权限问题，节点无法打开 `canable0`。
2. `set_mode` service 类型命名错误（代码缺陷）导致服务调用失败。
3. 修复 service 类型后，`controller_manager` 相关调用出现阻塞或控制器未加载，导致 `/arm_position_controller/command` 话题不存在，位置命令无法被消费。

## 2. 已观察到的关键症状（按时间）

### 2.1 启动失败（权限）
日志：

- `CanopenMaster start failed: CanController: Operation not permitted`
- `Init: master start failed`
- `canopen_hw_node ... exit code 1`

结论：进程无权限打开 CAN 设备（`cap_net_raw/cap_net_admin` 缺失或失效）。

### 2.2 抓包显示命令未生效
用户提供抓包中长期出现：

- `0x205` 一直为 `00 00 00 00 00 00 00`
- 可见 `0x285`，但未稳定看到预期 `0x185` 路径

结论：控制字/目标未形成有效推进，命令路径被门控或控制器链路未通。

### 2.3 service 类型错误（已确认代码缺陷）
用户执行：

- `rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 8}"`

错误：

- `Unable to load type [canopen_hw/SetMode]`

分析：当前包名是 `Eyou_Canopen_Master`，但节点代码使用了旧命名空间 `canopen_hw::SetMode`。

### 2.4 controller_manager 调用异常
用户反馈：

- `rosservice call /controller_manager/list_controllers "{}"` 多次阻塞/卡住
- `rostopic info /arm_position_controller/command` -> `Unknown topic`
- `rostopic echo -n1 /diagnostics` 无输出

另一次状态中可见：

- `rosservice list` 能看到 `/controller_manager/*`
- `rosservice call /controller_manager/list_controllers "{}"` 返回 `controller: []`

结论：控制器未真正加载/启动；在部分运行上下文中 service 回调还存在阻塞现象。

### 2.5 ROS Master 未启动（一次独立问题）
用户直接 `rosrun` 节点时出现：

- `Failed to contact master at [localhost:11311]`
- `Connection refused`

结论：该次仅因 `roscore` 未运行，不是核心逻辑问题。

## 3. 已确认的代码级问题与修复

### 3.1 SetMode service 命名空间错误（已修）
文件：`src/canopen_hw_ros_node.cpp`

修复内容：

- `#include "canopen_hw/SetMode.h"` -> `#include "Eyou_Canopen_Master/SetMode.h"`
- `canopen_hw::SetMode::{Request,Response}` -> `Eyou_Canopen_Master::SetMode::{Request,Response}`

编译结果：

- `catkin_make --pkg Eyou_Canopen_Master -j12 -l12` 通过
- 运行时验证：`rosservice type /canopen_hw_node/set_mode` 返回 `Eyou_Canopen_Master/SetMode`

## 4. 当前系统状态（最新）

- `set_mode` 类型问题已解决。
- `/controller_manager/*` 服务存在，但控制器加载不稳定（阻塞或空列表）。
- `/arm_position_controller/command` 仍不存在（控制器未 running）。
- 位置命令发布后无动作，符合“无控制器消费者”的现象。

## 5. 影响范围

- 无法进行 CSP 位置控制联调。
- `halt/recover/set_mode` 等生命周期联调受阻。
- 现场验收中的控制器层验证项不可通过。

## 6. 复现步骤（当前可复现）

1. 启动节点（roslaunch 或 roscore+rosrun）。
2. 调用：`rosservice call /controller_manager/list_controllers "{}"`。
3. 观察：调用阻塞，或返回空控制器列表。
4. 验证：`rostopic info /arm_position_controller/command` 显示 `Unknown topic`。
5. 发布位置轨迹命令无动作。

## 7. 技术分析结论

### 7.1 已闭环结论

- CAN 权限问题会导致节点直接启动失败（已识别）。
- `SetMode` 服务类型不匹配是确定性代码缺陷（已修复并验证）。
- 控制器未加载/未运行是“命令发布无效”的直接原因。

### 7.2 未闭环结论（待继续排查）

`controller_manager` service 在当前节点上下文下出现阻塞，尚未定位到最终根因。可疑方向：

1. 控制器插件加载链路异常（pluginlib/依赖包/参数一致性）。
2. 节点主循环在某阶段阻塞，导致 service 回调无法及时处理。
3. 启动方式切换（roslaunch vs roscore+rosrun）导致参数与 spawner 时序不一致。

## 8. 建议的下一步排查（按优先级）

1. 先固定单一启动方式：`roscore` + `rosrun canopen_hw_ros_node`，不带 spawner。
2. 手动依次调用：
   - `/controller_manager/list_controller_types`
   - `/controller_manager/load_controller` (joint_state)
   - `/controller_manager/load_controller` (arm_position)
   - `/controller_manager/switch_controller`
3. 若 `load_controller` 阻塞，抓节点栈：
   - `gdb -p <canopen_hw_ros_node_pid>`
   - `thread apply all bt`
4. 增加主循环节拍日志（`read/update/write/spinOnce` 周期）确认是否卡在 `cm.update` 或 `rate.sleep`。
5. 在控制器成功 running 后再验证话题：
   - `/arm_position_controller/command` 是否出现 subscriber
   - 抓包 `0x205` 是否脱离全零

## 9. 附：与当前问题直接相关的关键证据

- 启动失败日志：`CanController: Operation not permitted`
- service 类型错误：`Unable to load type [canopen_hw/SetMode]`
- 修复后类型验证：`rosservice type /canopen_hw_node/set_mode = Eyou_Canopen_Master/SetMode`
- 控制器不可用证据：`Unknown topic /arm_position_controller/command`
- 控制器空列表证据：`rosservice call /controller_manager/list_controllers "{}" -> controller: []`


## 10. 新增证据（2026-03-21 晚间）

- `rosservice type /canopen_hw_node/set_mode` 已返回 `Eyou_Canopen_Master/SetMode`（说明 service 命名空间修复生效）。
- 关键分水岭测试：`timeout 5s rosservice call /controller_manager/list_controller_types "{}"` / `list_controllers` 出现超时卡住。

新增结论：

- 在 service 类型修复后，controller_manager 仍存在回调饥饿或主循环阻塞问题。
- 问题不再属于服务定义错误，而是运行时调度/阻塞类问题。

建议下一步（强制执行）：

1. 复现场景下附加 `gdb -p <canopen_hw_ros_node_pid>`。
2. 执行 `thread apply all bt` 导出所有线程栈。
3. 优先确认主线程是否卡在 `read/poll/select`（CAN I/O）或互斥锁等待。
