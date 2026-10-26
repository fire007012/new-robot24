# CANopen 后端迁移：实现状态与实机待办

更新：2026-10-26。ROS2 正式后端为 `ros2_canopen`。原 `Eyou_Canopen_Master` 已解除子模块关联，和 `Eyou_ROS1_Master` 一并移入 `reference/ros1/`，仅供协议及行为参考；不再安排 ROS1 回退或等待实机后才整理旧包。

## 当前实现

- 唯一 controller_manager 加载 `canopen_ros2_control/RobotSystem` 与 `can_driver`，CANopen 的 DeviceContainer 在硬件插件内部；维护与运动不另起第二主站。
- `robot_canopen_config` 从可校验输入生成整车及六份单轴 bus/DCF/bin，审计 PDO、模式、节点及启动 SDO。配置路径相对安装目录，可搬迁。
- 四摆臂 Node-ID 1–4：CSV 9 / CSP 8；肩 yaw=6、pitch=5：IP 7。六轴 JTC 的另外四轴由私有 CAN 拥有。
- 双后端遵循 `robot_safety` 分组许可、epoch、新鲜度和心跳租约。未知或过期关联条件限制运行，故障恢复不自动 Running。
- 维护包括身份核验、零点/限位、保存、读回和不确定 journal；不在普通启动时写持久化零点/软限位。
- 分组 P0–P5 软件工作和离线验证已完成；当前实现和接口见 [31](31_分组隔离与局部恢复实施验收.md)，整车软件验收见 [34](34_ROS2全量软件集成与验收.md)。

## 实机出口

| 阶段 | 待完成项 |
| --- | --- |
| M0 基线 | 实际设备型号/身份/固件/接线，方向和机械标签，原持久化零点/限位读回，commissioning 现场副本 |
| M1 配置 | 每轴 PDO/COB-ID、CSV/CSP/IP 模式确认、单位/比例、心跳/EMCY、IP 双通道与 halt/release 时序 |
| M2 低速与停止 | 首次使能无跳变、命令超时、Quick Stop、模式切换、重启/重连及真实制动行为 |
| M3 双后端与维护 | 跨后端六轴轨迹/取消、通道故障、维护失败与保存/掉电/恢复读回 |
| M4 性能 | 200 Hz/5 ms 周期的现场抖动、SYNC、总线负载、丢帧及停止时延；阈值由现场确认 |
| P6 分组与发布 | 真实姿态/支撑/载荷条件、局部故障及恢复、健康组连续运行、匹配版本和 ROS2 恢复说明 |

按 [操作单 28](28_CANopen软件交付与实机验收.md)、[分组计划 29](29_分组故障隔离与局部恢复执行计划.md) 和 [现场模板 32](32_分组隔离现场验收记录模板.md) 执行。模拟硬件、虚拟 CAN 或离线读写不能填作真实制动/保存证据。

## 软件复现

```bash
# 从仓库根目录执行；结果写到 /tmp。
bash robot_hardware/scripts/validate_offline.sh /tmp/robot-canopen-offline
# 当前整车核心/全量集成：
bash tools/validate_ros2.sh /tmp/robot24-ros2 full
```

持续保留测试源码与采集工具；已完成批次的 logs/evidence 不再放源码树。现场新采集须使用唯一外部输出目录。恢复旧 ROS2 版本前 halt/shutdown 并确认主站退出；改写过设备参数时按维护前基线恢复、读回及必要的掉电核验，软件版本切换不能替代参数恢复。
