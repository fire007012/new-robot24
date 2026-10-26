# ROS1 迁移 ROS2 实施计划：当前状态与剩余工作

更新：2026-10-26。项目已确定使用 ROS2 Humble，正式代码不再支持 Catkin，不再要求 ROS1 回退基线。历史代码已移到 [reference/ros1](../reference/ros1/README.md)，不会被 colcon 发现。

## 已完成的软件工作

- [x] CANopen 与私有 CAN 统一到一个 controller_manager 和两个硬件组件；13 个关节所有权明确。
- [x] 六轴 JTC、夹爪 CSP 积分、履带速度超时、摆臂 CSV/CSP 互斥及维护入口。
- [x] arm/base/flippers/gripper 分组、关联联锁、命令 epoch、新鲜度、协调器租约、故障锁存和显式恢复。
- [x] ROS2 模型和有限 TF、MoveIt 2 OMPL/KDL、ExecuteArmGoal 规划执行、取消与并发门控。
- [x] Servo 与 JTC 互斥、腕部服务、键盘/手柄、原生 Qt 界面。
- [x] Fortress 物理仿真、RGB/深度/IMU/CameraInfo、仿真暂停与过期输入验证。
- [x] ROS2 相机驱动与 SDK 版本匹配、ArUco/easy_handeye2、自动采样客户端。
- [x] 视觉、H265、TCP/JSON 上位机桥、视频管理、CO₂ 和热成像 ROS2 入口。
- [x] 37 包全量构建、11 包 41 个 CTest 入口及软件集成验收，范围见 [34](34_ROS2全量软件集成与验收.md)。
- [x] 工作区整理：ROS1 包和混合包旧源码集中参考、旧 CANopen 子模块解除关联、`host_bridge`/`eye_in_hand` 目录统一、Noetic CI 改 Humble、清除完成批次日志与 evidence。

软件已实现不表示真实设备验收已完成。具体架构见 [01](01_系统概述与硬件拓扑.md)、[03](03_架构设计与决策过程.md)、[06](06_项目架构图与数据流向图.md)。

## 剩余交付出口

| 工作 | 当前状态与验收要求 |
| --- | --- |
| CAN 单轴与整车 | 待实机。核验身份/固件、PDO/IP、单位/方向、零点/软限位、CSV/CSP、真实停止/制动、断线及恢复、总线负载和周期抖动。按 [28](28_CANopen软件交付与实机验收.md) 执行 |
| 分组关联条件 | 待现场传感器/姿态/制动/载荷输入、超时及共享通道故障验证。按 [31](31_分组隔离与局部恢复实施验收.md)、[32](32_分组隔离现场验收记录模板.md) 登记，恢复不允许自动 Running |
| 几何与规划 | 待实测轮距、方向、夹爪行程和六轴 KDL 目标集精度；当前 wheel_separation=0.36 不是现场测量结论 |
| 相机 | 待真实 RealSense/D405 与交付范围内 Astra+ 验证；固定序列号、USB/固件、缺失/多设备、彩色/对齐深度、内参/光学 frame/时间戳 |
| marker 与手眼标定 | 待核对实际字典、ID、尺寸，真实图像与 CameraInfo 一致性、自动采样、取消、缺失 marker/TF 和标定精度；auto_poses_ros2.yaml 保持 commissioned:false |
| 视觉与视频 | 待真实 YOLO 权重、镜头标定、全景、H265、MediaMTX 与完整 RTSP 发布/拉流；空源视频控制测试不能证明视频传输 |
| 外围与上位机 | 待真实 CO₂、热成像、HMI 模式按钮、视频启停及整车回归 |
| 发布与部署 | 待保存父仓库和子模块匹配提交、现场参数、依赖版本、安装搬迁结果及 ROS2 版本恢复/设备参数恢复说明；不包含 ROS1 恢复 |

旧 `6x6_250 / 582` 组合超出字典范围，不得直接沿用。实际 ID 必须由现场指定；相机安装姿态和标定外参各只有一个权威发布者。

## 依赖与工作区约定

- 正式平台：Ubuntu 22.04 / ROS2 Humble；MoveIt 2 的 OMPL + KDL；Gazebo Fortress 6。
- RealSense 驱动 `4.58.1 / 53aceeac` 配 librealsense `2.58.4`；Orbbec SDK v1 `main / 0ea4ab68`；ArUco `5.0.6 / 2872a00`；easy_handeye2 `0.5.0 / 29bd50a`。
- 活动子模块共 6 个，见 [子模块工作流](git_submodule_workflow.md)。本地未提交修复须纳入匹配发布，不能仅依赖旧 gitlink。
- `.deps/librealsense` 是当前 SDK，不是测试缓存。`robot_canopen_config/config/legacy_*` 是活动配置生成和校验输入，继续保留。
- 持续回归源码保留；构建、日志、批次 evidence 和截图输出到源码树外。已完成测试只保留结论和重跑命令。

## 继续执行

构建和全量验证命令见 [README](../README.md)。下一步以现场设备登记、单轴低速验收和真实关联条件接入为先，再做整车、外围和发布出口。不在缺少实物输入时勾选现场验收。
