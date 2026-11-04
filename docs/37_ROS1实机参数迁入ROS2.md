# ROS1 实机参数迁入 ROS2（首版）

日期：2026-10-30。根据设备使用者确认，ROS1 历史参数已有实机验证。本次将含义明确的设备参数和控制器约束迁入当前 ROS2；历史参数验证不代替当前 ROS2 驱动、SDK 和控制链的实机验收。

## 版本与来源

保持 ROS2 Humble、RealSense ROS2 `4.58.1 / 53aceeac` 和现有 librealsense `2.58.4`。未切换 ROS1 分支、替换驱动或修改子模块 gitlink。

- 相机来源：`reference/ros1/vision_pkg/launch/vision.launch` 和 `d405_driver_only.launch`。`realsense-ros` 的 `origin/ros1-legacy` 仅只读核对参数接口；其中驱动默认值不视为项目实测参数。
- 控制来源：`reference/ros1/Eyou_ROS1_Master/config/controllers_jtc.yaml`、`reference/ros1/mobility_control/config/base_cmd.yaml`、`reference/ros1/Eyou_Canopen_Master/config/joints.yaml`。

## 已迁入的相机参数

入口：`vision_pkg/launch/cameras.launch.py`。参数可由 launch 覆盖，手爪相机仍需显式 `enable_paw:=true`。

| 项目 | ROS2 值/接口 |
| --- | --- |
| D405 序列号 | `paw_serial=260322273334` |
| 后置 RealSense 序列号 | `behind_camera_type=realsense` 且 `behind_serial` 为空时使用 `112222070518` |
| 后置 Astra+ 序列号 | `behind_camera_type=astra` 且 `behind_serial` 为空时使用 `219323070286` |
| D405 彩色流 | `depth_module.color_profile=1280,720,30`，不是 RGB 模块参数 |
| D405 深度流 | `depth_module.depth_profile=848,480,30` |
| D405 对齐、点云 | `align_depth.enable=true`、`pointcloud.enable=false` |
| 后置 RealSense 点云 | `pointcloud.enable=true`；分辨率/帧率沿用驱动选择，历史配置没有指定 |
| RealSense 红外、同步 | 关闭 |

RealSense 数字序列号传给驱动时增加 `_` 前缀，驱动会剥离，防止 YAML 按整数解释。校验两台 RealSense 的序列号不同。

后置深度对齐保留当前 ROS2 的 `true`，以维持对齐深度消费者接口；ROS1 后置配置未显式开启对齐。这是 ROS2 保留行为，不能标记为 ROS1 实测结论。Astra+ 保留 ROS2 现有红外关闭、深度配准设置。

```bash
ros2 launch vision_pkg cameras.launch.py enable_paw:=true
# 只启动手爪相机
ros2 launch vision_pkg cameras.launch.py behind_camera_type:=none enable_paw:=true
# 使用历史 Astra+ 序列号
ros2 launch vision_pkg cameras.launch.py behind_camera_type:=astra
```

## 控制参数

入口：`robot_hardware/config/controllers.yaml`。

| 项目 | 迁入或核对结果 |
| --- | --- |
| 六轴 JTC | 迁入 goal_time `2.0 s`、停止速度容差 `0.05 rad/s`、各轴轨迹容差 `0.10 rad`、目标容差 `0.02 rad` |
| 摆臂 CSP JTC | 迁入 goal_time `1.0 s`、停止速度容差 `0.05 rad/s`、各轴轨迹容差 `0.10 rad`、目标容差 `0.03 rad`；状态发布 `50 Hz`、动作监测 `20 Hz` |
| 底盘 | 补齐发布 `50 Hz`、位姿/速度协方差对角线 `[0.001, 0.001, 1000000, 1000000, 1000000, 0.03]` |
| 底盘速度/加速度 | 补齐控制器线速度上限 `0.8 m/s`、角速度上限 `1.5 rad/s`、线加速度 `1.0 m/s²`、角加速度 `2.0 rad/s²`，与已迁入中继限幅一致 |
| 履带几何 | 保留最终 ROS1 控制器的等效轮距 `0.36 m`、半径 `0.075 m` 和超时 `0.3 s` |
| CANopen | `legacy_joints.yaml` 与历史 `joints.yaml` 内容相同，节点 ID、编码器比例及 IP 等轴参数已继承，本次不重复覆盖 |
| 底盘中继 | 速度、加速度、jerk、`50 Hz` 和 `0.3 s` 已在 `mobility_control/config/base_cmd.yaml` 继承 |

ROS2 保留统一 controller_manager `200 Hz`、控制器会话隔离、命令超时、Servo/JTC 互斥和摆臂完整目标要求。ROS1 的控制器类型、`stop_trajectory_duration` 和 `allow_partial_joints_goal=true` 不直接覆盖当前 ROS2 行为。

## 尚待处理与实机复核

- 历史底盘中继的 `wheel_separation=0.438` 与最终控制器 `0.36` 不同：本版使用最终控制器的等效值，仍需通过现场测量与转向/里程计测试核验。
- ROS1 夹爪速度 `±0.8` 标注 rad/s；当前 ROS2 是米制位置积分，保留 `±0.02 m/s` 与 `0.2 s` 保持时间，缺少传动换算依据时不迁入旧数值。
- ROS1 摆臂 `direction_corrections` 是上层操作方向约定；当前 JointJog 使用关节坐标，不能直接作为硬件编码器方向修正。手柄映射与操作方向仍需独立对照，未覆盖 ROS2 现有按住运行、epoch 清除和失焦停止行为。
- 不迁入旧 marker `6x6_250 / 582`、未经 ROS2 核验的自动采样点或设备持久化零点；设备 `commissioned:false`、自动采样 `commissioned:false` 保持。
- 需使用上述参数重新验证 ROS2 实机出流、USB/固件、设备重连、轨迹跟踪、真实停止、几何/里程计和安全关联条件；本次不勾选清单 36 的实机验收项。

## 本次软件验证

- `bash tools/build_ros2.sh /tmp/robot24-param-migration core`：18 个核心包构建通过；仅有未使用 `INSTALL_UDEV_RULES` 的 CMake 提示。
- 相机 launch 参数展开检查通过：双 RealSense、D405 profile、Astra+ 序列号选择、关闭相机、覆盖参数、重复序列号拒绝；传入 RealSense 的参数名全部存在于当前 ROS2 驱动。检查脚本 `/tmp/check_ros2_migrated_cameras.py`。
- 控制 YAML 与历史容差、底盘限幅/协方差逐项比对通过；确认统一 `200 Hz` 及两个 `commissioned:false` 保持。
- `robot_safety` 2 个、`can_driver` 10 个、`robot_hardware` 3 个 CTest 入口通过。私有 CAN 分组测试首次受沙箱日志目录/本地 DDS 限制影响，使用 `/tmp` 日志和允许本地通信的环境重跑通过；硬件测试在 source 完整安装环境后通过。
- 安装后的 `full_system_smoke.py` 和 `moveit_smoke.py` 均通过，覆盖公共底盘/夹爪命令、会话复位、规划执行与取消等。报告 `/tmp/robot24-param-migration-control.log`、`/tmp/robot24-param-migration-moveit.log`；构建和 CTest 日志位于 `/tmp/robot24-param-migration-*`。
- 本次没有重跑 37 包 full 回归、真实相机出流或真实电机测试；以上软件结果不能勾选现场验收项。
