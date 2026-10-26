# ROS2 主控制链与 MoveIt 2 实施记录

> 后续已继续完成上层、外围与 Fortress 的软件集成，并修复控制器退出问题。本文保留第一轮批次范围；当前状态与操作见 [记录 34](34_ROS2全量软件集成与验收.md)。

日期：2026-10-26。范围：计划 25 中当前已有依赖的构建、模型／TF、主控制入口和 MoveIt 2。CANopen 和私有协议子仓库复用现有 ROS2 实现；实机、Fortress、视觉、标定、HMI 和视频的完成状态分别记录，不以 mock 代替实测。

## 本轮实现

- `robot_bringup`、`car_urdf`、`car_moveit_config`、`mobility_control`、`arm_control` 的正式构建入口切换为 ament。
- ROS1 的 XML launch、脚本和原始模型移至 `reference/ros1/` 作为参考；项目不再安排 ROS1 回退，正式构建只支持 ament。
- 新增整车 ROS2 实机／mock 启动入口：一个 controller_manager，双硬件组件，默认启用分组策略；启动不会自动 init、enable 或 resume。
- 从原整车模型提取 visual、collision、inertial 和关节，移除 ROS1 transmission 和 Classic 插件。机械臂组、末端、碰撞矩阵及 named targets 保留。ROS2 运动学模型以 `base_link` 为根，移除导出器占位根及旧 fixed virtual joint，避免与底盘唯一的 `odom → base_link` 发布者形成双父节点。模型专用 joint_states 转发器只转发原始测量中的有限位置，保留原时间戳；未配置或关闭的轴不会用 NaN 生成 TF，也不补造零位。硬件安全层和执行就绪检查仍读取原始 `/joint_states`。修正右夹爪 mimic 的米制限位，使其与左夹爪一致。
- 底盘使用 C++ rclcpp + Ruckig 速度模式，保留加速度和 jerk 限制；输出 `/wheel_controller/cmd_vel_unstamped`。不再在上层重复定义轮距或方向修正，几何参数由硬件包控制器配置提供。当前 `wheel_separation=0.36` 是沿用配置，仍须现场测量确认，不能视为已解决旧 `0.438` 差异。
- 夹爪保留北向 Bool／Float64，南向改为单元素 Float64MultiArray，单位 m/s。Bool 默认 ±0.02 m/s、保持 0.2 s；速度输入也有保持超时。南向复用已有超时、限位及 CSP 位置积分控制器。
- 底盘和夹爪适配器使用单调时间，校验非有限值，运行态过期或会话变化时清空目标，停机期间收到的目标不用于后续启动。
- 摆臂复用 `robot_hardware` 已迁移的 JointJog、CSP 轨迹及 profile／联动模式接口，不另启动 ROS1 `flipper_manager_node`。
- MoveIt 2 使用 Humble 已安装的 OMPL、KDL 和单个六轴 JTC。原 TRAC-IK 依赖未安装，首版选择标准 KDL；该变更的求解性能仍需实机目标集对照。
- 本机 MoveIt 2.5.10 在退出时出现 CallbackGroup 析构崩溃，gdb 定位到插件卸载后的节点清理路径；规划插件对已加载的相关库使用 `RTLD_NOLOAD | RTLD_NODELETE` 保持代码映射到进程退出，不修改系统安装或提前退出进程。正式规划进程改用只返回空控制器列表、不创建执行句柄的 `PlanningOnlyControllerManager`；真实六轴执行仍由项目 Action 送入同一个 JTC，无伪造执行结果。正式 `move_group` 设置 `allow_trajectory_execution=false`，提供规划和场景服务；执行通过项目 Action 门控后送往 JTC，避免并行 MoveGroup 执行入口绕过项目仲裁。该开关见 [Humble 官方启动实现](https://moveit.picknik.ai/humble/api/html/launches_8py_source.html)。C++ `arm_goal_executor` 采用 MoveGroupInterface 规划，将规划轨迹送往同一个六轴 JTC，保留 ExecuteArmGoal 的目标类型、反馈阶段和错误码。检查运行许可、反馈新鲜度及控制器 active 状态，支持取消和停机收敛。一次接受一个项目目标；并发新目标明确拒绝，客户端应先取消并等待旧目标终止。
- 本轮主链完成后，Servo 互斥仲裁、腕部服务、遥控与 HMI 已继续完成软件迁移，详见记录 34。

## 构建与使用

从仓库 `src` 执行，构建脚本显式选择 ROS2 包路径，排除保留的 ROS1 包和旧 vendor 示例：

```bash
bash tools/build_ros2.sh /tmp/robot24-ros2
source /tmp/robot24-ros2-install/setup.bash
ros2 launch robot_bringup full_system_mock.launch.py
# 实机：ros2 launch robot_bringup full_system.launch.py
# 仅控制及 TF：在上述命令末尾加 enable_moveit:=false
```

mock 仍要求显式设置测试用的安全条件，正式设备不得使用这些 mock 接口：

```bash
ros2 service call /robot_runtime/init std_srvs/srv/Trigger '{}'
ros2 service call /mock/conditions/arm_secured std_srvs/srv/SetBool '{data: true}'
ros2 service call /mock/conditions/support_trusted std_srvs/srv/SetBool '{data: true}'
ros2 service call /mock/conditions/payload_retained std_srvs/srv/SetBool '{data: true}'
ros2 service call /robot_runtime/enable std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/resume std_srvs/srv/Trigger '{}'
ros2 run robot_bringup check_ready --ros-args -p require_running:=true -p require_moveit:=true
ros2 run arm_control send_execute_arm_goal --named zero
```

实际硬件应先完成文档 28、31、32 的登记和验收；`commissioning.yaml` 不完整时 init 拒绝放行。mock 的 identity、位置跟随和安全条件不能用于证明设备能力。

## 验证

统一入口：`bash tools/validate_ros2.sh /tmp/robot24-ros2`。脚本使用独立 build／install／log，运行底层回归、Action 合同、整车控制入口及 MoveIt 规划执行测试。本机须允许回环 DDS 通信。该批次统一验证已通过，原始输出已清理，结果摘要如下：

| 检查 | 结果 |
| --- | --- |
| 显式 ROS2 包构建 | 16 个包通过，单一 `/tmp/ros2-migration-install` 安装前缀 |
| 底层／合同回归 | 11 个包、41 项 CTest 全部通过；其中 ExecuteArmGoal 合同含 5 个断言用例 |
| 整车控制入口 | 通过：默认 inactive、显式生命周期、跨后端 JTC、底盘／夹爪超时、Bool 输入、NaN、会话恢复、有限 TF 及就绪检查 |
| MoveIt 2 | 通过：named、joints、基于实测 TF 的 Pose、缺失 TF、并发拒绝、运动中取消、halt 撤权、错误码及退出无崩溃 |

Pose 可选择等效 IK 分支，低速轨迹可能超过 20 秒；测试按执行实际情况保留 90 秒等待上限，执行器另按规划轨迹时长加余量监测超时。SIGINT／SIGTERM 由执行器先处理取消与目标结束，再关闭 ROS 上下文。初始化为 unconfigured、关闭硬件、以及物理条件未知时的状态均保留真实含义，不用模型显示补造就绪。

## 当前剩余出口

后续批次已完成 Servo、腕部/遥控、Fortress、相机驱动/SDK 匹配、ArUco/标定、视觉、CO₂、热成像和 HMI/视频软件迁移。当前 Orbbec 为支持 Astra+ 的 SDK v1 `main / 0ea4ab68`。真实设备、真实 marker/模型/视频源、精度和匹配发布仍待验收，以 [计划 25](25_ROS1迁移ROS2实施计划.md) 和 [记录 34](34_ROS2全量软件集成与验收.md) 为准。
