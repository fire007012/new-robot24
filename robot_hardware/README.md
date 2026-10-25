# ROS2 双后端硬件运行入口

本包与 `robot_canopen_config`、`robot_safety` 组成 CANopen 迁移后的 ROS2 控制链。目标环境为 ROS2 Humble，使用一个 controller_manager 管理六个 CANopen 轴和七个私有协议轴。六轴机械臂的 JTC 跨两个后端；摆臂 CSV/CSP 控制器互斥。旧 ROS1 包保留供行为对照及实机回退。

完整操作及现场出口见 [CANopen 软件交付与实机验收](../docs/28_CANopen软件交付与实机验收.md)。

```bash
# 在仓库 src 目录执行；测试需要 ROS2 本机回环通信。
robot_hardware/scripts/validate_offline.sh /tmp/canopen-final
source /tmp/canopen-final-install/setup.bash
ros2 launch robot_hardware hardware.launch.py mock:=true journal:=/tmp/mock-maintenance.json
```

启动后依次显式调用 `/robot_runtime/init`、`/robot_runtime/enable`、`/robot_runtime/resume`，类型均为 `std_srvs/srv/Trigger`。`enable` 进入 Armed；`resume` 才启动运动控制器并进入 Running。`halt` 撤销运行许可；`recover` 重建两侧硬件并重新校验，最终保持 Armed；`shutdown` 清理两侧。

| ROS2 接口 | 合同 |
| --- | --- |
| `/arm_position_controller/follow_joint_trajectory` | 六轴唯一机械臂 FollowJointTrajectory 入口 |
| `/robot_runtime/set_flipper_profile` | `robot_hardware/srv/SetProfile`，`profile: csv` 或 `csp`；串行停用旧控制器再启动新控制器 |
| `/flipper_control/jog_cmd` | `control_msgs/msg/JointJog`，SI rad/s，支持现有五种联动模式 |
| `/flipper_control/command` | CSP 的四摆臂 `JointTrajectory`，检查关节、时间、有限值与 URDF 范围 |
| `/flipper_control/set_linkage_mode` | `SetProfile`，profile 为 independent/left_right_mirror/front_rear_sync/side_pair/diagonal_pair |
| `/gripper_controller/command` | `Float64MultiArray` 一元素，单位 m/s；带 250 ms 超时、限速、行程限制的 CSP 积分 |
| `/wheel_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist`，标准底盘速度，300 ms 超时 |
| `/robot_runtime/maintain` | `Maintain`；set_zero/set_offset/limits/limits_from_urdf/disable_limits/restore；运行时拒绝写入 |
| `/robot_runtime/snapshot` | `Trigger`，只读导出身份、固件及持久化基线，保持 commissioned=false |
| `/robot_runtime/state` | JSON 状态、当前摆臂 profile、维护不确定结果标志 |

项目配置禁用启动 Homing 和自动写限位。原始单轴运动 service/PDO topic 写入在托管模式下关闭；控制器资源切换是模式切换入口。维护 SDO 复用同一主站并单飞，读回失败不会使用缓存值代替设备响应。

`hardware.launch.py` 默认真实插件，但硬件保持 unconfigured；没有现场 `commissioning.yaml` 的完整登记，init 校验不能通过。`mock:=true` 加载专用模拟插件与明确标识的模拟身份，不连接 CAN。单轴实机入口为 `single_axis.launch.py`，只配置目标节点，控制器同样默认 inactive。
