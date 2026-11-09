# ROS2 双后端硬件运行入口

本包与 `robot_canopen_config`、`robot_safety` 组成 CANopen 迁移后的 ROS2 控制链。目标环境为 ROS2 Humble，使用一个 controller_manager 管理六个 CANopen 轴和七个私有协议轴。六轴机械臂的 JTC 跨两个后端；摆臂 CSV/CSP 控制器互斥。旧 ROS1 包已归入 `reference/ros1/`，仅供行为参考，项目不再要求 ROS1 回退。

完整操作及现场出口见 [CANopen 软件交付与实机验收](../docs/28_CANopen软件交付与实机验收.md)。

控制周期默认 5 ms / 200 Hz。调试安装可在构建 `robot_canopen_config` 时指定
`--cmake-args -DROBOT_CONTROL_PERIOD_MS=20`，生成 20 ms / 50 Hz 的全轴及单轴配置。
本包两种 launch 根据已安装的 bus/DCF 推导 manager 频率，启动前检查驱动、SYNC、
插补周期与 master DCF 一致；不能只改单独的 controllers.yaml。切换后需加载匹配安装环境。
现有诊断字段 `period_over_7_5ms` 仍表示超过 7.5 ms 的原始次数，以 200 Hz 为参考；
在 50 Hz 调试档下正常周期也会计入，不能据此单项判断周期异常。

只读采集 CAN 统计可在源码根目录执行
`python3 tools/capture_can_health.py canable1 --samples 60 --interval 1 --output /tmp/can-health.json`。
工具不发送 CAN 帧或重配接口，保留完整内核统计与逐段增量；接口重建、计数回退时标记不可比较。
该结果用于通信排查，实际目标帧积压和机械停止时延还需要抓包及实机测量。

```bash
# 在仓库 src 目录执行；测试需要 ROS2 本机回环通信。
robot_hardware/scripts/validate_offline.sh /tmp/canopen-final
source /tmp/canopen-final-install/setup.bash
ros2 launch robot_hardware hardware.launch.py mock:=true journal:=/tmp/mock-maintenance.json
```

上述空 `group_policy` 示例使用全局联锁诊断模式。启动后依次显式调用 `/robot_runtime/init`、`/robot_runtime/enable`、`/robot_runtime/resume`，类型均为 `std_srvs/srv/Trigger`。`enable` 进入 Armed；`resume` 才启动运动控制器并进入 Running。`halt` 撤销运行许可；该模式的 `recover` 重建两侧硬件并重新校验，最终保持 Armed；`shutdown` 清理两侧。整车默认分组模式下，普通组恢复不重建总线，确需重建后端时使用 `recover_resource`；真实关联证据缺失会阻止相应组运行。

| ROS2 接口 | 合同 |
| --- | --- |
| `/arm_position_controller/follow_joint_trajectory` | 六轴唯一机械臂 FollowJointTrajectory 入口 |
| `/robot_runtime/set_flipper_profile` | `robot_hardware/srv/SetProfile`，`profile: csv` 或 `csp`；串行停用旧控制器再启动新控制器 |
| `/flipper_control/jog_cmd` | `control_msgs/msg/JointJog`，SI rad/s，支持现有五种联动模式 |
| `/flipper_control/command` | CSP 的四摆臂 `JointTrajectory`，检查关节、时间、有限值与 URDF 范围 |
| `/flipper_control/set_linkage_mode` | `SetProfile`，profile 为 independent/left_right_mirror/front_rear_sync/side_pair/diagonal_pair |
| `/gripper_controller/command` | `Float64MultiArray` 一元素，单位 m/s；带 250 ms 超时、限速、行程限制的 CSP 积分 |
| `/wheel_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist`，标准底盘速度，300 ms 超时 |
| `/robot_runtime/maintain` | `Maintain`；set_zero/set_offset/limits/limits_from_urdf/disable_limits/restore；全局模式要求先 halt，分组模式先停止目标所属组并按关联条件限制其他组 |
| `/robot_runtime/snapshot` | `Trigger`，只读导出身份、固件及持久化基线，保持 commissioned=false |
| `/robot_runtime/state` | JSON 状态、当前摆臂 profile、维护不确定结果标志 |

项目配置禁用启动 Homing 和自动写限位。原始单轴运动 service/PDO topic 写入在托管模式下关闭；控制器资源切换是模式切换入口。维护 SDO 复用同一主站并单飞，读回失败不会使用缓存值代替设备响应。

生成的 Eyou 配置启用 `enable_eyou_zero_mode_support_fallback`，仅对真实身份为 `1000=0x00220811`、厂商 `0x668`、产品 `0x00221401`、修订 `16`、固件 `V133-0262` 且 `6502=0` 的 PH14，允许尝试 IP 7 / CSP 8 / CSV 9。通用驱动默认关闭该兼容；任何非零能力位掩码仍以设备读回为准，SDO 失败不回退到 EDS 默认值。模式必须通过实际 `6061` 和使能状态确认才能建立新会话。项目以 `switching_state: 4` 在 Switched On 状态切换模式；Quick Stop 完成于 Switch On Disabled 时，disable 保持失能，不重新上电。位置控制指 CSP，不是 Profile Position 1；本兼容不开放额外转矩或 Homing 模式。

`hardware.launch.py` 默认真实插件，但硬件保持 unconfigured；没有现场 `commissioning.yaml` 的完整登记，init 校验不能通过。`mock:=true` 加载专用模拟插件与明确标识的模拟身份，不连接 CAN。单轴实机入口为 `single_axis.launch.py`，只配置目标节点，控制器同样默认 inactive。

整车 `robot_bringup/full_system.launch.py` 默认启用四组策略；本包 `hardware.launch.py` 的空 `group_policy` 使用 ROS2 全局联锁诊断模式。分组 API、关联条件与恢复见 [实施验收 31](../docs/31_分组隔离与局部恢复实施验收.md)。
