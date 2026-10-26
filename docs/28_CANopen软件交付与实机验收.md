# CANopen 软件交付与实机验收

对应 [迁移计划](26_CANopen后端迁移计划.md) 和 [执行记录](../reference/ros1/docs/历史文档/27_CANopen迁移执行记录.md)。本文给出软件验证入口与现场验收步骤；实测数据不得由模拟结果代填。

> 状态更新：整车 `robot_bringup/full_system.launch.py` 默认启用分组策略；底层 `robot_hardware/hardware.launch.py` 在 group_policy 为空时使用全局联锁。分组隔离与局部恢复的软件实现、操作增量和离线证据见 [实施验收 31](31_分组隔离与局部恢复实施验收.md) 及 [计划 29](29_分组故障隔离与局部恢复执行计划.md)。直接启动底层硬件入口时，分组模式须显式提供 group_policy，真实姿态/制动/支撑/载荷证据尚待现场接入和验证，不能用模拟条件替代。下文旧全车操作与分组操作需按启动模式选择。

## 1. 软件组成与边界

- `robot_canopen_config`：六轴和六份独立单轴 bus/DCF/bin、EDS、可重复生成与映射/启动写入审计。安装的 bus 使用 `dcf_path: .`，在运行时按 bus 文件目录解析，可迁移安装目录。
- `ros2_canopen` 项目补丁：RobotSystem 接口切换、生命周期清理、模式/状态确认、命令 epoch、通信新鲜度、NMT/心跳/EMCY/总线状态门控、肩轴 IP 双通道及 SDO 超时/字符串上传。
- `robot_safety`：同一进程的两个硬件组件共享许可；全局诊断模式使用统一状态/epoch，整车分组模式使用四组独立状态/epoch、关联证据、全局禁止和协调器心跳租约；故障不自动恢复 Running。
- `can_driver` 插件补丁：首次使能前按反馈预置位置/零速，等待使能及模式反馈，资源停用、反馈故障、越界或写入失败撤销命令并停止。
- `robot_hardware`：唯一 controller_manager 的启动入口、统一生命周期、六轴 JTC、摆臂互斥控制器和联动参考、夹爪 CSP 速度积分、受控维护、模拟后端与验收采集工具。

六轴机械臂为 shoulder_yaw、shoulder_pitch、elbow_pitch、wrist_pitch、wrist_yaw、wrist_roll；前两轴使用 CANopen，其余四轴使用私有 CAN。四摆臂属于 CANopen，夹爪和两履带电机属于私有 CAN。共 13 个物理关节，所有权不重叠。

正式 ROS2 硬件入口不依赖旧 ROS1 主站或桥。上层及外围已完成 ROS2 软件迁移，现场验收按计划 25 继续推进；本次保留统一 JTC 的 ROS2 Action 合同，并迁移 CANopen 运行、维护和摆臂调用路径。未发现本项目上层需要独立 CANopen Ruckig Action 的调用，因此不另设插补或 Action 入口。

## 2. 离线复现

在 `src` 目录：

```bash
robot_hardware/scripts/validate_offline.sh /tmp/canopen-final
source /tmp/canopen-final-install/setup.bash
```

脚本在指定 `/tmp` 前缀构建，使用系统已安装的 Lely，执行上游单元/组件测试、项目安全/维护/接口测试、Lely 用户态虚拟 CAN、双模拟后端整链，以及真实单轴插件的未配置加载检查。ROS2 进程测试需要本机回环 DDS 套接字；无需真实 CAN 接口，也不激活真实硬件。

结果保留在该前缀的 `-build`、`-log`、`-roslogs`、`-hybrid.log`、`-groups.log`、`-install-relocated-<随机后缀>`、`-report.json`。报告包含基础提交、工作树源文件 SHA-256、生成产物及安装库 SHA-256 和系统依赖版本。未提交的子模块补丁必须随交付一并保存，父仓库原 gitlink 不能单独代表本次软件状态。

启动写入审计拒绝 bin 中的 `0x607C/0x607D/0x2003/0x1010/0x1011`，验证单轴与全轴的对应 bin 一致、所有 bin 引用相对路径、所有节点 `1F81=0x00000005`。当前 dcfgen 的默认 retry_factor=3 曾产生 `0x305`；项目显式配置 retry_factor=0 恢复原启动语义。新增 TIME 对象为生成器的主站字典结构，原有控制映射保持严格比对。

### 模拟运行

```bash
ros2 launch robot_hardware hardware.launch.py mock:=true journal:=/tmp/mock-maintenance.json
ros2 service call /robot_runtime/init std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/enable std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/resume std_srvs/srv/Trigger '{}'
```

模拟数据使用 `MOCK` 身份，不能作为现场 commissioning.yaml。完整自动回归还覆盖六轴轨迹完成、取消、CSV 超时归零、CSV/CSP 切换、分别注入两个后端故障、共同停止、恢复后保持 Armed、维护读回和关闭。

## 3. 首次现场登记：先只读

1. 保存当前 ROS2 匹配源码、启动参数和设备原始参数，确认总线终端、供电、急停和电机标签。按设备记录或失能后的手动编码器观测登记方向。停止旧主站及旧硬件控制进程后再启动新主站。
2. 记录 `ip -d -s link show canable0` 和 `canable1` 输出、实际波特率及接线。仓库默认值为 1 Mbit/s，不替代现场确认。
3. 新入口默认不自动激活硬件、运动控制器或执行 Homing。DCF 启动会配置易失 PDO、模式、心跳与 IP 周期；不会写零点、软限位或保存对象。
4. 按下一节逐轴启动并只读导出。`axis_commission` 不带 `--activate` 或 `--operation` 时仅配置主站和读取对象，不使能运动。

```bash
# 终端 1，仅注册目标节点，其他 CANopen 节点不参与这次启动。
ros2 launch robot_hardware single_axis.launch.py joint:=shoulder_yaw_joint can_interface:=canable0

# 终端 2，只读身份、固件、零点和软限位。
ros2 run robot_hardware axis_commission shoulder_yaw_joint
```

分别对 Node-ID 1/2/3/4/6/5 执行并保存输出，停掉前一个单轴主站后再换轴。将结果填入 `robot_hardware/config/commissioning.yaml` 的现场副本，保留：

- `1000:00`，`1018:01..04`，`100A:00` 的真实读回值；分别用于设备类型、厂商/产品/修订/序列号和固件字符串核验。
- `607C:00`，`2003:00`，`607D:01/02` 的原始无符号 32 位读回值及有符号解释；单独记录保存支持状态、旧方向/零位及物理标签。
- 每轴 `physical_label`、实际方向的证据，确认后才设置 `direction_verified: true`；资料齐全后设置 `commissioned: true`。

不要把 EDS 中的身份默认值填成设备实测值。原始 EDS 与生成器 EDS 的身份默认值不同，启动身份检查以现场副本为依据。

## 4. 单轴低速验收

完成对应节点的现场基线后：

```bash
ros2 run robot_hardware axis_commission shoulder_yaw_joint --baseline /实际路径/commissioning.yaml --activate
```

该命令先比对身份、固件、已启用 PDO 的 COB-ID/类型/映射和禁用 PDO 的关闭状态，然后激活硬件及单轴控制器。禁用 PDO 的映射内容不参与运动合同；不要求 dcfgen 改写已禁用通道的无效映射。

- 肩轴：`/axis_controller/follow_joint_trajectory`，`control_msgs/action/FollowJointTrajectory`。以实时反馈为起点，先做约 0.01 rad、至少 2 s 的轨迹。目标数值由现场反馈计算，不能假定当前位置为 0。
- 摆臂：`/axis_controller/command`，`std_msgs/msg/Float64MultiArray` 的一元素 rad/s 速度。先用低于 0.02 rad/s 的短时命令；确认停发后 250 ms 内控制器归零，并读回真实停止结果。
- 记录正反向、位置/速度单位、零位、URDF 边界、启用后无新命令、停用/再启用、Quick Stop/恢复、模式往返及断电重连。肩轴同时抓取 `0x60C1:01` 和 `0x607A`，确认镜像、控制字 halt/release 的真实时序。

若固件 Quick Stop 选项使设备最终进入其他状态，保留状态字序列和 `0x605A` 读回；软件等待 Quick Stop 确认失败时保持会话关闭，不通过反复 resume 绕过。

## 5. 六轴与双后端验收

停止单轴主站，使用完整现场基线：

```bash
ros2 launch robot_hardware hardware.launch.py baseline:=/实际路径/commissioning.yaml journal:=/实际路径/maintenance.json
ros2 service call /robot_runtime/init std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/enable std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/resume std_srvs/srv/Trigger '{}'
```

init 只配置、读回和核验；enable 最终为 Armed；resume 建立全新控制器会话并进入 Running。默认摆臂为 CSV。

```bash
ros2 service call /robot_runtime/set_flipper_profile robot_hardware/srv/SetProfile '{profile: csp}'
ros2 service call /robot_runtime/halt std_srvs/srv/Trigger '{}'
ros2 service call /robot_runtime/recover std_srvs/srv/Trigger '{}'
# recover 完成后保持 Armed，需要重新 resume 和发送新目标。
```

使用唯一 `/arm_position_controller/follow_joint_trajectory` 验证跨 CANopen/私有 CAN 的完整六轴轨迹及取消。依次验证控制器停用、CANopen 心跳丢失/EMCY、任一 CAN 接口断线、任一私有协议轴反馈失效，观察双方停止和旧命令不重放。

同时验证摆臂 CSV/CSP 互斥、五种联动、速度超时、软件范围及方向；夹爪 `/gripper_controller/command` 为一元素 m/s 命令，控制器通过受限积分输出 CSP 位置，不能继续按旧 rad/s 注释使用。履带接口为标准 Twist，并验收 300 ms 停发超时。

## 6. 零点、软限位与保存

仅在原持久化基线完整且机械状态允许维护时执行。全局诊断模式先 halt，维护入口停用运动控制器和两硬件组件，再逐轴 disable；分组模式只关闭目标所属组及必要关联许可，不停用完整组件，按 [31 号操作单](31_分组隔离与局部恢复实施验收.md) 执行。SDO 始终使用唯一主站。

```bash
ros2 service call /robot_runtime/maintain robot_hardware/srv/Maintain \
  '{joint: shoulder_yaw_joint, operation: limits_from_urdf, save: false}'
```

操作包括 set_zero、set_offset（lower 为 rad）、limits（lower/upper 为 rad）、limits_from_urdf、disable_limits 和 restore。没有自动启动写限位。

- 写后逐对象读回；部分失败尝试恢复操作前 RAM 值并读回。限位先关闭厂商使能，再写范围，最后恢复使能。
- set_zero 使用 `旧 0x607C - 当前 0x6064`，避免已存在硬件偏置时重复叠加；ROS 软件 offset 固定为零。现场必须确认该固件 `0x6064` 是包含硬件偏置的位置反馈，再进行零点试验。
- `save: true` 写 `0x1010:01 = 0x65766173`，读回支持状态并记录需要重启核验；该读回不代表已经证明掉电保持。
- 保存超时视为持久化结果不确定。日志以 dirty 标记跨进程保存，全局模式阻止 init/运动，分组模式锁定目标轴所属组及关联范围；显式 restore 按操作前基线恢复并重新保存。恢复后仍须掉电/重启读回。

单轴同样支持 `axis_commission --operation ...`；参数和基线校验一致。每次维护完成需重新校验和 enable/resume，不自动恢复先前运动。

## 7. 证据采集、性能与回退

```bash
ros2 run robot_hardware capture_acceptance --interfaces canable0 canable1 \
  --seconds 60 --bitrate 1000000 --output /实际路径/acceptance/run01
```

该工具被动接收 CAN 帧和 ROS 状态，输出 CAN JSONL、反馈与运行态、SYNC 间隔以及运行期管理器周期统计。总线负载是根据经典 CAN 帧长度和填充估算；精确物理负载、总线错误电气原因及急停效果仍需现场测量。控制周期输出 min/mean/max 及超过 7.5 ms 的次数，用于对照 200 Hz/5 ms 配置，不代表已经达到交付实时性阈值。

输出目录须每次唯一，已存在时拒绝覆盖。CAN 每轮批量读取，`kernel_time_ns` 为 Linux 内核 CLOCK_REALTIME 接收时间，SYNC 间隔由此计算；`t` 保留用户态 CLOCK_MONOTONIC 读取时间。报告附起止墙钟与单调时钟锚点，外部测量需注明时钟同步误差；墙钟回退计入 `timestamp_clock_regressions`，出现时不能直接采用该次 SYNC 统计。`rx_dropped_total` 记录最后收到的内核套接字队列累计丢帧数，非零时采集不完整，零也不能证明物理总线无丢帧；结束时尚未送达的计数可能不可见。时间戳缺失或元数据截断会失败并保留已写帧，不能把部分文件当完整报告。`status_errors` 和 `status_request_pending` 展示状态请求失败/未完成；周期样本为空时没有控制周期验收证据。采集工具增量回归已通过，历史输出已清理；新采集继续使用外部唯一目录。

每项保留软件清单、实际设备/固件、基线副本、维护日志、步骤、预期结果、抓包和实测结果；将阈值与现场负责人确认后写入验收记录。

恢复原可用 ROS2 版本前先 halt/shutdown，确认主站退出；若改过设备零点、限位或保存参数，按原始基线恢复并读回。项目不再要求恢复 ROS1。历史 CANopen 包已经归入 reference/ros1，不参与活动构建。
