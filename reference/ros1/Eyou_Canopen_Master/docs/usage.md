# CANopen 主站使用文档

本文档面向上机联调与日常使用，覆盖：环境准备、配置、生成 DCF、编译运行、验证点、关机与故障排查。

当前安全行为基线见：`docs/2026-03-25_现行安全行为规范.md`。
若与 `docs/archive/2026-04-19_deprecated/0324import.md` 存在语义差异，以现行安全行为规范为准。

---

## 1. 环境准备

### 1.1 依赖工具

```bash
sudo apt install can-utils
```

### 1.2 启动 CAN 接口

```bash
sudo ip link set canable0 up type can bitrate 1000000
ip -details -statistics link show canable0
```

### 1.3 抓包检查（建议常开）

```bash
# 实时抓包
candump canable0

# 仅看关键帧（SYNC + PDO + Heartbeat + EMCY）
candump canable0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

预期：
- SYNC 周期约 10ms
- Node 1-6 心跳帧可见（0x701~0x706）
- 运行后可见 PDO 帧（0x18x/0x28x/0x20x）

---

## 2. 配置文件

### 2.1 joints.yaml

路径：`config/joints.yaml`

重点字段（每轴）：
- `counts_per_rev`
- `rated_torque_nm`
- `velocity_scale`
- `torque_scale`
- `canopen.verify_pdo_mapping`（启动时是否验证 PDO 映射）

建议每轴都填完整参数，避免默认值掩盖问题。

### 2.2 master.yaml / master.dcf

路径：
- `config/master.yaml`
- `config/master.dcf`

`master.dcf` 由 `dcfgen` 根据 `master.yaml` 生成，且在启动时必须存在。

---

## 3. 生成 DCF

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh full
rosrun Eyou_Canopen_Master generate_dcf.sh arm_only
```

说明：
- 若 `dcfgen` 报 EDS 格式错误，请改用 `YiyouServo_V1.4.dcfgen.eds`。
- `full` 会刷新默认全量 `config/master.dcf`。
- `arm_only` 会刷新机械臂肩部专用 `config/master_arm_only.dcf`。
- 4 轴摆臂配置可用：

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh flipper_only
```

- 一次刷新全部三套 CANopen 配置：

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh all
```

---

## 4. 编译

### 4.1 独立编译（不依赖 ROS）

```bash
cmake -S . -B build
cmake --build build -j
```

### 4.2 catkin 编译（ROS 模式）

```bash
cd ~/Robot24_catkin_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

---

## 5. 启动

### 5.1 独立模式

```bash
./build/canopen_hw_node \
  --dcf config/master.dcf \
  --joints config/joints.yaml
```

说明：
- `--dcf` 必须指向存在的 DCF 文件，否则程序直接退出。
- `--joints` 必须指向存在的 `joints.yaml`，否则程序直接退出。
- 路径建议使用绝对路径，避免工作目录变化导致失败。

### 5.2 ROS 模式

```bash
# 默认手动初始化（推荐）
roslaunch Eyou_Canopen_Master canopen_hw.launch auto_init:=false

# 首次启动必须手动初始化电机
rosservice call /canopen_hw_node/init "{}"
```

可选参数：
- `dcf_path:=<path>` — DCF 文件路径
- `joints_path:=<path>` — joints.yaml 路径
- `robot_urdf:=<path>` — URDF 文件路径；默认使用 `car_urdf/urdf/car_urdf.urdf`
- `loop_hz:=200.0` — 控制循环频率
- `auto_init:=false|true` — 是否启动后自动 `~/init`（默认 `false`）
- `auto_enable:=false|true` — 是否在 `auto_init` 后自动 `~/enable`（默认 `false`）
- `auto_release:=false|true` — 是否在 `auto_enable` 后自动 `~/resume`（默认 `false`）
- `use_ip_executor:=false|true` — 是否启用 IP 轨迹执行器（默认 `false`）
- `ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory` — action 名称

节点私有参数（通过 ROS 参数服务器覆盖）：
- `/canopen_hw_node/ip_executor_rate_hz` — 执行器更新频率（默认跟随 `loop_hz`）

说明：
- `bringup.launch` / `canopen_hw.launch` 现在默认从 `car_urdf` 加载正式 `robot_description`。
- 包内 `urdf/robot.urdf` 已弃用，仅保留作历史参考与轻量测试，不再建议作为运行时默认模型。

### 5.3 启用 IP 轨迹执行器（多轴）

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory

# 可选：单独覆盖 executor 频率
rosparam set /canopen_hw_node/ip_executor_rate_hz 100.0
```

说明：
- executor 为单实例多轴执行，轴数由 `joints.yaml` 的 `joints` 列表决定。
- 每轴约束来自 `joints.yaml`：`ip_max_velocity/ip_max_acceleration/ip_max_jerk/ip_goal_tolerance`。
- action goal 必须覆盖全部配置轴；`joint_names` 顺序可与配置不同，内部会做映射。

---

## 6. ROS 接口

### 6.1 Services

| Service | 类型 | 说明 |
|---------|------|------|
| `~init` | `std_srvs/Trigger` | `Configured -> Armed`（启动主站并自动上电到冻结态；成功后触发一次命令重同步） |
| `~enable` | `std_srvs/Trigger` | `Standby -> Armed`（使能并冻结输出；若存在 fault / heartbeat_lost / global fault 则拒绝） |
| `~disable` | `std_srvs/Trigger` | `Running/Armed/Standby -> Standby`（去使能但不断开通信） |
| `~halt` | `std_srvs/Trigger` | `Running -> Armed`（置 Halt bit，冻结输出） |
| `~resume` | `std_srvs/Trigger` | `Armed -> Running`；若存在 fault / heartbeat_lost / global fault 则拒绝 |
| `~recover` | `std_srvs/Trigger` | `Faulted -> Standby`（等待 fault 与 heartbeat 全清后才成功，不自动上电） |
| `~shutdown` | `std_srvs/Trigger` | 停通信并回 `Configured`，节点不退出（随后需 `~init`；成功后触发一次命令重同步） |
| `~set_mode` | `Eyou_Canopen_Master/SetMode` | 仅在 `Standby` 允许（典型：`~disable` 后） |
| `~set_zero` | `Eyou_Canopen_Master/SetZero` | 仅在 `Standby` 允许；zero-only，支持“按当前点归零”或“显式写入零偏” |
| `~apply_limits` | `Eyou_Canopen_Master/ApplyLimits` | 仅在 `Standby` 允许；单独应用 URDF 或手工限位到 `0x2003/0x607D` |

### 6.1.2 命令协议（epoch-ready + command_sync_sequence）

当前控制链路采用 `AxisCommand.valid + AxisCommand.arm_epoch + SharedSnapshot.command_sync_sequence` 协议：

1. `arm_epoch=0` 永远无效；
2. 进入新的使能会话后，底层会更新 `AxisFeedback.arm_epoch`；
3. `~init`、`~recover`、`~shutdown` 成功后，协调层会递增 `command_sync_sequence`，显式声明“旧命令源作废，需要重新对齐”；
4. ROS 适配层在 `read()` 观察到 `command_sync_sequence` 变化、`arm_epoch` 变化沿或 fault-halt 上升沿时，会强制把位置命令对齐到当前位置，并把 `valid` 拉低进入 guard 窗口；
5. guard 结束且系统不处于 fault-halt 时，`write()` 才会重新把 `valid=true` 写回下层；
6. `valid=true` 但 `arm_epoch` 不匹配时，状态机仍保持锁定，不会透传目标。

工程建议：

1. `~recover` 返回成功，只表示 fault/heartbeat 已经恢复到健康快照；后续动作仍应是 `~enable -> ~resume`。
2. `~init` / `~recover` 成功后，不要复用旧控制器缓存的目标；ROS 适配层会先把位置命令拉回当前位置，再经过 guard 窗口重新放开 `valid`。
3. 若上层绕过 `CanopenRobotHwRos` 直接写 `SharedState`，则必须自行监听 `command_sync_sequence`，并在序列变化后显式重同步 setpoint。

### 6.1.3 shutdown/recover/init 关系（Coordinator 语义）

- `~shutdown`：停通信并回到 `Configured`，节点进程不退出。
- `~recover`：仅在全部轴 `fault=false` 且 `heartbeat_lost=false` 后才返回成功；成功后回到 `Standby`，不自动上电。
- `~init`：`~shutdown` 后重新建立通信并进入 `Armed`；成功后会触发一次命令重同步。若 `canopen.auto_write_soft_limits_from_urdf=true`，会在 `init` 成功后按 URDF 关节限位同步 `0x2003/0x607D`；`revolute` 使用 `counts_per_rev` 换算，`prismatic` 使用 `counts_per_meter` 换算；若关节在 URDF 中没有位置上下限，则仅写 `0x2003:00=0x00000000` 关闭该轴软限位，不写 `0x607D`；写入失败则本次 `init` 返回失败并回滚到 `Configured`。
- `~enable`：将 `Standby` 推到 `Armed`；若快照仍有 fault / heartbeat_lost / global fault，则拒绝。
- `~disable`：将 `Running/Armed` 回到 `Standby`，但保持通信在线。
- `~halt` / `~resume`：在 `Running <-> Armed` 之间切换。
- `~resume`：仅在健康快照下允许进入 `Running`；若存在 fault、heartbeat 丢失或 `global_fault` 闩锁，则会被拒绝。
- `~set_zero`：仅允许 `Standby`；当前统一为 zero-only 语义。`use_current_position_as_zero=true` 时执行 `0x607C=0 -> 读0x6064 -> 0x607C=-actual_position -> 0x1010:01='save'`；否则按请求值直接写 `0x607C` 并保存。
- `~apply_limits`：仅允许 `Standby`；负责单独写该轴 `0x2003/0x607D`。`use_urdf_limits=true` 时采用 URDF 限位，`require_current_inside_limits=true` 时才会在当前点超限时拒绝。
- 故障恢复标准顺序为：`~recover -> ~enable -> ~resume`。

### 6.1.4 调零与限位推荐顺序

```bash
# 1) 进入 Standby
rosservice call /canopen_hw_node/disable "{}"

# 2) 当前点设零（zero-only）
rosservice call /canopen_hw_node/set_zero "{axis_index: 0, zero_offset_rad: 0.0, use_current_position_as_zero: true}"

# 3) 单独应用 URDF 限位
rosservice call /canopen_hw_node/apply_limits "{axis_index: 0, min_position: 0.0, max_position: 0.0, use_urdf_limits: true, require_current_inside_limits: false}"

# 4) 重新进入运行态
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume "{}"
```

### 6.2 模式切换流程

```bash
# 0. 首次启动先执行 init（若 auto_init:=false）
rosservice call /canopen_hw_node/init "{}"

# 1. 去使能到 Standby（保持通信，不断总线）
rosservice call /canopen_hw_node/disable "{}"

# 2. 切换到 CSV 模式（所有轴）
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 9}"
done

# 3. 切换 controller
rosrun controller_manager controller_manager stop arm_position_controller
rosrun controller_manager controller_manager start arm_velocity_controller

# 4. 重新上电并恢复运动
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume
```

切换到 IP 模式（mode=7）：

```bash
# 1) 去使能到 Standby
rosservice call /canopen_hw_node/disable "{}"

# 2) 切到 IP
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 7}"
done

# 3) 重新上电并恢复
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume "{}"
```

说明：
- IP 模式位置目标优先走 `0x60C1:01`（RPDO2 映射）。  
- 若驱动器拒绝 `0x60C1:01` PDO 写入，运行时会回退到 `0x607A`，并打印一次告警日志。  
- `0x60C2:01`（插补周期，ms）在 boot 时通过 SDO 下发，取自 `joints.yaml` 的 `ip_interpolation_period_ms`（未配置时按 `loop_hz` 推导）。
- 开启 `use_ip_executor:=true` 后，MoveIt / action client 直接向 `/<ip_executor_action_ns>` 发送 `FollowJointTrajectory` 即可。

### 6.3 Controllers

| Controller | 类型 | 模式 | 默认状态 |
|-----------|------|------|---------|
| `joint_state_controller` | JointStateController | — | 启动 |
| `arm_position_controller` | position_controllers/JointTrajectoryController | CSP | 默认启动（`use_ip_executor=false`） |
| `arm_velocity_controller` | velocity_controllers/JointTrajectoryController | CSV | 已加载未启动 |

### 6.4 Diagnostics

`/diagnostics` topic 每轴上报：
- `heartbeat_lost` — 心跳丢失计数
- `emcy_count` — EMCY 计数
- `fault_reset_attempts` — 故障复位尝试次数
- `is_operational` / `is_fault` / `heartbeat_lost_flag`

---

## 7. 运行期验证点

基础验证：
- 主站启动无报错
- 心跳可见
- PDO 正常刷新

状态机验证：
- 状态能够进入 `OPERATION_ENABLED`
- 位置反馈更新后 `all_operational` 变为 true
- 在 `valid=false` 或 `epoch` 失配时，轴保持锁定且 `safe_target` 跟随实际位置
- `all_axes_halted_by_fault=true` 时，上层不应继续发送有效运动命令

---

## 8. 关机与停通信流程

### 8.1 `~shutdown`（节点不退出）

`~shutdown` 被调用时按以下顺序执行：
1. `Disable Operation` (0x6040=0x0007)
2. 等待状态进入 `SwitchedOn`（超时 2s）
3. `Shutdown` (0x6040=0x0006)
4. 等待状态进入 `ReadyToSwitchOn`（超时 1s）
5. `NMT Stop` + `master_->Stop()` 停通信
6. 返回 `Configured`；随后 `~recover` 会因状态不合法被拒绝，需先 `~init`

若第 2/4 步超时：
- 仍继续执行第 5 步，避免 service 卡死
- service 返回 `success=false`，`message` 给出超时轴信息
- 状态仍回到 `Configured`，后续需先 `~init`

### 8.2 Ctrl+C（进程级 teardown）

Ctrl+C 才会触发进程级退出，释放 ROS 节点与内部对象。

---

## 9. 启动身份失配诊断

当启动日志出现 `OnConfig failed (es=...)` 时，驱动会额外输出以下诊断信息：

- `expected identity from master.dcf`：主站 DCF 对该节点期望的 `1000:00 / 1018:01 / 1018:02 / 1018:03`
- `actual identity snapshot`：启动失败当下通过 SDO 读回的实际值
- `boot identity mismatch fields`：直接列出不一致字段（如 `1000:00(device_type)`, `1018:02(product_code)`）

建议排查顺序：
1. 对比 `boot identity mismatch fields` 与驱动器实测对象字典。
2. 若字段不一致，先修正 EDS/DCF（尤其 `1F84/1F85/1F86`）再重新 `dcfgen`。
3. 若字段一致但仍失败，继续排查 PDO 映射或上电时序问题。

---

## 10. 现场验证清单（最终版）

> 目标：现场按此清单一次性完成“启动可用 + 抓包可证 + 关机可控”的验收。

### 10.1 前置准备

1. CAN 链路已连通，`canable0` 已 up，波特率与驱动器一致。
2. `master.dcf` 与现场固件版本匹配（更新固件后必须重生成 DCF）。
3. 启动抓包：

```bash
# Node=5 示例：SYNC + TPDO/RPDO + Heartbeat + NMT
candump -L canable0,080:7FF,185:7FF,205:7FF,285:7FF,705:7FF,000:7FF
```

### 10.2 启动与 Boot 身份判据

启动后检查日志：

- 通过：无 `OnConfig failed`；或出现后自动恢复且最终进入运行态。
- 失败：持续出现 `OnConfig failed` 且无法进入运行态。

若失败，必须核对 C03 诊断日志：

- `expected identity from master.dcf`
- `actual identity snapshot`
- `boot identity mismatch fields`

字段判据（必须一致）：

- `1000:00`（Device Type）
- `1018:01`（Vendor ID）
- `1018:02`（Product Code）

### 10.3 PDO 抓包判据（C01/C02 关键）

1. 使能阶段（C01）：在 `0x205` 中可看到控制字低字节序列 `06 00 -> 0F 00`（little-endian，对应 0x0006 -> 0x000F）。
2. 运行阶段：`0x185` 反馈应持续周期出现（无长时间稀疏/中断）。
3. 退出阶段（C02）：Ctrl+C 后应看到控制字回落序列 `07 00 -> 06 00`，随后出现 NMT Stop（`0x000`）。

### 10.4 运行态判据

- `all_operational` 最终为 `true`。
- `/diagnostics` 中各轴 `is_operational=true`，且无持续 `is_fault=true`。
- 无持续 heartbeat 丢失累计增长。

### 10.5 启动负路径判据（C04）

- 缺失 `--dcf` 文件：进程应直接失败退出（返回码 1）。
- 缺失 `--joints` 文件：进程应直接失败退出（返回码 1）。

### 10.6 验收结论模板

| 检查项 | 结果 | 证据 |
|---|---|---|
| Boot 身份一致（1000/1018） | PASS/FAIL | 启动日志片段 |
| 控制字使能序列（06 00 -> 0F 00） | PASS/FAIL | candump 片段 |
| 运行期 TPDO 连续性（0x185） | PASS/FAIL | candump 片段 |
| 关机序列（07 00 -> 06 00 + NMT Stop） | PASS/FAIL | Ctrl+C 后抓包 |
| all_operational 与 diagnostics | PASS/FAIL | topic/日志截图 |

---

## 11. 单测

```bash
# 独立编译模式
cmake --build build -j && cd build && ctest --output-on-failure

# catkin 模式
catkin_make run_tests_Eyou_Canopen_Master
```

---

## 12. 常见问题

1. 无任何帧：优先检查终端电阻、电源、CAN_H/CAN_L 线序。
2. 有心跳无 PDO：确认 NMT 已进入 Operational，检查 PDO COB-ID 是否被禁用(bit31)。
3. 反复 FAULT：检查 statusword 与 EMCY 码，确认复位节流参数是否过短。
4. 退出后行为异常：做“断 SYNC/强杀进程”实验并确认驱动器策略。

---

## 13. 联调记录建议

建议在项目 issue 或现场日志中固定记录以下内容：

1. 启动参数（`dcf_path/joints_path/loop_hz/use_ip_executor`）。
2. 关键 service 调用序列与时间戳。
3. 故障时的 `candump` 片段、`/diagnostics` 快照与节点日志。
