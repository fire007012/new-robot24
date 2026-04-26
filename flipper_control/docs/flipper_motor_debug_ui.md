# flipper_motor_debug_ui 使用说明

`flipper_motor_debug_ui.py` 是 `flipper_control` 的摆臂电机联调界面，用来同时观察上层模式管理状态、下游后端生命周期状态、控制器状态，以及直接发送轨迹和点动命令。

脚本文件：

- [../scripts/flipper_motor_debug_ui.py](/home/rera/robot24_ws/src/flipper_control/scripts/flipper_motor_debug_ui.py)

适用场景：

- 验证摆臂命令是否经过 `flipper_control -> controller -> backend` 这一整条链路。
- 区分当前下游到底在走 `hybrid` 还是 `canopen`。
- 在不改下游包代码的前提下，做生命周期按钮联调。
- 现场确认控制器是否已加载、是否在运行、当前档位是否匹配。

## 功能概览

界面分成四块：

- `Runtime Summary`
  - 显示 `backend`、`backend_ns`、`active_profile`、`hardware_mode`、`active_controller`、`linkage_mode`、`switch_state`、`lifecycle`、`lifecycle_src`、`ready`、`switching`、`timed_out`、`degraded` 和 `detail`。
- `Backend Ops`
  - 显示 `joint_state_controller`、`flipper_csp_controller`、`flipper_csv_forward_controller` 的当前状态。
  - 提供 `init / enable / disable / halt / resume / recover / shutdown` 按钮。
  - 显示后端侧概览信息 `backend_detail`。
- `Mode Control`
  - 切换 `profile` 和 `linkage`。
  - 设置轨迹时间 `traj_time` 和点动持续时间 `jog_duration`。
- `Joint Commands`
  - 按关节显示 `online / enabled / fault / hb_lost / measured / reference / cmd_vel / lifecycle`。
  - 可直接填写 `target_pos` 发送轨迹，或填写 `target_vel` 发送点动。

## 后端适配逻辑

UI 会优先读取 `/flipper_control/backend_type`，自动适配下游后端，也可以通过命令行强制指定。

`hybrid` 模式：

- 订阅 `JointRuntimeStateArray`。
- 生命周期来自真实 runtime state。
- 每个关节的 `online / enabled / fault / lifecycle` 都是实时值。

`canopen` 模式：

- 订阅 `/diagnostics`，解析每个关节的 `is_operational`、`is_fault`、`heartbeat_lost_flag`。
- 生命周期按钮直接调用 `Trigger` 服务。
- 顶部 `lifecycle` 是由以下信息综合得到：
  - 生命周期服务返回结果。
  - `flipper_control/state` 里的 manager 状态。
  - `/diagnostics` 故障与心跳状态。
  - `auto_init / auto_enable / auto_release` 启动参数。

注意：

- `hybrid` 的生命周期是实时真实状态。
- `canopen` 的生命周期显示是“尽量贴近真实”的估计结果，因为这版适配明确不修改 `Eyou_Canopen_Master`。
- 判断来源会写在 `lifecycle_src`，常见值有 `runtime`、`manager`、`service`、`diagnostics`、`estimated:auto_startup`、`estimated:diagnostics`。

## 启动方式

默认自动识别后端：

```bash
rosrun flipper_control flipper_motor_debug_ui.py \
  --flipper-ns /flipper_control \
  --backend-type auto \
  --hybrid-ns /hybrid_motor_hw_node \
  --canopen-ns /canopen_hw_node
```

强制按 `hybrid` 打开：

```bash
rosrun flipper_control flipper_motor_debug_ui.py \
  --flipper-ns /flipper_control \
  --backend-type hybrid \
  --hybrid-ns /hybrid_motor_hw_node
```

强制按 `canopen` 打开：

```bash
rosrun flipper_control flipper_motor_debug_ui.py \
  --flipper-ns /flipper_control \
  --backend-type canopen \
  --canopen-ns /canopen_hw_node
```

推荐在后端和 `flipper_control` 节点都已经启动后再打开 UI，这样启动后就能立刻看到状态，而不是先停在 `-` 或 `waiting for diagnostics`。

## 依赖的 ROS 接口

上层状态：

- 订阅 `/flipper_control/state`
- 调用 `/flipper_control/set_control_profile`
- 调用 `/flipper_control/set_linkage_mode`
- 发布 `/flipper_control/command`
- 发布 `/flipper_control/jog_cmd`

后端相关：

- `hybrid`
  - 订阅 `runtime_state_topic`，默认是 `/hybrid_motor_hw_node/joint_runtime_states`
- `canopen`
  - 订阅 `canopen_diagnostics_topic`，默认是 `/diagnostics`
  - 调用 `/<backend_ns>/init`
  - 调用 `/<backend_ns>/enable`
  - 调用 `/<backend_ns>/disable`
  - 调用 `/<backend_ns>/halt`
  - 调用 `/<backend_ns>/resume`
  - 调用 `/<backend_ns>/recover`
  - 调用 `/<backend_ns>/shutdown`

控制器状态：

- 调用 `/<controller_manager_ns>/list_controllers`

## 常见联调流程

### 1. 确认当前到底走哪个后端

- 看 `Runtime Summary` 里的 `backend`。
- 看 `backend_ns` 是否是预期命名空间。
- 如果和现场配置不一致，可以重新用 `--backend-type` 强制指定。

### 2. 确认命令有没有经过控制器

- 先看 `active_controller`。
- 再看 `Backend Ops` 里的三个 controller state。
- 正常情况下：
  - `joint_state_controller` 应处于 `running`。
  - 当前使用的 `flipper_csp_controller` 或 `flipper_csv_forward_controller` 应处于 `running`。
  - 未使用的那个控制器可能是 `stopped`。

如果 `active_controller` 已经切过来，但 `list_controllers` 里目标控制器不是 `running`，就说明链路没有真正切通。

### 3. 切换档位并验证

- 通过 `profile` 切换 `csp_position / csp_jog / csv_velocity`。
- 看 `active_profile`、`hardware_mode`、`active_controller` 是否同步变化。
- 看 `switch_state` 是否最终回到稳定状态。
- 如果 `switching=true` 长时间不恢复，优先检查底层生命周期和控制器状态。

### 4. 使用生命周期按钮

推荐顺序：

1. `init`
2. `enable`
3. `resume`

停机常见顺序：

1. `halt`
2. `disable`
3. `shutdown`

故障恢复常见顺序：

1. `recover`
2. `enable`
3. `resume`

注意：

- `canopen` 下 `halt` 和 `resume` 更像轻量级冻结/恢复，不等于完整上下电。
- 某些按钮在当前状态下会被后端拒绝，按钮本身不会灰掉，结果看底部状态栏和 `lifecycle_src`。

### 5. 发命令前先看这几个字段

- `ready=true`
- `switching=false`
- `timed_out=false`
- `degraded=false`
- 当前档位与发送命令类型匹配

命令类型约束：

- `Send Position Trajectory`
  - 适合 `csp_position`
- `Send Velocity Jog`
  - 适合 `csp_jog` 或 `csv_velocity`

## 参数说明

命令行参数：

- `--flipper-ns`
  - `flipper_control` 节点命名空间，默认 `/flipper_control`
- `--backend-type`
  - `auto | hybrid | canopen`
- `--hybrid-ns`
  - `hybrid` 后端命名空间，默认 `/hybrid_motor_hw_node`
- `--canopen-ns`
  - `canopen` 后端命名空间，默认 `/canopen_hw_node`

脚本还会读取这些参数：

- `/flipper_control/backend_type`
- `/flipper_control/runtime_state_topic`
- `/flipper_control/canopen_diagnostics_topic`
- `/flipper_control/controller_manager_ns`
- `/flipper_control/controllers/csp`
- `/flipper_control/controllers/csv`
- `/<canopen_ns>/auto_init`
- `/<canopen_ns>/auto_enable`
- `/<canopen_ns>/auto_release`

## 状态字段怎么理解

顶部字段：

- `lifecycle`
  - 当前综合生命周期判断。
- `lifecycle_src`
  - 生命周期来自哪里。
- `detail`
  - `flipper_control` manager 的汇总说明。
- `backend_detail`
  - 下游后端健康摘要。

关节字段：

- `online`
  - `hybrid` 下来自 runtime state。
  - `canopen` 下等价于诊断里的 `is_operational`。
- `enabled`
  - `hybrid` 下来自 runtime state。
  - `canopen` 下表示该关节已 operational，且总体 lifecycle 已进入 `Armed` 或 `Running`，同时没有 fault 和 heartbeat lost。
- `fault`
  - 关节故障标志。
- `hb_lost`
  - 仅 `canopen` 有意义，表示心跳丢失。
- `measured`
  - 实测位置。
- `reference`
  - `flipper_control` 当前参考位置。
- `cmd_vel`
  - `flipper_control` 当前输出速度参考。

## 常见问题排查

现象：界面里所有 lifecycle 都是 `-`

- 先确认 `/flipper_control/state` 是否在发布。
- 再确认 `--flipper-ns` 是否和实际节点一致。

现象：`canopen` 下只看到 `waiting for diagnostics`

- 确认 `/diagnostics` 是否真的包含摆臂关节项。
- 确认诊断项名字末尾能匹配 `joint_names`。
- 确认 `canopen_diagnostics_topic` 没被改到别的 topic。

现象：按钮点了没效果

- 先看底部状态栏是否提示 `service not available` 或超时。
- 再确认 `backend_ns` 下对应服务是否存在。
- 对 `canopen` 来说，部分状态切换本来就要求严格顺序，失败时以后端返回消息为准。

现象：`active_profile` 已切换，但电机不动

- 看 `active_controller` 和 controller state 是否一致。
- 看 `ready / switching / degraded / timed_out`。
- 看当前发送命令类型是否与 profile 匹配。

现象：`canopen` 下 lifecycle 看起来不太像现场真实状态

- 先看 `lifecycle_src`。
- 如果来源是 `estimated:*`，说明当前没有足够直接的 lifecycle 观测，只能根据启动参数和诊断推断。
- 这不是 UI bug，而是这版实现刻意避免改动 `Eyou_Canopen_Master` 的结果。

## 相关文档

- [../README.md](/home/rera/robot24_ws/src/flipper_control/README.md)
- [README.md](/home/rera/robot24_ws/src/flipper_control/docs/README.md)
- [../config/flipper_control.yaml](/home/rera/robot24_ws/src/flipper_control/config/flipper_control.yaml)
- [../launch/flipper_control.launch](/home/rera/robot24_ws/src/flipper_control/launch/flipper_control.launch)
