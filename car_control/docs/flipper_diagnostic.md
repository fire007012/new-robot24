# flipper_diagnostic_node

`flipper_diagnostic_node` 是一个旁路诊断脚本，用来在复现摆臂“发黏/变慢”时同步抓取三层链路：

- 键盘链路：`/car_control/keyboard_teleop/status`
- flipper manager：`/flipper_control/jog_cmd`、`/flipper_control/state`
- 执行层：`/joint_states`

节点不会发布控制命令，只做观测与记录。

## 启动

```bash
roslaunch car_control flipper_diagnostic.launch
```

指定观测关节和阈值：

```bash
roslaunch car_control flipper_diagnostic.launch \
  watch_joint:=left_front_arm_joint \
  threshold_deg:=30 \
  output_path:=/tmp/flipper_diag_left_front.jsonl
```

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `watch_joint` | `left_front_arm_joint` | 用来判定“是否过阈值”的观测关节 |
| `long_travel_sign` | `0.0` | 长行程方向符号；`0` 表示自动推断，前摆臂为负向，后摆臂为正向 |
| `threshold_deg` | `30.0` | 相对零位的阈值角度 |
| `sample_rate_hz` | `50.0` | 诊断采样频率 |
| `sticky_ratio_threshold` | `0.65` | 当 `|velocity| / |manager_cmd|` 低于该值时，记为可疑掉速 |
| `sticky_min_effort` | `0.2` | 只有关节 effort 明显抬升时才把掉速记为粘滞事件，避免采样空洞误报 |
| `sticky_min_samples` | `3` | 连续满足多少个样本后触发一次粘滞事件提示 |
| `output_path` | 自动生成到 `/tmp` | JSONL 输出文件路径 |

## 输出内容

终端会周期打印摘要，包括：

- `pressed` 当前按键
- `ready / degraded`
- 当前 `travel_deg`
- 活跃关节的 `jog_cmd / manager_cmd / velocity / velocity_ratio / effort`

当节点检测到“命令速度保持不变，但实测速度连续掉到较低比例”时，会额外打印 `sticky event` 告警。

同时会把完整记录写入 JSONL，每行一条样本，包含：

- `keyboard_status`
- `jog`
- `flipper_state`
- `active_joints`
- `watch`
