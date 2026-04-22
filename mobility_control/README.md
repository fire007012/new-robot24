# mobility_control

`mobility_control` 是底盘正式北向控制包，当前已落地的核心能力是 `base_cmd_node`：在进入 `wheel_controller` 前把 `/cmd_vel` 作为 Ruckig 跟踪目标做速度平滑，同时保留速度限幅和超时刹停。它不负责 CAN 直写、差速逆解或整机总控。

当前正式链路：

```text
/cmd_vel -> mobility_control/base_cmd_node -> /wheel_controller/cmd_vel -> wheel_controller -> can_driver
```

## 包职责

- 统一承接底盘北向速度指令。
- 在控制器前做安全整形：Ruckig 平滑、限幅、超时自动归零。
- 作为后续底盘仿真桥接和里程计适配的正式归属包。

## 包结构

```text
mobility_control/
|-- config/
|   `-- base_cmd.yaml
|-- docs/
|   |-- README.md
|   |-- 归档/
|   `-- base_cmd_node.md
|-- launch/
|   `-- base_cmd.launch
`-- scripts/
    `-- base_cmd_node.py
```

## 快速开始

```bash
cd /home/rera/robot24_ws
catkin_make --pkg mobility_control
source devel/setup.bash
```

```bash
roslaunch mobility_control base_cmd.launch
```

覆盖默认 topic 或限幅：

```bash
roslaunch mobility_control base_cmd.launch \
  input_topic:=/cmd_vel \
  output_topic:=/wheel_controller/cmd_vel \
  max_linear_x:=0.6 \
  timeout_sec:=0.2
```

## 常用命令

```bash
# 发送底盘速度
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'

# 观察限幅后的输出
rostopic echo /wheel_controller/cmd_vel

# 验证超时刹停
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 1.5}, angular: {z: 2.0}}'
```

## 节点 / API 摘要

| 节点 | 接口 | 说明 |
| --- | --- | --- |
| `base_cmd_node` | 默认订阅 `/cmd_vel`；默认发布 `/wheel_controller/cmd_vel` | 对 `linear.x` 和 `angular.z` 做 Ruckig 速度跟踪；同时做限幅，超过 `timeout_sec` 自动平滑减速到零。 |

默认参数：

- `input_topic=/cmd_vel`
- `output_topic=/wheel_controller/cmd_vel`
- `max_linear_x=0.8`
- `max_angular_z=1.5`
- `control_rate_hz=50.0`
- `max_linear_acc=1.0`
- `max_linear_jerk=4.0`
- `max_angular_acc=2.0`
- `max_angular_jerk=8.0`
- `wheel_separation=0.438`
- `cmd_linear_direction_correction=1.0`
- `cmd_angular_direction_correction=1.0`
- `left_track_direction_correction=1.0`
- `right_track_direction_correction=1.0`
- `timeout_sec=0.3`

方向修正说明：

- `cmd_linear_direction_correction` / `cmd_angular_direction_correction`
  - 作用在 `cmd_vel` 语义层，适合修正整车前进方向或原地转向方向。
- `left_track_direction_correction` / `right_track_direction_correction`
  - 作用在左右履带速度层。节点会先把平滑后的 `linear.x` / `angular.z` 解算成左右履带速度，应用修正后再合成为等效 Twist 发给 `wheel_controller`。

## 详细文档索引

- [`docs/README.md`](docs/README.md)：当前文档导航
- [`docs/base_cmd_node.md`](docs/base_cmd_node.md)：节点参数与测试命令
- [`config/base_cmd.yaml`](config/base_cmd.yaml)：默认参数
- [`launch/base_cmd.launch`](launch/base_cmd.launch)：标准启动入口
