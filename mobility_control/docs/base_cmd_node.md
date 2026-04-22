# base_cmd_node

`base_cmd_node` 是 `mobility_control` 当前唯一的正式运行节点，用来在 `wheel_controller` 前做底盘命令安全整形。

当前实现把 `/cmd_vel` 作为 Ruckig 的目标速度，在 `linear.x` 和 `angular.z` 两个自由度上做连续加速度/加加速度受限的速度跟踪。

## 默认链路

```text
/cmd_vel -> base_cmd_node -> /wheel_controller/cmd_vel
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_topic` | `/cmd_vel` | 上游速度输入 |
| `output_topic` | `/wheel_controller/cmd_vel` | 发送给控制器的话题 |
| `max_linear_x` | `0.8` | `linear.x` 限幅 |
| `max_angular_z` | `1.5` | `angular.z` 限幅 |
| `control_rate_hz` | `50.0` | Ruckig 更新频率 |
| `max_linear_acc` | `1.0` | `linear.x` 最大加速度 |
| `max_linear_jerk` | `4.0` | `linear.x` 最大加加速度 |
| `max_angular_acc` | `2.0` | `angular.z` 最大加速度 |
| `max_angular_jerk` | `8.0` | `angular.z` 最大加加速度 |
| `wheel_separation` | `0.438` | 左右履带中心距，用于 `Twist <-> track velocity` 换算 |
| `cmd_linear_direction_correction` | `1.0` | `cmd_vel.linear.x` 方向修正，常用值 `1.0/-1.0` |
| `cmd_angular_direction_correction` | `1.0` | `cmd_vel.angular.z` 方向修正，常用值 `1.0/-1.0` |
| `left_track_direction_correction` | `1.0` | 左履带方向修正，常用值 `1.0/-1.0` |
| `right_track_direction_correction` | `1.0` | 右履带方向修正，常用值 `1.0/-1.0` |
| `timeout_sec` | `0.3` | 超时后自动发零速度 |

## 启动

```bash
roslaunch mobility_control base_cmd.launch
```

```bash
roslaunch mobility_control base_cmd.launch \
  input_topic:=/cmd_vel \
  output_topic:=/wheel_controller/cmd_vel \
  max_linear_x:=0.6 \
  max_angular_z:=1.0 \
  timeout_sec:=0.2
```

## 测试

```bash
# 正常速度
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.2}}'

# 超限输入，观察输出是否被截断
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 1.5}, angular: {z: 2.0}}'

rostopic echo /wheel_controller/cmd_vel
```

## 边界

- 不直接写 CAN
- 内部会临时做 `Twist -> 左右履带速度 -> Twist` 的换算，仅用于方向修正
- 不承担机械臂、摆臂或总控逻辑
- 平滑发生在 `linear.x` / `angular.z` 命令层，不直接读取车轮反馈闭环
