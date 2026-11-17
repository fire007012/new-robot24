# arm_control · ROS2 机械臂执行与适配

`src/arm_goal_executor.cpp` 提供 ExecuteArmGoal，使用 MoveIt 2 规划后向唯一六轴 JTC 派发，含许可/反馈检查、并发拒绝和取消。`scripts/` 提供夹爪 Bool/Float64 → m/s 适配、腕部 Trigger 服务及目标客户端。

```bash
ros2 run arm_control send_execute_arm_goal --named zero
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。

## Servo 准备和限位

启动时带 `enable_moveit:=true enable_servo:=true`，机械臂处于轨迹模式 Running 后，可显式规划到历史 `ready` 姿态，再切入 Servo：

```bash
ros2 service call /arm_control/prepare_servo std_srvs/srv/Trigger '{}'
ros2 service call /arm_control/set_servo_mode std_srvs/srv/SetBool '{data: true}'
```

第一条会产生经过 MoveIt 规划的六轴运动，等待返回 success 后再执行第二条；也可使用 `send_execute_arm_goal --named ready`。准备失败时不自动重试，不切换模式。切换模式本身不会自动移动到准备姿态。

`zero`、`home` 与六轴位置/速度限位保持 `reference/ros1` 的原值。Servo 切入和恢复前要求六轴反馈完整、新鲜、静止，雅可比条件数小于历史减速阈值 17，并保持历史 0.1 rad 限位余量。zero 奇异，home 的肩俯仰距下限约 0.083 rad，因此这两个姿态仍可用于关节空间规划，但不能直接作为 Servo 起点。切回轨迹模式后可规划到 ready。

执行器只使用本组六轴位置及速度反馈；其他组离线的 NaN 不会阻塞机械臂。派发前实际关节与规划起点/首点须在 0.01 rad 内、实测速度不超过历史 JTC 的 0.05 rad/s 静止容差；不满足时返回 NOT_READY，须重新请求规划。六轴均为有限位关节，腕滚转比较不按 2π 回绕。轨迹点另外检查位置和速度限位；不替代底层控制器及驱动保护。
