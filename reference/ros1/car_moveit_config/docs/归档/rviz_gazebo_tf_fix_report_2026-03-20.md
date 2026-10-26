# RViz/Gazebo 联调问题修复报告（2026-03-20）

## 1. 背景与现象
在 Gazebo + MoveIt + RViz 联调中出现以下问题：

1. RViz 中机器人底盘不随 Gazebo 运动。
2. 深度相机显示方向异常（画面偏右/朝下）。
3. MoveIt 持续告警缺失轮子关节状态：
   - `left_front_wheel_joint`
   - `left_rear_wheel_joint`
   - `right_front_wheel_joint`
   - `right_rear_wheel_joint`
4. TF 出现 `zhua1/zhua2` 的 NaN 变换报错，机械臂模型异常。

## 2. 根因分析

### 2.1 RViz 不跟随 Gazebo
RViz 的显示依赖 TF。底盘插件发布的里程计帧与机器人根帧配置不一致时，RViz 会表现为不跟随或跟随异常。

### 2.2 深度相机方向异常
相机使用了标准机器人坐标系与光学坐标系的混用配置时，会出现画面偏移/方向异常。

### 2.3 MoveIt 缺轮子关节状态
MoveIt 监听 `/joint_states`，轮子关节状态未稳定出现在它使用的状态流中，会触发完整状态缺失告警。

### 2.4 `zhua1/zhua2` NaN TF
日志中存在 `gazebo_ros_control` 对 `left_gripper_finger_joint` 的 PID 缺失报错，导致夹爪关节状态异常，连带触发 `zhua1/zhua2` 的 NaN TF。

## 3. 已实施修复

### 3.1 相机与深度话题坐标修复（URDF）
- 保证抓手相机使用 `catch_camera_optical_frame`。
- 深度点云与相机信息统一发布到 optical frame。
- 抓手相机安装姿态做了定向调整，避免“朝下拍”误差。
- 注意：这只是传感器发布坐标系；MoveIt Servo / 遥操作控制仍建议使用 `catch_camera` 本体坐标系，而不是 optical frame。

### 3.2 底盘 TF 对齐（URDF）
- 底盘插件使用 `odom` 作为里程计帧并广播 TF。
- `robotBaseFrame` 对齐机器人根帧链路，恢复 RViz 与 Gazebo 一致性。

### 3.3 Gazebo 控制 PID 修复（MoveIt config）
文件：`car_moveit_config/config/gazebo_controllers.yaml`

新增：
```yaml
gazebo_ros_control:
  pid_gains:
    left_gripper_finger_joint: {p: 50.0, i: 0.0, d: 1.0}
```
用于消除夹爪关节控制器的 PID 缺失，避免 NaN 关节状态传播到 TF。

### 3.4 RViz 固定坐标修正
文件：`car_moveit_config/launch/moveit.rviz`

- `Fixed Frame` 调整为 `odom`，确保可视化随里程计运动更新。

## 4. 验证结果（现场）

当前用户反馈：上述主要问题已恢复正常。

建议保留以下回归检查命令：

```bash
rosrun tf tf_echo odom base_link_root
rosrun tf tf_echo base_link_root base_link
rostopic echo -n1 /gripper_camera/depth/camera_info/header/frame_id
rostopic echo -n1 /joint_states/name
```

验收标准：

1. `odom -> base_link_root` 随底盘运动变化。
2. `base_link_root -> base_link` 为稳定固定变换。
3. 深度相机 `frame_id` 为 `catch_camera_optical_frame`。
4. `/joint_states` 中无 NaN 且包含关键关节状态。

## 5. 备注

- 日志中若再次出现 `SpawnModel: Failure - entity already exists` 或 `new node registered with same name`，应先清理重复 Gazebo/roslaunch 进程后再单实例启动。
- 本次修复后，建议后续将底盘轮子状态发布方案和 MoveIt 状态监听策略统一文档化，避免多命名空间引起的二次告警。
