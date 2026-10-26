# CANopen 联调命令速查

更新时间：2026-03-25  
适用包：`Eyou_Canopen_Master`

## 1. CAN 启动与检查

```bash
# 1) 加载内核模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

# 2) 配置并拉起 canable0（1Mbps）
sudo ip link set canable0 down 2>/dev/null || true
sudo ip link set canable0 type can bitrate 1000000
sudo ip link set canable0 up

# 3) 查看状态
ip -details -statistics link show canable0
```

抓包：

```bash
# 全量抓包
candump canable0

# 关键帧抓包（SYNC + PDO + Heartbeat + EMCY）
candump canable0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

## 2. 赋权命令（可选）

推荐直接用 `sudo` 配置 CAN。  
如需减少 sudo 频率，可给工具加 capability（按需使用）：
```bash
# 先找到节点二进制
realpath ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node

# 给节点提权（打开 RAW CAN 一般需要 NET_RAW；加 NET_ADMIN 更保险）
sudo setcap cap_net_raw,cap_net_admin+eip ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node

# 检查
getcap ~/Robot24_catkin_ws/devel/lib/Eyou_Canopen_Master/canopen_hw_ros_node
```

若 USB-CAN 走串口设备（`/dev/ttyUSB*`）：

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

## 3. 生成 DCF

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh full
rosrun Eyou_Canopen_Master generate_dcf.sh arm_only
```

## 4. 编译

```bash
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master
cmake --build build -j
```

或 catkin：

```bash
cd ~/Robot24_catkin_ws
catkin_make --pkg Eyou_Canopen_Master
source devel/setup.bash
```

## 5. 启动节点

```bash
cd ~/Robot24_catkin_ws
source devel/setup.bash
roslaunch Eyou_Canopen_Master bringup.launch auto_init:=false
```

常用参数：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  dcf_path:=/home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master/config/master.dcf \
  joints_path:=/home/dianhua/Robot24_catkin_ws/src/Eyou_Canopen_Master/config/joints.yaml \
  loop_hz:=200.0 auto_init:=false auto_enable:=false auto_release:=false
```

启用 IP 轨迹执行器：

```bash
roslaunch Eyou_Canopen_Master bringup.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=arm_position_controller/follow_joint_trajectory

# 可选：覆盖执行器频率
rosparam set /canopen_hw_node/ip_executor_rate_hz 100.0
```

4 轴摆臂如果只是验证 `flipper_csp_controller` 的 action/轨迹接口，可以走 IP executor：

```bash
cd ~/robot24_ws
source devel/setup.bash

roslaunch Eyou_Canopen_Master bringup_flipper_4axis.launch \
  use_ip_executor:=true \
  ip_executor_action_ns:=flipper_csp_controller/follow_joint_trajectory
```

如果要联调 `flipper_control`：

- `csv_velocity` 独立摆臂测试，推荐直接走 `canopen_hw_node`
- 全车 `CSP` 混合后端联调，再走 `hybrid_motor_hw_node`

`flipper_control` 的三个 profile 与底层模式对应关系：

- `csp_position` -> `CSP`
- `csp_jog` -> `CSP`
- `csv_velocity` -> `CSV`

若 4 轴启动时报下面这类错误：

```text
error: .../left_front_arm_joint.bin: No such file or directory
error: SDO abort code 08000020 on upload request of object 1F22:01 (Concise DCF)
```

说明当前 `master_flipper_4axis.dcf` 是在别的机器上生成的旧产物，里面的 `UploadFile=` 仍指向旧绝对路径。需要在本机重新生成 4 轴 DCF 与 4 个从站 `.bin`：

```bash
rosrun Eyou_Canopen_Master generate_dcf.sh flipper_only
```

生成后可检查：

```bash
ls "$CFG_DIR"/left_front_arm_joint.bin \
   "$CFG_DIR"/right_front_arm_joint.bin \
   "$CFG_DIR"/left_rear_arm_joint.bin \
   "$CFG_DIR"/right_rear_arm_joint.bin

grep -n "UploadFile=" "$CFG_DIR/master_flipper_4axis.dcf" | head
```

若启动时报：

```text
Could not load controller 'flipper_csp_controller' because controller type
'position_controllers/JointTrajectoryController' does not exist.
```

先检查当前环境是否缺少 JTC 插件：

```bash
rosrun controller_manager controller_manager list-types | grep JointTrajectoryController
dpkg -l | grep ros-noetic-joint-trajectory-controller
```

为空则安装：

```bash
sudo apt update
sudo apt install ros-noetic-joint-trajectory-controller
```

4 轴 UI 调试：

```bash
cd ~/robot24_ws
source devel/setup.bash

rosrun Eyou_Canopen_Master joint_action_ui.py \
  --joints-yaml ~/robot24_ws/src/Eyou_Canopen_Master/config/joints_flipper_4axis.yaml \
  --action-ns /flipper_csp_controller/follow_joint_trajectory \
  --service-ns /canopen_hw_node
```

注意：

- `joint_action_ui.py` 适合 `FollowJointTrajectory` / action 调试
- 它不覆盖 `flipper_control` 的 `csv_velocity` 测试链路
- `csv_velocity` 更适合通过 `flipper_control/set_control_profile`、`/flipper_control/jog_cmd`、`/flipper_control/state`、`/diagnostics` 联调

独立摆臂 `CSV` 联调示例：

```bash
cd ~/robot24_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch Eyou_Canopen_Master bringup_flipper_4axis.launch \
  start_csp_controller:=false \
  start_csv_controller:=true
```

```bash
cd ~/robot24_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch flipper_control flipper_control.launch \
  backend_type:=canopen \
  canopen_ns:=/canopen_hw_node
```

```bash
rosservice call /flipper_control/set_control_profile "{profile: 'csv_velocity'}"

rostopic pub -1 /flipper_control/jog_cmd control_msgs/JointJog \
"{
  joint_names: ['left_front_arm_joint','right_front_arm_joint'],
  velocities: [0.2, 0.2],
  duration: 0.2
}"
```

## 6. Service 调用

列出服务：

```bash
rosservice list | grep canopen_hw_node
```

生命周期：

```bash
rosservice call /canopen_hw_node/init "{}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/disable "{}"
rosservice call /canopen_hw_node/halt "{}"
rosservice call /canopen_hw_node/resume "{}"
rosservice call /canopen_hw_node/recover "{}"
rosservice call /canopen_hw_node/set_zero "{axis_index: 0}"
rosservice call /canopen_hw_node/shutdown "{}"
```

状态语义（当前）：
- `init`: `Configured -> Armed`
- `enable`: `Standby -> Armed`
- `disable`: `Running/Armed/Standby -> Standby`
- `halt`: `Running -> Armed`
- `resume`: `Armed -> Running`
- `recover`: `Faulted -> Standby`（不自动上电）
- `set_zero`: 仅 `Standby` 允许；`0x607C=0 -> 读0x6064 -> 0x607C=-actual -> 0x1010:01='save'`
- `shutdown`: `* -> Configured`

设置零点（推荐顺序）：

```bash
rosservice call /canopen_hw_node/disable "{}"
rosservice call /canopen_hw_node/set_zero "{axis_index: 0}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume "{}"
```

模式切换（白名单：`7/8/9/10`）：

```bash
# 指定轴切换到 IP
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 7}"

# 全轴切换到 CSV
for i in 0 1 2 3 4 5; do
  rosservice call /canopen_hw_node/set_mode "{axis_index: $i, mode: 9}"
done
```

模式定义：
- `7`: IP
- `8`: CSP
- `9`: CSV
- `10`: CST

## 7. Controller 切换命令

```bash
# 查询
rosservice call /controller_manager/list_controllers

# 切到速度控制器（先停 position，再启 velocity）
rosrun controller_manager controller_manager stop arm_position_controller
rosrun controller_manager controller_manager start arm_velocity_controller

# 切回位置控制器
rosrun controller_manager controller_manager stop arm_velocity_controller
rosrun controller_manager controller_manager start arm_position_controller
```

## 8. Topic 查看与发布

查看：

```bash
rostopic list | grep -E "joint_states|diagnostics|controller"
rostopic echo /joint_states
rostopic echo /joint_states/position
rostopic echo /joint_states/velocity
rostopic echo /diagnostics
rostopic hz /joint_states
```

只看单关节（`joint_1`）位置/速度反馈：

```bash
rostopic echo /joint_states | grep -E "name|position|velocity"
```

发布位置轨迹（单关节示例）：

```bash
rostopic pub -1 /arm_position_controller/command trajectory_msgs/JointTrajectory \
"{joint_names: ['joint_1'], points: [{positions: [0.2], time_from_start: {secs: 2, nsecs: 0}}]}"
```

发布速度轨迹（先切到 CSV）：

```bash
rostopic pub -1 /arm_velocity_controller/command trajectory_msgs/JointTrajectory \
"{joint_names: ['joint_1'], points: [{velocities: [0.5], time_from_start: {secs: 1, nsecs: 0}}]}"
```

发送多轴 `FollowJointTrajectory`（IP executor / MoveIt 路径）：

```bash
rostopic pub -1 /arm_position_controller/follow_joint_trajectory/goal \
control_msgs/FollowJointTrajectoryActionGoal \
"{
  goal: {
    trajectory: {
      joint_names: ['joint_1','joint_2'],
      points: [
        {positions: [0.2, -0.1], time_from_start: {secs: 1, nsecs: 0}},
        {positions: [0.4,  0.0], time_from_start: {secs: 2, nsecs: 0}}
      ]
    }
  }
}"
```

## 9. 一键联调顺序（推荐）

```bash
# 1) 拉起 CAN
# 1. 先关闭接口
sudo ip link set canable0 down

# 2. 设置波特率（示例 1000Kbps）
sudo ip link set canable0 type can bitrate 1000000

# 3. 设置缓冲区大小（推荐值）
sudo ip link set canable0 txqueuelen 10000     # 或 20000 / 5000，根据需求

# 4. 重新启动
sudo ip link set canable0 up
```bash
# 1) 拉起 CAN
# 1. 先关闭接口
sudo ip link set canable1 down

# 2. 设置波特率（示例 1000Kbps）
sudo ip link set canable1 type can bitrate 1000000

# 3. 设置缓冲区大小（推荐值）
sudo ip link set canable1 txqueuelen 10000     # 或 20000 / 5000，根据需求

# 4. 重新启动
sudo ip link set canable1 up

# 2) 生成 DCF
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master/config
dcfgen -S -r -d . master.yaml

# 3) 启动 ROS 节点

roslaunch Eyou_Canopen_Master bringup.launch auto_init:=false

# 4) 初始化
rosservice call /canopen_hw_node/init "{}"

# 5) （可选）切到 IP
rosservice call /canopen_hw_node/disable "{}"
rosservice call /canopen_hw_node/set_mode "{axis_index: 0, mode: 7}"
rosservice call /canopen_hw_node/enable "{}"
rosservice call /canopen_hw_node/resume "{}"
```
