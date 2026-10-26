# car_control 包梳理报告

## 包总览

**功能定位**：负责底盘移动、夹爪控制，以及 DS4/DS5 手柄遥控 MoveIt Servo 的完整遥操作系统。

---

## 目录结构

```
car_control/
├── config/
│   ├── control_params.yaml      # 所有节点参数配置
│   └── moveit_servo.yaml        # MoveIt Servo 配置
├── docs/
│   └── car_control_report.md    # 本报告
├── launch/
│   ├── control_nodes.launch     # 仅启动底盘+夹爪节点
│   ├── moveit_servo.launch      # 仅启动 MoveIt Servo
│   ├── servo_twist_frame_bridge.launch  # optical frame -> control frame 桥接
│   ├── teleop_system.launch     # 手柄+底盘+夹爪（常用入口）
│   └── sim_test.launch          # Gazebo 仿真完整测试
├── scripts/
│   ├── base_cmd_node.py         # 底盘速度安全节点
│   ├── ds5_teleop_node.py       # 手柄遥控节点
│   ├── gripper_cmd_node.py      # 夹爪控制节点
│   └── servo_twist_frame_bridge_node.py # Twist 坐标系桥接
├── README.md
├── CMakeLists.txt
└── package.xml
```

---

## 三个核心节点

### 1. `base_cmd_node.py` — 底盘安全中继

| 项目 | 内容 |
|------|------|
| 订阅话题 | `/car_control/cmd_vel`（geometry_msgs/Twist） |
| 发布话题 | `/car_urdf/cmd_vel`（geometry_msgs/Twist） |
| 功能 | 速度限幅（linear.x ≤ 0.8 m/s，angular.z ≤ 1.5 rad/s）+ 超时 0.5s 自动刹停（20Hz watchdog） |

**行为说明**：
- 接收上游速度指令，对线速度和角速度做限幅后转发给底盘驱动器。
- 内置 watchdog 定时器（20Hz），若超过 `timeout_sec`（默认 0.5s）未收到新指令，则自动发布零速度以刹停底盘，防止失控。

---

### 2. `gripper_cmd_node.py` — 夹爪控制

| 项目 | 内容 |
|------|------|
| 订阅话题1 | `/car_control/gripper_position`（std_msgs/Float64）— 精确位置控制 |
| 订阅话题2 | `/car_control/gripper_open`（std_msgs/Bool）— 一键开/合 |
| 发布话题 | `/gripper_controller/command`（trajectory_msgs/JointTrajectory） |
| 关节名称 | `left_gripper_finger_joint` |

**行为说明**：
- 接收 `Bool` 指令时：`true` → 发送 open_pos（0.03m），`false` → 发送 close_pos（-0.022m）。
- 接收 `Float64` 指令时：直接设定目标位置（会做范围限幅）。
- 所有指令都转换为 `JointTrajectory`，运动时间默认 0.8s。

---

### 3. `ds5_teleop_node.py` — 手柄遥控（核心）

| 项目 | 内容 |
|------|------|
| 订阅话题 | `/joy`（sensor_msgs/Joy） |
| 发布话题1 | `/car_control/cmd_vel`（geometry_msgs/Twist）— 底盘指令 |
| 发布话题2 | `/servo_server/delta_twist_cmds`（geometry_msgs/TwistStamped）— MoveIt Servo，frame=`catch_camera` |
| 发布话题3 | `/car_control/gripper_open`（std_msgs/Bool）— 夹爪开闭 |

**两种工作模式（`Options` 键切换）：**

| 模式 | 底盘 | 机械臂 Servo |
|------|------|------|
| CHASSIS | 左摇杆 LY → 前后，LX/十字键 X → 转向 | 发布零速度 |
| ARM_SERVO | 发布零速度 | 左摇杆、右摇杆、扳机控制末端 Twist |

**ARM_SERVO 模式轴映射：**

| 轴/按键 | 控制量 |
|---------|--------|
| 左摇杆 LX | linear.y |
| 左摇杆 LY | linear.z |
| L2 / R2 | linear.x 正 / 负 |
| L1 / R1 | angular.x（roll）正 / 负 |
| 右摇杆 RY | angular.y（pitch，方向已反转） |
| 右摇杆 RX | angular.z（yaw） |

**夹爪按键（任意模式均有效）：**

| 按键 | 动作 |
|------|------|
| Square（□） | 打开夹爪 |
| Circle（○） | 闭合夹爪 |

**安全机制**：内置 watchdog（20Hz），若超过 `cmd_timeout`（默认 0.25s）未收到手柄数据，自动发布零速度。

**坐标系约定**：
- 相机传感器话题默认使用 `catch_camera_optical_frame`
- `ds5_teleop_node.py` 与 MoveIt Servo 控制命令使用 `catch_camera`
- 若上游输入来自 optical frame，需先经过 `servo_twist_frame_bridge_node.py`

### 4. `servo_twist_frame_bridge_node.py` — Twist 坐标系桥接

| 项目 | 内容 |
|------|------|
| 订阅话题 | `/car_control/delta_twist_cmds_optical`（geometry_msgs/TwistStamped） |
| 发布话题 | `/servo_server/delta_twist_cmds`（geometry_msgs/TwistStamped） |
| 功能 | 将 `catch_camera_optical_frame` 下的 Twist 转到 `catch_camera`，供 MoveIt Servo 使用 |

---

## 话题数据流图

```
[/dev/input/js0]
      │
   joy_node / game_controller_node
      │
      │  /joy (sensor_msgs/Joy)
      ▼
ds5_teleop_node
      │                    │                        │
      │ /car_control/      │ /servo_server/          │ /car_control/
      │  cmd_vel           │  delta_twist_cmds       │  gripper_open
      ▼                    ▼                         ▼
base_cmd_node         servo_server             gripper_cmd_node
      │               (MoveIt Servo)                 │
      │ /car_urdf/         │                         │ /gripper_controller/command
      │  cmd_vel           │ 关节位置指令             ▼
      ▼                    ▼                    夹爪控制器
  底盘驱动器           机械臂关节
```

---

## Launch 文件关系

```
sim_test.launch                    （完整仿真入口）
  ├── car_moveit_config/
  │     demo_gazebo.launch         （Gazebo + MoveIt + RViz）
  ├── moveit_servo.launch           （启动 servo_server）
  ├── servo_twist_frame_bridge.launch（由 teleop_system 默认带起）
  └── teleop_system.launch          （手柄遥操作系统）
        ├── joy_node /
        │   game_controller_node   （手柄驱动）
        ├── control_nodes.launch
        │     ├── base_cmd_node    （底盘安全中继）
        │     └── gripper_cmd_node （夹爪控制）
        └── ds5_teleop_node        （手柄解析与分发）
```

**常用启动命令：**

```bash
# 仅底盘 + 夹爪
roslaunch car_control control_nodes.launch

# 手柄 + 底盘 + 夹爪（不含 MoveIt Servo）
roslaunch car_control teleop_system.launch joy_dev:=/dev/input/js0

# 完整仿真（Gazebo + MoveIt + 手柄）
roslaunch car_control sim_test.launch joy_dev:=/dev/input/js0
```

---

## 关键参数（control_params.yaml）

### base_cmd_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| input_topic | /car_control/cmd_vel | 输入话题 |
| output_topic | /car_urdf/cmd_vel | 输出话题 |
| max_linear_x | 0.8 m/s | 底盘最大线速度 |
| max_angular_z | 1.5 rad/s | 底盘最大角速度 |
| timeout_sec | 0.5 s | 超时刹停阈值 |

### gripper_cmd_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| joint_name | left_gripper_finger_joint | 夹爪关节名 |
| open_position | 0.03 m | 开爪目标位置 |
| close_position | -0.022 m | 闭爪目标位置 |
| min_position | -0.022 m | 位置下限 |
| max_position | 0.03 m | 位置上限 |
| move_duration | 0.8 s | 轨迹执行时间 |

### ds5_teleop_node

| 参数 | 默认值 | 说明 |
|------|--------|------|
| max_chassis_vx | 0.8 m/s | 底盘最大线速度 |
| max_chassis_wz | 1.5 rad/s | 底盘最大角速度 |
| max_arm_linear | 0.15 m/s | 机械臂末端最大线速度 |
| max_arm_angular | 0.6 rad/s | 机械臂末端最大角速度 |
| servo_frame | catch_camera | DS5 遥操作发送给 MoveIt Servo 的控制坐标系 |
| deadzone | 0.12 | 摇杆死区 |
| cmd_timeout | 0.25 s | 手柄超时阈值 |
| mode_switch_buttons | [9, 10] | 模式切换按键索引 |
| chassis_turn_axis_candidates | [0, 6] | 转向轴候选（按优先级选择） |

---

## 快速测试命令

```bash
# 底盘前进
rostopic pub -1 /car_control/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 夹爪打开
rostopic pub -1 /car_control/gripper_open std_msgs/Bool 'data: true'

# 夹爪闭合
rostopic pub -1 /car_control/gripper_open std_msgs/Bool 'data: false'

# MoveIt Servo 手动测试（末端沿 x 轴前移）
rostopic pub -1 /servo_server/delta_twist_cmds geometry_msgs/TwistStamped \
  '{header: {frame_id: "catch_camera"}, twist: {linear: {x: 0.05}}}'

# optical frame 输入示例：相机前方为 optical z 正方向
roslaunch car_control servo_twist_frame_bridge.launch
rostopic pub -1 /car_control/delta_twist_cmds_optical geometry_msgs/TwistStamped \
  '{header: {frame_id: "catch_camera_optical_frame"}, twist: {linear: {z: 0.05}}}'
```
