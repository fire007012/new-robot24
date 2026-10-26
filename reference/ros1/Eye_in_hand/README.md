# Eye_in_hand — D405 手眼标定包

把装在机械臂末端 `tool0` 上的 RealSense D405（paw_camera）做一次手眼标定，求出 `tool0 → paw_camera_color_optical_frame` 的精确变换。

## ROS2 依赖替换状态

主仓库现已用 [`easy_handeye2`](../easy_handeye2/README.md) 子模块替换旧 `easy_handeye`，锁定提交 `29bd50a`，目标为 Ubuntu 22.04 / ROS2 Humble。使用 `git submodule update --init easy_handeye2` 恢复依赖，不要在当前源码树重新克隆 ROS1 easy_handeye。

本包正式入口已迁移为 ROS2 `eye_in_hand`：rclpy 自动采样通过项目 ExecuteArmGoal 运动，easy_handeye2 负责求解、保存和发布。当前用法及合成 TF 验收见 [记录 34](../docs/34_ROS2全量软件集成与验收.md)；下文 ROS1 流程保留作历史参考。旧 `6x6_250 / 582` 配置矛盾，不能作为实际 marker 参数，必须核对打印标记后显式传入 ID。

## 目录结构
```
Eye_in_hand/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── calibration_params.yaml      # marker ID、边长、话题、frame 命名
│   └── auto_poses.yaml              # 自动模式的 home 位姿和 16 组扰动
├── launch/
│   ├── aruco_detect.launch          # 只启动 ArUco 检测（调试用）
│   ├── eye_in_hand_calibrate.launch      # 手动模式（需要能拖动机械臂）
│   ├── eye_in_hand_auto_calibrate.launch # 自动模式（MoveIt 走预设位姿）
│   └── publish_calibration.launch   # 日常发布标定结果 TF
└── scripts/
    └── auto_sample.py               # 自动采样：走位姿 + take_sample + compute + save
```

## 一、旧 ROS1 依赖安装（历史参考，当前工作区不适用）

```bash
cd ~/catkin_ws/src

# 1. easy_handeye（手眼标定主包）
git clone https://github.com/IFL-CAMP/easy_handeye.git

# 2. aruco_ros（ArUco marker 检测）
git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros.git

# 3. 依赖安装
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
catkin_make
source devel/setup.bash
```

## 二、打印 ArUco Marker

默认参数：
- 字典：`6x6_250`（标准 ArUco 字典）
- ID：`582`
- 边长：`50mm`（即 `marker_size: 0.05`）

生成 marker：https://chev.me/arucogen/
（Dictionary 选 `6x6 (250 markers)`，Marker ID 填 `582`，Marker size 填 `50`）

贴到硬纸板上**平整固定**到桌面（标定过程中不能动）。

> ⚠️ 打印后**务必用尺子量实际边长**（打印缩放常有 1-2mm 偏差），把实测值写到 `config/calibration_params.yaml` 的 `marker_size`。

## 三、标定流程（二选一）

### 方式 A：自动模式（推荐 —— 机械臂拖不动时用）

#### 1. 先手动确定 home 位姿
机械臂手动/键盘控制到一个姿态，让 D405 正对 marker、距离 15-25cm、marker 在画面中心。
```bash
rostopic echo /joint_states -n 1
```
把这 6 个关节角填到 `config/auto_poses.yaml` 的 `home_pose` 字段。

#### 2. 启动自动标定
```bash
roslaunch Eye_in_hand eye_in_hand_auto_calibrate.launch
```
这会启动：
- Eyou 机械臂硬件 + MoveIt（auto_release=true，机械臂使能）
- D405 相机
- ArUco 检测
- easy_handeye 标定（`freehand_robot_movement=false`）
- `auto_sample.py` 自动走 17 个位姿，每个位姿 take_sample 一次
- 全部采完自动 compute + save，结果在 `~/.ros/easy_handeye/paw_d405_handeye_eye_on_hand.yaml`

> ⚠️ **安全提醒**：第一次运行建议先把 `auto_poses.yaml` 里的 `velocity_scaling` 和 `acceleration_scaling` 调到 0.1（很慢），并站在急停旁边看着，避免预设位姿不合理撞东西。

如果有位姿 MoveIt 规划失败（报"不可达"），改 `auto_poses.yaml` 把该位姿的 delta 减小一点。

### 方式 B：手动模式（机械臂可拖动时用）

```bash
roslaunch Eye_in_hand eye_in_hand_calibrate.launch
```

这会启动：
- Eyou 机械臂硬件（只开 joint_state_controller，不使能电机，关节可手动拖动）
- D405 相机、aruco 检测、easy_handeye GUI

### 2. 先在 RViz 里看一眼（方式 B 手动模式）

打开后应该能看到：
- `base_link` → ... → `tool0` 的机械臂 TF 树
- `paw_camera_color_optical_frame` → `aruco_marker_frame` 的视觉 TF（只要 marker 在画面里）

如果 marker 没被检测到，先跑 `roslaunch Eye_in_hand aruco_detect.launch`，用 `rqt_image_view` 看 `/aruco_single/result` 确认识别正常。

### 3. 采样 15~20 组（方式 B）

在 easy_handeye 的 rqt 面板里：
1. **手动把机械臂拖到某个位姿**（需要末端无力矩）
2. 保证 marker 仍在 D405 视野里、清晰可见
3. 点 "**Check starting pose**" 确认 TF 正常
4. 点 "**Take sample**" 采集一次

**关键：位姿要多样**
- 平移：在 marker 正上方不同距离（10~40cm）
- 旋转：至少绕 XYZ 三个轴各变化 ±30°
- 不要只是简单平移（AX=XB 会欠定）

### 4. 求解 + 保存（方式 B）

- 点 "**Compute**" 求解
- 查看结果合理性（平移应该在几 cm 量级，和 URDF 里的估计值 `xyz="-0.0463, 0.0013, -0.0338"` 量级一致）
- 点 "**Save**"，结果写到 `~/.ros/easy_handeye/paw_d405_handeye_eye_on_hand.yaml`

## 四、日常使用标定结果

标定做完后，每次开机跑这个就能把 paw_camera 挂到 URDF 的 TF 树上：

```bash
roslaunch Eye_in_hand publish_calibration.launch
```

验证：
```bash
rosrun tf tf_echo base_link paw_camera_color_optical_frame
```

应该能看到一个稳定的变换（机械臂动的时候也跟着变）。

## 五、和 URDF 里 catch_camera 的关系

目前 URDF (`car_urdf.urdf`) 里定义了 `tool0 → catch_camera → catch_camera_optical_frame`，但这是 SolidWorks 导出的**估计值**，且 `catch_camera_optical_frame` 和实机驱动发布的 `paw_camera_color_optical_frame` 是两套不相关的 frame。

**推荐后续处理（可选）**：
- 把 URDF 里的 `catch_camera` 重命名为 `paw_camera_link`、`catch_camera_optical_frame` 重命名为 `paw_camera_color_optical_frame`，**让 URDF 的 frame 和驱动发的 frame 名字对齐**
- 或者保留 URDF 现有命名，只用标定结果发一条 `tool0 → paw_camera_color_optical_frame` static TF（本包 `publish_calibration.launch` 就是这个方案）
- `D:/vstest/arm_control/` 里的 `servo_twist_frame_bridge_node.py` 依赖 `catch_camera` 这个 frame，改名需要同步修改它

## 六、常见问题

| 问题 | 排查 |
|------|------|
| Marker 检测不到 | `/paw_camera/color/image_rect_color` 有没有？ID 和 size 对不对？光照？离得太远？ |
| TF 有 `paw_camera_color_optical_frame` 但 easy_handeye 报错 | `robot_base_frame` / `robot_effector_frame` 填对没？`tool0` 是否真的在 /joint_states 驱动下被 robot_state_publisher 发出来了？ |
| Compute 报错"not enough samples" | 至少采 10 组；位姿必须有旋转多样性 |
| 求解结果平移偏大（>20cm） | 大概率采样位姿旋转不够或 marker 边长填错 |
| D405 近距离对焦差 | D405 最佳工作距离 7~50cm，marker 别贴到镜头上 |
