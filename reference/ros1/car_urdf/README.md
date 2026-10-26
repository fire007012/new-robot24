> 2026-10-26：本包正式构建已切换 ROS2 Humble／ament。当前命令与验证见 [ROS2 实施记录](../docs/33_ROS2主控制链与MoveIt2实施记录.md)；下文 ROS1 示例保留作历史参考，不用于当前构建。

# car_urdf

机器人本体描述包，维护 URDF、网格模型、Gazebo 控制器配置和基础展示/仿真启动文件。

## 包职责

- 提供整车、机械臂、夹爪、摆臂、轮组和传感器的模型描述。
- 维护 Gazebo 控制器参数与 joint 名称映射。
- 为 RViz/Gazebo 提供基础模型加载入口。

## 包结构

```text
car_urdf/
|-- config/
|   |-- car_controllers.yaml
|   `-- joint_names_car_urdf.yaml
|-- launch/
|   |-- bringup.launch
|   |-- display.launch
|   `-- gazebo.launch
|-- meshes/
|   `-- *.STL
`-- urdf/
    |-- car_urdf.csv
    `-- car_urdf.urdf
```

## 快速开始

```bash
cd ~/robot24_ws
catkin_make --pkg car_urdf
source devel/setup.bash
roslaunch car_urdf bringup.launch
```

如果只需要最小 Gazebo 模型加载：

```bash
roslaunch car_urdf gazebo.launch
```

## 常用命令

```bash
# 检查 URDF 语法
check_urdf $(rospack find car_urdf)/urdf/car_urdf.urdf

# 完整 bringup（Gazebo + 控制器 + RViz）
roslaunch car_urdf bringup.launch

# 最小 Gazebo 模型加载
roslaunch car_urdf gazebo.launch

# 查看关键 TF
rosrun tf tf_echo base_link_root base_link
```

## 资源摘要

- 主模型：`urdf/car_urdf.urdf`
- 关键末端与传感器帧：
  - `tool0`
  - `grasp_tcp`
  - `camera_optical_frame`
  - `catch_camera_optical_frame`
- 控制器配置：`config/car_controllers.yaml`
  - `joint_state_controller`
  - `arm_controller`
  - `gripper_controller`
  - 四个摆臂控制器
  - `wheel_controller`
- 网格资源：`meshes/*.STL`

## 使用说明

- `display.launch` 目前引用了仓库中不存在的 `urdf.rviz`；日常建议优先使用 `bringup.launch` 或手动打开 RViz。

## 详细文档索引

- `README.md`：当前维护入口。
- `docs/README.md`：包内文档索引。
