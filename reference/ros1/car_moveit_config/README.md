> 2026-10-26：本包正式构建已切换 ROS2 Humble／ament。当前命令与验证见 [ROS2 实施记录](../docs/33_ROS2主控制链与MoveIt2实施记录.md)；下文 ROS1 示例保留作历史参考，不用于当前构建。

# car_moveit_config

`car_urdf` 的 MoveIt 配置包，负责规划组、运动学、控制器映射、RViz 配置和 Gazebo 联调入口。

## 包职责

- 提供 `arm`、`gripper` 两个 MoveIt 规划组的语义配置。
- 维护 OMPL/CHOMP/STOMP/Pilz 规划参数与关节约束。
- 统一 MoveIt、Gazebo、`ros_control`、RViz 的启动入口。

## 包结构

```text
car_moveit_config/
|-- config/
|   |-- car_urdf.srdf
|   |-- kinematics.yaml
|   |-- joint_limits.yaml
|   |-- ompl_planning.yaml
|   `-- ros_controllers.yaml
|-- launch/
|   |-- demo.launch
|   |-- demo_gazebo.launch
|   |-- gazebo.launch
|   |-- move_group.launch
|   `-- moveit_rviz.launch
`-- docs/
    `-- 归档/
```

## 快速开始

先做纯 MoveIt 规划验证：

```bash
cd ~/robot24_ws
catkin_make --pkg car_urdf car_moveit_config
source devel/setup.bash
roslaunch car_moveit_config demo.launch
```

需要 Gazebo + MoveIt 联调时：

```bash
roslaunch car_moveit_config demo_gazebo.launch
```

## 常用命令

```bash
# 只启动 move_group
roslaunch car_moveit_config move_group.launch pipeline:=ompl

# 单独打开 MoveIt RViz
roslaunch car_moveit_config moveit_rviz.launch \
  rviz_config:=$(rospack find car_moveit_config)/launch/moveit.rviz

# 只起 Gazebo 与 ros_control 控制器
roslaunch car_moveit_config gazebo.launch

# OMPL benchmark
roslaunch car_moveit_config run_benchmark_ompl.launch cfg:=/absolute/path/bench.cfg
```

## 节点与接口摘要

- 规划组：`arm`（`base_link -> grasp_tcp`）、`gripper`（`left_gripper_finger_joint`）。
- 主要节点：`move_group`、`robot_state_publisher`、`controller_spawner`、`rviz`。
- 轨迹执行接口：
  - `/arm_position_controller/follow_joint_trajectory`
  - `/gripper_controller/follow_joint_trajectory`
- 关键参数：
  - `robot_description`
  - `robot_description_semantic`
  - `robot_description_planning/*`
  - `robot_description_kinematics/*`
- 关键资源：
  - `config/car_urdf.srdf`
  - `config/joint_limits.yaml`
  - `config/ros_controllers.yaml`
  - `config/ompl_planning.yaml`

## 详细文档索引

- `README.md`：当前维护入口。
- `docs/README.md`：包内文档与归档索引。
- `docs/归档/rviz_gazebo_tf_fix_report_2026-03-20.md`：2026-03-20 联调修复记录，属于项目联调资料，保留为历史参考，不作为当前接口说明。
