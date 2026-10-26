# eye_in_hand · ROS2 手眼标定

正式目录与包名均为 eye_in_hand。launch/ 组合相机、ArUco 与 easy_handeye2；scripts/auto_sample.py 通过项目 ExecuteArmGoal 自动走点。config/auto_poses_ros2.yaml 保持 commissioned:false，核对现场无碰撞目标后使用独立配置。必须显式指定实际 marker ID 与边长，不能沿用旧 6x6_250 / 582。标定与保存后的静态外参发布不能同时占用同一父子 frame。

```bash
ros2 launch eye_in_hand eye_in_hand_calibrate.launch.py marker_id:=ACTUAL_ID marker_size:=0.05
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
