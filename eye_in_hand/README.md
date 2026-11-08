# eye_in_hand · ROS2 手眼标定

正式目录与包名均为 eye_in_hand。标定 launch 组合 ArUco、easy_handeye2 服务端与 rqt 界面，不启动相机驱动；须先从 `vision_pkg` 或整车入口启用夹爪相机，提供 `/paw_camera/color/image_raw` 和 `/paw_camera/color/camera_info`。scripts/auto_sample.py 通过项目 ExecuteArmGoal 自动走点，需另行启动并放行主控制链。config/auto_poses_ros2.yaml 保持 commissioned:false，核对现场无碰撞目标后使用独立配置。必须显式指定实际 marker ID 与边长，不能沿用旧 6x6_250 / 582。标定与保存后的静态外参发布不能同时占用同一父子 frame。

```bash
ros2 launch eye_in_hand eye_in_hand_calibrate.launch.py marker_id:=ACTUAL_ID marker_size:=0.05
```

`tools/validate_ros2.sh ... full` 会验证安装后的自动采样脚本：使用合成项目 Action 服务和 marker，检查正常采样保存、缺失 marker 中止、未确认配置拒绝运动；另外用合成 TF 验证真实标定求解、保存及重启发布。上游通用 `easy_handeye2` 运动辅助节点不在本项目使用范围内，不能将其视为已完成 ROS2 适配；自动走点使用本包的项目 Action 路径。

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
