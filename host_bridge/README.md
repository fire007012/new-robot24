# host_bridge · ROS2 TCP/JSON 上位机与视频桥

目录已由 ros1_bridge 改名，正式包名 host_bridge。提供 TCP/JSON 控制、心跳重连、运行会话适配、视频管理 IPC、ROS 图像转 RTSP 及可选 MediaMTX。video_stack.sh 和 launch/video_stack.launch.py 可从安装目录使用；必须显式传入现场视频配置。video_sources.local*.yaml 是本地配置，整理时保留；示例配置不等于真实源已验收。

```bash
ros2 launch host_bridge video_stack.launch.py video_config:=/absolute/path/video_sources.yaml start_mediamtx:=false
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。

控制 v2 可协商 `motion_presets_v1`：摆臂 `flippers`、底盘 `base`、机械臂含夹爪 `arm` 三组任意多选。`control_hello.presets` 确认初始选择；`select_presets` 更新选择并撤销旧授权；生命周期 `system_command.params.presets` 可显式操作已选预设的子集，统一调用运行协调器的组服务。未协商扩展的旧 v2 客户端保留全车语义。协议、状态机、部署限制和可复制的上位机开发提示词见 [文档 51](../docs/51_三组预设与上位机对接提示词.md)。
