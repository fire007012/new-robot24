# host_bridge · ROS2 TCP/JSON 上位机与视频桥

目录已由 ros1_bridge 改名，正式包名 host_bridge。提供 TCP/JSON 控制、心跳重连、运行会话适配、视频管理 IPC、ROS 图像转 RTSP 及可选 MediaMTX。video_stack.sh 和 launch/video_stack.launch.py 可从安装目录使用；必须显式传入现场视频配置。video_sources.local*.yaml 是本地配置，整理时保留；示例配置不等于真实源已验收。

```bash
ros2 launch host_bridge video_stack.launch.py video_config:=/absolute/path/video_sources.yaml start_mediamtx:=false
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
