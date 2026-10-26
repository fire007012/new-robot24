# senseair_co2 · ROS2 CO₂ 接口

scripts/co2_publisher.py 提供 TCP/Modbus CO₂ 采集；launch/ 提供 ROS2 参数入口。软件回归覆盖模拟服务端的分片帧和 CRC；真实传感器地址、链路和读数待现场核验。

```bash
ros2 launch senseair_co2 senseair_co2.launch.py
```

从仓库根目录使用 `bash tools/build_ros2.sh /tmp/robot24-ros2 full` 构建并 source 安装环境。组件示例须配合整车运行入口，运动需要显式许可。当前架构、完整参数与验证边界见 [根 README](../README.md) 和 [文档索引](../docs/README.md)。历史 ROS1 内容位于 `reference/ros1/`；持续回归源码保留。
