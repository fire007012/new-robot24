# ROS1 历史参考

当前项目只维护 ROS2 Humble，不再保留 ROS1 回退交付要求。本目录供协议、设备配置和历史行为对照，不属于活动工作区。

`COLCON_IGNORE` 和 `CATKIN_IGNORE` 阻止构建工具递归发现这里的历史包。不要从此目录启动主站、硬件控制器或旧 launch。

| 路径 | 内容 |
| --- | --- |
| Eyou_Canopen_Master | 旧 Catkin CANopen 主站源码快照；原子模块已解除关联 |
| Eyou_ROS1_Master | 旧统一硬件层 |
| tcp_bridge | 旧 TCP 桥 |
| vision_opencv | 旧 ROS1 cv_bridge/image_geometry/opencv_tests |
| can_driver | 活动 ROS2 驱动中移出的旧 RobotHW、XmlRpc 解析、XML launch、rostest、控制器配置及 Noetic CI/说明；ROS2 子模块关联继续保留 |
| arm_control、car_control、car_moveit_config、car_urdf、flipper_control、mobility_control、robot_bringup | 已迁移包中不再使用的 ROS1 launch、控制源码、配置及旧说明 |
| Eye_in_hand、ros1_bridge、vision_pkg、senseair_co2、thermal_camera | 标定、上位机和外围旧启动配置及说明；活动包名见根 README |
| docs/历史文档 | 原 docs/归档 的历史方案、故障分析与阶段记录 |
| ci/noetic.yml | 原 Noetic/Catkin CI 配置，仅供参考 |
| catkin_toplevel.txt | 原根 Catkin CMakeLists.txt 符号链接的目标记录 |

Eyou_Canopen_Master 原来源：`git@github.com:fire007012/Eyou_Canopen_Master.git`，分支 `feature/staged-integration`，快照提交 `c890355b6fc2151c2ea9d3dd7a9d1bf21c9799e9`。目录现为普通文件，已去掉 `.git` 指针、父仓库 gitlink、`.gitmodules` 声明及本地子模块配置。父仓库内部可能仍保存历史 Git 对象，不构成活动关联。

这里的旧说明保留原文，所述路径、依赖和架构可能已经失效。当前入口以 [主 README](../../README.md) 和 [当前文档](../../docs/README.md) 为准。完成测试产生的 evidence、日志、TF 导出图和缓存不随参考代码保留。
