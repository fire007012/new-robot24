# flipper_control 文档导航

`flipper_control` 的主入口是 [../README.md](/home/rera/robot24_ws/src/flipper_control/README.md)。先看包根 README，就能完成启动、切档、联动切换和接口对接。

## 你通常只需要看这些

- [../README.md](/home/rera/robot24_ws/src/flipper_control/README.md)
  - 包职责、结构图、快速开始、接口速查、常用命令。
- [flipper_motor_debug_ui.md](/home/rera/robot24_ws/src/flipper_control/docs/flipper_motor_debug_ui.md)
  - 调试 UI 的启动方式、生命周期适配逻辑、控制器联调方法与排障说明。
- [../config/flipper_control.yaml](/home/rera/robot24_ws/src/flipper_control/config/flipper_control.yaml)
  - 参数默认值与控制器命名。
- [../launch/flipper_control.launch](/home/rera/robot24_ws/src/flipper_control/launch/flipper_control.launch)
  - 标准启动参数入口。

## 接口落点

- 节点实现：
  - [../src/flipper_manager_node.cpp](/home/rera/robot24_ws/src/flipper_control/src/flipper_manager_node.cpp)
- 参考生成器：
  - [../include/flipper_control/flipper_reference_generator.hpp](/home/rera/robot24_ws/src/flipper_control/include/flipper_control/flipper_reference_generator.hpp)
  - [../src/flipper_reference_generator.cpp](/home/rera/robot24_ws/src/flipper_control/src/flipper_reference_generator.cpp)
- ROS 接口：
  - [../msg/FlipperControlState.msg](/home/rera/robot24_ws/src/flipper_control/msg/FlipperControlState.msg)
  - [../srv/SetControlProfile.srv](/home/rera/robot24_ws/src/flipper_control/srv/SetControlProfile.srv)
  - [../srv/SetLinkageMode.srv](/home/rera/robot24_ws/src/flipper_control/srv/SetLinkageMode.srv)
- 调试工具：
  - [../scripts/flipper_motor_debug_ui.py](/home/rera/robot24_ws/src/flipper_control/scripts/flipper_motor_debug_ui.py)
  - [flipper_motor_debug_ui.md](/home/rera/robot24_ws/src/flipper_control/docs/flipper_motor_debug_ui.md)

## 历史设计参考

当前包内没有需要单独归档的历史报告。与摆臂控制相关的阶段性设计/实施记录已归入项目级 `docs/归档/`：

- [/home/rera/robot24_ws/src/docs/归档/19_控制包拆分与当前结构说明_2026-04-14.md](/home/rera/robot24_ws/src/docs/归档/19_控制包拆分与当前结构说明_2026-04-14.md)
- [/home/rera/robot24_ws/src/docs/归档/21_摆臂控制器模式与冷切换设计_2026-04-15.md](/home/rera/robot24_ws/src/docs/归档/21_摆臂控制器模式与冷切换设计_2026-04-15.md)
- [/home/rera/robot24_ws/src/docs/归档/22_摆臂控制落地与提交路线图_2026-04-15.md](/home/rera/robot24_ws/src/docs/归档/22_摆臂控制落地与提交路线图_2026-04-15.md)
