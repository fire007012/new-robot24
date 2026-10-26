# Eyou_Canopen_Master 文档导航

主入口是 [`../README.md`](../README.md)。包根 README 负责快速理解当前正式入口；本目录保留详细说明、命令速查和历史归档。

## 配置教程

- [`yaml_config_guide.md`](yaml_config_guide.md)
  - `master.yaml/master.dcf/joints.yaml` 说明
- [`电机配置变更指南.md`](电机配置变更指南.md)
  - 电机配置变更时 `joints.yaml`、`master.yaml`、`master.dcf` 与控制器文件的同步规则

## API 参考

- [`../README.md`](../README.md)
  - 包职责、结构图、快速开始、接口速查
- [`api_reference.md`](api_reference.md)
  - 核心模块与 ROS 接口摘要
- [`ip_follow_joint_trajectory_executor.md`](ip_follow_joint_trajectory_executor.md)
  - IP 轨迹执行器语义、代码位置与运行约束
- [`2026-03-25_现行安全行为规范.md`](2026-03-25_%E7%8E%B0%E8%A1%8C%E5%AE%89%E5%85%A8%E8%A1%8C%E4%B8%BA%E8%A7%84%E8%8C%83.md)
  - 当前安全语义基线

## 使用教程

- [`usage.md`](usage.md)
  - 上机部署、启动步骤、联调操作
- [`command_cheatsheet.md`](command_cheatsheet.md)
  - 联调命令速查

## 归档

历史总览、发布检查、测试清单、bug 报告、实施计划和设计草案统一保留在 [`archive/`](archive/)：

- [`archive/project_overview.md`](archive/project_overview.md)
- [`archive/release_readiness.md`](archive/release_readiness.md)
- [`archive/fault_injection_checklist.md`](archive/fault_injection_checklist.md)
- [`archive/soak_test_plan.md`](archive/soak_test_plan.md)

- `archive/2026-03-21_deprecated/`
- `archive/2026-03-25_deprecated/`
- `archive/2026-04-19_deprecated/0324import.md`
- `archive/*.md` 里的更早期评审、修复记录和 roadmap

这些内容用于追溯，不再作为当前执行口径。

## 相关包

- [`../../Eyou_ROS1_Master/README.md`](../../Eyou_ROS1_Master/README.md)
  - 当前系统正式统一外观层
- [`../../flipper_control/README.md`](../../flipper_control/README.md)
  - 四摆臂上层控制入口
