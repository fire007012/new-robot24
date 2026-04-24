# Eyou_ROS1_Master 文档导航

主入口是 [`../README.md`](../README.md)。当前包内长期维护的信息尽量收口在包根 README、`launch/`、`config/`、`msg/` 和 `srv/` 中。

## 当前应先看这些

- [`../README.md`](../README.md)
  - 包职责、结构图、快速开始、统一服务与运行态接口
- [`../launch/hybrid_motor_hw.launch`](../launch/hybrid_motor_hw.launch)
  - 正式启动入口
- [`../config/controllers_jtc.yaml`](../config/controllers_jtc.yaml)
  - 默认控制器集合
- [`../config/joint_mode_mappings.yaml`](../config/joint_mode_mappings.yaml)
  - 统一 mode 映射
- [`../msg/JointRuntimeStateArray.msg`](../msg/JointRuntimeStateArray.msg)
  - 关节运行态汇总消息
- [`../scripts/hybrid_joint_action_ui.py`](../scripts/hybrid_joint_action_ui.py)
  - 手工联调 UI

## 归档

历史实施计划、修复报告和复盘已移动到 [`归档/`](%E5%BD%92%E6%A1%A3/)：

- `实施计划.md`
- `18_Eyou_ROS1_Master事务一致性问题修复报告_2026-04-09.md`
- `19_Eyou_ROS1_Master事务一致性按commit实施计划_2026-04-09.md`
- `20_Eyou_ROS1_Master修复复盘_2026-04-09.md`
- `21_Eyou_ROS1_Master剩余问题修复计划_2026-04-09.md`

这些文档只保留历史追溯价值，不再作为当前执行入口。

## 项目级相关文档

- [`../../docs/20_电机生命周期管理约定_2026-04-15.md`](../../docs/20_%E7%94%B5%E6%9C%BA%E7%94%9F%E5%91%BD%E5%91%A8%E6%9C%9F%E7%AE%A1%E7%90%86%E7%BA%A6%E5%AE%9A_2026-04-15.md)
  - 当前统一生命周期接口口径
- [`../../docs/归档/17_can_driver解耦与Eyou_ROS1_Master事务回滚修复报告_2026-04-09.md`](../../docs/%E5%BD%92%E6%A1%A3/17_can_driver%E8%A7%A3%E8%80%A6%E4%B8%8EEyou_ROS1_Master%E4%BA%8B%E5%8A%A1%E5%9B%9E%E6%BB%9A%E4%BF%AE%E5%A4%8D%E6%8A%A5%E5%91%8A_2026-04-09.md)
  - 历史修复背景
