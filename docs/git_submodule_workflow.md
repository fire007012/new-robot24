# ROS2 子模块工作流

当前活动子模块为 `can_driver`、`ros2_canopen`、`realsense-ros`、`OrbbecSDK_ROS2`、`aruco_ros`、`easy_handeye2` 共 6 个，来源与跟踪分支以根 `.gitmodules` 为准。父仓库 gitlink 锁定具体提交，分支名不意味着自动升级。

旧 `Eyou_Canopen_Master` 已解除 gitlink、`.gitmodules`、本地配置及工作树 `.git` 指针，源码在 `reference/ros1/`。它不再是子模块，不需要 init/update。不使用 ROS1 checkout 作为交付回退。

## 获取与检查

从仓库根目录执行；私有 SSH 来源需要相应 GitHub 权限：

```bash
git submodule update --init --recursive
git submodule status --recursive
git status --short
```

有本地修改时先查看各子模块 `git status`/`git diff` 并保存，不能用强制 checkout/update 丢弃补丁。此工作区两个驱动子模块仍有配套安全与退出修复，父仓库旧 gitlink 无法代表未提交改动。

## 修改与匹配发布

在对应子模块的开发分支提交并推送补丁后，回父仓库 `git add <子模块路径>` 更新 gitlink；父仓库同时提交配置、接口、依赖版本和文档。正式发布前用同一组源码重建并回归，再记录父仓库与子模块匹配版本。设备参数按现场副本管理，恢复 ROS2 软件版本不自动恢复设备参数。

## CI

`.github/workflows/ci.yml` 使用 Humble/Ubuntu 22.04，递归检出子模块并执行核心构建与回归，不再使用 Noetic/Catkin 或临时克隆替换驱动。私有子模块可通过仓库 secret `SUBMODULE_TOKEN` 提供读取权限；CI 会将 GitHub SSH 来源映射为 HTTPS，默认 GITHUB_TOKEN 必须拥有相应访问权限。CI 核心检查不代替需 SDK 和显示服务的全量 `tools/validate_ros2.sh ... full`。

当前相机依赖版本和实际设备边界见 [全量实施记录](34_ROS2全量软件集成与验收.md)。
