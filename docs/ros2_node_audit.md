# ROS 节点版本检查

检查日期：2026-11-03。

范围为当前源码工作区的包清单、C++/Python 节点、测试及启动/构建入口。除 `reference/ros1` 历史参考目录和 `.deps` 外，共发现 45 个 ROS 包，42 个使用 `ament_cmake`，3 个使用 `ament_python`。主控制、MoveIt、视觉、传感器和标定主链已经使用 ROS2。

`reference/ros1/COLCON_IGNORE` 隔离了历史 ROS1 源码；这些源码保留作迁移对照，不作为当前 ROS2 节点构建。`can_driver/ros2_compat.hpp` 中的 `ROS_INFO` 等宏由本地实现提供，使用标准 C++ 时间和日志，不依赖 ROS1。

本次修复：

- `easy_handeye2` 自动运动辅助节点移除 `moveit_commander`，改用 ROS2 `GetMotionPlan` 服务、`ExecuteTrajectory` Action 和 TF2；补齐 ROS2 服务回调、响应类型、参数提供器、多线程执行器及依赖声明。
- 修复标定位姿生成、空/非有限轨迹处理、目标未选定及旧计划重复执行；规划/执行异常返回失败，执行超时请求取消。关节行程检查统一使用保守的 90° 上限，支持不同轴数。
- 修复 `easy_handeye2` 两处不兼容的日志调用。
- RealSense `rs2_listener.py` 的乱序消息分支改用 ROS2 日志。
- 两份 CAN 测试移除 ROS1 时间初始化和头文件，纳入 ament/gtest 构建。

自动运动辅助节点需要显式配置 `robot_base_frame`、`robot_effector_frame`，并可配置 `move_group`（默认 `arm`）、`move_group_namespace`（默认 `/`）。MoveIt 端必须提供对应的规划服务及轨迹执行 Action。

验证入口：

```bash
source /opt/ros/humble/setup.bash
# 构建消息与标定包；产物放到源码树外。
colcon --log-base /tmp/ros2-node-audit-log build --base-paths easy_handeye2 \
  --packages-select easy_handeye2_msgs easy_handeye2 \
  --build-base /tmp/ros2-node-audit-build --install-base /tmp/ros2-node-audit-install
source /tmp/ros2-node-audit-install/setup.bash
python3 -m unittest discover -s easy_handeye2/easy_handeye2/test -v
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 \
  python3 easy_handeye2/easy_handeye2/test/robot_service_smoke.py
```

服务 smoke 使用模拟运动对象，验证 ROS2 服务通信及失败处理，不操作硬件。单元测试检查 MoveIt 请求字段、规划失败、目标生成、空/超限/非有限轨迹、拒绝执行与超时取消。软件检查不代表真实机器人运动、相机或整车实机验收。

本次修改位于 `can_driver`、`easy_handeye2`、`realsense-ros` 子模块工作树，尚未提交或更新主仓库子模块指针。

本次结果：

- `easy_handeye2_msgs`、`easy_handeye2`、`robot_safety`、`can_driver` 构建成功。
- 标定单元测试 4 项通过；本地服务 smoke 通过。
- CAN 包 CTest 12/12 通过；依赖 `vcan0` 的用例在接口不存在时跳过，不计实机验证。
- 修改涉及的 Python 源码语法检查及三个子模块 `git diff --check` 通过。
- 未执行整车全量构建/集成回归或实机验收。
