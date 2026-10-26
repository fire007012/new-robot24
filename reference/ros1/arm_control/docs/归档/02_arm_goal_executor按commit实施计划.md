# `arm_goal_executor_node` 第 3 步实施计划（含按 Commit 路线图）

## 1. Summary

- v1 在 `arm_control` 内新增 `arm_goal_executor_node`，提供 `/arm_control/execute_goal` 自定义 Action，支持 `全量关节目标 / named target / PoseStamped` 三类离散目标。
- 节点基于 `moveit_commander`，固定使用 `arm` 规划组、`base_link` 参考系、MoveIt 当前默认 tip link；不改 `robot_bringup`，不替换现有 `keyboard_moveit_server`。
- readiness 只检查并失败，不自动调用 `/hybrid_motor_hw_node/init`、`/hybrid_motor_hw_node/enable`、`/hybrid_motor_hw_node/resume`。
- 文档落地采用“双文档”方式：保留现有设计指南做架构说明，新增一份按 Commit 实施计划写进 `arm_control/docs`。

---

## 2. Public API / Interfaces

- 新增 `arm_control/action/ExecuteArmGoal.action`。
- Goal 固定包含：`target_type`、`joint_names`、`joint_positions`、`named_target`、`pose_target`。
- Result 固定包含：`success`、`error_code`、`message`。
- Feedback 固定包含：`phase`、`message`。

### 2.1 `target_type`

- `TARGET_JOINTS`
- `TARGET_NAMED`
- `TARGET_POSE`

### 2.2 `error_code`

- `ERROR_NONE`
- `ERROR_INVALID_GOAL`
- `ERROR_NOT_READY`
- `ERROR_TF_FAILED`
- `ERROR_PLANNING_FAILED`
- `ERROR_EXECUTION_FAILED`
- `ERROR_PREEMPTED`

### 2.3 输入约束

- `JOINTS` 目标必须提供完整 6 轴 `joint_names + joint_positions`，名称集合必须与 `arm` group 完全一致；顺序允许打乱，节点内部按名字重排。
- `NAMED` 目标只接受当前 MoveIt/SRDF 中真实存在的 named target；仓库现状下至少有 `home`、`zero`、`ready`。
- `POSE` 目标只接受 `geometry_msgs/PoseStamped`；节点内部做 TF 转换，`frame_id` 为空直接判错。
- Action server 固定发布在 `/arm_control/execute_goal`。

---

## 3. Commit 路线图

状态维护约定：

- Commit 1 到 Commit 6 初始状态统一记为 `待执行`
- 对应 commit 落地并完成验证后，将该条目的状态更新为 `已完成`
- 如实现过程中发生拆分或回退，保留原条目并补充备注，不删除既有路线记录

### Commit 0

建议提交信息：

- `docs(arm_control): add arm goal executor commit plan`

目标：

- 新增 `arm_control/docs/02_arm_goal_executor按commit实施计划.md`
- 更新 `arm_control/docs/README.md`
- 更新 `arm_control/README.md`
- 在 `arm_control/docs/01_arm_control包设计指南_视觉抓取扩展.md` 增加一条“实施计划见 02 文档”的交叉引用

### Commit 1

状态：`已完成`

建议提交信息：

- `feat(arm_control): add action and catkin plumbing for arm goal executor`

目标：

- 新增 `ExecuteArmGoal.action`
- 补 `message_generation`
- 补 `message_runtime`
- 补 `actionlib`
- 补 `actionlib_msgs`
- 补 `controller_manager_msgs`
- 补 `tf2_geometry_msgs`
- 让 `arm_control` 能生成 Action 消息并安装新节点脚本

### Commit 2

状态：`已完成`

建议提交信息：

- `feat(arm_control): add arm goal executor node skeleton and readiness checks`

目标：

- 新增 `scripts/arm_goal_executor_node.py`
- 新增 `config/goal_executor.yaml`
- 新增 `launch/arm_goal_executor.launch`
- 节点启动时初始化 `MoveGroupCommander(group=arm)`
- 对 goal 执行前固定检查：
  - `/controller_manager/list_controllers`
  - `joint_state_controller` 运行态
  - `arm_position_controller` 运行态
  - `/arm_position_controller/follow_joint_trajectory` 可达

### Commit 3

状态：`已完成`

建议提交信息：

- `feat(arm_control): support full joint and named targets`

目标：

- 完成自定义 Action server
- 完成 joint goal 全量校验
- 完成 named target 查询与执行
- 完成错误码映射
- 完成 result 与 feedback 回传
- 保持“新 goal 抢占旧 goal”

### Commit 4

状态：`已完成`

建议提交信息：

- `feat(arm_control): support pose goals and TF transform`

目标：

- 补 `PoseStamped` 目标处理
- 补 TF 转换
- 补参考系统一
- 补 pose planning / execution
- 固化 TF 失败、规划失败、执行失败的稳定回报

### Commit 5

状态：`已完成`

建议提交信息：

- `test(arm_control): add smoke clients and executor regression coverage`

目标：

- 新增最小测试客户端脚本
- 覆盖 named、joint、pose 三类 happy path
- 覆盖 invalid goal
- 覆盖 TF failure
- 覆盖 controller not ready
- 覆盖 new goal preempts old goal

### Commit 6

状态：`已完成`

建议提交信息：

- `docs(arm_control): finalize executor usage and acceptance notes`

目标：

- 把最终 Action 合同、启动方式、默认参数和验收命令同步回本实施计划文档
- 更新 `arm_control/README.md`
- 形成第 3 步交付入口

---

## 4. Implementation Notes

- 节点继续使用 `moveit_commander`，直接复用现有口径：
  - `group=arm`
  - `reference_frame=base_link`
  - `velocity_scaling=0.2`
  - `acceleration_scaling=0.2`
  - `planning_time=5.0`
  - `num_planning_attempts=1`
- v1 不新增 `setup.py`，也不在这一步引入 `src/arm_control/` Python 包结构；逻辑先收敛在 `scripts/arm_goal_executor_node.py`，避免把第 3 步扩大成 Python packaging 重构。
- preempt 固定采用 `SimpleActionServer + group.stop()`；旧 goal 统一返回 `ERROR_PREEMPTED`，不做内部排队。
- readiness 不读取底层 lifecycle mode，因为当前仓库没有稳定的只读 lifecycle API；v1 以 controller 和 action server readiness 作为“可执行”判据。
- 文档写法固定遵循仓库现有“按 Commit 实施计划”风格，使用单独文档而不是把路线图直接塞进设计指南正文。

---

## 5. Test Plan

- 包级检查：Action 生成成功，Python 语法通过，isolated catkin build 通过。
- Fake MoveIt smoke：在 `car_moveit_config` demo/fake 环境下，`named target=ready`、一组完整 joint goal、一个 `base_link` pose goal 都能成功。
- 失败场景：
  - 缺关节、重复关节、长度不一致返回 `ERROR_INVALID_GOAL`
  - 不存在的 named target 返回 `ERROR_INVALID_GOAL`
  - 空 `frame_id` 或 TF 不存在返回 `ERROR_TF_FAILED`
  - controller 未 running 或 action server 不可达返回 `ERROR_NOT_READY`
- 抢占场景：
  - 长 goal 执行中发送新 goal
  - 第一个 goal 被 `PREEMPTED`
  - 第二个 goal 正常规划执行
- 主链 smoke：
  - 在 `hybrid_motor_hw.launch + move_group.launch + arm_goal_executor.launch` 组合下
  - 至少能完成一个 `named target=ready`
  - 当 controller 不 ready 时快速失败而不是卡住

---

## 6. 使用方式占位

以下命令为第 3 步当前已落地的使用入口：

- 启动节点：`roslaunch arm_control arm_goal_executor.launch`
- 发送 named target：`rosrun arm_control send_execute_arm_goal.py named --name ready`
- 发送 joint goal：`rosrun arm_control send_execute_arm_goal.py joints --joint shoulder_yaw_joint=0.0 --joint shoulder_pitch_joint=-0.2 --joint elbow_pitch_joint=1.0 --joint wrist_pitch_joint=0.0 --joint wrist_roll_joint=0.0 --joint wrist_yaw_joint=0.0`
- 发送 pose goal：`rosrun arm_control send_execute_arm_goal.py pose --frame base_link --x 0.30 --y 0.00 --z 0.25 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0`
- 查看 Action 状态：`rostopic echo /arm_control/execute_goal/status`

---

## 7. Assumptions

- 当前正式机械臂规划组是 `arm`，不是 `manipulator`。
- 当前 MoveIt/SRDF 中的 named targets 以运行时查询为准，仓库现状下已存在 `home`、`zero`、`ready`。
- 第 3 步范围只覆盖 `arm_control` 包内节点、launch、测试脚本和文档，不触碰 `robot_bringup` 集成入口。
- 文档落地采用“新增独立实施计划文档”方案：
  - `01_设计指南` 负责架构
  - `02_按commit实施计划` 负责执行路线
