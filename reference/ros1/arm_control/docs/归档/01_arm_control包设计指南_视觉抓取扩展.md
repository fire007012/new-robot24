# arm_control 包设计指南（视觉抓取扩展）

## 1. 文档目的

本文将“视觉检测 + 抓取规划 + 抓取执行”的方案整理为 `arm_control` 包内的设计指南，用于指导后续节点迁移、新增实现和接口收敛。

本文描述的是 `arm_control` 的目标形态，不代表当前仓库已经完成全部实现。

`arm_goal_executor_node` 的第 3 步实施路线图见：

- `02_arm_goal_executor按commit实施计划.md`

---

## 2. 包定位与边界

### 2.1 包定位

`arm_control` 是机械臂上层控制包，负责：

- 接收目标位姿、目标关节或抓取请求
- 调用 MoveIt 完成离散规划与执行
- 维护抓取过程中的任务编排、场景管理和安全约束
- 提供夹爪控制与视觉 Servo 桥接能力

### 2.2 不负责的内容

以下职责不应放入 `arm_control`：

- 电机驱动、控制器生命周期与硬件写入
- 相机驱动和底层感知驱动
- 底盘控制、摆臂控制等跨子系统总控逻辑

当前正式执行入口仍为 `Eyou_ROS1_Master/hybrid_motor_hw_node`。`arm_control` 只负责机械臂上层能力，不重复实现底层执行链。

### 2.3 设计原则

- 感知与控制解耦：视觉侧可来自 `vision_pkg` 或其他感知包，`arm_control` 只消费标准化位姿输入
- 任务与执行分层：抓取任务编排不直接耦合到底层电机
- 离散规划与实时修正并存：大范围位姿调整走 MoveIt，局部微调可接 MoveIt Servo
- 统一坐标契约：所有输入输出都必须明确 `frame_id`

---

## 3. 目标能力

面向视觉抓取扩展时，`arm_control` 推荐覆盖以下能力：

1. 接收视觉检测结果并转换到机械臂规划坐标系
2. 基于规划场景生成安全抓取轨迹
3. 按阶段执行预抓取、接近、闭合、抬升、放置等动作
4. 在需要时通过 Servo 输入进行末端小步修正
5. 维护碰撞物体、抓取对象附着状态和任务执行状态

---

## 4. 推荐总体架构

```text
┌─────────────────────────────────────────────────────────┐
│                    arm_control 包内架构                 │
├─────────────────────────────────────────────────────────┤
│ 应用/任务层                                              │
│   grasp_task_node                                        │
│   - 任务状态机                                            │
│   - 抓取/放置编排                                         │
├─────────────────────────────────────────────────────────┤
│ 通用控制层                                                │
│   arm_goal_executor_node                                 │
│   gripper_cmd_node                                       │
│   servo_twist_frame_bridge_node                          │
├─────────────────────────────────────────────────────────┤
│ 规划与场景层                                              │
│   move_group / planning_scene / MoveIt Task Constructor  │
├─────────────────────────────────────────────────────────┤
│ 外部输入                                                  │
│   视觉感知包 -> PoseStamped / TF                         │
├─────────────────────────────────────────────────────────┤
│ 执行层（包外）                                            │
│   hybrid_motor_hw_node / controller_manager             │
└─────────────────────────────────────────────────────────┘
```

### 4.1 推荐数据流

```text
感知包 -> 目标位姿 -> grasp_task_node
      -> TF 转换 -> 规划场景更新
      -> MoveIt / MTC 规划
      -> gripper_cmd_node + 轨迹执行
      -> hybrid_motor_hw_node
```

### 4.2 分层建议

- `grasp_task_node`：只负责视觉抓取任务，不扩展成整机总控
- `arm_goal_executor_node`：提供通用目标位姿/关节执行能力，供非抓取场景复用
- `gripper_cmd_node`：继续作为独立夹爪控制节点，避免抓取任务节点直接拼装底层轨迹消息
- `servo_twist_frame_bridge_node`：只处理坐标系转换，不承担任务逻辑

---

## 5. 节点设计

### 5.1 `grasp_task_node`

职责：

- 接收目标物体位姿或抓取触发请求
- 将目标位姿从视觉坐标系转换到规划坐标系
- 构建抓取任务阶段
- 更新 `planning_scene`
- 触发夹爪控制、执行规划并跟踪状态

建议输入：

- Topic：`/arm_control/target_object_pose` (`geometry_msgs/PoseStamped`)
- TF：`camera_frame -> target_object`
- Service/Action：`/arm_control/execute_grasp`

建议输出：

- Topic：`/arm_control/task_state`
- Topic：`/arm_control/debug/grasp_pose`
- Topic：`/move_group/display_planned_path`

建议内部能力：

- `tf2_ros.Buffer`
- `moveit_commander`
- `PlanningSceneInterface`
- 场景物体附着/分离
- 失败重试与超时处理

### 5.2 `arm_goal_executor_node`

职责：

- 接收目标位姿或目标关节
- 调用 MoveIt 做单次规划与执行
- 作为包内通用离散运动入口

建议保留为独立节点，不把视觉抓取逻辑塞进该节点内部。

### 5.3 `gripper_cmd_node`

职责：

- 接收“打开/关闭”或“目标开度”命令
- 输出夹爪控制轨迹
- 向抓取任务提供独立、可测试的夹爪接口

该节点可沿用迁移期已有实现，只需要把命名空间和参数收敛到 `arm_control`。

### 5.4 `servo_twist_frame_bridge_node`

职责：

- 将视觉链路产生的 `TwistStamped` 从 optical frame 转换到控制 frame
- 输出到 MoveIt Servo 输入接口

适用场景：

- 相机跟随末端安装，存在 optical frame 与控制 frame 轴定义差异
- 抓取前需要做小范围视觉闭环微调

---

## 6. 抓取任务流程

推荐将抓取拆成可独立验证的阶段：

```text
CurrentState
  -> OpenGripper
  -> MoveToPregrasp
  -> Approach
  -> CloseGripper
  -> Lift
  -> Transfer
  -> Place
  -> Release
  -> Retreat
```

### 6.1 阶段含义

| 阶段 | 目标 |
|------|------|
| `OpenGripper` | 打开夹爪，进入可抓取状态 |
| `MoveToPregrasp` | 移动到预抓取位姿，避免直接贴近目标 |
| `Approach` | 沿抓取方向做短距离接近 |
| `CloseGripper` | 闭合夹爪并建立附着关系 |
| `Lift` | 抬升目标，脱离桌面或障碍物 |
| `Transfer` | 转运到放置区域 |
| `Place` | 接近放置位姿 |
| `Release` | 打开夹爪释放物体 |
| `Retreat` | 末端撤离，避免碰撞 |

### 6.2 实现建议

- 优先使用 MTC 表达阶段化任务与碰撞约束
- 若目标环境中 MTC 绑定受限，仍应保留相同阶段语义，不要退化成无状态的大脚本
- `Approach`、`Lift`、`Retreat` 推荐使用笛卡尔段表达，便于约束方向与安全距离

---

## 7. 接口约定

### 7.1 Topic 约定

| Topic | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/arm_control/target_object_pose` | `geometry_msgs/PoseStamped` | 感知 -> 抓取任务 | 目标物体位姿 |
| `/arm_control/task_state` | `std_msgs/String` | 抓取任务 -> 外部 | 当前任务状态 |
| `/arm_control/gripper_open` | `std_msgs/Bool` | 任务/上层 -> 夹爪 | 一键开合 |
| `/arm_control/gripper_position` | `std_msgs/Float64` | 任务/上层 -> 夹爪 | 目标开度 |
| `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | 桥接 -> Servo | 实时微调输入 |

### 7.2 TF 约定

推荐最少包含以下坐标关系：

```text
world
  └─ base_link
      └─ tool0
      └─ camera_link
          └─ camera_optical_frame
              └─ target_object
```

要求：

- 视觉输入必须带时间戳和 `frame_id`
- 抓取前必须能稳定查询 `base_link <- target_object`
- 如果相机安装在末端，必须明确 optical frame 与控制 frame 的转换关系

### 7.3 执行链约定

- 离散规划执行走 MoveIt 控制器接口
- 实时微调走 MoveIt Servo
- Servo 的下游执行应接入当前正式执行链，而不是新增旁路电机写入逻辑

---

## 8. 包内目录建议

推荐在 `arm_control` 内逐步收敛为以下结构：

```text
arm_control/
├── docs/
│   ├── README.md
│   └── 01_arm_control包设计指南_视觉抓取扩展.md
├── launch/
│   ├── arm_goal_executor.launch
│   ├── grasp_task.launch
│   └── servo_twist_frame_bridge.launch
├── config/
│   ├── grasp_params.yaml
│   ├── mtc_config.yaml
│   └── servo_bridge.yaml
├── scripts/
│   ├── arm_goal_executor_node.py
│   ├── grasp_task_node.py
│   ├── gripper_cmd_node.py
│   └── servo_twist_frame_bridge_node.py
└── src/arm_control/
    ├── __init__.py
    ├── pose_utils.py
    ├── scene_manager.py
    └── task_builder.py
```

说明：

- `scripts/` 放节点入口
- `src/arm_control/` 放可复用逻辑
- `config/` 只放包内参数，不放硬件驱动配置
- 顶层整机启动仍应由 `robot_bringup` 负责编排

---

## 9. 参数组织建议

### 9.1 `grasp_params.yaml`

```yaml
grasp:
  planning_group: "manipulator"
  gripper_group: "gripper"
  end_effector_link: "tool0"

  approach:
    direction: [0.0, 0.0, -1.0]
    min_distance: 0.05
    max_distance: 0.15

  lift:
    direction: [0.0, 0.0, 1.0]
    min_distance: 0.08
    max_distance: 0.12

  gripper:
    open_position: 0.08
    close_position: 0.02

  planning:
    velocity_scaling: 0.3
    acceleration_scaling: 0.3
    planning_time: 5.0
    max_attempts: 10
```

### 9.2 `mtc_config.yaml`

```yaml
mtc:
  task_name: "pick_and_place"

  stage_timeouts:
    approach: 10.0
    lift: 8.0
    place: 10.0

  cartesian_path:
    step_size: 0.005
    jump_threshold: 0.0

  collision_checking:
    safety_distance: 0.01
```

---

## 10. Launch 组织建议

### 10.1 `arm_control` 包内 launch

`arm_control` 内只提供与自身节点相关的子 launch，例如：

- `grasp_task.launch`
- `arm_goal_executor.launch`
- `servo_twist_frame_bridge.launch`

### 10.2 顶层启动归属

整机启动应由 `robot_bringup` 统一编排，而不是在 `arm_control` 内部直接承担全部 bringup 责任。

建议的集成方式：

```text
robot_bringup
  -> MoveIt / robot_description / controller_manager
  -> 感知包
  -> arm_control/grasp_task.launch
```

这样可以保持：

- `arm_control` 只关注机械臂控制能力
- 感知、执行、整机编排相互解耦
- 后续替换视觉方案时不需要改动抓取任务主逻辑

---

## 11. 测试建议

推荐按“先单模块、后全链路”的方式验证：

1. TF 变换验证
2. 单次目标位姿规划验证
3. 夹爪开合验证
4. 场景障碍物碰撞验证
5. 完整抓取流程验证

建议关注的验收指标：

- 目标位姿转换稳定，无坐标系跳变
- 空载规划成功率高且可重复
- 抓取前后场景物体状态一致
- 接近与抬升阶段无明显碰撞风险
- 抓取失败时能回退到可恢复状态

---

## 12. 主要风险与约束

| 风险项 | 影响 | 对策 |
|-------|------|------|
| 相机外参与 TF 不稳定 | 抓取位姿偏差大 | 固化标定流程，启动时做 TF 健康检查 |
| 抓取目标姿态抖动 | 规划失败或接近不稳 | 多帧滤波、时间窗平均、有效性判定 |
| 任务节点职责膨胀 | 变成新的“大总控节点” | 保持只负责机械臂抓取，不吞并其他子系统 |
| MTC 或规划链超时 | 任务卡死 | 增加阶段超时、失败重试和状态上报 |
| Servo 输入坐标定义不一致 | 末端运动方向错误 | 强制校验 frame，并保留桥接节点独立测试 |

---

## 13. 推荐实施顺序

1. 先迁移 `gripper_cmd_node` 和 `servo_twist_frame_bridge_node`
2. 新增 `arm_goal_executor_node`，打通通用 MoveIt 执行链
3. 在此基础上新增 `grasp_task_node`
4. 最后补充场景管理、抓取重试和可视化调试接口

这样可以保证：

- 每一步都有独立可测的成果
- 迁移路径与当前控制包拆分方案一致
- 不会把视觉抓取需求直接压成一次性大改
