# 按 Commit 执行的修复指南（排除 MoveIt 与测试 UI）

本文将修复任务拆成可独立提交、可回滚、可验证的最小 commit。

范围约束：
- 暂不处理：`robot_moveit_config`
- 暂不处理：`can_driver_ui`（测试包）
- 聚焦：`arm_traj_splitter`、`yiyou_canopen`、`robot_bringup`、`car_total`、文档一致性

---

## Commit 0：建立基线与决策记录

### 目的
冻结当前行为，避免后续“改了但不知道影响了什么”。

### 修改内容
- 新增本指南（当前文件）
- 在 `docs/08_阶段变更记录.md` 追加一条“本轮修复范围说明”（可选）

### 建议提交信息
- `docs: add commit-based remediation plan excluding moveit/ui`

### 验收
- 文档可读，范围明确
- 团队对“先不管 MoveIt/UI”达成一致

---

## Commit 1：修复 arm_traj_splitter 的容差映射正确性（P0）

### 为什么优先
这是功能正确性问题，不是纯优化。

### 现象
当前 `splitTolerances()` 只按 `JointTolerance.name` 过滤：
- 当上游容差 `name` 为空时，容差会被全部丢弃
- `index_maps` 参数未使用，行为与 `splitTrajectory` 的映射逻辑不一致

### 修改文件
- `arm_traj_splitter/src/SplitterNode.cpp`

### 修改策略
1. 保留“按名称映射”路径（`name` 非空时）
2. 增加“按索引映射”回退路径：
   - 当 `name` 为空且容差数组长度与 `full_goal.trajectory.joint_names` 对齐时，按 `index_maps` 拆分
3. 对无法安全映射的情况打印 `ROS_WARN`，并显式记录降级行为

### 验收命令
```bash
catkin_make --pkg arm_traj_splitter
```

### 功能验收
- 发送 `name` 为空的容差 goal，后端仍能收到拆分后的容差
- 发送 `name` 非空的容差 goal，结果与原逻辑一致

### 建议提交信息
- `fix(arm_traj_splitter): make tolerance split robust for unnamed entries`

---

## Commit 2：优化 arm_traj_splitter 的索引构建复杂度（P1）

### 目的
把 `splitGoal()` 里关节索引构建从 O(n²) 改为 O(n)。

### 现象
`splitGoal()` 中存在双层循环字符串匹配，且中间 `lookup` 变量构建后未有效利用。

### 修改文件
- `arm_traj_splitter/src/SplitterNode.cpp`

### 修改策略
1. 先构建 `full_joint_name -> full_index` 的哈希表
2. 单次遍历 `part_trajectories[b].joint_names` 直接查索引
3. 删除无用变量与重复搜索路径

### 验收命令
```bash
catkin_make --pkg arm_traj_splitter
```

### 性能验收
- 行为不变（同样输入输出）
- 代码路径更短、复杂度更低、可读性更好

### 建议提交信息
- `refactor(arm_traj_splitter): replace nested joint index search with hash lookup`

---

## Commit 3：启用 robot_bringup 的启动后自动校验（P1）

### 目的
把已有的 `wait_for_graph.sh` 从“手动工具”变成“可选自动校验”。

### 修改文件
- `robot_bringup/launch/acceptance_smoke.launch`

### 修改策略
1. 新增参数：`enable_graph_check`（默认建议 `true`）
2. 条件启动 `wait_for_graph.sh`：
   - `if="$(arg enable_graph_check)"`
   - `args="$(arg check_timeout_sec)"`
   - `required="true"`（验收场景建议开启）

### 验收命令
```bash
roslaunch robot_bringup acceptance_smoke.launch enable_graph_check:=true check_timeout_sec:=30
```

### 结果判定
- 成功：脚本输出 PASS 并退出 0
- 失败：明确打印缺失话题列表并非 0 退出

### 建议提交信息
- `feat(robot_bringup): enable optional graph readiness check in smoke launch`

---

## Commit 4：提高 yiyou_canopen 的运行稳定性（P1）

### 回答你的问题
`yiyou_canopen` 虽然“直接走 ros_canopen”，但**你仍需在你的 launch/配置层做进程级与运维级兜底**，否则现场异常（节点退出、总线瞬断）时恢复能力不足。

### 注意
不要盲加未经验证的 ros_canopen 参数键。先做“确定有效”的增强：

### 修改文件
- `yiyou_canopen/launch/yiyou_canopen.launch`
- `yiyou_canopen/package.xml`（补运行依赖）

### 修改策略（安全且确定有效）
1. 给 `canopen_motor_node` 增加：
   - `respawn="true"`
   - `respawn_delay="2"`
   - 是否 `required="true"` 取决于你希望“故障即全系统退出”还是“自动拉起继续跑”
2. 给 `controller_spawner` 增加合理超时参数（避免早启动误判）
3. 在 `package.xml` 明确声明 launch 用到的运行依赖（如 `controller_manager`）

### 可选（需先查 ros_canopen 文档后再做）
- 心跳/超时/NMT 等更细粒度策略键

### 验收
- 手动 kill `canopen_motor_node` 后，2 秒内自动拉起
- `canopen_arm_controller`、`flipper_controller` 可恢复到 running

### 建议提交信息
- `feat(yiyou_canopen): add respawn-based resilience and runtime deps`

---

## Commit 5：为 yiyou_canopen 转换因子补可追溯文档（P1）

### 目的
把“硬编码数字”变成“可维护参数”。

### 修改文件
- `yiyou_canopen/config/motor.yaml`
- `yiyou_canopen/docs/` 下新增说明文档（例如 `12_canopen参数说明.md`）

### 修改策略
1. 在 `motor.yaml` 就地增加注释，说明：
   - 该常数的物理含义（编码器分辨率/减速比组合）
   - 单位（counts per revolution）
   - 变更方法（换电机/减速器时如何重算）
2. 文档里给出计算公式与样例

### 验收
- 新成员不查聊天记录也能理解并修改该参数
- 参数来源可追溯

### 建议提交信息
- `docs(yiyou_canopen): document conversion factor source and recalculation method`

---

## Commit 6：修复 car_total 的最小规范问题（P2）

### 目的
不改几何/运动学前提下，先清理低风险规范问题。

### 修改文件
- `car_total/urdf/car_total.urdf`
- `car_total/launch/display.launch`（若 `urdf.rviz` 缺失）

### 修改策略
1. 若 `urdf.rviz` 不存在：
   - 补文件，或移除 `-d $(find car_total)/urdf.rviz` 参数
2. 统一修正文档性错误（如明显拼写问题），但避免改变实际关节名（除非你已完成全链路映射统一）
3. 材料空名、fixed joint 冗余 axis 等问题按“低风险清理”处理

### 验收
```bash
roslaunch car_total display.launch
```
- 启动不因缺文件报错
- 模型显示正常

### 建议提交信息
- `chore(car_total): clean urdf conventions and fix rviz launch usability`

---

## Commit 7：收敛文档状态，消除“计划与现状冲突”（P2）

### 目的
当前 `07/08/09/10` 对阶段完成度描述不一致，容易误导后续开发。

### 修改文件
- `docs/08_细化开发步骤与检查清单.md`
- `docs/09_代码评估报告.md`
- `docs/08_阶段变更记录.md`

### 修改策略
1. 把“已完成/未完成”统一到同一口径（以代码现状为准）
2. 明确“本轮排除项”：`robot_moveit_config`、`can_driver_ui`
3. 给出下一轮唯一入口文档（建议本文件）

### 建议提交信息
- `docs: reconcile progress status across planning and review documents`

---

## 推荐执行顺序
1. Commit 1（容差正确性）
2. Commit 2（性能优化）
3. Commit 3（启动自动验收）
4. Commit 4（canopen 进程韧性）
5. Commit 5（转换因子文档化）
6. Commit 6（URDF 低风险规范修复）
7. Commit 7（文档状态收敛）

---

## 每个 Commit 的统一验收模板

```bash
# 1) 编译最小影响范围
catkin_make --pkg <changed_pkg>

# 2) 启动相关 launch（如有）
roslaunch <pkg> <file>.launch

# 3) 关键接口检查
rostopic list | grep -E "joint_states|follow_joint_trajectory"
rosservice list | grep controller_manager
```

---

## 明确不在本轮处理
- `robot_moveit_config` 完整配置生成（你已明确先不管）
- `can_driver_ui` 健壮性与输入容错（测试包，先不投入）
