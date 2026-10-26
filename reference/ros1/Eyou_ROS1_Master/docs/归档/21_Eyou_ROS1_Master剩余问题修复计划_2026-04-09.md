# Eyou_ROS1_Master 剩余问题修复计划（2026-04-09）

## 1. 目的

本计划用于处理当前 `Eyou_ROS1_Master` 在“事务一致性主链修复完成”之后，
仍然存在的剩余问题。

这些问题不再是“主链没有 rollback”，而是更偏系统入口、契约表达和集成层验证：

1. `hybrid_motor_hw.launch` 仍不是完整统一入口
2. auto startup 已经收口，但 launch 层没有把参数正式暴露出来
3. `already initialized` 语义仍依赖消息字符串前缀
4. 缺少 launch/节点级自动化验证

## 2. 总体策略

后续修复不再继续重写 runtime 或 facade 主逻辑，而是进入“收口阶段”：

1. 收口 launch
2. 收口显式语义
3. 补集成测试
4. 最后做文档与入口说明

## 3. Commit 1：补全 hybrid launch 的正式入口能力

### 目的

让 `hybrid_motor_hw.launch` 从“节点启动脚本”升级为“包级统一入口”。

### 需要处理的问题

当前 launch：

- 可以加载 `controllers_file`
- 但不会主动启动 controller spawner
- 也没有把 `auto_init / auto_enable / auto_release` 作为正式参数暴露

### 修改文件

- `Eyou_ROS1_Master/launch/hybrid_motor_hw.launch`

### 修改策略

1. 增加参数：
   - `auto_init`
   - `auto_enable`
   - `auto_release`
2. 将这些参数透传给 `hybrid_motor_hw_node`
3. 增加可选 controller spawner：
   - 至少支持 `joint_state_controller`
   - 允许用户通过参数指定需要启动的 controllers

### 验收

- 仅通过 launch 参数即可完整控制 auto startup 行为
- 启动后 controller_manager 中的目标 controller 能实际进入 running

### 建议提交信息

- `feat(Eyou_ROS1_Master): expose auto startup and controller spawners in launch`

## 4. Commit 2：去掉 “already ” 前缀的隐式契约

### 目的

消除：

- `HybridServiceGateway` 通过消息字符串前缀判断流程分支

这种脆弱设计。

### 修改文件

- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_operational_coordinator.hpp`
- `Eyou_ROS1_Master/src/hybrid_operational_coordinator.cpp`
- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_service_gateway.hpp`
- `Eyou_ROS1_Master/src/hybrid_service_gateway.cpp`

### 修改策略

1. 给 `HybridOperationalCoordinator::Result` 增加显式字段，例如：
   - `bool already{false};`
2. `RequestInit()` 在幂等命中时：
   - `ok=true`
   - `already=true`
   - `message="already initialized"`
3. `HybridServiceGateway::RunInitSequence()` 直接读 `already` 字段，
   不再解析字符串

### 验收

- 代码中不再存在用 `"already "` 前缀决定流程分支的逻辑
- 现有 hook 测试保持通过

### 建议提交信息

- `refactor(Eyou_ROS1_Master): make hybrid init idempotency explicit`

## 5. Commit 3：补 launch/节点级自动启动测试

### 目的

当前已有：

- `HybridOperationalCoordinator` 单测
- `HybridServiceGateway` 单测

但还缺：

- launch / node 级“真正从参数进入 auto startup”的测试

### 修改文件

- 新增：
  - `Eyou_ROS1_Master/tests/test_hybrid_auto_startup.cpp`
  - 或 `rostest` 配套 `.test`
- 修改：
  - `Eyou_ROS1_Master/CMakeLists.txt`

### 测试点

1. `auto_init=false` 时不自动推进
2. `auto_init=true` 时会走 unified init 路径
3. `auto_enable=true` 时两边都进入 `Armed`
4. `auto_release=true` 时两边都进入 `Running`
5. 非法组合：
   - `auto_enable=true && auto_init=false`
   - `auto_release=true && auto_enable=false`
   应明确失败

### 验收

- 不通过 service 手动调用，也能验证 auto startup 逻辑
- 测试覆盖的是 hybrid facade 路径，而不是 CANopen 原生路径

### 建议提交信息

- `test(Eyou_ROS1_Master): cover unified auto startup path`

## 6. Commit 4：补 facade 最终入口说明文档

### 目的

当 launch 和自动启动能力都整理好后，需要把“用户应该怎么用这个包”说清楚。

### 修改文件

- `Eyou_ROS1_Master/docs/README.md`
- 可新增：
  - `Eyou_ROS1_Master/docs/22_统一入口使用说明_2026-04-09.md`

### 文档内容建议

1. 正式入口 launch 是什么
2. 哪些 service 对外暴露
3. auto startup 参数怎么配
4. 哪些原始 backend 接口在 facade 模式下不建议直接使用
5. 故障时预期会收敛到什么状态

### 建议提交信息

- `docs(Eyou_ROS1_Master): document unified facade usage and startup semantics`

## 7. 推荐顺序

1. Commit 1
2. Commit 2
3. Commit 3
4. Commit 4

## 8. 统一验收模板

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='can_driver;Eyou_Canopen_Master;Eyou_ROS1_Master' \
  --force-cmake \
  --pkg can_driver Eyou_Canopen_Master Eyou_ROS1_Master

/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_operational_coordinator
/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_service_gateway
```

若新增 auto startup 测试，再追加该测试目标。

## 9. 结论

当前 `Eyou_ROS1_Master` 的“事务一致性主链”已经补齐。  
剩余工作主要是把它从“逻辑正确”推进到“入口完整、契约清晰、测试覆盖到系统边界”。

这份计划就是下一阶段的收尾路线图。
