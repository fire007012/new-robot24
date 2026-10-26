# can_driver 解耦与 Eyou_ROS1_Master 事务回滚修复报告（2026-04-09）

## 1. 背景

本轮修复围绕两个直接相关的问题展开：

1. `can_driver` 作为独立节点时可以工作，但结构边界不够清晰，嵌入 `Eyou_ROS1_Master` 时会把自身的 ROS service/topic/timer 一并带出，导致统一外观层难以收口。
2. `Eyou_ROS1_Master` 在组合 `can_driver` 与 `Eyou_Canopen_Master` 时，生命周期请求采用“先后端 A，再后端 B”的串行策略；后半段失败时没有补偿回滚，容易留下“半成功”的中间态。

本轮工作的目标不是继续堆补丁，而是：

- 把 `can_driver` 收敛成更适合嵌入的结构
- 让 `Eyou_ROS1_Master` 在生命周期失败时具备“全局失败并回滚/收敛到安全态”的能力
- 补齐最关键的自动化测试，便于后续代码审查

## 2. 关键结论

### 2.1 `can_driver` 已从“大一统节点类”演进为可嵌入结构

本轮之后，`can_driver` 的主要职责边界如下：

- `CanDriverRuntime`
  - 持有 `DeviceManager`、`MotorActionExecutor`、`LifecycleDriverOps`、`CommandGate`、`OperationalCoordinator`
  - 负责 lifecycle/runtime 编排主链
- `DriverRosEndpoints`
  - 持有 `motor_command`、`set_zero_limit`、direct command topics、`motor_states`、`lifecycle_state` 与对应 timer
- `CanDriverHW`
  - 主要保留 `RobotHW` 适配和数据面职责

这意味着 `can_driver` 已经不再强依赖“单节点独占运行”的假设。

### 2.2 `can_driver` 新增了嵌入模式

`CanDriverHW` 现在支持通过 `InitOptions` 显式关闭 ROS endpoints。

效果是：

- 默认 `can_driver_node` 行为保持不变
- `Eyou_ROS1_Master` 可以只复用 `RobotHW + runtime`，不自动对外暴露 `can_driver` 自带的 service/topic

### 2.3 `Eyou_ROS1_Master` 的 hybrid 生命周期请求已具备事务补偿

`HybridOperationalCoordinator` 现在不再是“后半段失败直接返回”，而是：

- 尝试将先前已成功的一侧恢复到调用前状态
- 若无法精确恢复，则进一步执行 fail-safe shutdown，把系统收敛到 `Configured` 等安全态

这使得 facade 不再轻易留下“can_driver 已 Armed、canopen 却 init 失败”这类半成功状态。

### 2.4 hybrid 已补回 CANopen 原生 `post_init_hook` 语义

`HybridServiceGateway` 新增 `SetPostInitHook(...)`，并在 `init` 成功后执行 hook。

当前已接回：

- `canopen_aux.ApplySoftLimitAll(detail)`

如果 hook 失败，会统一执行 shutdown 回滚，避免“表面初始化成功，但软限位未正确应用”的不一致状态。

## 3. 具体修改

### 3.1 can_driver 结构解耦

新增文件：

- `can_driver/include/can_driver/CanDriverRuntime.h`
- `can_driver/src/CanDriverRuntime.cpp`
- `can_driver/include/can_driver/driver_ros_endpoints.hpp`
- `can_driver/src/driver_ros_endpoints.cpp`

关键调整：

1. 将 lifecycle/runtime 对象所有权迁移至 `CanDriverRuntime`
2. 将 ROS service/topic/publisher/timer 迁移至 `DriverRosEndpoints`
3. `MotorMaintenanceService` 从“直接持有内部容器裸指针”改为“通过明确回调访问数据面”
4. `CanDriverHW` 增加 `InitOptions`，支持 `enable_ros_endpoints = false`

### 3.2 Eyou_ROS1_Master hybrid 事务回滚

修改文件：

- `Eyou_ROS1_Master/src/hybrid_operational_coordinator.cpp`
- `Eyou_ROS1_Master/src/hybrid_service_gateway.cpp`
- `Eyou_ROS1_Master/include/Eyou_ROS1_Master/hybrid_service_gateway.hpp`
- `Eyou_ROS1_Master/src/hybrid_motor_hw_node.cpp`
- `Eyou_ROS1_Master/src/hybrid_robot_hw.cpp`

关键调整：

1. `HybridOperationalCoordinator`
   - 为 `RequestInit / Enable / Disable / Release / Halt / Recover` 增加失败补偿
   - 在无法回退到原状态时，执行 fail-safe shutdown
2. `HybridServiceGateway`
   - 支持 `SetPostInitHook(...)`
   - `OnInit()` 在 hook 失败时自动 shutdown 回滚
3. `HybridRobotHW`
   - 初始化 `can_driver` 时显式关闭其 ROS endpoints
4. `hybrid_motor_hw_node`
   - 将 `canopen_aux.ApplySoftLimitAll(...)` 接成 post-init hook

### 3.3 新增测试

#### can_driver

修改文件：

- `can_driver/tests/test_can_driver_hw_smoke.cpp`

新增护栏测试：

- `LifecycleStateTopicTracksCoordinatorMode`
- `InitRegistersRosControlInterfacesForJointMode`
- `InitCanDisableRosEndpointsForEmbeddedUse`

#### Eyou_ROS1_Master

新增文件：

- `Eyou_ROS1_Master/tests/test_hybrid_operational_coordinator.cpp`

新增测试覆盖：

- `Init` 后半失败时是否回滚到 `Configured`
- `Release` 后半失败时是否回滚到 `Armed`
- `Recover` 无法精确回退时是否收敛到 `Configured`

## 4. 提交记录

本轮相关提交分为两部分：

### 4.1 can_driver 子仓库

- Commit: `a748f6e`
- Message: `refactor(can_driver): split runtime and ROS endpoints for embedding`

### 4.2 外层 src 仓库

- Commit: `01abdeb`
- Message: `feat(Eyou_ROS1_Master): add transactional rollback for hybrid lifecycle`

## 5. 验证记录

### 5.1 can_driver 编译与测试

执行：

```bash
catkin_make --pkg can_driver
make -C /home/dianhua/Robot24_catkin_ws/build/can_driver test_can_driver_hw_smoke -j12
/home/dianhua/Robot24_catkin_ws/devel/lib/can_driver/test_can_driver_hw_smoke
```

结果：

- `can_driver` 编译通过
- `test_can_driver_hw_smoke` 共 `32` 条测试全部通过

### 5.2 Eyou_ROS1_Master 相关包联合编译

执行：

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='can_driver;Eyou_Canopen_Master;Eyou_ROS1_Master' \
  --force-cmake \
  --pkg can_driver Eyou_Canopen_Master Eyou_ROS1_Master
```

结果：

- `can_driver` 编译通过
- `Eyou_Canopen_Master` 编译通过
- `Eyou_ROS1_Master` 编译通过

### 5.3 Eyou_ROS1_Master 单元测试

执行：

```bash
make -C /home/dianhua/Robot24_catkin_ws/build/Eyou_ROS1_Master test_hybrid_operational_coordinator -j12
/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_operational_coordinator
```

结果：

- `test_hybrid_operational_coordinator` 共 `3` 条测试全部通过

## 6. 审查关注点

建议审查时重点看以下几点：

1. `can_driver` 的嵌入模式是否保持了默认行为兼容
2. `HybridOperationalCoordinator` 在各生命周期阶段的补偿路径是否足够保守
3. fail-safe shutdown 是否符合项目对“错误时全局失败”的预期
4. `HybridServiceGateway` 的 post-init hook 失败回滚是否与原 CANopen 节点语义一致
5. 新增单测是否覆盖了最关键的中间态失败场景

## 7. 当前剩余风险

本轮修复后，最大的结构性问题已经不再是“半成功不回滚”，但仍有几点值得后续关注：

1. `HybridOperationalCoordinator` 的补偿逻辑是基于当前公开 API 做“最保守恢复”，不是严格意义上的分布式事务。
2. 若底层驱动在某一步已经产生不可逆副作用，当前策略会优先选择 fail-safe 收敛到安全态，而不是强求恢复到原始态。
3. `can_driver` 虽已大幅解耦，但若要完全对齐 `Eyou_Canopen_Master` 的结构，还可继续将 `CanDriverHW` 内的数据面进一步拆成独立核心对象。

## 8. 结论

本轮修复完成后：

- `can_driver` 已经具备较好的可嵌入性
- `Eyou_ROS1_Master` 已从“串行转发请求”升级为“失败时全局回滚/安全收敛”的统一 facade
- 关键行为已通过编译与单测验证

这意味着后续对 `Eyou_ROS1_Master` 的评审重点，可以从“结构是否能组合”转向“补偿策略是否满足业务预期”。
