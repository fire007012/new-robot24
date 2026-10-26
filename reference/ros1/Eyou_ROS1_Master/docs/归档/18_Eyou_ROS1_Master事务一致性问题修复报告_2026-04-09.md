# Eyou_ROS1_Master 事务一致性问题修复报告（2026-04-09）

## 1. 背景

状态更新（2026-04-09）：

- Commit 1：已完成
- Commit 2：已完成
- Commit 3：已完成
- Commit 4：已完成
- Commit 5：已完成
- Commit 6：当前文档更新

本轮审查聚焦 `Eyou_ROS1_Master` 作为统一外观层时的生命周期一致性问题。

当前 facade 已经具备：

- 统一 `controller_manager`
- 统一 lifecycle service 入口
- `can_driver` 与 `Eyou_Canopen_Master` 的组合运行

但在“失败时是否仍保持全局一致”的要求下，仍存在几个关键缺口。  
本报告的目标不是给出泛化建议，而是明确：

1. 当前实现哪里与“全局失败并回滚”目标不一致
2. 哪些行为与现有报告承诺不一致
3. 后续应如何按最小风险顺序修复

## 2. 关键结论

### 2.1 高优问题：4 条事务回滚在 rollback 失败后不会进入 fail-safe shutdown

当前 `HybridOperationalCoordinator` 里：

- `RequestInit`
- `RequestRecover`

在 rollback 失败时会进一步执行 `RequestShutdown()`，把系统收敛到安全态。

但以下 4 条路径仍然没有做到这一点：

- `RequestEnable`
- `RequestDisable`
- `RequestRelease`
- `RequestHalt`

这些路径在“前半段成功、后半段失败、补偿回滚又失败”时，只是返回错误，不会强制把两边进一步收敛到统一安全态。

这会留下 facade 不应该暴露的“半成功”状态，例如：

- `can_driver` 仍处于 `Armed/Running`
- `canopen` 已经回到 `Standby/Faulted`

这与本轮既有修复报告中的承诺不一致。  
报告中已经明确写到：

- `RequestInit / Enable / Disable / Release / Halt / Recover`
  在无法回退到原状态时执行 fail-safe shutdown

但当前代码只对 `Init / Recover` 真正实现了该承诺。

### 2.2 中优问题：CANopen 自动启动序列绕过 hybrid facade

`hybrid_motor_hw_node.cpp` 当前直接调用：

`CanopenStartupSequence::Run(canopen_coord, canopen_aux, pnh)`

这条序列只推进 CANopen 一侧状态机，不经过 `HybridOperationalCoordinator`。

这意味着：

- 只要用户打开 `auto_init / auto_enable / auto_release`
- CANopen 就可能在启动阶段自行进入 `Armed/Running`
- 而 `can_driver` 仍停在 `Configured`

后果是：

- facade 的事务补偿没有机会介入
- “统一外观层”在自动启动路径上被旁路

也就是说，当前系统存在一条与统一 facade 并行的启动通道。

### 2.3 中优问题：`RequestInit()` 丢失 “already initialized” 语义

当前 `HybridServiceGateway::OnInit()` 通过返回消息是否以 `"already "` 开头，
来决定是否跳过 `post_init_hook`。

但 `HybridOperationalCoordinator::RequestInit()` 成功后固定返回：

- `"both backends initialized"`

而不是保留底层的：

- `"already initialized"`

这会导致重复调用 `~/init` 时：

- gateway 误以为是首次初始化
- `post_init_hook` 再次执行
- 当前接入的 `ApplySoftLimitAll()` 被重复调用

这与“补回 CANopen 原生 `post_init_hook` 语义”的目标不一致。

## 3. 现状影响

### 3.1 对系统一致性的影响

高优问题会导致 facade 在错误场景下无法保证：

- 两边后端生命周期状态一致
- 系统能在失败后回到统一安全态

这会直接削弱统一外观层的可信度。

### 3.2 对自动启动链路的影响

自动启动路径当前不受 hybrid facade 控制，这意味着：

- 手动 service 路径是一套逻辑
- 自动启动路径是另一套逻辑

两套逻辑并存会让问题定位和后续维护显著变复杂。

### 3.3 对 hook 语义的影响

重复执行 `ApplySoftLimitAll()` 不一定立刻造成故障，
但它会破坏“post-init hook 只在首次成功初始化后执行一次”的契约。

这类问题通常在联调或二次维护时才暴露，属于典型的语义回归。

## 4. 已完成修复

### 4.1 rollback-failure 统一收敛到安全态

已补齐：

- `RequestEnable`
- `RequestDisable`
- `RequestRelease`
- `RequestHalt`

在 rollback 失败时的 fail-safe shutdown。

### 4.2 自动启动路径已改为经由 hybrid facade

`hybrid_motor_hw_node.cpp` 已不再直接调用 CANopen 原生启动序列，
而是统一经由 hybrid facade authority 执行自动启动。

### 4.3 `already initialized` 语义已恢复

当两侧本来都已处于 `Armed` 时，`RequestInit()` 现在会返回：

- `ok=true`
- `message="already initialized"`

从而使 `post_init_hook` 不会在重复 `~/init` 时再次执行。

### 4.4 hook 语义测试已补齐

已新增：

- `test_hybrid_service_gateway.cpp`

用于验证：

- 首次 init 执行 hook
- 重复 init 跳过 hook
- hook 失败时 shutdown 回滚

## 5. 剩余关注点

本轮后续修复应明确达成以下目标：

1. `RequestEnable / Disable / Release / Halt` 在 rollback 失败时也进入 fail-safe shutdown
2. 自动启动路径不允许绕过 hybrid facade
3. `RequestInit()` 正确保留 `"already initialized"` 语义
4. 新增测试覆盖：
   - 4 条 rollback-failure 场景
   - `post_init_hook` 重复执行防护
   - 自动启动旁路防护

## 6. 修复原则

本轮建议坚持以下原则：

1. 优先“全局安全失败”，其次才是“尽量恢复到原状态”
2. 不引入第二套并行启动路径
3. 所有 facade 级承诺都必须有测试覆盖
4. 文档承诺必须与实现保持一致，不能让报告先于代码

## 7. 提交记录

- `9a2cdb3`
  - `fix(Eyou_ROS1_Master): fail safe on rollback failure for hybrid lifecycle`
- `01bac24`
  - `test(Eyou_ROS1_Master): cover rollback failure fail-safe paths`
- `5f06b60`
  - `fix(Eyou_ROS1_Master): report fail-safe shutdown in rollback errors`
- `2cfdc23`
  - `fix(Eyou_ROS1_Master): route auto init through hybrid facade`
- `23005e7`
  - `test(Eyou_ROS1_Master): verify hybrid init hook semantics`

## 8. 验证结果

执行并通过：

```bash
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES='can_driver;Eyou_Canopen_Master;Eyou_ROS1_Master' \
  --force-cmake \
  --pkg can_driver Eyou_Canopen_Master Eyou_ROS1_Master

/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_operational_coordinator
/home/dianhua/Robot24_catkin_ws/devel/lib/Eyou_ROS1_Master/test_hybrid_service_gateway
```

其中：

- `test_hybrid_operational_coordinator`：7 条测试通过
- `test_hybrid_service_gateway`：2 条测试通过

## 9. 结论

当前 `Eyou_ROS1_Master` 已完成本轮事务一致性主修复：

- rollback-failure 可收敛到安全态
- auto 启动不再绕开 hybrid facade
- `already initialized` 语义已恢复
- hook 语义已有测试覆盖

剩余问题不再是“事务一致性主链缺失”，而是后续是否要继续增强更多异常分支覆盖与集成测试。
