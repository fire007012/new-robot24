# Eyou_ROS1_Master 修复复盘（2026-04-09）

## 1. 复盘目的

本复盘不讨论“某一行代码写错了”，而是回看这轮改造为什么会出现：

- 主链修好了，但旁路没收口
- 单测通过了，但 launch/auto 路径仍然有问题
- 过早宣布“修复完成”，后续又被审查指出新的问题

目标是把这次失误沉淀成后续做架构级修复时必须执行的检查清单。

## 2. 本轮问题的本质

这次出现的问题，并不是单纯的实现粗心，而是典型的“跨层修复不完整”：

1. 修改范围跨越了 `can_driver`、`Eyou_Canopen_Master`、`Eyou_ROS1_Master`
2. 同时涉及：
   - runtime
   - ROS gateway
   - lifecycle facade
   - auto startup
   - hook 语义
   - 测试
3. 代码里存在多条入口：
   - 手动 service 调用
   - auto startup
   - launch 默认参数
   - 重复 init
   - rollback 失败路径

结果是：

- 修主链不等于修完整系统
- 单个类的单测通过，不等于 facade 整体语义成立

## 3. 这次为什么会“修了又冒问题”

### 3.1 先修主流程，后看入口矩阵

这轮工作一开始优先处理的是：

- `can_driver` 结构解耦
- hybrid rollback 主链
- hook 语义

这些改动本身方向是对的，但缺点是：

- 先修了“最明显的主流程”
- 没有先把所有入口列成矩阵逐条核对

结果导致：

- `RequestInit / Recover` 有 fail-safe shutdown
- 但 `Enable / Disable / Release / Halt` 在 rollback 失败后没有同样处理

也就是主链补上了，分支语义没有一起闭环。

### 3.2 对“旁路路径”不够警惕

本轮最典型的旁路是：

- `CanopenStartupSequence::Run(...)`

一开始虽然 facade service 已经统一了，但节点启动时仍然直接跑 CANopen 原生启动序列。

这说明：

- 逻辑上的统一入口已经有了
- 但系统级真正执行时，仍然存在另一条并行入口

这类问题不会在单个类的 review 中自然暴露，只会在“按系统入口检查”时出现。

### 3.3 低估了“隐式契约”的脆弱性

一个典型例子是：

- `HybridServiceGateway` 依赖返回消息是否以 `"already "` 开头
- 才决定是否跳过 `post_init_hook`

这种设计在短期看很方便，但它建立在一个很脆弱的隐式契约上：

- 下游消息格式不能变

结果 `HybridOperationalCoordinator::RequestInit()` 改成固定返回 `"both backends initialized"` 后，
hook 语义立刻被破坏。

问题不在字符串本身，而在于：

- 用消息文本承载控制语义

### 3.4 过早把“阶段性可用”表述成“问题已完成”

本轮还有一个明显的过程问题：

- 当 rollback 主链和部分测试通过后，过早把状态描述成“修复完成”

但当时实际上还缺：

- auto startup 入口收口
- already 语义恢复
- hook 行为测试
- launch 层可用性检查

所以更准确的说法应该是：

- 主链已修通
- 系统级语义尚未完全闭环

## 4. 这次暴露出的流程缺口

### 4.1 缺少“入口矩阵”

做 facade 相关修复前，应该先列出所有入口：

1. 手动 `~/init`
2. 手动 `~/enable`
3. 手动 `~/resume`
4. auto_init
5. auto_enable
6. auto_release
7. 重复 `~/init`
8. rollback 失败
9. hook 失败

这次没有先做这张表，所以修复天然偏向“先想到的路径”。

### 4.2 缺少“承诺矩阵”

每个入口都应该先明确：

- 成功时状态应是什么
- 失败时状态应收敛到哪里
- 是否允许部分成功
- 是否应当幂等
- 是否应触发 hook

例如：

- 重复 `~/init` 应幂等
- rollback 失败应 fail-safe shutdown
- auto startup 不应绕开 facade

这次这些要求是后审查阶段才被重新拉齐的。

### 4.3 缺少“文档与代码一一对照”

这次有一个典型失误：

- 报告中已经写了 `Enable / Disable / Release / Halt` 在 rollback 失败时也会 fail-safe
- 但代码实际上没有全补齐

说明流程上缺了最后一步：

- 写完报告后，逐条回到代码做反向校验

## 5. 后续必须遵守的规则

### 5.1 先画入口矩阵，再动 facade 代码

以后所有 facade / lifecycle / startup 改造，必须先列入口矩阵。

### 5.2 不允许用消息字符串承载控制语义

像 `"already initialized"` 这种返回语义，应该尽量由显式布尔值或状态表达，
不能再让上层靠字符串前缀判断流程分支。

### 5.3 修“主流程”时必须同步检查旁路

至少强制检查：

- auto startup
- launch 默认参数
- 测试入口
- shutdown 路径

### 5.4 阶段性完成必须带限定语

以后不能直接说“修复完成”，而应区分：

- 主链修通
- 系统语义闭环
- launch 可用
- 测试覆盖完成

## 6. 这次复盘的结论

本轮暴露的问题，不是“修复方向错了”，而是：

- 主链先走通了
- 入口矩阵和语义矩阵后补
- 因而造成多轮来回修补

这次的经验应该沉淀为一条明确规则：

> 架构级修复不能只看主流程，必须先列入口矩阵、承诺矩阵，再写代码，再对照文档验收。

如果后续继续做 `Eyou_ROS1_Master` 的高层收尾，这份复盘应作为前置检查清单使用。
