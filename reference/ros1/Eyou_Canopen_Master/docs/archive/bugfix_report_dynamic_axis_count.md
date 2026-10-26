# Bugfix 报告：轴数硬编码动态化

- 日期：2026-03-19
- 项目：`canopen_hw`
- 严重级别：P1（可扩展性阻断）

## 1. 问题描述

`SharedState`、`SharedSnapshot`、`CanopenRobotHw` 中轴数硬编码为 `kAxisCount = 6`，所有数据数组使用 `std::array<T, 6>`。这导致项目无法复用到非 6 轴机器人（如 4 轴 SCARA、7 轴协作臂），每次适配都需要修改源码并重新编译。

## 2. 根因

- `SharedState::kAxisCount = 6` 为编译期常量，所有数组维度在编译时固定。
- `SharedSnapshot` 使用 `std::array<AxisFeedback, 6>` 等固定大小结构。
- `CanopenRobotHw` 通过 `static constexpr kAxisCount = SharedState::kAxisCount` 传播硬编码。
- 虽然存在 `active_axis_count_` 运行时变量，但底层存储仍为固定 6 轴。

## 3. 修复方案

采用"构造时确定 + vector"方案：

- `SharedState` 构造函数接收 `axis_count` 参数，内部 `std::array` 全部替换为 `std::vector`，构造时一次性 resize，运行期大小不变（零额外堆分配）。
- `kAxisCount = 6` 替换为 `kMaxAxisCount = 16` 作为安全上限。
- `CanopenRobotHw` 从 `SharedState` 获取 `axis_count()`，内部数组同样改为 vector。
- 删除 `SetActiveAxisCount()`，轴数由构造函数一��性确定，不可运行期变更。
- 默认值保持 6，向后兼容。

## 4. 修改文件

| 文件 | 变更内容 |
|------|----------|
| `include/canopen_hw/shared_state.hpp` | `kAxisCount→kMaxAxisCount=16`，`array→vector`，新增构造函数 |
| `src/shared_state.cpp` | 构造函数实现（std::clamp），删除 `SetActiveAxisCount` |
| `include/canopen_hw/canopen_robot_hw.hpp` | 删除 `static constexpr kAxisCount`，`array→vector`，新增 `axis_count()` |
| `src/canopen_robot_hw.cpp` | 构造函数从 SharedState 获取 axis_count 并 resize |
| `include/canopen_hw/canopen_master.hpp` | `axis_count` 默认值注释更新 |
| `src/canopen_master.cpp` | `kAxisCount→kMaxAxisCount` |
| `src/main.cpp` | SharedState 构造传参，删除 `SetActiveAxisCount` 调用 |
| `src/joints_config.cpp` | `CanopenRobotHw::kAxisCount→robot_hw->axis_count()` |
| `test/test_boundary_cases.cpp` | 适配构造函数，新增 `AxisCountZeroClampedToOne` 测试 |
| `test/test_shared_state*.cpp` | 适配构造函数传参 |
| `test/test_canopen_master.cpp` | 适配构造函数传参 |
| `test/test_canopen_robot_hw.cpp` | 适配构造函数传参 |
| `test/test_joints_config.cpp` | 适配构造函数传参 |
| `test/test_unit_conversion.cpp` | 适配构造函数传参 |

## 5. 验证结果

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
cd build && ctest --output-on-failure

100% tests passed, 0 tests failed out of 30
Total Test time (real) = 0.44 sec
```

- 编译零警告（-Wall -Wextra -Werror）
- 30/30 测试全部通过（比改造前多 1 个新增边界测试）

## 6. 关键约束

- 运行期零分配：vector 在构造时一次性 resize，主循环中不触发堆分配
- 线程安全不变：mutex 保护逻辑不变，Snapshot() 仍为锁内拷贝
- 向后兼容：SharedState 默认构造 axis_count=6，现有代码无需修改即可编译
