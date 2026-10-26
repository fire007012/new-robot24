# Bug 报告：CI 质量门失真与状态机极值解锁缺陷

- 报告日期：2026-03-19
- 项目：`canopen_hw`
- 严重级别：
  - Bug A（CI 测试未执行误通过）：P0
  - Bug B（clang-tidy 非阻断）：P1
  - Bug C（`AbsDiff` 截断误解锁）：P1（安全语义相关）

## 1. 摘要

当前项目存在两个“质量门可信度”缺陷和一个“状态机边界值”缺陷：

1. CI 中 `ctest --test-dir` 在当前工具链（`ctest 3.16.3`）下不生效，可能出现“0 测试也通过”。
2. CI 中 `clang-tidy` 通过 `|| echo ...` 被降级为非阻断，静态检查无法拦截风险变更。
3. `CiA402StateMachine` 使用 `int32_t` 返回绝对差值，在 `INT32_MAX` 与 `INT32_MIN` 场景发生截断，导致应锁定时误解锁。

这三项会直接拉低发布可信度：前两项导致“门禁失真”，第三项是逻辑正确性缺陷。

---

## 2. 影响范围

- 代码路径：
  - CI：`.github/workflows/ci.yml`
  - 状态机：`src/cia402_state_machine.cpp`
  - 关联测试：`test/test_boundary_cases.cpp`
- 影响对象：
  - 所有 PR/主干流水线结果可信度
  - 极端位置差输入下的安全锁定逻辑

---

## 3. Bug A：CI 测试步骤可能“0 测试通过”

### 3.1 现象

CI 测试步骤当前为：

- 文件：`.github/workflows/ci.yml:59`
- 内容：`ctest --test-dir build/${{ matrix.build_type }} --output-on-failure`

本地环境验证：

```bash
ctest --version
# ctest version 3.16.3

ctest --help | rg -n "test-dir|--test-dir"
# 无输出（该版本无此参数）

ctest --test-dir build --output-on-failure
# Test project /home/dianhua/robot_test
# No tests were found!!!
# (退出码为 0)
```

说明在该版本上，测试步骤可能未真正执行测试但流水线仍通过。

### 3.2 根因

- 使用了与当前 CTest 版本不兼容的参数。
- 质量门仅依赖退出码，未对“测试数量”做下限约束。

### 3.3 风险

- 高风险回归可直接进入主干。
- `release_readiness` 文档中的“29/29 通过”在 CI 维度不再可靠。

### 3.4 修复方案

方案 A（推荐，兼容性最好）：

```yaml
- name: Test (${{ matrix.build_type }})
  working-directory: build/${{ matrix.build_type }}
  run: ctest --output-on-failure
```

方案 B（可选）：升级 CI 中 CMake/CTest 并固定最小版本，再保留 `--test-dir`。

### 3.5 验收标准

- 在 CI 日志中看到 `Test project .../build/<type>`。
- 明确出现 `Total Tests: N` 且 `N > 0`。
- 故意引入一个失败测试时，流水线应失败。

---

## 4. Bug B：clang-tidy 质量门被降级为非阻断

### 4.1 现象

- 文件：`.github/workflows/ci.yml:70`
- 当前逻辑：`clang-tidy ... || echo "...non-blocking..."`

即使 `clang-tidy` 报错，最终步骤也可能返回成功。

### 4.2 根因

- 为避免噪声告警阻塞开发，使用了“吞错”逻辑，但没有分级阻断策略。

### 4.3 风险

- `bugprone-*` 与 `performance-*` 检查结果无法作为准入门禁。

### 4.4 修复方案

- 去掉 `|| echo ...`，让步骤按真实退出码失败。
- 若担心一次性噪声过大，可临时仅保留 `bugprone-*` 为阻断规则。

### 4.5 验收标准

- 人工引入一个可被 `bugprone-*` 捕获的问题，CI 必须失败。

---

## 5. Bug C：`AbsDiff` 极值截断导致误解锁

### 5.1 现象

状态机中绝对差值计算：

- 文件：`src/cia402_state_machine.cpp:9-12`

```cpp
int32_t AbsDiff(int32_t a, int32_t b) {
  const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
  return static_cast<int32_t>(d < 0 ? -d : d);
}
```

当 `a=INT32_MAX`, `b=INT32_MIN` 时：
- 正确差值应为 `4294967295`（`int64_t`）
- 被强转为 `int32_t` 后截断为 `-1`
- 比较 `AbsDiff(...) <= position_lock_threshold_` 会误判为 true，从而错误解锁

该行为在测试中已被“文档化为当前行为”：

- 文件：`test/test_boundary_cases.cpp:118-131`
- 当前断言：`EXPECT_FALSE(sm.is_position_locked());`

### 5.2 根因

- 中间结果虽用 `int64_t`，但返回类型仍为 `int32_t`，导致窄化截断。

### 5.3 风险

- 属于安全语义相关逻辑错误：锁定机制在极值输入下可能失效。
- 虽然工程上概率低，但不应依赖“概率低”接受逻辑错误。

### 5.4 修复方案（推荐）

1. 将 `AbsDiff` 返回类型改为 `int64_t`。
2. 比较时将阈值显式提升为 `int64_t`：

```cpp
if (AbsDiff(ros_target_, actual_position) <=
    static_cast<int64_t>(position_lock_threshold_)) {
  ...
}
```

3. 更新边界测试预期：
- `Boundary.Int32ExtremeAbsDiffTruncation` 改为 `EXPECT_TRUE(sm.is_position_locked());`

### 5.5 验收标准

- 更新后该测试断言变为“保持锁定”，并在本地/CI 通过。
- 其余状态机测试不回归。

---

## 6. 建议的修复顺序

1. 先修 Bug A + Bug B（恢复 CI 门禁可信度）。
2. 再修 Bug C 并更新测试（保证逻辑正确性）。
3. 在 `docs/release_readiness.md` 中将 P2-5 从“接受风险”调整为“已修复”。

---

## 7. 回归验证清单

```bash
cmake -S . -B build
cmake --build build -j
cd build && ctest --output-on-failure
```

附加检查：

```bash
clang-tidy -p build --warnings-as-errors='*' \
  --checks='-*,bugprone-*,performance-*,readability-braces-around-statements' \
  --header-filter='include/canopen_hw/.*' src/*.cpp
```

预期：
- 测试数 > 0 且全通过
- `clang-tidy` 无错误（或按策略只允许白名单例外）

---

## 8. 结论

该问题组不是“代码无法运行”的故障，而是“质量门失真 + 边界值逻辑错误”的组合风险。建议在下一次发布前全部修复并完成一次完整 CI 复验。
