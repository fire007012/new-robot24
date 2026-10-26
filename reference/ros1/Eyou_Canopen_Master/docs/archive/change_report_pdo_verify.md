# PDO 映射验证改动报告

日期：2026-03-17  
范围：按“PDO 映射验证 — 最终方案”实现（不含降级控制）

---

## 1. 目标与结论

目标：启动时验证从站 PDO 映射与 DCF 是否一致，不一致则该轴不进入 Operational。  
结论：已完成核心功能的代码落地与配置支持，验证逻辑在 `OnBoot` 内执行并输出差异明细。

---

## 2. 变更摘要

### 2.1 配置与解析

新增解析 `joints[].canopen.verify_pdo_mapping`，并支持每轴 `node_id`。

影响：
- `CanopenMasterConfig` 改为按轴接收 `node_id` 与 `verify_pdo_mapping`。
- `main.cpp` 先加载配置再创建 master，保证 node_id/verify 标志生效。
- 更新 `config/joints.yaml` 示例与单测。
- 文档更新使用说明。

### 2.2 PDO 验证逻辑

新增 PDO 映射读取与比对模块：

- `PdoMappingReader`：通过 SDO 读回实际 COB-ID 与映射条目  
  - RPDO：0x1400/0x1600  
  - TPDO：0x1800/0x1A00
- `DiffPdoMapping`：对比 DCF 期望值与读回结果，输出差异明细。
- `AxisDriver::OnBoot()`：执行验证流程。
  - 验证失败或未完成时，强制 `is_operational=false`，阻断上层命令。

---

## 3. 详细改动清单

### 新增文件

- [pdo_mapping.hpp](/home/dianhua/robot_test/include/canopen_hw/pdo_mapping.hpp)  
  定义 PDO 映射结构、Reader、Diff 接口。

- [pdo_mapping.cpp](/home/dianhua/robot_test/src/pdo_mapping.cpp)  
  实现 DCF 期望映射读取、SDO 读回与差异比对。

### 修改文件

- [joints_config.hpp](/home/dianhua/robot_test/include/canopen_hw/joints_config.hpp)  
  新增 `JointCanopenConfig` 与 `CanopenRuntimeConfig::joints`。

- [joints_config.cpp](/home/dianhua/robot_test/src/joints_config.cpp)  
  解析 `joints[].canopen.node_id` 与 `verify_pdo_mapping`，并保持兼容原 `node_id` 写法。

- [canopen_master.hpp](/home/dianhua/robot_test/include/canopen_hw/canopen_master.hpp)  
  `CanopenMasterConfig` 新增 `node_ids` 与 `verify_pdo_mapping` 数组。

- [canopen_master.cpp](/home/dianhua/robot_test/src/canopen_master.cpp)  
  `CreateAxisDrivers()` 使用配置的 node_id 与 verify 标志。

- [axis_driver.hpp](/home/dianhua/robot_test/include/canopen_hw/axis_driver.hpp)  
  构造函数新增 `verify_pdo_mapping` 与 `dcf_path` 参数，保存验证状态。

- [axis_driver.cpp](/home/dianhua/robot_test/src/axis_driver.cpp)  
  `OnBoot` 触发验证；验证失败时阻断 `is_operational`。

- [main.cpp](/home/dianhua/robot_test/src/main.cpp)  
  先读取 joints.yaml，再创建 master，传入 per-axis 配置。

- [CMakeLists.txt](/home/dianhua/robot_test/CMakeLists.txt)  
  新增 `src/pdo_mapping.cpp`。

- [test_joints_config.cpp](/home/dianhua/robot_test/test/test_joints_config.cpp)  
  测试 `verify_pdo_mapping` 解析。

- [config/joints.yaml](/home/dianhua/robot_test/config/joints.yaml)  
  示例调整为 `joints[].canopen.verify_pdo_mapping`。

- [usage.md](/home/dianhua/robot_test/docs/usage.md)  
  补充 `verify_pdo_mapping` 字段说明。

---

## 4. 行为变化

### 4.1 正常情况

- `verify_pdo_mapping=false`：行为与原系统一致，不做验证。
- `verify_pdo_mapping=true` 且映射一致：进入 Operational，PDO 流程正常。

### 4.2 异常情况

- SDO 读回失败或映射不一致：
  - 打印差异明细（ERROR）。
  - 该轴 `is_operational=false`，`all_operational` 无法为 true。

---

## 5. 日志样例

```
[INFO]  Axis 0 (node 1): PDO mapping verified
[ERROR] Axis 1 (node 2): PDO mapping mismatch
  TPDO1 entry[1] mismatch: expected 6041:0/16 actual 606C:0/32
```

---

## 6. 编译验证

```
cmake -S . -B build
cmake --build build -j
```

---

## 7. 未覆盖项与后续建议

1. **OnBoot 超时兜底**  
   当前验证逻辑放在 `OnBoot`，但“超时未触发 OnBoot”的兜底未实现。  
   需要在接入 Lely 事件循环后增加定时器超时处理。

2. **DCF 期望映射来源**  
   当前直接读取 `master_dcf_path` 作为期望映射来源；若未来改为分轴 DCF，需要扩展配置。

---

## 8. 相关提交

1. `config: parse verify_pdo_mapping in joints yaml`
2. `canopen: add pdo mapping reader and verify on boot`

