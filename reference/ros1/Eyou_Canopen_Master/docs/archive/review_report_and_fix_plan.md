# PDO 映射验证实现评估与修复计划

日期：2026-03-17  
范围：基于 `change_report_pdo_verify.md` 的现有实现

---

## 1. 评估结论

当前实现已完成核心链路（配置解析、映射读回、差异比对与 OnBoot 触发），并能在验证失败时阻断 Operational。结构清晰，可用性方向正确，但仍存在稳定性与实时性风险，需在上线前补齐超时、I/O 位置和回调生命周期等问题。

---

## 2. 主要问题清单（按严重性）

### 2.1 高严重

1. **缺少超时兜底**
   - 现状：SDO 读未返回时会永久卡住 `pdo_verification_done_ = false`，轴始终非 Operational。
   - 影响：系统无法恢复，需要人工干预或重启。

2. **回调线程内同步读取 DCF**
   - 现状：`OnBoot` 回调中读取 DCF 文件（阻塞 I/O）。
   - 影响：阻塞 Lely 事件循环，影响实时性与启动稳定性。

### 2.2 中严重

3. **OnBoot 状态判断不稳妥**
   - 现状：`st == BOOTUP` 直接跳过验证。
   - 风险：若 `OnBoot` 常态为 `BOOTUP`，将导致永不验证或误判失败。
   - 建议：仅以 `es != 0` 作为失败条件，`st` 需按 Lely 语义再决定。

4. **回调生命周期安全不明确**
   - 现状：SDO 回调捕获 `this`，Stop/析构时若仍有挂起读请求，可能访问已析构对象。
   - 风险：潜在崩溃或未定义行为。

### 2.3 低严重

5. **日志可观测性不足**
   - 现状：SDO 读失败日志不包含 idx/sub；PDO 编号从 0 开始不直观。
   - 影响：现场排查成本高。

---

## 3. 修复计划（按 commit 划分）

### Commit 1：预加载 DCF，回调内只做比对

目标：消除回调线程内 I/O，提高实时性稳定性。

内容：
- 在 `AxisDriver` 构造或 `CreateAxisDrivers` 阶段加载 expected 映射并缓存。
- `OnBoot` 只做 `DiffPdoMapping`。
- 失败判断仅保留 `es != 0`，不依赖 `st == BOOTUP`。

编译：
```
cmake -S . -B build
cmake --build build -j
```

提交信息：
```
canopen: preload expected PDO mapping before OnBoot
```

---

### Commit 2：加入 PDO 验证超时兜底

目标：避免 SDO 读挂死导致永久阻塞。

内容：
- `PdoMappingReader` 增加超时机制（建议 2s）。
- 超时触发 `Finish(false, "timeout")`，轴进入验证失败态。
- 若已接入 Lely 事件循环，优先用 timer；否则先用时间戳轮询兜底。

编译：
```
cmake -S . -B build
cmake --build build -j
```

提交信息：
```
canopen: add PDO verify timeout fallback
```

---

### Commit 3：日志增强与编号修正

目标：提升现场定位效率。

内容：
- SDO 读失败日志包含 idx/sub。
- RPDO/TPDO 日志编号从 1 开始。

编译：
```
cmake -S . -B build
cmake --build build -j
```

提交信息：
```
canopen: improve PDO verify logging
```

---

## 4. 备注

当前工作区存在未处理改动（`.gitignore` 修改、`main.cpp` 删除），已按要求不处理。后续执行每个 commit 前会先确认工作区状态，避免混入无关变更。
