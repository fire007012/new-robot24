# 计划书：IP 模式（Mode=7）支持改造 Commit 路线表

日期：2026-03-23  
状态：规划中（待实施）

## 1. 目标与边界

目标：
1. 在现有 `CSP/CSV` 基础上新增可用的 `IP`（Mode=7）运行链路。  
2. 运行时可通过 `~set_mode` 在 `CSP <-> CSV <-> IP` 切换。  
3. IP 模式下采用 `0x60C1:01` 作为插补位置数据对象，`0x60C2` 仅做 SDO 配置。  

边界（本轮不做）：
1. 不引入 CST 控制器链路。  
2. 不做多节点异构映射自动探测（先按当前关节驱动统一配置）。  
3. 不改控制器算法层（仅打通模式与对象通道）。

## 2. 前置风险（必须显式管理）

本地 EDS 文件中 `0x60C1:01` 当前标记为 `PDOMapping=0`，与现场手册“可映射 RPDO”描述冲突。  
因此路线中包含“能力门禁与回退策略”：

1. 先按手册路线实现并验证。  
2. 若现场驱动拒绝 `0x60C1:01` 映射，则回退到“Mode=7 + 0x607A 通道”的兼容方案，并记录为限制项。

## 3. Commit 路线表

| Commit | 建议提交信息 | 主要改动 | 关键文件 | 代码量估算 | 验收点 |
|---|---|---|---|---:|---|
| C01 | `test(ip): add mode7 baseline and regression guards` | 先补失败用例，锁定 IP 预期行为（模式常量、命令流、OnBoot 模式保持） | `test/test_multi_mode.cpp`, `test/test_state_machine.cpp`, `test/test_axis_logic.cpp` | +120 ~ +180 | 新增用例在未改功能前应失败（或至少暴露缺口） |
| C02 | `feat(core): introduce kMode_IP and explicit mode routing` | 新增 `kMode_IP=7`，核心逻辑显式识别 IP（位置路径语义） | `include/canopen_hw/cia402_defs.hpp`, `src/cia402_state_machine.cpp`, `src/canopen_robot_hw_ros.cpp`, `include/canopen_hw/canopen_robot_hw_ros.hpp`, `docs/api_reference.md` | +40 ~ +80 | `~set_mode mode=7` 后链路不再依赖 default 分支隐式行为 |
| C03 | `fix(boot): remove hardcoded CSP pre-init mode` | 去掉 OnBoot 预写 `kMode_CSP` 的硬编码，改为按“当前目标模式”预写，避免 IP/CSV 被 boot 覆盖回 8 | `src/axis_driver.cpp`, `src/axis_logic.cpp`(如需 accessor), `include/canopen_hw/axis_logic.hpp` | +35 ~ +70 | 复电后若当前模式是 7/9，不再被强制改回 8 |
| C04 | `feat(ip-config): add 60C2 SDO init and mode config knobs` | 增加 IP 配置项（插补周期值等），启动时通过 SDO 配置 `0x60C2:01`；`0x60C2:02` 只读不写 | `include/canopen_hw/canopen_master.hpp`, `src/joints_config.cpp`, `config/joints.yaml`, `docs/yaml_config_guide.md`, `docs/usage.md` | +80 ~ +140 | 参数可配置，日志可观测，`0x60C2` 不走 PDO |
| C05 | `feat(ip-pdo): route IP position to 0x60C1:01` | IP 模式下位置目标写入 `0x60C1:01`；保留 CSP 走 `0x607A`；补齐 PDO 映射（建议 RPDO2） | `src/axis_driver.cpp`, `config/master.yaml`, `config/master.dcf`, `docs/usage.md` | +90 ~ +160 | candump 可见 IP 数据对象通道，驱动在 mode=7 下跟随有效 |
| C06 | `test+docs: finalize IP bringup checklist and fallback path` | 完成联调手册与回归；写明 EDS/手册冲突下的回退方案 | `docs/fault_injection_checklist.md`, `docs/api_reference.md`, `docs/2026-03-23_ip_mode_support_commit_plan.md`(更新状态), `test/*`(按需) | +60 ~ +120 | 有明确“通过/回退”判据，可重复执行 |

## 4. 关键设计决策（实施时遵循）

1. `0x60C2` 只走 SDO，不映射 PDO。  
2. IP 模式对象切换要显式实现，不依赖 `switch default`。  
3. OnBoot 预写模式必须与“当前目标模式”一致，不能固定 CSP。  
4. 保留 CSP 现有行为不回归（现网优先）。

## 5. 验收清单（最终）

1. `set_mode=7` 后，`mode_display==7` 稳定。  
2. IP 模式下位置命令经 `0x60C1:01` 下发（抓包可见）。  
3. `0x60C2:01` 配置值与主站周期规划一致。  
4. 切回 `mode=8` 后，链路回到 `0x607A`，运动正常。  
5. 掉线复电后不被 OnBoot 强制改回 CSP。  

## 6. 代码量总评估

仅代码：
- 新增约 `+365 ~ +630` 行  
- 删除约 `-20 ~ -70` 行  
- 净增约 `+300 ~ +560` 行

含文档与测试：
- 总净增约 `+450 ~ +800` 行

## 7. 回滚策略

1. C05（对象通道切换）独立回滚，不影响 C01-C04 的模式与配置改造。  
2. 若现场证实 `0x60C1:01` 不可映射 PDO：  
   - 保留 C01-C04/C06，回滚 C05；  
   - 进入兼容分支：Mode=7 仍走 `0x607A`，并在文档标记“非标准 IP 数据对象实现”。  

