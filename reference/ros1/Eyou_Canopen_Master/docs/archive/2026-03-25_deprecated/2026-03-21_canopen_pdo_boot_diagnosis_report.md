# CANopen 主站通信异常排查与修复报告（完整修订版）

**项目：** Eyou_Canopen_Master（Lely Core + ROS）  
**日期：** 2026-03-21  
**总线：** `canable0`  
**从站：** Node ID = 5  
**模式：** CSP

---

## 1. 问题概述

现场出现过以下典型问题：

1. `dcfgen` 失败（`invalid DCF`），导致配置生成链路中断。
2. 主站与从站身份信息不一致，boot 配置阶段不稳定。
3. 总线观测出现 TPDO 反馈异常（特别是 `0x185` 稀疏/缺失阶段）。
4. 控制器加载失败：`Could not find joint 'joint_2' in URDF model.`
5. 关闭阶段存在卡住风险（联调阶段曾见 `controller_spawner` 等未及时退出）。

---

## 2. 统一根因结论

### 2.1 EDS 格式兼容问题（生成阶段）

`YiyouServo_V1.4.dcfgen.eds` 包含两类导致 `dcfgen` 严格校验失败的内容：

- 非兼容段：`[DynamicChannels]`
- 越界字段：`[2023]`、`[2025]` 中 `UNSIGNED16` 的 `HighLimit=65536`

### 2.2 身份字段不一致（boot/配置阶段）

实测设备身份如下：

| 对象 | 含义 | 实测值 |
|---|---|---|
| `0x1000:00` | Device Type | `0x00220811` |
| `0x1018:01` | Vendor ID | `0x00000668` |
| `0x1018:02` | Product Code | `0x00221701` |

旧 EDS 中 `ProductNumber` 与 `1018sub2` 不一致，造成主站期望值与从站实值可能偏离。

### 2.3 URDF 与控制器关节配置不一致（运行阶段）

`controllers.yaml` 依赖 `joint_1~joint_6`，而旧 `urdf/robot.urdf` 仅含 `joint_1`，触发控制器初始化失败。

---

## 3. 已实施修复（已落地）

### 3.1 EDS 修复

文件：`config/YiyouServo_V1.4.dcfgen.eds`

1. 删除 `[DynamicChannels]`
2. 修复越界：
   - `[2023] HighLimit: 65536 -> 65535`
   - `[2025] HighLimit: 65536 -> 65535`
3. 统一产品码：
   - `[DeviceInfo] ProductNumber = 0x00221701`
   - `[1018sub2] DefaultValue = 0x00221701`

### 3.2 重新生成 DCF 产物

执行 `dcfgen master.yaml` 后更新：

- `config/master.dcf`
- `config/master.bin`
- `config/joint_1.bin`

### 3.3 URDF 修复

文件：`urdf/robot.urdf`

- 从单关节扩展为 `joint_1~joint_6`
- 与 `config/ros/controllers.yaml` 关节列表对齐

---

## 4. 验证结论

1. `dcfgen master.yaml` 可通过（无 fatal error）。
2. `master.dcf` 中设备身份检查值与实测一致：
   - `1F84:5 = 0x00220811`
   - `1F85:5 = 0x00000668`
   - `1F86:5 = 0x00221701`
3. URDF 与控制器关节命名一致，`joint_2` 缺失问题已消除。

---

## 5. 提交记录（精确到 commit）

### 5.1 当前生效提交（截至 2026-03-21）

- `7605dbd` `feat: add full Eyou_Canopen_Master package (code, urdf, docs, dcf)`
- `e9fda64` `fix(canopen): publish controlword in rpdo cycle`
- `f23c328` `fix(canopen): align shutdown flow with disable request`
- `12b457b` `feat(canopen): add boot identity mismatch diagnostics`
- `2475555` `test(canopen): add startup and operational regression`

---

## 6. 修复计划完成状态（精确到 commit）

| 计划ID | 目标 commit message | 状态 | 结果/提交 |
|---|---|---|---|
| C01 | `fix(canopen): publish controlword in rpdo cycle` | ✅ 完成 | `e9fda64` |
| C02 | `fix(canopen): align shutdown flow with disable request` | ✅ 完成 | `f23c328` |
| C03 | `feat(canopen): add boot identity mismatch diagnostics` | ✅ 完成 | `12b457b` |
| C04 | `test(canopen): add startup and operational regression` | ✅ 完成 | `2475555` |
| C05 | `docs(canopen): finalize validation checklist for field` | ✅ 完成 | 当前提交 |

---

## 7. 现场验证清单（最终版）

### 7.1 启动前检查

1. `canable0` 已 up，波特率与驱动器一致。
2. `master.dcf` 与现场固件版本匹配。
3. 打开抓包并包含关键 COB-ID：`0x080`、`0x185`、`0x205`、`0x285`、`0x705`、`0x000`。

### 7.2 启动与身份判据

通过标准：

- 无持续 `OnConfig failed`。
- 若失败日志出现，`expected identity from master.dcf` 与 `actual identity snapshot` 可直接对比 `1000:00/1018:01/1018:02` 并定位失配字段。

### 7.3 抓包判据

1. 使能阶段：`0x205` 前 2 字节出现 `06 00 -> 0F 00`。
2. 运行阶段：`0x185` 持续周期刷新，无长时间稀疏/丢失。
3. 退出阶段（Ctrl+C）：控制字回落 `07 00 -> 06 00`，随后可见 NMT Stop（`0x000`）。

### 7.4 运行态判据

- `all_operational = true`。
- 各轴诊断为 `is_operational=true` 且无持续 `is_fault=true`。

### 7.5 负路径判据

- 缺失 `--dcf` 文件：启动失败退出（返回码 1）。
- 缺失 `--joints` 文件：启动失败退出（返回码 1）。

---

## 8. 备注

- 本报告为当前执行基线；此前 2026-03-19 阶段报告已归档至 `docs/archive/2026-03-21_deprecated/`。
- 若后续固件版本变化，需优先复核 `0x1000` 与 `0x1018` 三元组，再生成 DCF。
