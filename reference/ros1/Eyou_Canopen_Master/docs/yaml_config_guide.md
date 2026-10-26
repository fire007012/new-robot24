# YAML 配置指南

日期：2026-03-25  
范围：`config/master.yaml` 与 `config/joints.yaml`

---

## 1. 总览

本项目涉及两份 YAML 配置：

- `config/master.yaml`：用于生成 `master.dcf`（由 `dcfgen` 读取）。包含主站节点、SYNC 周期、PDO/SDO 配置等。
- `config/joints.yaml`：运行时配置，供程序读取；用于关节参数与每轴 PDO 验证开关。

二者作用不同：
- `master.yaml` 影响 **CANopen 对象字典配置**（通过 DCF 下发）。
- `joints.yaml` 影响 **本地计算与运行逻辑**（不写入设备）。

---

## 2. master.yaml 指南

### 2.1 文件结构

```yaml
options:
  dcf_path: .
  heartbeat_multiplier: 3.0

master:
  node_id: 127
  heartbeat_producer: 500
  sync_period: 10000
  sync_overflow: 0

joint_1:
  node_id: 1
  dcf: YiyouServo_V1.4.dcfgen.eds
  heartbeat_producer: 500
  rpdo:
    1:
      cob_id: 0x201
      transmission: 1
      mapping:
        - {index: 0x6040, sub_index: 0, bits: 16}
        - {index: 0x607A, sub_index: 0, bits: 32}
        - {index: 0x6060, sub_index: 0, bits: 8}
    2:
      enabled: false
  tpdo:
    1:
      cob_id: 0x181
      transmission: 1
      mapping:
        - {index: 0x6041, sub_index: 0, bits: 16}
        - {index: 0x6064, sub_index: 0, bits: 32}
        - {index: 0x6061, sub_index: 0, bits: 8}
    2:
      cob_id: 0x281
      transmission: 1
      mapping:
        - {index: 0x606C, sub_index: 0, bits: 32}
        - {index: 0x6077, sub_index: 0, bits: 16}
  sdo:
    - {index: 0x6060, sub_index: 0, value: 8}
```

### 2.2 关键字段说明

**options**
- `dcf_path`：`dcfgen` 生成 DCF 的输出路径。
- `heartbeat_multiplier`：与生成逻辑相关，一般保持默认。

**master**
- `node_id`：主站节点号（建议 127）。
- `heartbeat_producer`：主站心跳周期（ms）。
- `sync_period`：SYNC 周期（微秒）。例：10000 = 10ms。
- `sync_overflow`：SYNC 计数器溢出，通常为 0。

**joint_X**
- `node_id`：从站节点号，通常 1~6。
- `dcf`：EDS 文件路径。
- `heartbeat_producer`：从站心跳周期（ms）。

**rpdo/tpdo**
- `cob_id`：PDO COB-ID。
- `transmission`：传输类型，CSP 通常为 1（同步）。
- `mapping`：PDO 映射列表。
  - `index` / `sub_index` / `bits` 三元组。

**sdo**
- 启动时通过 SDO 写入固定值。
- 常用例：`0x6060` 模式设为 8（CSP）。

### 2.3 修改 SYNC 周期

1) 修改 `master.sync_period`（单位微秒）。
2) 重新运行 `dcfgen`：

```bash
cd ~/Robot24_catkin_ws/src/Eyou_Canopen_Master/config
dcfgen -S -r -d . master.yaml
```

3) 启动时指定新的 `master.dcf`。

---

## 3. joints.yaml 指南

### 3.1 最小示例

```yaml
joints:
  - name: joint_1
    counts_per_rev: 1000
    rated_torque_nm: 10
    position_lock_threshold: 15000
    max_velocity_for_clamp: 500000
    velocity_scale: 1.0
    torque_scale: 1.0
    canopen:
      node_id: 1
      verify_pdo_mapping: true
```

### 3.2 字段说明

`joints.yaml` 顶层 `canopen`（运行时生效）：
- `interface`：CAN 接口名（例如 `canable0`）。
- `master_node_id`：主站节点号。
- `loop_hz`：ROS 控制循环频率（Hz）。
- `auto_write_soft_limits_from_urdf`：是否在每次 `init` 成功后自动按 URDF 限位写入 `0x2003/0x607D`（默认 `false`）。

说明：
- `bitrate` / `sync_period_us` 不属于运行时 `joints.yaml` 生效字段。
- 波特率由系统 `ip link` 配置；SYNC 周期由 `master.dcf`（`master.yaml` 生成）决定。

- `name`：关节名称（仅标识）。
- `counts_per_rev`：每圈计数，用于位置换算。
- `counts_per_meter`：每米计数，仅用于 `prismatic` 关节软限位换算（可选；配置后必须 > 0）。
- `rated_torque_nm`：额定扭矩，Nm。
- `position_lock_threshold`：CSP 解锁阈值（ticks）。
- `ip_interpolation_period_ms`：IP 模式插补周期（ms，对应 SDO `0x60C2:01`，范围 1..255）。
- `ip_max_velocity`：IP 轨迹执行器每轴最大速度（rad/s）。
- `ip_max_acceleration`：IP 轨迹执行器每轴最大加速度（rad/s²）。
- `ip_max_jerk`：IP 轨迹执行器每轴最大 jerk（rad/s³）。
- `ip_goal_tolerance`：IP 轨迹执行器每轴终点容差（rad）。
- `max_velocity_for_clamp`：位置目标每周期限幅依据的最大速度（ticks/s，必须 > 0）。
- `velocity_scale`：速度缩放（来自设备定义）。
- `torque_scale`：力矩缩放（来自设备定义）。

**canopen 子项**
- `node_id`：该关节对应 CANopen 节点号。
- `verify_pdo_mapping`：是否在启动时验证 PDO 映射。

### 3.3 解析行为

- `joints` 为数组，顺序对应轴索引（0 开始，长度由 `joints` 数组决定）。
- 若 `canopen.node_id` 未设置，则按顺序分配 node_id = index + 1。
- `verify_pdo_mapping` 仅影响启动校验，不写入设备。
- `auto_write_soft_limits_from_urdf=true` 时，`init` 后会自动执行：
  - `SoftLimitState(0x2003:00)=0x4C494D54`
  - 若关节类型为 `revolute`：`0x607D:02/01=round(limit_rad/(2π)*counts_per_rev)`
  - 若关节类型为 `prismatic`：`0x607D:02/01=round(limit_meter*counts_per_meter)`
  - 若 URDF 中对应关节没有位置上下限（例如 `continuous`，或解析结果无 position limits），则改为写 `0x2003:00=0x00000000` 关闭该轴软限位，不写 `0x607D`。
  若 URDF 缺少对应关节，则本次 `init` 失败。
  若关节为 `prismatic` 但未配置 `counts_per_meter`，本次 `init` 失败。
- `ip_interpolation_period_ms` 未配置时，默认由 `loop_hz` 推导：
  `period_ms = round(1000 / loop_hz)`，并限制到 `1..255`。
- `max_velocity_for_clamp` 缺省时使用默认值 `500000.0`（ticks/s），
  显式配置值必须大于 0。

---

## 4. 常见修改场景

### 4.1 调整 PDO 映射

- 修改 `master.yaml` 中 `rpdo/tpdo` 映射。
- 重新 `dcfgen` 生成 `master.dcf`。
- 若开启 `verify_pdo_mapping`，会在启动时校验。

### 4.2 修改节点号

- `master.yaml` 内各 `joint_X.node_id`。
- `joints.yaml` 内 `canopen.node_id`。

注意：两者必须一致，否则启动后会出现 PDO 映射不匹配或控制轴错位。

### 4.3 修改应用周期

- 修改 `master.sync_period`（单位微秒）。
- 保证应用线程周期（`joints.yaml` 的 `canopen.loop_hz`，可被 launch 参数覆盖）与 SYNC 对齐。

---

## 5. 校验清单

- `master.yaml` 已更新并重新生成 `master.dcf`。
- `joints.yaml` 的 node_id 与 `master.yaml` 一致。
- SYNC 周期与应用线程周期一致。
- 如果开启 `verify_pdo_mapping`，确保 DCF 映射与设备一致。
