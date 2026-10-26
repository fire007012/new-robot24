# PDO 映射验证 — 最终方案

---

## 一、目标

启动时验证从站 PDO 映射是否与 DCF 一致。不一致则报错，该轴不进入 Operational。不做降级控制。

---

## 二、配置

```yaml
joints:
  - name: axis_1
    canopen:
      node_id: 2
      verify_pdo_mapping: true

  - name: axis_2
    canopen:
      node_id: 3
      verify_pdo_mapping: false
```

一个开关，没有其他参数。

---

## 三、流程

```
1. 加载 joints.yaml
2. 加载 DCF，初始化 Lely Master
3. 从站上线 → Lely OnConfig() 自动下发
4. OnBoot(id) 触发（或超时）
   │
   ├─ 超时 → ERROR，该轴标记失败
   ├─ OnConfig 报失败 → ERROR，该轴标记失败
   ├─ verify_pdo_mapping=false → 跳过，正常启动
   └─ verify_pdo_mapping=true →
        Reader 读回实际映射
        Diff 对比 DCF 期望
        ├─ 一致 → 正常启动
        └─ 不一致 → ERROR（含差异明细），该轴不进入 OP
5. 正常轴进入 Operational，失败轴停在 Pre-Operational
```

---

## 四、验证范围

读取：

| 对象 | 含义 |
|------|------|
| 0x1400~0x1403 sub1 | RPDO COB-ID |
| 0x1600~0x1603 sub0~subN | RPDO 映射 |
| 0x1800~0x1803 sub1 | TPDO COB-ID |
| 0x1A00~0x1A03 sub0~subN | TPDO 映射 |

对比：

| 项目 | 比较 |
|------|------|
| COB-ID | ✅ |
| 映射数量 | ✅ |
| 映射条目（index:subindex:bitlength） | ✅ |
| 传输类型、inhibit time、event timer | ❌ |

判定：任何一项不一致或读取失败 → 该轴验证失败。

---

## 五、模块

### PdoMappingReader

- 通过 SDO 读回从站实际 PDO 配置
- 输入：node_id
- 输出：该从站的 COB-ID + 映射列表
- 任何 SDO 读取失败 → 返回错误

### PdoMappingDiff

- 对比 DCF 期望与 Reader 读回结果
- 输出：是否一致 + 差异明细
- 差异明细用于日志输出

---

## 六、日志

```
[INFO]  轴1 (node 2): PDO 映射验证通过
[ERROR] 轴2 (node 3): TPDO1 映射不一致
          期望: [6041:00/16, 6064:00/32]
          实际: [6041:00/16, 606C:00/32]
[ERROR] 轴2 (node 3): PDO 验证失败，该轴不进入 Operational
[INFO]  轴3 (node 4): verify_pdo_mapping=false，跳过验证
```

---

## 七、不做的事情

- 不自己写入 PDO 映射
- 不重试 OnConfig
- 不降级到 SDO 轮询
- 不切换操作模式
- 不因单轴失败影响其他轴

---

## 八、实现任务

| 序号 | 任务 | 备注 |
|------|------|------|
| 0 | 核实 Lely API 签名 | 最先做 |
| 1 | PdoMappingReader | SDO 读取 |
| 2 | PdoMappingDiff | 对比逻辑 |
| 3 | OnBoot 接入验证 + 超时兜底 | 集成 |
| 4 | joints.yaml 解析 verify_pdo_mapping | 配置 |
| 5 | 日志 | 差异明细输出 |

顺序：0 → 4 → 1 → 2 → 3 → 5

---

## 九、验收标准

1. verify_pdo_mapping=false 时行为与现有系统完全一致
2. 映射一致时正常启动，额外耗时不超过 2 秒
3. 映射不一致时 ERROR 日志输出具体差异，该轴不进入 Operational
4. 单轴失败不影响其他轴

