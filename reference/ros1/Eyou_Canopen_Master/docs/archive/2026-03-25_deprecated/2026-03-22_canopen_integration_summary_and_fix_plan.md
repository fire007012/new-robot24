# CANopen 实机联调问题总结与修复方案（2026-03-22）

## 1. 目标与结论

本轮联调目标是让 Node 5 在 CSP 模式下从主站稳定进入可运行态，并接受位置命令。

当前结论：

1. 主链路已打通（可进入 `OperationEnabled`，控制字可稳定下发 `0x000F`，模式字可稳定下发 `0x08`）。
2. 已确认过多个独立问题并分阶段修复（权限、服务类型、controller_manager 死锁、DCF 映射不匹配、额外 COB-ID 噪声）。
3. 现场仍存在“目标过激时抽动 + 偶发 heartbeat 抖动 + 故障灯触发”的风险，需要补充运行期限幅和恢复策略。

---

## 2. 问题时间线（按根因分类）

### 2.1 启动权限问题

现象：

- `CanController: Operation not permitted`

根因：

- `canopen_hw_ros_node` 无 `cap_net_raw/cap_net_admin` 能力，无法打开 SocketCAN。

处理：

- 对可执行文件补 capability。

### 2.2 SetMode 服务类型不匹配

现象：

- `Unable to load type [canopen_hw/SetMode]`

根因：

- 节点代码使用旧服务命名空间，实际包名是 `Eyou_Canopen_Master`。

处理：

- 服务头文件与模板参数改为 `Eyou_Canopen_Master::SetMode`。

### 2.3 controller_manager 调用阻塞（死锁）

现象：

- `list_controllers/load_controller` 长时间卡住。

根因：

- 主线程在 `ros::spinOnce()` 内执行 `loadController` 回调并等待初始化，而 `cm.update()` 又在主循环里等待 `spinOnce` 返回，形成同线程互锁。

处理：

- `canopen_hw_ros_node` 改用 `ros::AsyncSpinner(2)`，移除循环内 `spinOnce()`。
- 主循环与 lifecycle 服务回调增加互斥保护，避免并发竞态。

### 2.4 DCF 映射不匹配导致主站持续发 0

现象：

- `0x205` 长期 `00000000000000`
- `statusword` 长期停在 `SwitchOnDisabled`

根因：

- 活跃 `master.dcf` 与代码预期 PDO 映射不一致（使用了 `0x2000/0x2200` 风格映射，缺少代码依赖的 `0x6040/0x607A/0x6060` 链路）。

处理：

- 切换到带 `5E00/5A00` 远端 PDO 映射的 DCF（`master.dcf.bak_diag`）进行验证。

### 2.5 额外 COB-ID 0x405 噪声

现象：

- 主站持续发送 `0x405` 报文。

根因：

- DCF 中 TPDO3/TPDO4 COB-ID 处于启用态。

处理：

- 在 `master.dcf.bak_diag` 中禁用：
  - `1801sub1 = 0x80000305`
  - `1802sub1 = 0x80000405`

结果：

- `0x405` 噪声消失。

### 2.6 CSP 启动阶段 `is_operational` 卡住

现象：

- 轴状态显示 `not operational`，命令链路难收敛。

根因（历史逻辑）：

- `WriteToSharedState()` 只有 `all_operational=true` 才写命令，导致 `ros_target` 无法更新，CSP 锁定无法解开（环依赖）。

处理：

- 修改为：始终写入位置与模式；当 `all_operational=false` 时强制速度/力矩为 0。

---

## 3. 关键修复（代码）

### 3.1 已提交修复

- `5ba50b7`  
  `fix(ros): unblock controller_manager load path and startup command gating`

  主要内容：
  1. `SetMode` 命名空间修正。  
  2. `AsyncSpinner` 解除 controller_manager 死锁。  
  3. 启动命令门控修正（未 operational 时仍可写位置，速度/力矩归零）。

- `0e095e4`  
  `cia402: auto-align CSP target on large startup offset`

  主要内容：
  1. CSP 锁定阶段若 `|ros_target - actual|` 超阈值，自动重基准到 `actual`。  
  2. 补充状态机回归测试。

### 3.2 未提交但已现场应用的配置修复

- `config/master.dcf.bak_diag`：禁用 `0x305/0x405`（`1801sub1/1802sub1`）。

---

## 4. 你给出的 CSP 分析评估

你提供的分析方向总体正确，尤其以下点是准确的：

1. `is_operational` 取决于 `position_locked_` 是否释放。  
2. 在 CSP 下，解锁条件与 `|ros_target-actual_position| <= threshold` 相关。  
3. `mode_display != target_mode` 会影响进入可运行链路（在 ReadyToSwitchOn 分支无法发 `EnableOperation`）。

需要修正/补充的点：

1. “自动重基准后要下一帧才解锁”这一点在当前实现里不完全成立。  
   现版本在同一帧内先执行重基准，再立即再次比较阈值，因此可在同一 `Update()` 内解锁。  
2. “上层未 set_ros_target 会永久卡死”在旧门控下成立；在当前门控修复后，位置目标会持续刷新，不再构成永久闭环卡死条件。

建议结论：

- 你的根因框架可作为文档主线；需要按当前代码语义更新“解锁时机”和“是否永久卡死”的描述。

---

## 5. 集成测试现状（截至 2026-03-22）

已确认：

1. `controller_manager` 服务不再卡死。  
2. 轨道链路可进入 `operational`，`/diagnostics` 可见 `is_operational=True`。  
3. 抓包可见：
   - `0x205`：`0F 00 ... ... ... 08`（使能 + CSP）
   - `0x185`：模式显示 `0x08` 且状态字进入运行组合。

剩余风险：

1. 大步进目标会引发机械抽动。  
2. 偶发 heartbeat 状态变化与 CAN 写取消（`Operation canceled`）后自动恢复，表明现场稳定性仍需增强。

---

## 6. 后续修复方案（建议）

### 6.1 短期（现场可立即执行）

1. 强制小步进命令策略（限制单次位置变化）。
2. 出现故障灯时执行手动故障复位控制字序列（`0x0000 -> 0x0080 -> 0x0006 -> 0x000F`）。
3. 保持抓包窗口常开，记录 `0x205/0x185/0x705`。

### 6.2 中期（代码增强）

1. 在 `CanopenRobotHw::WriteToSharedState` 增加位置斜坡限幅（每周期最大增量）。
2. 在 recover 之后增加一次“启动预置帧”发送（显式写 `6040/6060`，避免缓冲初值全零窗口）。
3. 在 ROS 侧新增恢复服务（显式触发 fault reset 序列）。

### 6.3 长期（工程化）

1. 固化 DCF 生成流程，防止 `master.dcf` 与代码期望漂移。  
2. 把现场稳定性判据纳入回归（heartbeat 抖动率、写帧失败率、恢复时间）。

---

## 7. 建议的验收判据（最终版）

1. 启动 30s 内：`/diagnostics` 显示 `is_operational=True` 且 `is_fault=False`。  
2. 发位置命令后：`0x205` 中目标位置字段发生变化，电机平滑响应。  
3. 退出阶段：可观测到预期控制字回落序列，无长时间心跳丢失。  
4. 连续 10 次小步进动作，无抽动触发故障灯。

