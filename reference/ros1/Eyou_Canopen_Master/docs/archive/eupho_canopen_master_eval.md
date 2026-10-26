# 意优谐波关节 CANopen 主站方案评估与落地路径

版本: v0.1
日期: 2026-03-17

## 1. 结论概览

该方案在原型阶段是可落地的: 需求约束清晰、风险识别到位、状态机逻辑完整、总线负载评估充分。设计的核心优势是“工程可用性优先 + 复杂度可控”。在 USB-CAN + 非 RT 内核 + 10ms 周期的背景下, 方案避免了不必要的确定性追求, 这与原型目标匹配。

## 2. 主要优势

- 基于 EDS/手册的事实清单充分, 关键参数可复核, 避免魔数。
- PDO 映射紧凑且满足 CSP 控制最小闭环要求。
- 状态机以 statusword 驱动, 使用掩码匹配, 具备防御性。
- 复位流程采用节流和重试上限, 可避免复位风暴。
- “无扰切换”策略减少首次使能和故障恢复时的跳变风险。
- 对总线负载、SYNC 窗口、心跳策略给出了合理工程判断。
- 线程模型简洁明确, 互斥锁策略符合非 RT 环境的现实约束。

## 3. 关键风险与不确定点

### 3.1 SYNC 丢失后的驱动器行为
手册未明确 CSP 下 SYNC 丢失后的保护策略。若驱动器不会自动失能, 在主站异常退出时可能导致轴保持刚性锁定, 这在某些场景仍有风险。

建议: D1~D3 阶段必须做“断 SYNC 5s”实验, 明确行为并写入最终设计约束。

### 3.2 PDO COB-ID bit30 处理兼容性
EDS 默认带 bit30 的 COB-ID, 方案中明确覆盖为纯 11-bit 标准 COB-ID。需要确认 Lely dcfgen 写入后驱动器是否接受。

建议: D2 阶段抓包验证写入后的 COB-ID 是否生效, 如果设备拒绝, 需要切回 bit30=1 并验证 Lely 接收/发送是否正常。

### 3.3 从站不监控主站心跳
该决策在原型阶段可接受, 但需要通过实验验证“主站异常退出后驱动器不会继续运动”。

建议: 增加“强杀主站进程”测试, 观察驱动器行为。

### 3.4 速度/扭矩单位与量纲
0x606C/0x6077 的实际单位需再确认(分辨率/比例因驱动器而异)。方案里仅做基础换算, 仍存在量纲误差风险。

建议: 在手册或 EDS 中补齐单位说明; 若缺失, 通过实验标定。

### 3.5 fault reset 复位节奏
复位流程采用 50ms 保持 + 1 周期上升沿 + 1s 超时。该策略合理, 但需要确认驱动器是否要求更长的低电平保持。

建议: 初期保留参数可配置, 便于根据实机调整。

## 4. 方案可执行性评估

- 软硬件环境符合方案约束。
- 10ms CSP 在 USB-CAN 上可实现, 且总线负载余量充足。
- Lely-core 支持的 DCF 与 PDO 动态映射机制满足配置需求。
- 原型“3小时稳定运行”的目标在此设计下可达成。

## 5. 建议补充/修订项与解决方案(不改变总体架构)

### 5.1 SYNC 丢失/主站异常退出行为验证
解决方案:
- 增加专门测试用例(断 SYNC 5 秒与强杀主站进程), 观察驱动器是否自动失能或保持位置。
- 将观察结果写入最终方案约束与风险说明, 若不失能则增加外部安全回路或软件超时保护。

### 5.2 速度/扭矩单位与比例不确定
解决方案:
- 在 joints.yaml 为每轴加入 `velocity_scale` 与 `torque_scale`。
- 首次联调时用实验标定(已知速度/扭矩输入与反馈对比)确定比例, 并在文档中记录。

### 5.3 位置跳变保护
解决方案:
- 在状态机输出层引入 `position_jump_limit`(每周期最大 Δpos), 超限则限幅。
- 作为可选开关, 原型阶段默认关闭但预留参数。

### 5.4 心跳/启动超时参数边界
解决方案:
- 为 boot_timeout 与 heartbeat 超时添加上下限约束, 并在配置文件注释中说明推荐范围。
- 提供默认值: boot_timeout 5s, heartbeat_consumer 2s。

## 6. 小 commit 级落地路径(建议顺序)

该路径按“先可观察, 再可控制, 再可集成”的原则拆分, 每个 commit 都是可测试的最小增量。

1. `docs: add spec evaluation`
   - 新增评估文档, 固化风险/假设/测试点
   - 文件: docs/eupho_canopen_master_eval.md

2. `config: add initial YAML skeletons`
   - 新增 config 目录与空模板: master.yaml/joints.yaml
   - 只放结构和注释, 不含具体 PDO 配置
   - 文件: config/master.yaml, config/joints.yaml

3. `core: add cia402 defs and state enums`
   - 添加 cia402_defs.hpp, 定义掩码/控制字/状态枚举
   - 文件: include/canopen_hw/cia402_defs.hpp

4. `core: add pure cia402 state machine`
   - 实现纯逻辑状态机, 支持复位节流/无扰切换
   - 单元测试覆盖状态跃迁与复位节流
   - 文件: include/canopen_hw/cia402_state_machine.hpp, src/cia402_state_machine.cpp, test/test_state_machine.cpp

5. `core: add shared_state data model`
   - 定义 SharedAxisData/SharedState + 线程安全访问 API
   - 文件: include/canopen_hw/shared_state.hpp, src/shared_state.cpp

6. `canopen: add axis_driver skeleton`
   - 继承 BasicDriver, 实现 PDO 解析/回调框架, 先不接 ROS
   - 文件: include/canopen_hw/axis_driver.hpp, src/axis_driver.cpp

7. `canopen: add master bootstrap`
   - AsyncMaster/ev::Loop 启动流程 + 6 轴驱动实例
   - 文件: include/canopen_hw/canopen_master.hpp, src/canopen_master.cpp

8. `ros: add canopen_robot_hw skeleton`
   - RobotHW 框架 + JointInterface 注册 + 空读写
   - 文件: include/canopen_hw/canopen_robot_hw.hpp, src/canopen_robot_hw.cpp

9. `ros: wire shared_state into read/write`
   - read/write 与 SharedState 完成数据交换
   - 单元测试验证转换函数
   - 文件: src/canopen_robot_hw.cpp, test/test_unit_conversion.cpp

10. `app: add main loop and launch`
    - main.cpp 启动 Lely 线程与 controller_manager
    - launch 文件与参数加载
    - 文件: src/main.cpp, launch/canopen_hw.launch

11. `config: finalize master.yaml for 6 axes`
    - 完整 PDO 映射/COB-ID/heartbeat 参数
    - 配合 dcfgen 生成 master.dcf
    - 文件: config/master.yaml, config/master.dcf

12. `ops: add debug tooling notes`
    - 文档化 candump/cansend 操作与调试步骤
    - 文件: docs/debug_notes.md

## 7. 验收标准(原型阶段)

- 1 轴 D3: OPERATION_ENABLED 稳定保持 ≥ 10 分钟
- 1 轴 D4: 位置跟随正常, 无异常跳变
- 1 轴 D5: 断电/故障后自动复位成功
- 6 轴 D7: 同步使能与运动无丢帧
- 6 轴 D9: 连续运行 3 小时, 无人工干预
