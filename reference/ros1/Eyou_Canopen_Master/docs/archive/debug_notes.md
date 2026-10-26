# CANopen 主站调试操作手册

## 1. 环境准备

```bash
# 1) 启动 CAN 口
sudo ip link set canable0 up type can bitrate 1000000

# 2) 查看接口状态
ip -details -statistics link show canable0
```

## 2. 总线观测

```bash
# 实时抓包
candump canable0

# 仅看关键帧（SYNC + PDO + Heartbeat + EMCY）
candump canable0,080:7FF,180:7FF,200:7FF,280:7FF,700:7FF
```

预期：
- SYNC 周期约 10ms
- Node 1-6 心跳帧应可见（0x701~0x706）
- 运行时可见 `0x18x/0x28x` 与 `0x20x` PDO 帧

## 3. SDO 手工诊断（Node 1 示例）

```bash
# 读 statusword 0x6041
cansend canable0 601#40.41.60.00.00.00.00.00

# 设 mode_of_operation=8 (CSP)
cansend canable0 601#2F.60.60.00.08.00.00.00

# 发 shutdown (0x6040=0x0006)
cansend canable0 601#2B.40.60.00.06.00.00.00

# 发 enable operation (0x6040=0x000F)
cansend canable0 601#2B.40.60.00.0F.00.00.00
```

## 4. dcfgen 生成流程

```bash
# 从 master.yaml 生成主站 DCF 和各轴 concise bin
dcfgen -S -r -d /home/dianhua/robot_test/config \
       /home/dianhua/robot_test/config/master.yaml
```

说明：
- 当前仓库包含两份 EDS：
  - `YiyouServo_V1.4.eds`：原始文件
  - `YiyouServo_V1.4.dcfgen.eds`：为适配 dcf-tools 2.3.5 解析规则的兼容版本
- `master.yaml` 当前指向兼容版 EDS。

## 5. 本地编译与单测

```bash
# 状态机单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/cia402_state_machine.cpp \
    /home/dianhua/robot_test/test/test_state_machine.cpp \
    -o /tmp/test_state_machine && /tmp/test_state_machine

# shared_state 单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/shared_state.cpp \
    /home/dianhua/robot_test/test/test_shared_state.cpp \
    -o /tmp/test_shared_state && /tmp/test_shared_state

# 单位换算单测
g++ -std=c++17 -I/home/dianhua/robot_test/include \
    /home/dianhua/robot_test/src/canopen_robot_hw.cpp \
    /home/dianhua/robot_test/src/shared_state.cpp \
    /home/dianhua/robot_test/test/test_unit_conversion.cpp \
    -o /tmp/test_unit_conversion && /tmp/test_unit_conversion
```

## 6. 启动与联调流程（建议顺序）

```bash
# 1) 生成 DCF（如需）
dcfgen -S -r -d /home/dianhua/robot_test/config \
       /home/dianhua/robot_test/config/master.yaml

# 2) 编译
cmake -S /home/dianhua/robot_test -B /home/dianhua/robot_test/build
cmake --build /home/dianhua/robot_test/build -j

# 3) 启动主站（路径建议使用绝对路径）
/home/dianhua/robot_test/build/canopen_hw_node \
  --dcf /home/dianhua/robot_test/config/master.dcf \
  --joints /home/dianhua/robot_test/config/joints.yaml
```

验证点：
- 启动日志无 `master_dcf_path not found`
- 10ms 左右 SYNC 周期
- 0x701~0x706 心跳可见
- 运行后有 `0x18x/0x28x` 与 `0x20x` PDO

停止流程验证：
- Ctrl+C 后可观察到 Disable Operation -> Shutdown -> NMT Stop
- 轴进入 SwitchedOn / ReadyToSwitchOn 后再进入 Stopped

## 6. 故障排查清单

1. 看不到任何帧：优先检查终端电阻、电源、CAN_H/CAN_L 线序。
2. 有心跳但无 PDO：检查 NMT 是否进入 Operational、PDO COB-ID 是否被禁用(bit31)。
3. 反复 FAULT：检查 statusword 与 EMCY 码，确认复位节流参数是否过短。
4. dcfgen 报 EDS 格式错误：切换到 `YiyouServo_V1.4.dcfgen.eds` 并使用 `-S`。
5. 主站退出后行为异常：做“断 SYNC/强杀进程”实验并确认驱动器策略。

## 7. D1~D9 实验记录模板

```text
D1 SocketCAN 通讯验证
- 日期/负责人：
- 设备/环境：
- 步骤：
  1) candump canable0 观察心跳帧 0x701~0x706
- 结果：
- 结论/下一步：

D2 Lely 主站启动 + DCF 配置
- 日期/负责人：
- 设备/环境：
- 步骤：
  1) 启动 canopen_hw_node（记录 --dcf/--joints 路径）
  2) 抓包确认 NMT 进入 Operational
- 结果：
- 结论/下一步：

D3 状态机使能
- 日期/负责人：
- 步骤：
  1) 观察 statusword 进入 OPERATION_ENABLED
- 结果：
- 结论/下一步：

D4 位置控制
- 日期/负责人：
- 步骤：
  1) 写入不同 target_position，观察位置跟随
- 结果：
- 结论/下一步：

D5 故障复位
- 日期/负责人：
- 步骤：
  1) 人为触发故障，观察自动复位
- 结果：
- 结论/下一步：

D6 无扰切换
- 日期/负责人：
- 步骤：
  1) 故障恢复后无明显跳变
- 结果：
- 结论/下一步：

D7 多轴联调
- 日期/负责人：
- 步骤：
  1) 6轴同时使能与运动
- 结果：
- 结论/下一步：

D8 ROS 集成
- 日期/负责人：
- 步骤：
  1) controller_manager + JointTrajectoryController 正常工作
- 结果：
- 结论/下一步：

D9 耐久测试
- 日期/负责人：
- 步骤：
  1) 连续运行 3 小时，反复运动
- 结果：
- 结论/下一步：
```

## 8. 运行期内存约束

- 控制循环(`main.cpp` 的 `while` 循环)内禁止动态内存分配。
- 允许分配的阶段仅限初始化:
  - 读取配置
  - 创建主站/驱动对象
  - 容器 `reserve/resize`
- 代码审查重点:
  - 循环内禁止 `new/malloc`
  - 循环内禁止会触发扩容的容器操作(`push_back/emplace_back` 等)
  - 循环内避免构造大对象和字符串拼接
