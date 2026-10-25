# CANopen 迁移执行记录

日期：2026-10-24。对应 [迁移计划](26_CANopen后端迁移计划.md)。本记录只确认仓库和离线检查结果；未进行设备写入、运动或正式链路切换。

## 已固定的软件输入

| 输入 | 提交或 SHA-256 |
| --- | --- |
| `Eyou_Canopen_Master` | `c890355b6fc2151c2ea9d3dd7a9d1bf21c9799e9` |
| `ros2_canopen` | `fef50e54b1c94c50e908e2c5d0b8888eed907e8d` |
| `can_driver` | `0e6ca8de063b9b924db6cd4ccc334dd4976c02fc` |
| 旧 `master.yaml` | `48add4bc46b404d12c11287f6b086dbcc2c0c80a0e4e71f65fcf29b173615a86` |
| 旧 `joints.yaml` | `fc3d0adcb33896da2fc3d4bb871267d2d4a120b17d43b4ca06f3981cfc08618d` |
| 原始 `YiyouServo_V1.4.eds` | `ea95a8045eb0b4b61bf462287c98cd42d3b655499ea8cdc88fc7156acf3c55d9` |
| 生成器用 `YiyouServo_V1.4.dcfgen.eds` | `cb4ba5ef63935f01841ff33e5125e984d35df8b76c5c8f75928134af78eb4683` |
| 旧全轴 `master.dcf` | `1e1bfa57e6fbc4e3a6c67faf95af8f8613bb3e96c4159f3e3821f6dc4d8ffb9d` |
| 旧肩轴 `master_arm_only.dcf` | `8bbd2bbefa7d31b974507e5b59ea126ea63a85a999c1cf6d0a93d38a27ba0e8b` |
| 旧摆臂 `master_flipper_4axis.dcf` | `84f159800252af7f6624c6a113ea8b9af4dc843f0c93801c4ce16592ea91eb49` |

`dcfgen.eds` 从原始 EDS 删除了部分设备身份默认值并修正了对象数值边界，不能把它当作实际固件身份证据。旧配置中六轴 `verify_pdo_mapping=false`；新 DCF 生成成功仍必须在现场逐项读回。

## 当前软件状态（2026-10-25）

| 项目 | 实现与验证范围 |
| --- | --- |
| 六轴及单轴配置 | 六轴和六份独立节点配置、SI 比例、PDO 与启动 SDO 的生成、严格比对及禁止持久化启动写入审计 |
| 硬件插件 | 采用 RobotSystem；按 URDF 声明导出接口，覆盖停止/失败/同模式重启、CSP/CSV/IP、epoch、有限值与范围门控 |
| 通信故障 | Lely 回调接入 NMT、心跳、EMCY、总线状态及 100 ms PDO 新鲜度；恢复不重新放行旧会话 |
| IP 肩轴 | 项目配置启用 0x60C1:01 与 0x607A 镜像，启动禁用 Homing；真实设备边沿仍需验收 |
| 受控维护 | 零点、偏置、URDF/显式软限位、保存、读回、部分失败回滚和不确定结果持久化拦截；默认不自动写限位 |
| 启动校验 | 身份、修订、序列号、固件字符串及 PDO 读回；现场基线缺失不放行 |
| 双后端 | 一个管理器、共享 Armed/Running/Fault 与 epoch、跨后端六轴 JTC、取消、控制器切换及双方故障停止 |
| 上层调用 | ROS2 统一生命周期和维护接口、摆臂 CSV/CSP 互斥及联动、带超时/行程限制的夹爪 CSP 速度积分 |
| 离线回归 | 已完成用户态虚拟 CAN 和双模拟后端链路回归；最终空前缀整套验收结果见本记录末节 |
| 现场工作 | 设备登记、真实运动、持久化掉电保持及实时性/总线负载验收尚未执行 |

交付与操作入口见 [软件交付与实机验收](28_CANopen软件交付与实机验收.md)。离线验证以两个子模块的基础提交及工作树修改为输入；发布后的锁定提交见本记录末节。

### 首轮产物校验值归档

以下是早期 `/tmp/canopen-migration-*` 产物记录，不能代替当前终验清单。早期 dcfgen 默认 retry_factor=3 产生的 `1F81=0x305` 已通过显式 retry_factor=0 恢复为旧 `0x5`，并恢复严格比对；当前产物以终验报告为准。

| 产物 | SHA-256 |
| --- | --- |
| `master.dcf` | `dd920b7576ffd02b752d0e91acac1a2ba4fe25ee54dfcd353f95476886c42971` |
| `left_front_arm_joint.bin` | `9832a712b6db71394c694e1fb4d7235795273d00afb8cd90a8b20baa18153d5b` |
| `right_front_arm_joint.bin` | `3254d25b9825283392c390f0aa24d1a51761785bd9e20253edc1fcf051eb355e` |
| `left_rear_arm_joint.bin` | `a0c8e94801bbfd5cf7e9f15151454f30e4fe62c7589d254c7942b6d757046cc1` |
| `right_rear_arm_joint.bin` | `3141ce3fbf5f6abce6997d909c93d65636e3bf832c2b6ca7b0d697f6d22210f3` |
| `shoulder_yaw_joint.bin` | `5f2c363ef2a3a7793fba7b601f743f9917e9b89e33fa02c5acc32b63740912b5` |
| `shoulder_pitch_joint.bin` | `674056c72126eb5c96bcef839c449bd8b8e24f6374aa64472ebe8f368ea767ef` |


## 现场基线待填

在设备持久化值、接线和固件登记完成前，不执行零点、软限位或保存参数写入。

| Node-ID | 关节 | 物理电机/接线标签 | 设备型号 | 固件 | `0x607C` | `0x2003` | `0x607D:01/02` | `0x1010:01` 状态 |
| ---: | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | left_front_arm_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |
| 2 | right_front_arm_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |
| 3 | left_rear_arm_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |
| 4 | right_rear_arm_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |
| 6 | shoulder_yaw_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |
| 5 | shoulder_pitch_joint | 待现场登记 | 待现场登记 | 待现场登记 | 待读回 | 待读回 | 待读回 | 待确认 |

还需记录总线波特率、终端电阻、现场 `canable0` 参数、旧 ROS1 可运行启动命令及日志、典型轨迹与故障抓包、回退提交和每轴真实方向/零位。之后依次执行计划的虚拟总线、真实单轴和整车验收矩阵；每项附设备身份、配置版本、抓包、步骤和结果。

## 历史增量：M1 模式切换与命令会话补丁（2026-10-25）

本次继续完成离线可执行部分，未连接设备，未写入零点、限位或持久化对象。`ros2_canopen` 仍以 `fef50e5` 为基线，以下为其工作树补丁，不代表已产生新的锁定提交。

- `set_operation_mode()` 在驱动已激活且已处于请求模式时返回成功，保留未激活时失败。
- `Cia402Data` 消费并清空本轮 start/stop 请求；校验接口及单写入者约束。停止请求撤销运行接口并请求 halt；模式切换成功后才登记新运行接口，失败保持命令会话关闭。
- 关闭或新建命令会话时将目标清为 NaN；写入要求单一运行接口、模式匹配且目标有限，避免停用后继续写目标及重用上次控制器目标。
- `RobotSystem` 保留完整的 `joint/interface` 键。硬件激活前清除所有轴会话，任一轴初始化失败则逐轴请求 halt；硬件停用先清除所有轴命令，再逐轴 halt，即使某轴失败也继续停止其余轴；多轴切换部分失败时撤销全部命令会话并逐轴请求 halt。
- 新增 `test_cia402_data.cpp` 六项模拟驱动回归：同模式启动、新目标/有限值门控、仅停用、失败不登记运行、CSP/CSV/IP 切换、非法/冲突接口以及 halt 失败。

这些模拟测试只检查硬件接口侧的软件行为。`halt_motor()` 请求 Quick Stop；重新选择相同模式不保证重新进入 Operation Enabled。控制器重启、Quick Stop 恢复、反馈确认、心跳/EMCY 后撤销会话及命令 epoch 仍需后续实现和验证，不能据此宣称 M1 或 M3 出口通过。上游内部缓存目标、肩轴双通道镜像以及真实模式反馈也不在本次模拟验证范围内。

构建使用 `/tmp/canopen-migration-{build,install,log}`，避免混入 ROS1 构建目录。首次构建尝试下载 Lely 源码时 DNS 失败；随后通过 `--packages-ignore lely_core_libraries` 使用 `/opt/ros/humble` 已安装的 Lely。

本次复现命令（先加载 `/opt/ros/humble/setup.bash`）：

```bash
colcon --log-base /tmp/canopen-migration-log build \
  --base-paths ros2_canopen robot_canopen_config \
  --packages-up-to canopen_ros2_control robot_canopen_config \
  --packages-ignore lely_core_libraries \
  --build-base /tmp/canopen-migration-build \
  --install-base /tmp/canopen-migration-install \
  --cmake-args -DBUILD_TESTING=ON
source /tmp/canopen-migration-install/setup.bash
colcon --log-base /tmp/canopen-migration-test-log test \
  --base-paths ros2_canopen robot_canopen_config \
  --packages-select canopen_ros2_control robot_canopen_config \
  --build-base /tmp/canopen-migration-build
colcon test-result --test-result-base /tmp/canopen-migration-build --verbose
```

本次验证结果：8 个包构建成功；`canopen_ros2_control` 的 1 项 CTest（内含 6 项命令会话 gtest）通过，`robot_canopen_config` 的插件加载及配置生成 2 项 CTest 通过（配置生成内含 4 项 Python 单测）；两仓库 `git diff --check` 通过。构建依赖包的其余测试仅编译，本次未执行，不能计入通过项。

额外手动运行插件测试时曾漏加载安装环境，导致插件不可见；加载完整隔离安装环境后复测通过。最初 Lely 下载失败留下的临时安装前缀会产生缺少 Lely `local_setup.bash` 的提示，实际使用系统 Lely；这是本次临时构建环境残留，不是设备通信验证。正式干净环境应从一开始忽略源码 Lely 包并记录系统依赖版本。

## 历史增量：Quick Stop 与驱动侧命令会话（2026-10-25）

本轮在上一节工作树上继续实现，尚未产生新的锁定提交：

- 去掉节点层“同模式立即成功”捷径。每次请求都经过 `Motor402::enterModeAndWait()`：先撤销旧会话，重置模式目标缓存；若先前 Quick Stop 尚待确认，先等待其状态反馈，再通过状态转换路径检查 Operation Enabled；只有模式反馈一致且会话未被并发停止撤销时才开放目标。
- `Motor402` 在 mode mutex 下管理命令 epoch 和 ready。halt、init、recover、enable、disable、shutdown 均撤销会话；轮询读到非 Operation Enabled 或模式反馈变化也会撤销，反馈恢复本身不会重新开放命令。目标写入在同一锁内核对 epoch、反馈模式、使能状态及数值，拒绝延迟到达的旧会话目标。
- 周期写入在门关闭时不再调用运动模式缓存目标的 write；保留初始化流程内部 Homing 的处理。目标辅助类明确初始化为“无目标”并拒绝 NaN/Inf，避免未初始化目标标志。
- `RobotSystem` 记录驱动 epoch；会话失效或目标写入被拒绝时返回硬件 ERROR，并撤销全部轴的命令、逐轴请求 halt。重新开启接口仍需显式 start，且接口目标清为 NaN。

验证范围仍是离线软件逻辑。状态判断依赖现有驱动轮询缓存，尚无反馈新鲜度期限；心跳超时、EMCY、驱动 deactivation/重连事件尚未完整接入 epoch。Quick Stop 请求与确认的时序、服务和控制器的串行仲裁、同模式恢复的真实状态转换以及肩轴 IP 双通道仍须虚拟总线与设备验证。不得据此将 M1/M3 或整车 Running 验收标记为完成。

本轮验证：驱动侧新增 4 项 gtest、硬件接口侧 9 项 gtest（原 6 项加 3 项故障/恢复回归）、插件加载 1 项 gtest，以及配置生成 4 项 Python 单测均通过。对应 4 项 CTest 全部通过；未执行驱动包的 ROS 组件加载测试，也未运行总线或运动测试。新增回归验证会话与缓存合同，不是完整 CiA402 设备仿真。

```bash
source /opt/ros/humble/setup.bash
source /tmp/canopen-migration-install/setup.bash
ctest --test-dir /tmp/canopen-migration-build/canopen_402_driver \
  -R '^test_command_session$' --output-on-failure
ctest --test-dir /tmp/canopen-migration-build/canopen_ros2_control --output-on-failure
ctest --test-dir /tmp/canopen-migration-build/robot_canopen_config --output-on-failure
```

下一步仍需验证 Quick Stop 状态时序与反馈新鲜度，接入心跳/EMCY/重连撤销，并建立不依赖真实设备的通信故障注入；之后再处理肩轴 IP 镜像及受控维护操作。现场持久化基线未补齐前继续禁止零点、限位及保存对象写入。

## 2026-10-25：软件离线收官，进入实机验收

计划 26 的软件实现与离线验证已完成；M0 现场登记和真实设备验收保留为未通过，不以模拟数据填充。操作入口为 [软件交付与实机验收](28_CANopen软件交付与实机验收.md)。

### 最终实现

- 全六轴与六份单轴配置可从仓库重建；主站 Node-ID 127、SYNC 5 ms、心跳 500 ms、旧 PDO 与 counts/rev 合同保留。`dcf_path: .` 按 bus 所在目录解析，bin 引用保持相对路径；新增目录迁移测试。启动 bin 审计拒绝零点、限位、保存/恢复参数对象写入。
- 选定 RobotSystem 作为软件交付插件；导出接口严格服从 URDF，覆盖同模式重启、仅停用、失败关闭、范围/非有限值检查、Quick Stop 待确认时序和命令 epoch。生命周期销毁、失败重配置及重复清理可离线验证。
- Lely 通信事件与实际 PDO 更新时间接入驱动门控。NMT、心跳、EMCY、总线状态和超时缺口即使在控制周期之间恢复，也使旧运输 epoch 失效。驱动 boot/通知状态显式初始化，boot 等待使用条件谓词；项目配置显式设置启动/SDO 超时。
- 肩轴开启 IP 目标镜像；项目禁用自动 Homing。托管模式拒绝原始运动 service/PDO topic 的第二写入者，模式变更由 controller_manager 资源切换承接。维护 SDO 复用同一主站，限制对象、单飞并有界等待，失败不以字典缓存冒充读回；固件支持分段字符串上传。
- 新增 `robot_safety` 和 `robot_hardware`，同一管理器管理两个硬件组件。共享 Stopped/Armed/Running/Fault、运行 epoch 和管理器周期统计；六轴 JTC 跨后端。任一后端失败撤销运行许可，重新就绪不自动 Running。恢复后先退掉残留控制器，最终保持 Armed，需新 resume/目标。
- ROS2 摆臂调用路径包含 CSV/CSP 串行切换、联动、超时归零和范围检查；夹爪采用 m/s 速度输入、受限 CSP 积分，统一旧配置接口冲突。使用统一机械臂 JTC，不新增 Ruckig Action 入口。
- 维护入口覆盖零点、偏置、显式/URDF 范围、限位使能、保存、读回、RAM 回滚、持久化结果不确定日志及显式恢复；失败的恢复自身也持久化 dirty。启动不自动写限位。身份、固件、序列号、PDO 和现场基线不符时不使能。

### 最终验证与证据

命令：`robot_hardware/scripts/validate_offline.sh /tmp/canopen-final`。最初使用空的独立前缀完成构建，并在最后修改测试/安装内容后复验。

| 检查 | 结果 |
| --- | --- |
| Humble 构建 | 11 个包通过，独立 build/install/log 前缀，无 ROS1 构建依赖 |
| CTest | 10 个包中的 28 个目标全部通过；包含已启用的上游核心、基础、代理、主站和 CiA402 单元/组件测试 |
| 测试汇总 | `colcon test-result`：164 tests，0 errors，0 failures，0 skipped；其中 Python 策略/联动的单个 CTest 实际执行 14 项 unittest，配置生成执行 5 项 unittest |
| 用户态虚拟 CAN | Lely CAN 帧交换；真实 AsyncMaster/LelyDriverBridge 与模拟 SDO 设备进行数值、分段固件字符串上传、丢回复超时、超时后继续读取 |
| 双后端进程回归 | 真实 Humble controller_manager + 两个模拟插件：跨后端六轴轨迹、取消、CSV 超时、CSV/CSP 切换、两侧分别注入故障、共同停止、显式恢复、维护和 shutdown |
| 真实插件未激活检查 | 完整 URDF 通过实际两个插件 on_init/接口核对；单轴真实插件保持 unconfigured；不存在 CAN 接口时 configure 两次均干净失败，进程存活 |
| 配置审计 | 27 个生成产物留 SHA；全轴/单轴 bin 一致、相对引用、旧 NMT assignment 严格比对、无持久化启动写入 |
| 安装一致性 | Python 安装内容与源码一致；私有驱动 SDK 库随包安装，RUNPATH 为 `$ORIGIN`，动态依赖解析通过 |
| 工作树检查 | 主仓库和两个修改过的子模块 `git diff --check` 通过 |

证据已保存为 [离线验收日志](artifacts/canopen_offline_validation.log) 和 [源码/产物/依赖清单](artifacts/canopen_offline_manifest.json)。清单覆盖 87 个源文件和 27 个生成产物；真实 ROS2 进程日志另保留在 `/tmp/canopen-final-hybrid.log`。报告记录工作树补丁对应的基础提交，不声称已生成或推送新提交。

构建中曾因修改基类头文件期间的派生模板库残留产生 ABI 不一致；完整清理与空前缀复验后组件装载/销毁通过。默认沙箱不允许 DDS 回环套接字，进程级离线测试使用允许本机通信的执行环境；用户态虚拟 CAN 无需内核 vcan、管理员建接口或真实设备。

### 仅剩现场验收

1. 填齐六轴实际身份/固件、接线、方向/零位、原始持久化参数、总线参数及旧可运行日志；现场 commissioning.yaml 不得使用模拟身份。
2. 按操作单单轴低速验证单位、方向、IP 镜像、Quick Stop/halt/release、模式反馈、断线重连及限位；再验证保存后的真实掉电/重启保持和回退读回。
3. 在真实六轴和双后端上执行轨迹、取消、任一侧故障的停止收敛，采集控制周期、SYNC、总线负载和错误，确定实测交付阈值。
4. 全部现场矩阵通过后正式切换并执行旧 ROS1 构建路径的移除收尾，保留回退提交与设备基线。

本次没有连接真实 CAN、电机使能、运动或设备参数写入。ROS1 旧包按计划保留，正式 ROS2 新入口已经独立。


## 2026-10-25：发布子模块并更新主仓库

已将离线验收覆盖的子模块修改提交并推送到项目远程：

| 仓库 | 远程分支 | 锁定提交 |
| --- | --- | --- |
| `fire007012/ros2_canopen` | `origin/humble` | `78242f5c81809bac173051c28fe8154719f10d30` |
| `fire007012/can_driver` | `origin/main` | `0ba4e613e16fc233677db30ab7a07df1830e55ad` |

主仓库同步这两个 gitlink，并纳入 `robot_safety`、`robot_canopen_config`、`robot_hardware`、迁移文档和离线验收证据。子模块提交使用的文件内容与离线清单完全一致；本次发布只增加 Git 提交及文档版本记录，没有修改已验证的运行代码。

验收清单中的 `source_base_commits` 继续保留当时的基础输入；修改文件 SHA-256 对应上述新提交中的文件内容。现场验收仍按文档 28 执行。
