# Robot24 独立管理服务

2026-11-20 停止恢复更新：预检按所选组检查 `controller_manager`、MoveIt 及启用 Servo 时的 `moveit_servo` 包，依赖缺失时不派生启动组。若部分模拟启动失败且 runtime 无法确认 halt，仅对 `mode=mock` 的 core/bridge 软件残留提供回退清理：核对 boot ID、PID 启动时刻、会话、用户、可执行文件，以及 runtime 参数中的 `simulation=true` 和本部署 journal，随后使用 pidfd 逐个停止已验证进程。未知成员、PID 复用、未落盘身份或控制器/MoveIt 残留均拒绝此回退；hardware 始终保留原 halt 确认要求。最后仍经管理事务清除已停止资源、读回 stopped 和恢复标记，不能直接删除状态文件。

当前双 CAN 约定（2026-11-20）：can_driver/private_interface=can0，CANopen/canopen_interface=can1；hardware 与 hardware-full 模板按此映射，旧 can0 共享接线记录仅为历史。机械臂启动需完整执行程序、Servo 就绪服务和 MoveIt 过滤状态输入，依赖与启动健康检查按所选组执行。

历史记录（2026-11-17，已由上述双 CAN 约定取代）：hardware 档案使用显式 `can_interfaces`（canopen_interface/private_interface），与 devices/group_devices 一致校验。presets 继续传入实际 launch、裁剪组件/轴/控制器并限制 runtime 操作范围。base/flippers 分别使用 can0；未经核验的同接口跨后端组合报 CAN_TOPOLOGY。全车接线使用独立 hardware-full 档案。受控停机迁移工具为 `deploy_can0.py`；完整流程、限制和证据见 [实施验收 54](../docs/54_can0分组实机档案实施与验收.md)。

2026-11-14 分组更新：预检/start 请求可传 `presets: ["base"]`、`["arm"]`、`["flippers"]` 或组合；机械臂展开为 arm+gripper。本次选择持久化在运行 profile 的 requested_presets，源档案不变。健康检查只要求所选组的软件服务/控制器及后端；不传时保持整车检查。hardware 档案可用 group_devices 显式映射四个组所用设备，预检据此只检查所选设备；未配置映射的旧档案仍保守检查全部 devices。设备映射的并集必须与 devices 一致。整车默认策略升级为 group-policy-v2；旧 v1 文件仍可显式选择。v2 默认没有待登记的组间机械关联，现场真实联锁应在 v2 策略显式配置，配置后未知/过期证据仍阻止对应目标组。

管理服务核心仅依赖 Python 3.10 标准库；模拟回退清理读取启动参数时使用部署已有的 `python3-yaml`，不导入 ROS。ROS 健康探针在 source 部署安装环境后独立运行。入口 `manager.py --config /etc/robot24/manager.json`，客户端 `client.py` 不需要 ROS。协议和 Qt 对接详见 [管理协议 39](../docs/archive/qt/39_下位机管理协议与Qt对接.md)。

## 进程归属与安全

core 复用 `robot_bringup/full_system.launch.py`，始终 `enable_host:=false`；无视频时单独启动 `host_bridge.launch.py`，选视频时由现有 `video_stack.launch.py` 唯一拥有桥接、视频 IPC 和图像 RTSP。不再创建 controller_manager。视频档案复用 video_stack 的默认外部 MediaMTX 模式；现场应先部署配置所引用的 RTSP 服务，本管理服务不重复占用其8554端口。预检保留桥接9090、调试18080和所选视频IPC18081端口。视频可选表示流故障允许降级，桥接仍必需；视频 stack 整体故障导致桥接失败则整体失败。

运动预设通过控制桥 `motion_presets_v1` 选择，映射与上位机提示词见 [文档 51](../docs/archive/51_三组预设与上位机对接提示词.md)。部署档案可设置布尔 `enable_servo`，管理服务将其传入整车 launch；本目录 hardware/mock 示例启用该项以支持机械臂遥控，旧部署省略时仍为 false。它只加载 Servo 软件/控制器，不使能电机；切换 arm 模式仍需显式请求。该字段由部署管理员离线维护，配置事务接口只读。

ready 要求所选组的控制器清单、所用硬件组件、实时 runtime 及匹配桥接心跳 ACK 均实际通过；选择机械臂时还要求 MoveIt 项目 action，不传选组的旧请求保持整车检查。视频需要非空且全部 online 的源清单，并由 ffprobe 实际收到视频帧；探测预算4秒、单源最多2秒，缺少 ffprobe 或未收到帧时按必需/可选规则失败或降级。这不替代现场完整链路、质量和时延验收。runtime 的 Stopped、初始化、使能、分组、急停字段原样独立提供。管理服务从不调用 init/enable/resume。停止先通过实时 runtime、硬件生命周期和控制器清单确认：只有两个硬件均 unconfigured、所有控制器未运行、组均 Stopped 时，才以只读证据判定冷栈已停；其他情况必须成功调用 halt 并观察停止状态；无法确认时保留资源、返回 STOP_UNCONFIRMED，不能宣告停止成功。

默认使用部署目录只读JSON档案。部署可显式开启enable_config_transactions，提供受限字段的候选、校验、应用、回滚及审计；[配置事务41](../docs/archive/qt/41_配置维护事务与Qt对接.md)定义接口。基线和视频YAML仍必须位于原配置根目录，不接受网络文件路径。配置功能默认关闭，P0控制升级可独立发布。

进程资源持久记录 Linux boot ID、PID、启动时刻及独立进程组；同状态目录 flock 排除第二管理实例。写入先落盘 spawn intent，再启动并落盘身份。重启将未完成操作标为失败/结果未知，不重放。存在资源时阻止直接 start，要求受控 stop 后再启动。身份改变、未落盘身份或未知孤儿组均不误杀，要求人工对账。旧健康记录不能恢复 ready。日志和操作记录持久保留；操作日志接口保留前 1 MiB，进程原始日志位于 `<state_dir>/<operation_id>-<module>.process.log`，运维定期归档这些原始日志。

## 首次部署与本地终端入口（更新：2026-11-25）

- **开发/非最终环境**：直接运行 `bash ~/ros2_ws_car/src/tools/run_manager.sh`，自动初始化 `~/.local/share/robot24` 并通过退避守护拉起本服务，无需依赖 `robot24-portable` 中间目录。
- **目标车端一键自启动**：通过 `bash ~/ros2_ws_car/src/tools/install_autostart.sh plan` 预览或 `sudo bash ~/ros2_ws_car/src/tools/install_autostart.sh install --with-can` 完成以下标准部署（详述见 [下位机管理服务与自启动部署说明](../docs/下位机管理服务与自启动部署说明.md)）：
- **自恢复补充**：`monitor()` 在运行中检测到必需模块失联时同步设置 `recovery_required=true`；主机重启导致 `boot_id` 变化且当前启动无存活 ROS runtime 时，`stop` 可安全清理上一轮旧 `boot_id` 残留资源而不触发 PID 复用死锁；`state.json` 损坏时显式拒绝启动以避免丢失资源归属。

1. 使用已验证的安装产物放置 `/opt/robot24/install`；核对安装搬迁和所有依赖。开发时可使用已构建 `/tmp/...-install/setup.bash`，生产不使用临时路径。管理按钮不运行 colcon、apt 或下载。
2. 创建 `robot24` 专用用户，加入现场设备所需组；将本目录顶层的全部 `.py` 文件 复制到 `/opt/robot24/manager/`。这些文件与 ROS 安装、配置均应由部署管理员拥有，运行用户只读。
3. 将 `config/manager.example.json` 安装为 `/etc/robot24/manager.json`，档案放在 `/etc/robot24/profiles/`，只复制需要的档案。根据最终产物设置一致的 `build_version`；在完成部署目录准备后运行 `python3 release_manifest.py --root /opt/robot24 --build-version robot24-release-1` 生成 release.json；管理预检校验协议版本、构建标识和每个清单文件 SHA256。发布目录须使用实际文件，勿引用源码或外部可变文件；配置、日志和令牌放在发布目录外。hardware 示例故意引用尚未登记的 commissioning；不能直接用于现场运行。
4. 以 `python3 -c 'import secrets; print(secrets.token_urlsafe(32))'` 生成令牌写入 `/etc/robot24/manager.token`，0600，属运行用户。不得提交或打印令牌到日志。给 Qt 安全配置相同令牌。设置实际管理 bind、控制桥 bridge_bind 和可从 Qt 访问的 control_endpoint.host；bridge_bind 默认127.0.0.1，现场远程接入时明确设置0.0.0.0；默认回环仅供本机测试。网络明文 TCP 仅限隔离可信网，跨网使用受认证 TLS 隧道；本版不内置 TLS。
5. 把 systemd 文件安装到 `/etc/systemd/system/robot24-manager.service`，确认 `/opt/robot24/install` 和配置可读、状态目录可写，再执行 `systemctl daemon-reload` 和 `systemctl enable --now robot24-manager`。以上真实安装需按项目现场规则执行，本任务只提交文件，不修改系统服务或防火墙。
6. Qt 连接 9091，启动后通过快照发现 9090 控制桥。升级/关闭管理服务前，先经管理 API stop，确认 succeeded/stopped 和无资源。unit 使用 `KillMode=process` 使管理进程崩溃后保留受管启动组用于对账；不可把 `systemctl stop` 当作整车停止。systemd 不自动重启 ROS/桥接，恢复必须经显式 stop/start，不能自动恢复运动。

## 本机 mock 验证

将 manager 示例配置复制到源码树外临时目录，设置 config_root/state_dir/token_file/setup；复制 mock.json 并使 build_version 匹配。独立运行：

```bash
python3 robot_manager/manager.py --config /tmp/robot-manager/manager.json
python3 robot_manager/client.py --token-file /tmp/robot-manager/token hello
python3 robot_manager/client.py --token-file /tmp/robot-manager/token start --profile mock --revision 1
python3 robot_manager/client.py --token-file /tmp/robot-manager/token operation
python3 robot_manager/client.py --token-file /tmp/robot-manager/token snapshot
python3 robot_manager/client.py --token-file /tmp/robot-manager/token stop
```

回归入口：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robot_manager/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 robot_manager/tests/integration.py \
  --setup /tmp/robot24-manager-overlay-install/setup.bash \
  --output /tmp/robot24-manager-validation --rounds 20
```

集成测试在 ROS_DOMAIN_ID=94、回环网络中运行真实 ROS mock/MoveIt/桥接，日志写指定临时目录；不要同时运行占用 19090/19092 或桥接调试端口 18080 的其他用例。集成测试包含管理服务强制终止后的重启对账，不连接硬件。

`systemd/robot24-manager.logrotate` 可安装为 `/etc/logrotate.d/robot24-manager`，对原始进程日志按日/10MiB 轮转并保留7份；API 操作日志独立使用固定行游标及截断语义。未部署此文件时原始进程日志须由运维自行归档。

附加边界验收可在新的输出目录执行：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot_manager/tests/integration.py \
  --setup /tmp/robot24-manager-overlay-install/setup.bash \
  --output /tmp/robot24-manager-video-validation --rounds 20 \
  --optional-video --restart-during-start
```

此模式故意提供空视频源，期望 `degraded` 且 core/bridge 健康；首轮在启动任务 health 阶段强制重启管理进程，旧任务必须报告结果未知，经 stop 对账后重新启动。第二轮杀死本次桥接进程组，要求管理查询可用并受控恢复。测试夹具的异常退出清理仅用于其硬编码的隔离 mock；生产服务始终坚持 halt 确认后才清理。

如需验证已运行状态的停止路径，另用新输出目录和 `--rounds 1 --exercise-running`。该标志仅在隔离 mock 测试脚本里显式建立运行许可，不属于管理 API，也不得用于实机。

当前控制端点版本为2，旧Qt不能直接运动。部署控制journal位于state_dir/control，保持持久权限0700；不要删除该目录规避未知结果或清除急停。发布清单control_protocol=2，管理version仍为1。只读部署检查：`python3 deployment_check.py --config /etc/robot24/manager.json`；安装/卸载回退与本轮证据见[交付与部署42](../docs/archive/qt/42_P0配置事务交付与验证.md)。

统一软件回归已包含本服务：`bash tools/validate_ros2.sh /tmp/robot24-ros2 full`。仅运行无 ROS 专项可用 `bash tools/validate_management.sh /tmp/robot24-units core`；输出前缀须位于源码树外。full 还运行控制 ROS mock、配置事务及管理进程/桥接异常恢复、Running 停止。最新结果见 [验证记录 43](../docs/archive/43_ROS2软件验证闭环记录.md)。

## 迁移现有上下位机联动部署

提供 `tools/portable_link.py pack`：停机后导出独立压缩包，保留令牌、现场基线和控制 journal，使用相对路径启动；目标机执行 `portable_link.py check` 只读检查文件、系统兼容性和动态库依赖。步骤与 Qt 地址配置见 [上下位机联动迁移说明](../docs/上下位机联动迁移说明.md)。

## ROS 通信运行依赖

发布默认使用 `rmw_cyclonedds_cpp`，依赖准备脚本包含 `ros-humble-rmw-cyclonedds-cpp`。启动树和只读健康探针共用 `config/cyclonedds.xml`，本机非组播网络允许最多 121 个自动参与者索引。文件位置按管理程序自身路径定位，支持独立部署迁移；显式 `RMW_IMPLEMENTATION`/`CYCLONEDDS_URI` 站点设置优先。Qt 仍通过 TCP 连接管理/控制端口。
