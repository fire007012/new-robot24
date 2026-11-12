# Robot24 独立管理服务

管理服务仅依赖 Python 3.10 标准库；ROS 健康探针在 source 部署安装环境后独立运行。入口 `manager.py --config /etc/robot24/manager.json`，客户端 `client.py` 不需要 ROS。协议和 Qt 对接详见 [管理协议 39](../docs/archive/qt/39_下位机管理协议与Qt对接.md)。

## 进程归属与安全

core 复用 `robot_bringup/full_system.launch.py`，始终 `enable_host:=false`；无视频时单独启动 `host_bridge.launch.py`，选视频时由现有 `video_stack.launch.py` 唯一拥有桥接、视频 IPC 和图像 RTSP。不再创建 controller_manager。视频档案复用 video_stack 的默认外部 MediaMTX 模式；现场应先部署配置所引用的 RTSP 服务，本管理服务不重复占用其8554端口。预检保留桥接9090、调试18080和所选视频IPC18081端口。视频可选表示流故障允许降级，桥接仍必需；视频 stack 整体故障导致桥接失败则整体失败。

运动预设通过控制桥 `motion_presets_v1` 选择，映射与上位机提示词见 [文档 51](../docs/51_三组预设与上位机对接提示词.md)。部署档案可设置布尔 `enable_servo`，管理服务将其传入整车 launch；本目录 hardware/mock 示例启用该项以支持机械臂遥控，旧部署省略时仍为 false。它只加载 Servo 软件/控制器，不使能电机；切换 arm 模式仍需显式请求。该字段由部署管理员离线维护，配置事务接口只读。

ready 要求控制器清单、两个硬件组件、实时 runtime、MoveIt 项目 action 及匹配桥接心跳 ACK 均实际通过。视频需要非空且全部 online 的源清单，并由 ffprobe 实际收到视频帧；探测预算4秒、单源最多2秒，缺少 ffprobe 或未收到帧时按必需/可选规则失败或降级。这不替代现场完整链路、质量和时延验收。runtime 的 Stopped、初始化、使能、分组、急停字段原样独立提供。管理服务从不调用 init/enable/resume。停止先通过实时 runtime、硬件生命周期和控制器清单确认：只有两个硬件均 unconfigured、所有控制器未运行、组均 Stopped 时，才以只读证据判定冷栈已停；其他情况必须成功调用 halt 并观察停止状态；无法确认时保留资源、返回 STOP_UNCONFIRMED，不能宣告停止成功。

默认使用部署目录只读JSON档案。部署可显式开启enable_config_transactions，提供受限字段的候选、校验、应用、回滚及审计；[配置事务41](../docs/archive/qt/41_配置维护事务与Qt对接.md)定义接口。基线和视频YAML仍必须位于原配置根目录，不接受网络文件路径。配置功能默认关闭，P0控制升级可独立发布。

进程资源持久记录 Linux boot ID、PID、启动时刻及独立进程组；同状态目录 flock 排除第二管理实例。写入先落盘 spawn intent，再启动并落盘身份。重启将未完成操作标为失败/结果未知，不重放。存在资源时阻止直接 start，要求受控 stop 后再启动。身份改变、未落盘身份或未知孤儿组均不误杀，要求人工对账。旧健康记录不能恢复 ready。日志和操作记录持久保留；操作日志接口保留前 1 MiB，进程原始日志位于 `<state_dir>/<operation_id>-<module>.process.log`，运维定期归档这些原始日志。

## 首次部署（本次未在真实机器执行）

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
