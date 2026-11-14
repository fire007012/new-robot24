# DM 可调速度运行脚本

**新版已加入自动识别ID，并统一到[电机测试脚本使用说明](电机测试脚本使用说明.md)。** 下列基本运行示例仍适用；默认节点和Master ID现在均为自动识别，可用 `--motor-id 7 --master-id 8` 固定目标。只扫描使用 `--scan-only`。

`run_dm_velocity.py` 自动启动仅包含一个 DM 电机的 ROS2 驱动，初始化、使能，以指定速度运行，到时发送零速、halt、disable、shutdown 并退出。Ctrl+C 也会执行停车流程。每半秒打印一次实际速度，异常时尝试停车，退出码非零。日志和本次配置保留在打印的 `/tmp/dm-velocity-*` 目录。

此前实测配置为 `can0`、1 Mbit/s、节点 **7**、反馈 Master ID **8**。新版默认自动扫描ID，+1 rad/s、运行1秒；可修改文件顶部 `DEFAULT_*`，或用命令行覆盖。正负号决定协议转向。

在本机当前终端执行：

```bash
cd /home/fire/ros2_ws_car/src
source /opt/ros/humble/setup.bash
source /tmp/dm-1rad-install/setup.bash
export ROS_DOMAIN_ID=79
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dm-velocity-ros-logs
python3 tools/run_dm_velocity.py --speed 1.0 --duration 1.0
```

例如反转0.5 rad/s、运行3秒：

```bash
python3 tools/run_dm_velocity.py --speed -0.5 --duration 3
```

更换接口或节点时，指定实际配置：

```bash
python3 tools/run_dm_velocity.py --interface can0 --motor-id 7 --master-id 8 --speed 1 --duration 2
```

`--speed-limit` 默认2 rad/s，限制指令和监测反馈；它是脚本的软件阈值，不是电机的硬件限速。速度单位均为 rad/s，时间单位为秒。反馈缺失、过期、故障、模式异常或速度超限会终止运动并尝试停车。

脚本需要已经编译的当前 `can_driver` 和已开启的 CAN 接口。`/tmp/dm-1rad-install` 是本次测试构建，重启清理后如不存在，可在工作区重新构建并 source 对应安装目录：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/dm-1rad-log build --base-paths robot_safety can_driver --build-base /tmp/dm-1rad-build --install-base /tmp/dm-1rad-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /tmp/dm-1rad-install/setup.bash
```

若 `can0` 关闭，先执行以下命令（仅在接口关闭时配置速率）：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

运行前停止控制同一电机的其他程序；脚本的启停服务会作用于它自己启动的单轴驱动。脚本不修改 ID、Master ID、量程、零点或保存参数，初始化会确认速度模式。当前电机 TIMEOUT=0，脚本的0.2秒命令超时依赖驱动仍运行；断线、进程被强杀或主机掉电时不能保证自动停车。
