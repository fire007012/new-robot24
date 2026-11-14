"""Shared single-motor velocity runner; protocol wrappers expose editable defaults."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from motor_discovery import scan_can, scan_ecb, select_target


def integer(value):
    return int(value, 0)


def arguments(protocol, defaults, argv=None):
    parser = argparse.ArgumentParser(description=f"{protocol} 单电机测试：只读识别ID，再以指定速度运行并停车")
    parser.add_argument("--speed", type=float, default=defaults["speed"], help="rad/s，负数反转")
    parser.add_argument("--duration", type=float, default=defaults["duration"], help="运行秒数")
    parser.add_argument("--motor-id", type=integer, default=defaults["motor_id"], help="十进制或0x十六进制；不填自动扫描")
    parser.add_argument("--device", "--interface", dest="device", default=defaults["device"], help="CAN接口，ECB使用ecb://auto或ecb://IP")
    parser.add_argument("--master-id", type=integer, default=defaults.get("master_id"), help="DM反馈ID；不填自动读取")
    parser.add_argument("--position-scale", type=float, default=defaults["position_scale"], help="SI位置/raw，或>=2的PPR")
    parser.add_argument("--velocity-scale", type=float, default=defaults["velocity_scale"], help="SI命令速度/raw，或>=2的PPR")
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=defaults["direction"])
    parser.add_argument("--speed-limit", type=float, default=2.0, help="指令和反馈速度上限rad/s")
    parser.add_argument("--scan-start", type=integer, default=0 if protocol == "DM" else 1)
    parser.add_argument("--scan-end", type=integer, default=15 if protocol == "DM" else 255)
    parser.add_argument("--scan-timeout", type=float, default=0.2, help="每节点每轮监听秒数")
    parser.add_argument("--scan-retries", type=int, default=2)
    parser.add_argument("--discovery-timeout", type=float, default=30, help="ECB SDK发现超时秒数")
    parser.add_argument("--feedback-timeout", type=float, default=0.5, help="状态话题超时秒数")
    parser.add_argument("--startup-timeout", type=float, default=5.0, help="等待首组有效反馈秒数")
    parser.add_argument("--stop-tolerance", type=float, default=0.05, help="停止判定速度阈值rad/s")
    parser.add_argument("--allow-sdk-cache", action="store_true", help="ECB：接受SDK缓存缺少完整新鲜度验证的限制")
    parser.add_argument("--scan-only", action="store_true", help="只读发现并列出全部结果，不运动")
    parser.add_argument("--dry-run", action="store_true", help="仅打印配置，不访问硬件；须指定ID，DM还须指定Master ID")
    args = parser.parse_args(argv)
    args.protocol = protocol
    positive = (args.duration, args.speed_limit, args.scan_timeout, args.discovery_timeout,
                args.feedback_timeout, args.startup_timeout, args.stop_tolerance,
                args.position_scale, args.velocity_scale)
    if not all(math.isfinite(v) and v > 0 for v in positive) or not math.isfinite(args.speed):
        parser.error("时间、比例和阈值须为有限正数；速度须为有限数")
    if abs(args.speed) > args.speed_limit:
        parser.error("目标速度超过 --speed-limit")
    minimum, maximum = (0, 15) if protocol == "DM" else (1, 255)
    if not minimum <= args.scan_start <= args.scan_end <= maximum:
        parser.error(f"扫描范围须在 {minimum}–{maximum} 内，起点不能超过终点")
    if args.motor_id is not None and not minimum <= args.motor_id <= maximum:
        parser.error(f"motor-id 范围 {minimum}–{maximum}；MT填节点号1，不填帧ID 0x141")
    if args.scan_retries < 1:
        parser.error("scan-retries 须>=1")
    if args.master_id is not None and (protocol != "DM" or not 0 <= args.master_id <= 0x7FF):
        parser.error("master-id 仅供DM使用，范围0–2047")
    if protocol == "DM" and (args.position_scale != 1e-4 or args.velocity_scale != 1e-4):
        parser.error("DM 的 position/velocity scale 必须为0.0001")
    if protocol == "MT" and (args.position_scale >= 2 or args.velocity_scale >= 2):
        parser.error("MT 使用角度单位换算系数，不使用编码器PPR")
    if protocol == "ECB" and not args.device.startswith("ecb://"):
        parser.error("ECB device须为ecb://auto或ecb://IP")
    if protocol != "ECB" and args.device.startswith("ecb://"):
        parser.error("CAN电机须指定can0等接口")
    for field in ("position_scale", "velocity_scale"):
        value = getattr(args, field)
        if value >= 2:
            setattr(args, field, 2 * math.pi / value)
    if args.dry_run and (args.scan_only or args.motor_id is None or (protocol == "DM" and args.master_id is None)):
        parser.error("dry-run不能与scan-only同时使用，须指定motor-id；DM还须指定master-id")
    if args.dry_run and protocol == "ECB" and args.device == "ecb://auto":
        parser.error("ECB dry-run须指定ecb://IP")
    # 提前拒绝舍入为0或超出协议整数表示范围的运动指令。
    raw = abs(args.speed / args.velocity_scale)
    if raw > 2147483647 or (args.speed != 0 and raw < 0.5):
        parser.error("速度换算后超出int32或小于半个原始单位；请检查比例/速度")
    return args


def make_config(args):
    axis = {"name": args.protocol.lower() + "_motor", "motor_id": args.motor_id,
            "protocol": args.protocol, "can_device": args.device, "control_mode": "velocity",
            "position_scale": args.position_scale, "velocity_scale": args.velocity_scale,
            "direction_sign": float(args.direction)}
    if args.protocol == "DM":
        axis["master_id"] = args.master_id
    return {"can_driver_node": {"control_frequency": 50.0, "motor_query_hz": 50.0,
                               "motor_state_period_sec": 0.02, "direct_cmd_timeout_sec": 0.2,
                               "debug_bypass_ros_control": False, "pp_fast_write_enabled": False,
                               "joints": [axis]}}


def main(protocol, defaults):
    args = arguments(protocol, defaults)
    try:
        if args.dry_run:
            print(json.dumps(make_config(args), indent=2))
            return 0
        if protocol == "ECB":
            candidates = scan_ecb(args.device, args.discovery_timeout)
        else:
            nodes = [args.motor_id] if args.motor_id is not None else list(range(args.scan_start, args.scan_end + 1))
            candidates = scan_can(protocol, args.device, nodes, args.scan_timeout, args.scan_retries)
        print("发现结果: " + json.dumps(candidates, ensure_ascii=False, indent=2), flush=True)
        if args.scan_only:
            return 0 if candidates else 1
        target = select_target(candidates, args.motor_id, args.master_id)
        args.motor_id, args.device = target["motor_id"], target["device"]
        if protocol == "DM":
            args.master_id = target["master_id"]
        if target.get("fault_code", 0):
            raise RuntimeError("MT 已报告故障；拒绝使能")
        if protocol == "ECB" and not args.allow_sdk_cache:
            raise RuntimeError("ID已识别；ECB当前仅提供SDK缓存，运行需显式加 --allow-sdk-cache；详见MD")
        return run(args)
    except KeyboardInterrupt:
        print("已取消发现，未发送运动命令。")
        return 130
    except Exception as error:
        print("错误: " + str(error), flush=True)
        return 1


def run(args):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from can_driver.msg import MotorState
    from can_driver.srv import Init, Shutdown
    from std_msgs.msg import Float64
    from std_srvs.srv import Trigger

    namespace = "/" + args.protocol.lower() + "_velocity_demo_" + str(os.getpid())
    joint = args.protocol.lower() + "_motor"
    executable = Path(get_package_prefix("can_driver")) / "lib/can_driver/can_driver_node"
    config = make_config(args)
    rclpy.init()
    node = rclpy.create_node(args.protocol.lower() + "_velocity_client")
    latest = None
    received = 0.0
    clients = {}

    def state(message):
        nonlocal latest, received
        if message.name == joint and message.motor_id == args.motor_id:
            latest, received = message, time.monotonic()

    subscription = node.create_subscription(MotorState, namespace + "/motor_states", state, 10)
    publisher = node.create_publisher(Float64, namespace + "/motor/" + joint + "/cmd_velocity", 1)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, end - time.monotonic())))

    def call(name, service=Trigger, request=None):
        if name not in clients:
            clients[name] = node.create_client(service, namespace + "/" + name)
        client = clients[name]
        if not client.wait_for_service(timeout_sec=4.0):
            raise RuntimeError("服务未就绪: " + name)
        future = client.call_async(request if request is not None else service.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            spin(0.01)
        if not future.done():
            raise RuntimeError("服务超时: " + name)
        response = future.result()
        if not response.success:
            raise RuntimeError(name + ": " + response.message)

    def healthy():
        if latest is None or time.monotonic() - received > args.feedback_timeout:
            raise RuntimeError("未收到新鲜反馈")
        if latest.fault:
            raise RuntimeError("电机故障")
        if args.protocol == "ECB":
            # 当前后端只发布 SDK 缓存，缺少共享反馈有效性，须显式接受该限制。
            if not args.allow_sdk_cache:
                raise RuntimeError("ECB 缺少完整有效性标志；需 --allow-sdk-cache 显式接受 SDK 缓存监测")
        elif args.protocol == "MT":
            if not all((latest.feedback_fresh, latest.position_valid, latest.velocity_valid)):
                raise RuntimeError("MT 位置/速度反馈无效或过期")
            # MT 无独立使能/模式读回；不把软件状态当作硬件确认。
        else:
            if not all((latest.feedback_fresh, latest.position_valid, latest.velocity_valid,
                        latest.status_valid, latest.mode_valid)):
                raise RuntimeError("反馈无效或过期")
            if latest.mode != MotorState.MODE_VELOCITY:
                raise RuntimeError("电机未处于速度模式")
        if not math.isfinite(velocity()) or abs(velocity()) > args.speed_limit:
            raise RuntimeError("实际速度超限或无效")

    def velocity():
        # MT A2 指令为0.01 deg/s，反馈为1 deg/s；不能共用一个换算系数。
        if args.protocol == "MT":
            return latest.velocity * args.velocity_scale * 100 * args.direction
        return latest.joint_velocity

    def command(speed, seconds, require_enabled=False):
        end = time.monotonic() + seconds
        next_print = time.monotonic()
        while time.monotonic() < end:
            publisher.publish(Float64(data=speed))
            spin(0.02)  # 50 Hz；不能只发送一次，否则驱动会因命令超时停车。
            healthy()
            if require_enabled and args.protocol != "MT" and not latest.enabled:
                raise RuntimeError("运动中电机意外失能")
            if require_enabled and time.monotonic() >= next_print:
                print(f"目标 {speed:+.3f} rad/s，实际 {velocity():+.4f} rad/s", flush=True)
                next_print = time.monotonic() + 0.5

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    old_handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    failed = False
    driver = None
    # 日志保留在 /tmp，退出后可查看。
    folder = Path(tempfile.mkdtemp(prefix=args.protocol.lower() + "-velocity-"))
    (folder / "config.yaml").write_text(json.dumps(config, indent=2))
    print(f"{args.protocol} 节点 {args.motor_id}，设备 {args.device}；日志: {folder / 'driver.log'}", flush=True)
    try:
        with (folder / "driver.log").open("w") as log:
            driver = subprocess.Popen([str(executable), "--ros-args", "-r", "__ns:=" + namespace,
                                       "-p", "config_file:=" + str(folder / "config.yaml")],
                                      stdout=log, stderr=subprocess.STDOUT)
        call("init", Init, Init.Request(device=args.device, loopback=False))
        deadline = time.monotonic() + args.startup_timeout
        while True:
            spin(0.05)
            try:
                healthy()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
        if latest.enabled or abs(velocity()) > args.stop_tolerance:
            raise RuntimeError("初始电机未失能或尚未停稳")
        call("resume")
        command(0.0, 0.2)
        call("enable")
        call("resume")
        command(0.0, 0.3, require_enabled=True)
        command(args.speed, args.duration, require_enabled=True)
    except KeyboardInterrupt:
        print("收到中断，正在停车。", flush=True)
    except Exception as error:
        failed = True
        print("运行失败: " + str(error), flush=True)
    finally:
        for sig in old_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if driver is not None:
            try:
                command(0.0, 0.6)
            except Exception as error:
                print("零速等待: " + str(error), flush=True)
            # 各步骤独立尝试，前一步失败仍继续失能和关闭。
            for name in ("halt", "disable"):
                try:
                    call(name)
                except Exception as error:
                    failed = True
                    print("停车失败: " + str(error), flush=True)
            try:
                spin(0.3)
                healthy()
                if (args.protocol != "MT" and latest.enabled) or abs(velocity()) > args.stop_tolerance:
                    raise RuntimeError("未确认停稳和失能")
                if args.protocol in ("MT", "ECB"):
                    print(f"已发送停止/失能，反馈速度 {velocity():+.4f} rad/s；硬件状态验证限制见说明", flush=True)
                else:
                    print(f"已停稳并失能，反馈速度 {velocity():+.4f} rad/s", flush=True)
            except Exception as error:
                failed = True
                print("停止确认失败: " + str(error), flush=True)
            try:
                call("shutdown", Shutdown, Shutdown.Request(force=False))
            except Exception as error:
                failed = True
                print("关闭失败: " + str(error), flush=True)
            if driver.poll() is None:
                driver.send_signal(signal.SIGINT)
                try:
                    driver.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    driver.kill()
                    driver.wait()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 1 if failed else 0
