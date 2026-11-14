#!/usr/bin/env python3
"""CANopen CiA402单电机：只读识别节点，Profile Velocity模式3短时运动。"""

import argparse
import json
import math
import signal
import socket
import struct
import time

from motor_discovery import link_state, scan_can, select_target

# None表示自动识别。速度比例须由该型号的606C/60FF单位确定，不从节点号推断。
DEFAULT_MOTOR_ID = None
DEFAULT_DEVICE = "canable0"
DEFAULT_SPEED = 0.1
DEFAULT_DURATION = 1.0
DEFAULT_VELOCITY_SCALE = None  # rad/s/raw；确认为counts/s时填2*pi/counts_per_rev


class SdoBus:
    def __init__(self, interface, node, timeout):
        self.node, self.timeout = node, timeout
        self.bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack("I", 0x1FFFFFFF))
        self.bus.bind((interface,))
        self.bus.settimeout(min(timeout, 0.05))

    def send(self, ident, data):
        self.bus.send(struct.pack("=IB3x8s", ident, len(data), data.ljust(8, b"\0")))

    def transfer(self, index, sub=0, value=None, size=4, signed=False):
        header = struct.pack("<HB", index, sub)
        if value is None:
            request = b"\x40" + header + bytes(4)
        else:
            request = bytes([{1: 0x2F, 2: 0x2B, 4: 0x23}[size]]) + header
            request += value.to_bytes(size, "little", signed=signed).ljust(4, b"\0")
        self.send(0x600 + self.node, request)
        end = time.monotonic() + self.timeout
        while time.monotonic() < end:
            try:
                raw = self.bus.recv(16)
            except TimeoutError:
                continue
            ident, count, data = struct.unpack("=IB3x8s", raw)
            if ident & 0x20000000:
                raise RuntimeError(f"CAN错误帧 {ident:#x}/{data.hex()}")
            if ident != 0x580 + self.node or count != 8 or data[1:4] != header:
                continue
            if data[0] == 0x80:
                raise RuntimeError(f"SDO {index:04X}:{sub:02X} abort=0x{int.from_bytes(data[4:], 'little'):08X}")
            if value is not None and data[0] == 0x60:
                return None
            if value is None and data[0] in (0x43, 0x47, 0x4B, 0x4F):
                length = 4 - ((data[0] >> 2) & 3)
                return int.from_bytes(data[4:4 + length], "little", signed=signed)
        raise RuntimeError(f"SDO {index:04X}:{sub:02X} 超时")

    def status(self):
        return self.transfer(0x6041)

    def control(self, word):
        self.transfer(0x6040, value=word, size=2)

    def wait_state(self, expected):
        end = time.monotonic() + 2
        while time.monotonic() < end:
            status = self.status()
            if status & 8:
                raise RuntimeError(f"CiA402故障，状态字0x{status:04X}")
            if status & 0x6F == expected:
                return
            time.sleep(0.02)
        raise RuntimeError(f"CiA402状态转换失败，目标0x{expected:04X}")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", "--interface", default=DEFAULT_DEVICE)
    parser.add_argument("--motor-id", type=lambda v: int(v, 0), default=DEFAULT_MOTOR_ID)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--velocity-scale", type=float, default=DEFAULT_VELOCITY_SCALE, help="rad/s/raw，必须按型号确认")
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--speed-limit", type=float, default=0.2)
    parser.add_argument("--stop-tolerance", type=float, default=0.02)
    parser.add_argument("--scan-start", type=int, default=1)
    parser.add_argument("--scan-end", type=int, default=127)
    parser.add_argument("--scan-timeout", type=float, default=0.2)
    parser.add_argument("--scan-retries", type=int, default=2)
    parser.add_argument("--sdo-timeout", type=float, default=0.5)
    parser.add_argument("--scan-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    values = (args.duration, args.speed_limit, args.stop_tolerance, args.scan_timeout, args.sdo_timeout)
    if not all(math.isfinite(v) and v > 0 for v in values) or not math.isfinite(args.speed):
        parser.error("时间和阈值须为有限正数，速度须为有限数")
    if not 1 <= args.scan_start <= args.scan_end <= 127 or args.scan_retries < 1:
        parser.error("扫描范围1–127，重试轮数须>=1")
    if args.motor_id is not None and not 1 <= args.motor_id <= 127:
        parser.error("motor-id范围1–127")
    if abs(args.speed) > args.speed_limit:
        parser.error("目标速度超过速度上限")
    if not args.scan_only:
        if args.velocity_scale is None or not math.isfinite(args.velocity_scale) or args.velocity_scale <= 0:
            parser.error("运动须显式设置 --velocity-scale；先确认606C/60FF的实际单位")
        raw = abs(args.speed / args.velocity_scale)
        if raw > 2147483647 or (args.speed != 0 and raw < 0.5):
            parser.error("速度换算后溢出或小于半个原始单位")
    if args.dry_run and (args.scan_only or args.motor_id is None):
        parser.error("dry-run须指定motor-id，不能与scan-only同时使用")
    return args


def main():
    args = arguments()
    if args.dry_run:
        print(json.dumps(vars(args), indent=2))
        return 0
    nodes = [args.motor_id] if args.motor_id is not None else list(range(args.scan_start, args.scan_end + 1))
    try:
        found = scan_can("CANOPEN", args.device, nodes, args.scan_timeout, args.scan_retries)
        print(json.dumps(found, ensure_ascii=False, indent=2), flush=True)
        if args.scan_only:
            return 0 if found else 1
        target = select_target(found, args.motor_id)
    except (Exception, KeyboardInterrupt) as error:
        print("发现未完成: " + str(error))
        return 1
    bus = SdoBus(args.device, target["motor_id"], args.sdo_timeout)
    failed = False
    original_mode = None
    mode_changed = False

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}

    def actual_velocity():
        return bus.transfer(0x606C, signed=True) * args.velocity_scale * args.direction

    try:
        status = bus.status()
        if status & 8 or status & 0x6F == 0x27 or abs(actual_velocity()) > args.stop_tolerance:
            raise RuntimeError("电机故障、已使能或未停稳，拒绝开始")
        supported = bus.transfer(0x6502)
        if not supported & (1 << 2):
            raise RuntimeError("该设备未声明支持Profile Velocity模式3；不尝试CSV/IP替代")
        original_mode = bus.transfer(0x6061, signed=True)
        bus.transfer(0x60FF, value=0, signed=True)
        mode_changed = True
        bus.transfer(0x6060, value=3, size=1, signed=True)
        if bus.transfer(0x6061, signed=True) != 3:
            raise RuntimeError("速度模式读回不符")
        bus.send(0, bytes([1, target["motor_id"]]))  # 只启动目标节点，不广播NMT
        for word, expected in [(6, 0x21), (7, 0x23), (15, 0x27)]:
            bus.control(word)
            bus.wait_state(expected)
        raw_speed = round(args.speed / args.velocity_scale * args.direction)
        end = time.monotonic() + args.duration
        next_print = 0.0
        while time.monotonic() < end:
            link_state(args.device)
            status = bus.status()
            speed = actual_velocity()
            if status & 0x6F != 0x27 or abs(speed) > args.speed_limit:
                raise RuntimeError("状态异常或速度超限")
            bus.transfer(0x60FF, value=raw_speed, signed=True)
            if time.monotonic() >= next_print:
                print(f"目标 {args.speed:+.3f} rad/s，实际 {speed:+.4f} rad/s", flush=True)
                next_print = time.monotonic() + 0.5
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("收到中断，正在停车。")
    except Exception as error:
        failed = True
        print("运行失败: " + str(error))
    finally:
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        for index, value, size in [(0x60FF, 0, 4), (0x6040, 2, 2), (0x6040, 0, 2)]:
            try:
                bus.transfer(index, value=value, size=size)
            except Exception as error:
                failed = True
                print("停车失败: " + str(error))
        try:
            bus.wait_state(0x40)
            deadline = time.monotonic() + 3
            while abs(actual_velocity()) > args.stop_tolerance:
                if time.monotonic() >= deadline:
                    raise RuntimeError("停车后速度未降至阈值")
                time.sleep(0.05)
            if mode_changed and original_mode is not None:
                bus.transfer(0x6060, value=original_mode, size=1, signed=True)
                if bus.transfer(0x6061, signed=True) != original_mode:
                    raise RuntimeError("原运行模式恢复读回失败")
            print("已确认停稳、关闭输出。")
        except Exception as error:
            failed = True
            print("停止/恢复确认失败: " + str(error))
        bus.bus.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
