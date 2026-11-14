"""Read-only ID discovery for the protocols implemented by this workspace."""

import ipaddress
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time


def query(protocol, node):
    if protocol == "DM":
        return 0x7FF, struct.pack("<HBBI", node, 0x33, 10, 0)
    if protocol == "MT":
        return 0x140 + node, bytes([0x9A]) + bytes(7)
    if protocol == "PP":
        return node, bytes([0x03, 0x02])  # 读序列号；短帧兼容旧固件
    if protocol == "CANOPEN":
        return 0x600 + node, bytes.fromhex("4018100100000000")  # 1018:01 Vendor ID
    raise ValueError(protocol)


def decode(protocol, ident, data):
    if ident & 0xE0000000:  # 拒绝扩展帧、远程帧、错误帧
        return None
    if protocol == "DM" and len(data) == 8 and data[2:4] == b"\x33\x0a":
        node = int.from_bytes(data[:2], "little")
        mode = int.from_bytes(data[4:], "little")
        if mode in (1, 2, 3, 4):
            return {"motor_id": node, "master_id": ident, "mode": mode}
    if protocol == "MT" and 0x241 <= ident <= 0x33F and len(data) == 8 and data[0] == 0x9A:
        return {"motor_id": ident - 0x240, "fault_code": int.from_bytes(data[6:], "little")}
    if protocol == "PP" and 1 <= ident <= 255 and len(data) >= 6 and data[:2] == b"\x04\x02":
        return {"motor_id": ident, "serial": data[2:6].hex()}
    if protocol == "CANOPEN" and 0x581 <= ident <= 0x5FF and len(data) == 8:
        if data[:4] == bytes.fromhex("43181001"):
            return {"motor_id": ident - 0x580, "vendor_id": int.from_bytes(data[4:], "little")}
    return None


def link_state(interface):
    info = json.loads(subprocess.check_output(
        ["ip", "-details", "-statistics", "-json", "link", "show", "dev", interface], text=True))[0]
    if info.get("linkinfo", {}).get("info_kind") != "can":
        raise RuntimeError(interface + " 不是物理 CAN 接口")
    state = info["linkinfo"]["info_data"].get("state")
    if "UP" not in info.get("flags", []) or state != "ERROR-ACTIVE":
        raise RuntimeError(f"{interface} 状态 {state}；请先恢复接口和接线，不继续扫描")
    return info["linkinfo"].get("info_xstats", {})


def scan_can(protocol, interface, nodes, timeout, retries):
    found = {}
    sent = set()
    initial_stats = link_state(interface)
    unspecified_notices = 0
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack("I", 0x1FFFFFFF))
        bus.bind((interface,))
        bus.settimeout(min(timeout, 0.05))
        for attempt in range(retries):
            print(f"只读扫描 {protocol} {interface}，第 {attempt + 1}/{retries} 轮", flush=True)
            for index, node in enumerate(nodes):
                link_state(interface)
                ident, data = query(protocol, node)
                sent.add(node)
                bus.send(struct.pack("=IB3x8s", ident, len(data), data.ljust(8, b"\0")))
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    try:
                        raw = bus.recv(16)
                    except TimeoutError:
                        continue
                    if len(raw) != 16:
                        continue
                    ident, size, payload = struct.unpack("=IB3x8s", raw)
                    if ident & 0x20000000:
                        # The connected gs_usb adapter was observed emitting this
                        # empty notice before valid replies. It carries no ACK/protocol/
                        # warning/passive indication. Defer acceptance until the
                        # final link state and kernel fault counters are checked.
                        if ident == 0x20000004 and size == 8 and payload == bytes(8):
                            unspecified_notices += 1
                            continue
                        raise RuntimeError(f"检测到 CAN 错误帧 {ident:#x}/{payload.hex()}，终止扫描")
                    row = decode(protocol, ident, payload[:size])
                    if row is not None and row["motor_id"] in sent:
                        key = (row["motor_id"], row.get("master_id"))
                        if key not in found:
                            found[key] = dict(row, device=interface)
                            print("发现: " + json.dumps(found[key], ensure_ascii=False), flush=True)
                if (index + 1) % 16 == 0:
                    print(f"已查询到 ID={node}，累计 {len(found)} 台", flush=True)
    final_stats = link_state(interface)
    if unspecified_notices:
        counters = ("restarts", "bus_error", "arbitration_lost", "error_warning", "error_passive", "bus_off")
        if not initial_stats or not final_stats or any(
                key not in initial_stats or key not in final_stats or initial_stats[key] != final_stats[key]
                for key in counters):
            raise RuntimeError("收到未注明原因的 CAN 控制器状态帧，且故障计数变化或不可核验；终止扫描")
        print(f"收到 {unspecified_notices} 条空控制器状态通知；接口保持 ERROR-ACTIVE，故障计数未增加", flush=True)
    return list(found.values())


def scan_ecb(device, timeout):
    """Compile a small read-only SDK helper in /tmp, then discover ID/IP pairs."""
    sdk = Path(__file__).resolve().parents[1] / "can_driver/lib/innfos-cpp-sdk"
    lib = sdk / "sdk/lib/linux_x86_64"
    folder = Path(tempfile.mkdtemp(prefix="ecb-lookup-"))
    executable = folder / "lookup"
    subprocess.run(["g++", "-std=c++11", str(Path(__file__).with_name("ecb_lookup_readonly.cpp")),
                    "-I" + str(sdk / "sdk/include"), "-L" + str(lib),
                    "-Wl,-rpath," + str(lib), "-lActuatorController", "-pthread", "-o", str(executable)],
                   check=True, timeout=60)
    env = os.environ.copy()
    env.setdefault("ACTUATOR_TOOL_PATH", str(sdk / "tools/logtool/linux_x86_64"))
    result = subprocess.run([str(executable)], env=env, capture_output=True,
                            text=True, timeout=timeout)
    (folder / "lookup.log").write_text(result.stdout + result.stderr)
    print(f"ECB 只读发现日志: {folder / 'lookup.log'}", flush=True)
    found = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[0] == "MOTOR":
            node = int(fields[1])
            ip = str(ipaddress.IPv4Address(fields[2]))
            endpoint = "ecb://" + ip
            if 1 <= node <= 255 and (device == "ecb://auto" or endpoint == device):
                found[node, ip] = {"motor_id": node, "device": endpoint}
    if result.returncode and not found:
        raise RuntimeError(f"ECB SDK 发现失败；见 {folder / 'lookup.log'}（网络/SDK权限配置）")
    return list(found.values())


def select_target(candidates, motor_id=None, master_id=None):
    matched = [row for row in candidates if motor_id is None or row["motor_id"] == motor_id]
    if master_id is not None:
        matched = [row for row in matched if row.get("master_id") == master_id]
    if not matched:
        raise RuntimeError("未找到符合条件的电机；未发送使能或运动命令")
    if len(matched) != 1:
        raise RuntimeError("发现多台电机；请用 --motor-id 指定，ECB重复ID还需 --device 指定IP；未发送使能或运动命令")
    return matched[0]
