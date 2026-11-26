#!/usr/bin/env python3
import argparse
from collections import deque
import json
import os
import time
import signal
import socket
import sys
import threading
from typing import Any, Dict, List, Optional, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from control_v2 import ControlV2, CAPABILITIES, BUILD
from bridge_core import BridgeCore, BridgeRuntime
from bridge_protocol import DEFAULT_WATCHDOG_MS, MAX_FRAME_BYTES, PROTOCOL_VERSION, now_ms
from bridge_protocol import json_line
from debug_ui import DebugHttpServer
from debug_events import CompositeEventSink, ConsoleEventSink, EventSink, RingBufferEventSink
from output_adapters import DryRunOutput, OutputAdapter, RosOutput
from video_manager import load_video_config
from video_manager_ipc import (
    DEFAULT_VIDEO_MANAGER_HOST,
    DEFAULT_VIDEO_MANAGER_PORT,
    VideoManagerClient,
    VideoManagerGateway,
)

DEFAULT_CAMERA_CONFIG = os.path.join(os.path.dirname(__file__), "video_sources.local.yaml")
if not os.path.isfile(DEFAULT_CAMERA_CONFIG):
    DEFAULT_CAMERA_CONFIG = os.path.join(os.path.dirname(__file__), "video_sources.example.yaml")
DEFAULT_BRIDGE_CONFIG = os.path.join(os.path.dirname(__file__), "bridge_control.yaml")
DEFAULT_FLIPPER_JOINT_NAMES = [
    "left_front_arm_joint",
    "right_front_arm_joint",
    "left_rear_arm_joint",
    "right_rear_arm_joint",
]
DEFAULT_BASE_LINEAR_LEVELS = {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8}
DEFAULT_BASE_ANGULAR_LEVELS = {1: 0.4, 2: 0.8, 3: 1.0, 4: 1.25, 5: 1.5}
DEFAULT_ARM_LINEAR_LEVELS = {1: 0.04, 2: 0.08, 3: 0.10, 4: 0.125, 5: 0.15}
DEFAULT_ARM_ANGULAR_LEVELS = {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8}
DEFAULT_GRIPPER_RATE_LEVELS = {1: 0.004, 2: 0.008, 3: 0.012, 4: 0.016, 5: 0.02}
DEFAULT_FLIPPER_VELOCITY_LEVELS = {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8}
LEVEL_CONFIG_KEYS = {
    "base-linear": "base_linear_levels",
    "base-angular": "base_angular_levels",
    "arm-linear": "arm_linear_levels",
    "arm-angular": "arm_angular_levels",
    "gripper-rate": "gripper_rate_levels",
    "flipper-velocity": "flipper_velocity_levels",
}


def add_level_args(
    parser: argparse.ArgumentParser,
    prefix: str,
    help_name: str,
    defaults: Dict[int, float],
) -> None:
    for level in range(1, 6):
        parser.add_argument(
            f"--{prefix}-level-{level}",
            type=float,
            default=float(defaults[level]),
            help=f"{help_name} speed/rate for level {level}",
        )


def collect_level_args(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    prefix: str,
    config: Optional[Dict[str, Any]] = None,
) -> Optional[Dict[int, float]]:
    values: Dict[int, float] = {}
    attr_prefix = prefix.replace("-", "_")
    config_levels = {}
    if config:
        config_key = LEVEL_CONFIG_KEYS.get(prefix, "")
        raw_config_levels = config.get(config_key, {})
        if raw_config_levels is not None:
            if not isinstance(raw_config_levels, dict):
                raise ValueError(f"{config_key} must be a mapping of speed levels")
            config_levels = {
                int(level): float(value)
                for level, value in raw_config_levels.items()
            }
    for level in range(1, 6):
        attr_name = f"{attr_prefix}_level_{level}"
        cli_value = getattr(args, attr_name, None)
        parser_default = parser.get_default(attr_name)
        config_value = config_levels.get(level)
        if cli_value is None:
            if config_value is not None:
                values[level] = config_value
            continue
        if config_value is not None and cli_value == parser_default:
            values[level] = config_value
            continue
        values[level] = float(cli_value)
    return values or None


def parse_csv_list(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_float_csv_list(value: str) -> List[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def load_bridge_control_config(path: str) -> Dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load bridge control config") from exc

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("bridge control config root must be a mapping")
    node_config = data.get("host_bridge_node", data)
    if not isinstance(node_config, dict):
        raise ValueError("bridge control config host_bridge_node must be a mapping")
    return node_config


def resolve_config_value(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    attr: str,
    config: Dict[str, Any],
    default: Any,
) -> Any:
    value = getattr(args, attr)
    parser_default = parser.get_default(attr)
    if value is None:
        return config.get(attr, default)
    if attr in config and value == parser_default:
        return config[attr]
    return value


def resolve_float_config(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    attr: str,
    config: Dict[str, Any],
    default: float,
) -> float:
    return float(resolve_config_value(args, parser, attr, config, default))


def resolve_int_config(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    attr: str,
    config: Dict[str, Any],
    default: int,
) -> int:
    return int(resolve_config_value(args, parser, attr, config, default))


def resolve_str_config(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    attr: str,
    config: Dict[str, Any],
    default: str,
) -> str:
    return str(resolve_config_value(args, parser, attr, config, default))


def resolve_flipper_direction_corrections(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    config: Dict[str, Any],
    joint_names: List[str],
) -> Optional[List[float]]:
    raw_value = resolve_config_value(
        args,
        parser,
        "flipper_direction_corrections",
        config,
        None,
    )
    if raw_value in (None, ""):
        return None
    if isinstance(raw_value, str):
        return parse_float_csv_list(raw_value)
    if isinstance(raw_value, list):
        return [float(value) for value in raw_value]
    if isinstance(raw_value, dict):
        return [float(raw_value.get(name, 1.0)) for name in joint_names]
    raise ValueError("flipper_direction_corrections must be a comma-separated string, list, or mapping")


def resolve_servo_angular_direction_corrections(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    config: Dict[str, Any],
) -> Optional[List[float]]:
    raw_value = resolve_config_value(
        args,
        parser,
        "servo_angular_direction_corrections",
        config,
        None,
    )
    if raw_value in (None, ""):
        return None
    if isinstance(raw_value, str):
        return parse_float_csv_list(raw_value)
    if isinstance(raw_value, list):
        return [float(value) for value in raw_value]
    if isinstance(raw_value, dict):
        return [
            float(raw_value.get("x", 1.0)),
            float(raw_value.get("y", 1.0)),
            float(raw_value.get("z", 1.0)),
        ]
    raise ValueError(
        "servo_angular_direction_corrections must be a comma-separated string, list, or mapping"
    )


def resolve_service_bindings_config(
    config: Dict[str, Any],
    attr: str,
) -> Dict[str, str]:
    raw_value = config.get(attr, {})
    if raw_value in (None, ""):
        return {}
    if not isinstance(raw_value, dict):
        raise ValueError(f"{attr} must be a mapping of input name to ROS service name")

    bindings: Dict[str, str] = {}
    for raw_name, raw_service in raw_value.items():
        name = str(raw_name).strip().lower()
        service = str(raw_service).strip()
        if not name or not service:
            raise ValueError(f"{attr} keys and values must be non-empty")
        bindings[name] = service
    return bindings


class HostBridgeServer:
    def __init__(
        self,
        host: str,
        port: int,
        output: OutputAdapter,
        watchdog_ms: int = DEFAULT_WATCHDOG_MS,
        linear_speed: float = 0.8,
        angular_speed: float = 1.5,
        servo_frame: str = "catch_camera",
        default_speed_level: int = 2,
        base_linear_levels: Optional[Dict[int, float]] = None,
        base_angular_levels: Optional[Dict[int, float]] = None,
        arm_linear_levels: Optional[Dict[int, float]] = None,
        arm_angular_levels: Optional[Dict[int, float]] = None,
        servo_angular_direction_corrections: Optional[List[float]] = None,
        gripper_rate_levels: Optional[Dict[int, float]] = None,
        flipper_velocity_levels: Optional[Dict[int, float]] = None,
        flipper_joint_names: Optional[List[str]] = None,
        flipper_direction_corrections: Optional[List[float]] = None,
        flipper_jog_duration: float = 0.15,
        flipper_target_profile: str = "csv_velocity",
        flipper_profile_retry_sec: float = 2.0,
        gamepad_deadzone_percent: float = 4.0,
        keyboard_service_bindings: Optional[Dict[str, str]] = None,
        gamepad_service_bindings: Optional[Dict[str, str]] = None,
        cameras: Optional[List[Dict[str, Any]]] = None,
        video_gateway: Optional[VideoManagerGateway] = None,
        video_poll_sec: float = 1.0,
        joint_runtime_topic: str = "",
        co2_topic: str = "",
        co2_status_topic: str = "",
        events: Optional[EventSink] = None,
        control_state_dir: Optional[str] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.stop_event = threading.Event()
        self.client_lock = threading.RLock()
        self.active_clients: List["BridgeClient"] = []
        self.events = events or RingBufferEventSink()
        self.video_gateway = video_gateway
        self.video_poll_sec = max(video_poll_sec, 0.1)
        self.joint_runtime_topic = joint_runtime_topic
        self.co2_topic = co2_topic
        self.co2_status_topic = co2_status_topic
        self.joint_runtime_subscriber = None
        self.co2_subscriber = None
        self.co2_status_subscriber = None
        self._joint_runtime_lock = threading.Lock()
        self._last_joint_runtime_states: Optional[List[Dict[str, Any]]] = None
        self._last_joint_runtime_sent = 0.0
        self._co2_status_lock = threading.Lock()
        self._last_co2_status: Optional[Tuple[bool, Optional[int], str]] = None
        self._last_co2_status_at = 0.0
        self.core = BridgeCore(
            output,
            watchdog_ms,
            linear_speed,
            angular_speed,
            servo_frame,
            default_speed_level,
            base_linear_levels,
            base_angular_levels,
            arm_linear_levels,
            arm_angular_levels,
            servo_angular_direction_corrections,
            gripper_rate_levels,
            flipper_velocity_levels,
            flipper_joint_names,
            flipper_direction_corrections,
            flipper_jog_duration,
            flipper_target_profile,
            flipper_profile_retry_sec,
            gamepad_deadzone_percent,
            keyboard_service_bindings,
            gamepad_service_bindings,
            cameras,
            self.video_gateway.camera_infos if self.video_gateway else None,
            self.handle_camera_stream_request if self.video_gateway else None,
            self.events,
        )
        self.runtime = BridgeRuntime(self.core, self.broadcast)
        self.control = ControlV2(self.core, state_dir=control_state_dir)

    def serve_forever(self) -> None:
        self.start_video_manager()
        self.start_joint_runtime_forwarder()
        self.start_co2_forwarder()
        threading.Thread(target=self._control_loop, daemon=True).start()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(5)
                server.settimeout(0.5)
                print(f"[bridge] listening on {self.host}:{self.port}", flush=True)
                self.events.emit("tcp", "bridge listening", data={"host": self.host, "port": self.port})
                while not self.stop_event.is_set():
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    with self.client_lock:
                        if len(self.active_clients) >= 16:
                            conn.close()
                            continue
                    client = BridgeClient(self, conn, addr)
                    with self.client_lock:
                        self.active_clients.append(client)
                    threading.Thread(target=client.run, daemon=True).start()
        finally:
            self.control.close()
            with self.client_lock:
                for client in self.active_clients:
                    client.close()
            self.stop_video_manager()

    def _control_loop(self):
        while not self.stop_event.wait(.01):
            self.control.tick()

    def start_video_manager(self) -> None:
        if not self.video_gateway:
            return
        changed = self.video_gateway.refresh()
        if self.video_gateway.last_error():
            self.events.emit("video", "external video manager unavailable", level="warning", data={
                "error": self.video_gateway.last_error(),
            })
        else:
            self.events.emit("video", "external video manager connected", data={
                "initial_changed": len(changed),
            })
        threading.Thread(target=self._video_monitor_loop, daemon=True).start()

    def stop_video_manager(self) -> None:
        if not self.video_gateway:
            return
        self.events.emit("video", "external video manager detached")

    @staticmethod
    def build_joint_runtime_states(state: Dict[str, Any]) -> List[Dict[str, Any]]:
        if not isinstance(state, dict):
            return []
        diagnostics = state.get("diagnostics")
        if not isinstance(diagnostics, dict):
            return []
        readable = diagnostics.get("readable") is True
        axes = diagnostics.get("axes")
        if not isinstance(axes, dict):
            return []
        groups = state.get("groups")
        groups = groups if isinstance(groups, dict) else {}
        events = diagnostics.get("events")
        events = events if isinstance(events, list) else []

        faulted_axes = set()
        faulted_groups = set()
        faulted_backends = set()
        global_fault = False
        for event in events:
            if not isinstance(event, dict):
                continue
            try:
                severity = int(event.get("severity", 0))
            except (TypeError, ValueError):
                severity = 0
            if severity < 2:
                continue
            if severity >= 3:
                global_fault = True
            ev_axis = event.get("axis") or ""
            ev_group = event.get("group") or ""
            ev_backend = event.get("backend") or ""
            if ev_axis:
                faulted_axes.add(ev_axis)
            elif ev_group:
                faulted_groups.add(ev_group)
            elif ev_backend:
                faulted_backends.add(ev_backend)

        states: List[Dict[str, Any]] = []
        for name, axis in axes.items():
            if not isinstance(axis, dict):
                continue
            group_name = axis.get("group", "")
            backend = axis.get("backend", "")
            lifecycle_state = groups.get(group_name, "")
            is_fault = (
                global_fault
                or lifecycle_state == "Fault"
                or name in faulted_axes
                or group_name in faulted_groups
                or backend in faulted_backends
            )
            states.append({
                "joint_name": name,
                "backend": backend,
                "lifecycle_state": lifecycle_state,
                "online": bool(readable and axis.get("ready", False)),
                "enabled": bool(readable and lifecycle_state == "Running"),
                "fault": is_fault,
            })
        return states

    def _on_runtime_state(self, state: Dict[str, Any]) -> None:
        if not self.joint_runtime_topic:
            return
        states = self.build_joint_runtime_states(state)
        now = time.monotonic()
        with self._joint_runtime_lock:
            changed = states != self._last_joint_runtime_states
            if not changed and now - self._last_joint_runtime_sent < 1.0:
                return
            self._last_joint_runtime_states = states
            self._last_joint_runtime_sent = now
        self.broadcast({
            "type": "joint_runtime_states",
            "protocol_version": PROTOCOL_VERSION,
            "seq": 0,
            "timestamp_ms": now_ms(),
            "states": states,
        })

    def start_joint_runtime_forwarder(self) -> None:
        if not self.joint_runtime_topic:
            return
        if (self.joint_runtime_topic == "/robot_runtime/state"
                and hasattr(self.core.output, "runtime_state_callback")):
            self.core.output.runtime_state_callback = self._on_runtime_state
            return
        node = getattr(self.core.output, "node", None)
        if node is None:
            return
        from std_msgs.msg import String
        def callback(msg: Any) -> None:
            try:
                state = json.loads(msg.data)
            except ValueError:
                return
            if isinstance(state, dict):
                self._on_runtime_state(state)
        self.joint_runtime_subscriber = node.create_subscription(
            String, self.joint_runtime_topic, callback, 1
        )

    def _on_co2_status(self, payload: Dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            return
        online = bool(payload.get("online", False))
        raw_ppm = payload.get("ppm")
        ppm: Optional[int] = None
        if raw_ppm is not None:
            try:
                ppm = int(raw_ppm)
            except (TypeError, ValueError):
                ppm = None
        error = str(payload.get("error") or "")
        raw_ts = payload.get("timestamp_ms")
        timestamp_ms: Optional[int] = None
        if raw_ts is not None:
            try:
                timestamp_ms = int(raw_ts)
            except (TypeError, ValueError):
                timestamp_ms = None
        key = (online, ppm, error)
        now = time.monotonic()
        with self._co2_status_lock:
            if self._last_co2_status == key and (now - self._last_co2_status_at) < 5.0:
                return
            self._last_co2_status = key
            self._last_co2_status_at = now
        self.broadcast(
            self.core.make_co2_status(
                online=online,
                ppm=ppm,
                error=error,
                timestamp_ms=timestamp_ms,
            )
        )

    def start_co2_forwarder(self):
        node = getattr(self.core.output, 'node', None)
        if node is None:
            return
        if self.co2_topic:
            from std_msgs.msg import Int32
            self.co2_subscriber = node.create_subscription(
                Int32,
                self.co2_topic,
                lambda msg: self.broadcast(self.core.make_co2_data(int(msg.data))),
                1,
            )
        if self.co2_status_topic:
            from std_msgs.msg import String
            def on_status_msg(msg: Any) -> None:
                try:
                    data = json.loads(msg.data)
                except ValueError:
                    return
                if isinstance(data, dict):
                    self._on_co2_status(data)
            self.co2_status_subscriber = node.create_subscription(
                String,
                self.co2_status_topic,
                on_status_msg,
                1,
            )

    def _video_monitor_loop(self) -> None:
        if not self.video_gateway:
            return
        while not self.stop_event.wait(self.video_poll_sec):
            for camera in self.video_gateway.refresh():
                self.broadcast(self.core.make_camera_info(camera))

    def handle_camera_stream_request(self, params: Dict[str, Any]) -> Tuple[bool, int, str, Optional[Dict[str, Any]]]:
        if not self.video_gateway:
            return False, 2400, "video manager unavailable", None
        ok, code, message, camera = self.video_gateway.stream_request(params)
        self.events.emit("video", "camera stream request handled", data={
            "camera_id": params.get("camera_id"),
            "action": params.get("action"),
            "online": camera.get("online", False) if camera else False,
        })
        return ok, code, message, camera

    def remove_client(self, client: "BridgeClient") -> None:
        with self.client_lock:
            if client in self.active_clients:
                self.active_clients.remove(client)
        self.control.disconnect(client)

    def broadcast(self, payload: Dict[str, Any]) -> None:
        with self.client_lock:
            clients = list(self.active_clients)
        for client in clients:
            client.send_json(payload)


class BridgeClient:
    CONTROL_QUEUE_MAX_ITEMS = 128
    CONTROL_QUEUE_MAX_BYTES = 512 * 1024
    IDLE_RX_TIMEOUT_SEC = 30.0
    DROPPABLE_TELEMETRY_TYPES = frozenset({"joint_runtime_states", "co2_data", "co2_status", "camera_info"})

    def __init__(self, server: HostBridgeServer, conn: socket.socket, addr: Tuple[str, int]) -> None:
        self.server = server
        self.conn = conn
        self.addr = addr
        self._send_lock = threading.Lock()
        self._send_cv = threading.Condition(self._send_lock)
        self._close_lock = threading.Lock()
        self._control_queue: deque = deque()
        self._control_bytes = 0
        self._telemetry_latest: Dict[Any, Tuple[float, bytes]] = {}
        self.connected_at = time.monotonic()
        self.last_rx_at = self.connected_at
        self.closed = False
        self._negotiated_v2 = False
        self.writer = None

    def run(self) -> None:
        is_loopback = bool(self.addr) and self.addr[0] in ("127.0.0.1", "::1")
        if not is_loopback:
            print(f"[bridge] client connected: {self.addr}", flush=True)
        self.server.events.emit(
            "tcp",
            "client connected",
            level="debug" if is_loopback else "info",
            data={"addr": f"{self.addr[0]}:{self.addr[1]}"},
        )
        self.conn.settimeout(1.0)
        if self.conn.family in (socket.AF_INET, socket.AF_INET6):
            self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                if hasattr(socket, "TCP_KEEPIDLE"):
                    self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                if hasattr(socket, "TCP_KEEPINTVL"):
                    self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                if hasattr(socket, "TCP_KEEPCNT"):
                    self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            except OSError:
                pass
        try:
            self.writer = threading.Thread(target=self._write_loop, daemon=True)
            self.writer.start()
            hello = dict(self.server.core.make_hello(), control_protocol_version=2, build_version=BUILD)
            self.send_json(hello)
            self.send_json(self.server.core.make_capabilities())
            self._recv_loop()
        finally:
            self.close()
            if self.writer:
                self.writer.join(1.2)
            self.server.remove_client(self)
            probe_only = is_loopback and not self._negotiated_v2
            self.server.events.emit(
                "tcp",
                "client disconnected",
                level="debug" if probe_only else "info",
                data={"addr": f"{self.addr[0]}:{self.addr[1]}"},
            )
            if not probe_only:
                print(f"[bridge] client disconnected: {self.addr}", flush=True)

    def close(self) -> None:
        with self._close_lock:
            if self.closed:
                return
            self.closed = True
            with self._send_cv:
                self._control_queue.clear()
                self._control_bytes = 0
                self._telemetry_latest.clear()
                self._send_cv.notify_all()
            try:
                self.conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.conn.close()
            except OSError:
                pass

    def send_json(self, payload: Dict[str, Any]) -> None:
        if self.closed:
            return
        data = json_line(payload)
        if len(data) > MAX_FRAME_BYTES:
            self.close()
            return
        now = time.monotonic()
        msg_type = payload.get("type")
        if msg_type in self.DROPPABLE_TELEMETRY_TYPES:
            key = ("camera_info", payload.get("camera_id")) if msg_type == "camera_info" else msg_type
            with self._send_cv:
                if self.closed:
                    return
                self._telemetry_latest[key] = (now, data)
                self._send_cv.notify()
            return
        close_now = False
        with self._send_cv:
            if self.closed:
                return
            if (len(self._control_queue) >= self.CONTROL_QUEUE_MAX_ITEMS
                    or self._control_bytes + len(data) > self.CONTROL_QUEUE_MAX_BYTES):
                close_now = True
            else:
                self._control_queue.append((now, data))
                self._control_bytes += len(data)
                self._send_cv.notify()
        if close_now:
            self.close()

    def _write_loop(self) -> None:
        while not self.closed and not self.server.stop_event.is_set():
            is_control = False
            with self._send_cv:
                while (not self.closed and not self.server.stop_event.is_set()
                       and not self._control_queue and not self._telemetry_latest):
                    self._send_cv.wait(timeout=0.1)
                if self.closed or self.server.stop_event.is_set():
                    break
                if self._control_queue:
                    queued, data = self._control_queue.popleft()
                    self._control_bytes = max(0, self._control_bytes - len(data))
                    is_control = True
                else:
                    key = next(iter(self._telemetry_latest))
                    queued, data = self._telemetry_latest.pop(key)
            if time.monotonic() - queued > 1.0:
                if is_control:
                    self.close()
                    break
                continue
            try:
                self.conn.sendall(data)
            except OSError:
                self.close()
                break

    def _recv_loop(self) -> None:
        buffer = b""
        while not self.server.stop_event.is_set() and not self.closed:
            try:
                chunk = self.conn.recv(4096)
            except socket.timeout:
                if time.monotonic() - self.last_rx_at > self.IDLE_RX_TIMEOUT_SEC:
                    self.server.events.emit("tcp", "closing idle client", level="warning",
                                            data={"addr": f"{self.addr[0]}:{self.addr[1]}"})
                    break
                continue
            except OSError:
                break

            if not chunk:
                break

            self.last_rx_at = time.monotonic()
            buffer += chunk
            if len(buffer) > MAX_FRAME_BYTES and b"\n" not in buffer:
                self.server.events.emit("protocol", "frame exceeds max length", level="error")
                self.send_json(self.server.core.make_protocol_error({}, 2001, "TCP frame exceeds max length"))
                break

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                # MAX_FRAME_BYTES includes the terminating newline on the wire.
                if len(line) + 1 > MAX_FRAME_BYTES:
                    self.server.events.emit("protocol", "line exceeds max length", level="error")
                    self.send_json(self.server.core.make_protocol_error({}, 2001, "TCP frame exceeds max length"))
                    return
                try:
                    msg = json.loads(line.decode("utf-8"))
                    if not isinstance(msg, dict):
                        raise ValueError("JSON frame is not an object")
                except Exception as exc:
                    self.server.events.emit("protocol", "invalid JSON", level="error", data={"error": str(exc)})
                    self.send_json(self.server.core.make_protocol_error({}, 2100, f"invalid JSON: {exc}"))
                    continue

                kind = msg.get('type')
                if msg.get('protocol_version', 1) == 2 or kind == 'emergency_stop':
                    self._negotiated_v2 = True
                if kind == 'emergency_stop':
                    try:
                        self.send_json(self.server.control.handle(self, msg))
                    except Exception as exc:
                        self.server.events.emit("protocol", "emergency_stop error", level="error", data={"error": str(exc)})
                        self.send_json(dict(
                            type='control_result', protocol_version=2,
                            request_id=msg.get('request_id'), seq=msg.get('seq'),
                            ok=False, error=dict(code='INTERNAL'),
                        ))
                    continue
                if kind == 'control_hello' and time.monotonic() - self.connected_at > 3:
                    self.send_json(dict(
                        type='control_result', protocol_version=2,
                        request_id=msg.get('request_id'), seq=msg.get('seq'),
                        ok=False, error=dict(code='NEGOTIATION_TIMEOUT'),
                    ))
                    continue
                if (msg.get('protocol_version', 1) == 2 and kind != 'control_hello'
                        and self not in self.server.control.sessions
                        and time.monotonic() - self.connected_at > 3):
                    self.send_json(dict(
                        type='control_result', protocol_version=2,
                        request_id=msg.get('request_id'), seq=msg.get('seq'),
                        ok=False, error=dict(code='NEGOTIATION_TIMEOUT'),
                    ))
                    continue
                # Legacy probes remain independent from the versioned control admission.
                if msg.get('protocol_version', 1) == 1 and kind in {'heartbeat', 'sync_request', 'system_snapshot_request', 'camera_list_request'}:
                    try:
                        for response in self.server.core.handle_message(msg):
                            self.send_json(response)
                    except (ValueError, TypeError):
                        self.send_json(self.server.core.make_protocol_error(msg, 2100, 'invalid fields'))
                    continue
                try:
                    self.send_json(self.server.control.handle(self, msg))
                except (ValueError, TypeError, KeyError):
                    self.send_json(dict(
                        type='control_result', protocol_version=2,
                        request_id=msg.get('request_id'), seq=msg.get('seq'),
                        ok=False, error=dict(code='BAD_REQUEST'),
                    ))
                except Exception as exc:
                    self.server.events.emit("protocol", "control handle error", level="error", data={"error": str(exc)})
                    self.send_json(dict(
                        type='control_result', protocol_version=2,
                        request_id=msg.get('request_id'), seq=msg.get('seq'),
                        ok=False, error=dict(code='INTERNAL'),
                    ))


def parse_camera(value: str) -> Dict[str, Any]:
    parts = value.split(",", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("camera format: id,name,rtsp_url")
    camera_id = int(parts[0])
    name = parts[1]
    rtsp_url = parts[2]
    return {
        "camera_id": camera_id,
        "name": name,
        "online": bool(rtsp_url),
        "rtsp_url": rtsp_url,
        "codec": "h264",
        "width": 1280,
        "height": 720,
        "fps": 25,
        "bitrate_kbps": 2500 if rtsp_url else 0,
    }


def parse_service_command(value: str) -> Tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("service command must be command=/service/name")
    command, service = value.split("=", 1)
    command = command.strip()
    service = service.strip()
    if not command or not service:
        raise argparse.ArgumentTypeError("service command and service name must be non-empty")
    return command, service


def load_cameras_from_yaml(path: str, publish_host_override: str = "") -> List[Dict[str, Any]]:
    config = load_video_config(path)
    if publish_host_override:
        config.rtsp.host = publish_host_override

    cameras = [
        source.camera_info(config.rtsp, online=True)
        for source in config.direct_sources
    ]
    cameras.extend(
        source.camera_info(config.rtsp, online=True)
        for source in config.stream_sources
    )
    deduped: Dict[int, Dict[str, Any]] = {}
    for camera in cameras:
        deduped[int(camera.get("camera_id", 1_000_000))] = camera
    return [
        deduped[camera_id]
        for camera_id in sorted(deduped.keys())
    ]


def detect_publish_host() -> str:
    for probe in ("8.8.8.8", "192.168.1.1", "192.168.0.1", "10.0.0.1", "172.16.0.1"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect((probe, 80))
                ip = sock.getsockname()[0]
                if ip and not ip.startswith("127.") and ip != "0.0.0.0":
                    return ip
        except OSError:
            continue
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip and not ip.startswith("127.") and ip != "0.0.0.0":
                return ip
    except OSError:
        pass
    return "127.0.0.1"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2 host bridge TCP/JSON node")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--control-state-dir", default=None, help="private persistent control operation journal directory")
    parser.add_argument(
        "--bridge-control-config",
        default=DEFAULT_BRIDGE_CONFIG,
        help="YAML config for host_bridge_node control translation",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="run without ROS and print bridge outputs")
    mode.add_argument("--ros", action="store_true", help="publish native ROS2 control topics; default mode")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--servo-topic", default="/servo_server/delta_twist_cmds")
    parser.add_argument("--servo-frame", default="grasp_tcp")
    parser.add_argument(
        "--gripper-velocity-topic",
        default="/arm_control/gripper_velocity",
    )
    parser.add_argument(
        "--moveit-group",
        default=None,
        help="MoveIt MoveGroup name used for named arm targets; overrides bridge_control.yaml",
    )
    parser.add_argument("--flipper-jog-topic", default="/flipper_control/jog_cmd")
    parser.add_argument("--flipper-profile-service", default="/flipper_control/set_control_profile")
    parser.add_argument(
        "--vision-detection-service",
        default="/paw_vision/set_detection_enabled",
        help="ROS service used for monitor-mode system_command routing",
    )
    parser.add_argument(
        "--hybrid-service-ns",
        default="/robot_runtime",
        help="ROS service namespace for lifecycle and joint mode commands",
    )
    parser.add_argument(
        "--service-command",
        action="append",
        type=parse_service_command,
        default=[],
        metavar="COMMAND=SERVICE",
        help=(
            "map a TCP command to a std_srvs/Trigger service; may be repeated. "
            "SERVICE may use {command}, {mode}, {source}, e.g. "
            "set_control_mode=/robot/set_{mode}_mode"
        ),
    )
    parser.add_argument("--default-speed-level", type=int, default=2)
    parser.add_argument("--flipper-target-profile", default="csv_velocity")
    parser.add_argument("--flipper-profile-retry-sec", type=float, default=2.0)
    parser.add_argument("--flipper-jog-duration", type=float, default=0.15)
    parser.add_argument(
        "--gamepad-deadzone-percent",
        type=float,
        default=None,
        help="ignore gamepad axes with absolute value below this percentage",
    )
    parser.add_argument(
        "--flipper-joint-names",
        default=",".join(DEFAULT_FLIPPER_JOINT_NAMES),
        help="comma-separated flipper joint names in command order",
    )
    parser.add_argument(
        "--flipper-direction-corrections",
        default=None,
        help="comma-separated flipper direction corrections aligned with --flipper-joint-names",
    )
    parser.add_argument(
        "--servo-angular-direction-corrections",
        default=None,
        help="comma-separated angular x,y,z direction corrections for arm servo output",
    )
    parser.add_argument("--watchdog-ms", type=int, default=DEFAULT_WATCHDOG_MS)
    parser.add_argument("--linear-speed", type=float, default=0.8, help="vehicle level-5 linear speed")
    parser.add_argument("--angular-speed", type=float, default=1.5, help="vehicle level-5 angular speed")
    add_level_args(parser, "base-linear", "base linear", DEFAULT_BASE_LINEAR_LEVELS)
    add_level_args(parser, "base-angular", "base angular", DEFAULT_BASE_ANGULAR_LEVELS)
    add_level_args(parser, "arm-linear", "arm linear", DEFAULT_ARM_LINEAR_LEVELS)
    add_level_args(parser, "arm-angular", "arm angular", DEFAULT_ARM_ANGULAR_LEVELS)
    add_level_args(parser, "gripper-rate", "gripper", DEFAULT_GRIPPER_RATE_LEVELS)
    add_level_args(parser, "flipper-velocity", "flipper", DEFAULT_FLIPPER_VELOCITY_LEVELS)
    parser.add_argument("--debug-ui", dest="debug_ui", action="store_true", default=True,
                        help="start read-only debug HTTP UI; enabled by default")
    parser.add_argument("--no-debug-ui", dest="debug_ui", action="store_false",
                        help="disable debug HTTP UI")
    parser.add_argument("--debug-host", default="0.0.0.0")
    parser.add_argument("--debug-port", type=int, default=18080)
    parser.add_argument(
        "--camera-config",
        default=DEFAULT_CAMERA_CONFIG,
        help="load camera list from a video_sources YAML file",
    )
    parser.add_argument(
        "--no-camera-config",
        action="store_true",
        help="disable static camera config loading; useful when --video-manager owns cameras",
    )
    parser.add_argument(
        "--camera-publish-host",
        default="",
        help="override rtsp.publish_host when loading --camera-config",
    )
    parser.add_argument(
        "--camera",
        action="append",
        type=parse_camera,
        help="camera entry as id,name,rtsp_url; may be repeated",
    )
    parser.add_argument(
        "--video-manager",
        action="store_true",
        help="use the standalone video_manager_node for camera list and stream lifecycle",
    )
    parser.add_argument("--video-manager-host", default=DEFAULT_VIDEO_MANAGER_HOST)
    parser.add_argument("--video-manager-port", type=int, default=DEFAULT_VIDEO_MANAGER_PORT)
    parser.add_argument("--video-manager-timeout", type=float, default=1.0)
    parser.add_argument(
        "--video-config",
        default="",
        help="deprecated: start host_bridge/video_manager_node.py with --config instead",
    )
    parser.add_argument(
        "--video-dry-run",
        action="store_true",
        help="deprecated: pass --dry-run to video_manager_node.py instead",
    )
    parser.add_argument(
        "--video-autostart",
        action="store_true",
        help="deprecated: pass --autostart to video_manager_node.py instead",
    )
    parser.add_argument("--video-poll-sec", type=float, default=1.0)
    parser.add_argument(
        "--joint-runtime-topic",
        default="/robot_runtime/state",
        help="ROS topic to forward as TCP joint_runtime_states when --ros is used; empty disables it",
    )
    parser.add_argument(
        "--co2-topic",
        default="/co2_ppm",
        help="ROS std_msgs/Int32 topic to forward as TCP co2_data; empty disables it",
    )
    parser.add_argument(
        "--co2-status-topic",
        default="/co2_status",
        help="ROS std_msgs/String JSON topic to forward as TCP co2_status; empty disables it",
    )
    return parser


def parse_node_args() -> Tuple[argparse.ArgumentParser, argparse.Namespace]:
    parser = build_arg_parser()
    parser.add_argument("--robot-id", default="loopback_robot")
    try:
        from rclpy.utilities import remove_ros_args
        raw_args = remove_ros_args(args=sys.argv)[1:]
    except ImportError:
        raw_args = list(sys.argv[1:])
        if "--ros-args" in raw_args:
            raw_args = raw_args[:raw_args.index("--ros-args")]
    args = parser.parse_args(raw_args)
    return parser, args


def main() -> None:
    parser, args = parse_node_args()
    buffer_events = RingBufferEventSink()
    events: EventSink = CompositeEventSink(
        buffer_events,
        ConsoleEventSink(min_level="warning"),
    )
    service_commands = dict(args.service_command or [])
    bridge_control_config = load_bridge_control_config(
        os.path.expanduser(args.bridge_control_config)
    ) if args.bridge_control_config else {}
    gamepad_deadzone_percent = resolve_float_config(
        args,
        parser,
        "gamepad_deadzone_percent",
        bridge_control_config,
        4.0,
    )
    moveit_group = resolve_str_config(
        args,
        parser,
        "moveit_group",
        bridge_control_config,
        "arm",
    )
    vision_detection_service = resolve_str_config(
        args,
        parser,
        "vision_detection_service",
        bridge_control_config,
        "/paw_vision/set_detection_enabled",
    )
    co2_topic = resolve_str_config(
        args,
        parser,
        "co2_topic",
        bridge_control_config,
        "/co2_ppm",
    )
    co2_status_topic = resolve_str_config(
        args,
        parser,
        "co2_status_topic",
        bridge_control_config,
        "/co2_status",
    )
    linear_speed = resolve_float_config(args, parser, "linear_speed", bridge_control_config, 0.8)
    angular_speed = resolve_float_config(args, parser, "angular_speed", bridge_control_config, 1.5)
    default_speed_level = resolve_int_config(
        args, parser, "default_speed_level", bridge_control_config, 2
    )
    flipper_jog_duration = resolve_float_config(
        args, parser, "flipper_jog_duration", bridge_control_config, 0.15
    )
    flipper_profile_retry_sec = resolve_float_config(
        args, parser, "flipper_profile_retry_sec", bridge_control_config, 2.0
    )
    base_linear_levels = collect_level_args(args, parser, "base-linear", bridge_control_config)
    base_angular_levels = collect_level_args(args, parser, "base-angular", bridge_control_config)
    arm_linear_levels = collect_level_args(args, parser, "arm-linear", bridge_control_config)
    arm_angular_levels = collect_level_args(args, parser, "arm-angular", bridge_control_config)
    gripper_rate_levels = collect_level_args(args, parser, "gripper-rate", bridge_control_config)
    flipper_velocity_levels = collect_level_args(
        args, parser, "flipper-velocity", bridge_control_config
    )
    flipper_joint_names = parse_csv_list(
        resolve_str_config(
            args,
            parser,
            "flipper_joint_names",
            bridge_control_config,
            ",".join(DEFAULT_FLIPPER_JOINT_NAMES),
        )
    )
    flipper_direction_corrections = resolve_flipper_direction_corrections(
        args,
        parser,
        bridge_control_config,
        flipper_joint_names,
    )
    servo_angular_direction_corrections = resolve_servo_angular_direction_corrections(
        args,
        parser,
        bridge_control_config,
    )
    flipper_target_profile = resolve_str_config(
        args,
        parser,
        "flipper_target_profile",
        bridge_control_config,
        "csv_velocity",
    )
    keyboard_service_bindings = resolve_service_bindings_config(
        bridge_control_config,
        "keyboard_service_bindings",
    )
    gamepad_service_bindings = resolve_service_bindings_config(
        bridge_control_config,
        "gamepad_service_bindings",
    )

    cameras = args.camera
    if args.no_camera_config:
        cameras = cameras or []
    elif args.camera_config:
        camera_config = os.path.expanduser(args.camera_config)
        camera_publish_host = args.camera_publish_host or detect_publish_host()
        cameras = load_cameras_from_yaml(camera_config, camera_publish_host)
        if args.camera:
            cameras.extend(args.camera)

    if not args.dry_run:
        output: OutputAdapter = RosOutput(
            "host_bridge_node",
            args.cmd_vel_topic,
            args.servo_topic,
            args.gripper_velocity_topic,
            args.flipper_jog_topic,
            args.flipper_profile_service,
            args.hybrid_service_ns,
            service_commands,
            vision_detection_service,
            moveit_group,
            events,
        )
    else:
        output = DryRunOutput(events, service_commands)

    if args.video_config or args.video_dry_run or args.video_autostart:
        raise SystemExit(
            "--video-config/--video-dry-run/--video-autostart moved to "
            "host_bridge/video_manager_node.py; start that node separately and pass "
            "--video-manager to host_bridge_node.py"
        )

    video_gateway: Optional[VideoManagerGateway] = None
    if args.video_manager:
        video_gateway = VideoManagerGateway(
            VideoManagerClient(
                args.video_manager_host,
                args.video_manager_port,
                args.video_manager_timeout,
            ),
            events,
        )
        events.emit("video", "external video manager configured", data={
            "host": args.video_manager_host,
            "port": args.video_manager_port,
        })

    server = HostBridgeServer(
        host=args.host,
        port=args.port,
        output=output,
        watchdog_ms=args.watchdog_ms,
        linear_speed=linear_speed,
        angular_speed=angular_speed,
        servo_frame=args.servo_frame,
        default_speed_level=default_speed_level,
        base_linear_levels=base_linear_levels,
        base_angular_levels=base_angular_levels,
        arm_linear_levels=arm_linear_levels,
        arm_angular_levels=arm_angular_levels,
        servo_angular_direction_corrections=servo_angular_direction_corrections,
        gripper_rate_levels=gripper_rate_levels,
        flipper_velocity_levels=flipper_velocity_levels,
        flipper_joint_names=flipper_joint_names,
        flipper_direction_corrections=flipper_direction_corrections,
        flipper_jog_duration=flipper_jog_duration,
        flipper_target_profile=flipper_target_profile,
        flipper_profile_retry_sec=flipper_profile_retry_sec,
        gamepad_deadzone_percent=gamepad_deadzone_percent,
        keyboard_service_bindings=keyboard_service_bindings,
        gamepad_service_bindings=gamepad_service_bindings,
        cameras=cameras,
        video_gateway=video_gateway,
        video_poll_sec=args.video_poll_sec,
        joint_runtime_topic=args.joint_runtime_topic if not args.dry_run else "",
        co2_topic=co2_topic if not args.dry_run else "",
        co2_status_topic=co2_status_topic if not args.dry_run else "",
        events=events,
        control_state_dir=args.control_state_dir or f"/tmp/robot24-control-{args.port}",
    )
    server.core.robot_id = args.robot_id
    if isinstance(output, RosOutput):output.runtime_reset_callback=server.control.runtime_changed
    if args.debug_ui:
        DebugHttpServer(args.debug_host, args.debug_port, server.core, buffer_events).start()

    def handle_signal(signum: int, frame: Any) -> None:
        del signum, frame
        server.stop_event.set()
        server.runtime.stop_event.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("[bridge] stopping", flush=True)
        server.stop_event.set()
        server.runtime.stop_event.set()

    finally:
        server.control.close()
        if isinstance(output,RosOutput):output.close()

if __name__ == "__main__":
    main()
