import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple

from debug_events import EventSink, NullEventSink
from bridge_protocol import MAX_FRAME_BYTES, PROTOCOL_VERSION, clamp, now_ms
from bridge_state import BridgeState, GripperCommand, ServoCommand, TwistCommand
from output_adapters import OutputAdapter


class BridgeCore:
    MODE_VEHICLE = "vehicle"
    MODE_ARM = "arm"
    LIFECYCLE_COMMANDS = {"init", "enable", "disable", "halt", "resume", "recover", "shutdown"}
    MODE_SWITCH_KEYS = {"1", "num_1"}
    EMERGENCY_KEYS = {"space"}
    SPEED_KEY_LEVELS = {
        "6": 1,
        "num_6": 1,
        "7": 2,
        "num_7": 2,
        "8": 3,
        "num_8": 3,
        "9": 4,
        "num_9": 4,
        "0": 5,
        "num_0": 5,
    }
    BASE_LINEAR_LEVELS = {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8}
    BASE_ANGULAR_LEVELS = {1: 0.4, 2: 0.8, 3: 1.0, 4: 1.25, 5: 1.5}
    ARM_LINEAR_LEVELS = {1: 0.04, 2: 0.08, 3: 0.10, 4: 0.125, 5: 0.15}
    ARM_ANGULAR_LEVELS = {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8}
    GRIPPER_RATE_LEVELS = {1: 0.015, 2: 0.03, 3: 0.04, 4: 0.05, 5: 0.06}

    def __init__(
        self,
        output: OutputAdapter,
        watchdog_ms: int,
        linear_speed: float,
        angular_speed: float,
        servo_frame: str = "catch_camera",
        gripper_min_position: float = 0.0,
        gripper_max_position: float = 0.044,
        gripper_initial_position: float = 0.022,
        cameras: Optional[List[Dict[str, Any]]] = None,
        camera_provider: Optional[Callable[[], List[Dict[str, Any]]]] = None,
        camera_stream_handler: Optional[Callable[[Dict[str, Any]], Tuple[bool, int, str, Optional[Dict[str, Any]]]]] = None,
        events: Optional[EventSink] = None,
    ) -> None:
        self.output = output
        self.events = events or NullEventSink()
        self.watchdog_ms = watchdog_ms
        self.base_linear = self.scaled_levels(
            self.BASE_LINEAR_LEVELS, linear_speed, self.BASE_LINEAR_LEVELS[5]
        )
        self.base_angular = self.scaled_levels(
            self.BASE_ANGULAR_LEVELS, angular_speed, self.BASE_ANGULAR_LEVELS[5]
        )
        self.arm_linear = dict(self.ARM_LINEAR_LEVELS)
        self.arm_angular = dict(self.ARM_ANGULAR_LEVELS)
        self.gripper_rates = dict(self.GRIPPER_RATE_LEVELS)
        self.servo_frame = servo_frame
        self.gripper_min_position = gripper_min_position
        self.gripper_max_position = gripper_max_position
        self.cameras = cameras if cameras is not None else self.default_cameras()
        self.camera_provider = camera_provider
        self.camera_stream_handler = camera_stream_handler
        self.state = BridgeState(
            control_mode=self.MODE_VEHICLE,
            gripper_target=self.clamp_gripper(gripper_initial_position),
            last_servo=ServoCommand(frame_id=servo_frame),
        )
        self.state_lock = threading.Lock()

    @staticmethod
    def scaled_levels(levels: Dict[int, float], configured_max: float, default_max: float) -> Dict[int, float]:
        if configured_max <= 0.0:
            return dict(levels)
        scale = configured_max / default_max
        return {level: value * scale for level, value in levels.items()}

    @staticmethod
    def default_cameras() -> List[Dict[str, Any]]:
        return [
            {
                "camera_id": 0,
                "name": "front",
                "online": False,
                "rtsp_url": "",
                "codec": "h264",
                "width": 1280,
                "height": 720,
                "fps": 25,
                "bitrate_kbps": 0,
            }
        ]

    def make_hello(self) -> Dict[str, Any]:
        return {
            "type": "hello",
            "protocol_version": PROTOCOL_VERSION,
            "seq": 0,
            "timestamp_ms": now_ms(),
            "bridge_name": "host_bridge_node",
            "bridge_version": "loopback-mvp",
            "robot_id": "loopback_robot",
        }

    def make_capabilities(self) -> Dict[str, Any]:
        return {
            "type": "capabilities",
            "protocol_version": PROTOCOL_VERSION,
            "seq": 0,
            "timestamp_ms": now_ms(),
            "supports": [
                "operator_input",
                "heartbeat_ack",
                "critical_ack",
                "sync_request",
                "system_snapshot",
                "camera_list_request",
                "camera_list_response",
                "camera_stream_request",
                "emergency_state",
                "watchdog",
                "keyboard_base_arm_mapping",
                "arm_servo_output",
                "gripper_position_output",
                "joint_runtime_states",
                "hybrid_lifecycle_services",
                "service_call_result",
            ],
            "max_frame_bytes": MAX_FRAME_BYTES,
            "watchdog_ms": self.watchdog_ms,
            "keyboard_mapping": {
                "mode_switch": "1",
                "emergency": "space or gamepad l3+r3",
                "speed_levels": {"6": 1, "7": 2, "8": 3, "9": 4, "0": 5},
                "vehicle": {
                    "linear_x": {"positive": "w", "negative": "s"},
                    "angular_z": {"positive": "a", "negative": "d"},
                },
                "arm": {
                    "linear_x": {"positive": "u", "negative": "o"},
                    "linear_y": {"positive": "a", "negative": "d"},
                    "linear_z": {"positive": "w", "negative": "s"},
                    "angular_x": {"positive": "q", "negative": "e"},
                    "angular_y": {"positive": "k", "negative": "i"},
                    "angular_z": {"positive": "j", "negative": "l"},
                    "gripper": {"open": "f", "close": "h"},
                },
            },
            "cameras": self.current_cameras(),
        }

    def make_system_snapshot(self, seq: int) -> Dict[str, Any]:
        cameras = self.current_cameras()
        with self.state_lock:
            return {
                "type": "system_snapshot",
                "protocol_version": PROTOCOL_VERSION,
                "seq": seq,
                "timestamp_ms": now_ms(),
                "control_mode": self.state.control_mode,
                "emergency": {
                    "active": self.state.emergency_active,
                    "source": self.state.emergency_source,
                },
                "motor": {
                    "initialized": self.state.motor_initialized,
                    "enabled": self.state.motor_enabled,
                },
                "modules": {
                    "base": "online",
                    "arm": "online",
                    "camera": "online" if any(cam.get("online", False) for cam in cameras) else "offline",
                },
                "last_error": {
                    "code": self.state.last_error_code,
                    "message": self.state.last_error_message,
                },
                "last_operator_input_seq": self.state.last_operator_seq,
                "watchdog_active": self.state.watchdog_active,
                "speed_level": self.state.speed_level,
                "last_twist": self.state.last_twist.to_dict(),
                "last_servo": self.state.last_servo.to_dict(),
                "gripper_target": self.state.gripper_target,
            }

    def make_camera_list_response(self, seq: int) -> Dict[str, Any]:
        return {
            "type": "camera_list_response",
            "protocol_version": PROTOCOL_VERSION,
            "seq": seq,
            "timestamp_ms": now_ms(),
            "cameras": self.current_cameras(),
        }

    def make_camera_info(self, camera: Dict[str, Any]) -> Dict[str, Any]:
        message = dict(camera)
        message["type"] = "camera_info"
        message["protocol_version"] = PROTOCOL_VERSION
        message["seq"] = 0
        message["timestamp_ms"] = now_ms()
        return message

    def state_snapshot(self) -> Dict[str, Any]:
        with self.state_lock:
            state = self.state.to_dict()
        state["cameras"] = self.current_cameras()
        state["watchdog_ms"] = self.watchdog_ms
        state["max_frame_bytes"] = MAX_FRAME_BYTES
        return state

    def current_cameras(self) -> List[Dict[str, Any]]:
        if self.camera_provider is None:
            return list(self.cameras)
        return self.camera_provider()

    def make_ack(self, msg: Dict[str, Any], ok: bool = True, code: int = 0, message: str = "ok") -> Dict[str, Any]:
        return {
            "type": "ack",
            "protocol_version": PROTOCOL_VERSION,
            "ack_type": str(msg.get("type", "unknown")),
            "seq": int(msg.get("seq", 0) or 0),
            "ok": ok,
            "code": code,
            "message": message,
            "timestamp_ms": now_ms(),
        }

    def make_protocol_error(self, msg: Dict[str, Any], code: int, message: str) -> Dict[str, Any]:
        with self.state_lock:
            self.state.last_error_code = code
            self.state.last_error_message = message
        return {
            "type": "protocol_error",
            "protocol_version": PROTOCOL_VERSION,
            "seq": int(msg.get("seq", 0) or 0),
            "code": code,
            "message": message,
            "timestamp_ms": now_ms(),
        }

    def make_emergency_state(self, message: str) -> Dict[str, Any]:
        with self.state_lock:
            active = self.state.emergency_active
            source = self.state.emergency_source
        return {
            "type": "emergency_state",
            "protocol_version": PROTOCOL_VERSION,
            "seq": 0,
            "timestamp_ms": now_ms(),
            "active": active,
            "source": source,
            "message": message,
        }

    def make_service_call_result(
        self,
        msg: Dict[str, Any],
        *,
        command: str,
        service: str,
        ok: bool,
        code: int,
        message: str,
        duration_ms: int,
    ) -> Dict[str, Any]:
        return {
            "type": "service_call_result",
            "protocol_version": PROTOCOL_VERSION,
            "seq": int(msg.get("seq", 0) or 0),
            "timestamp_ms": now_ms(),
            "request_type": str(msg.get("type", "unknown")),
            "command": command,
            "service": service,
            "ok": ok,
            "code": code,
            "message": message,
            "duration_ms": duration_ms,
        }

    def call_configured_service_result(
        self,
        msg: Dict[str, Any],
        command: str,
        params: Optional[Dict[str, Any]] = None,
    ) -> Optional[Tuple[bool, int, str, str, int]]:
        service = self.output.command_service_name(command, params)
        if not service:
            return None
        started = time.monotonic()
        ok, code, message = self.output.call_command_service(command, params)
        duration_ms = int((time.monotonic() - started) * 1000)
        return ok, code, message, service, duration_ms

    def publish_zero(self, source: str) -> None:
        self.publish_twist(TwistCommand(0.0, 0.0, source))
        self.publish_servo(ServoCommand(frame_id=self.servo_frame, source=source))

    def reset_operator_input_session(self, source: str) -> None:
        with self.state_lock:
            self.state.last_operator_seq = 0
            self.state.last_valid_input_ms = 0
            self.state.watchdog_active = False
            self.state.last_pressed_keys = []
            self.state.last_gamepad_buttons = []
        self.publish_zero(source)
        self.events.emit("operator_input", "operator input session reset", data={
            "source": source,
        })

    def publish_twist(self, twist: TwistCommand) -> None:
        with self.state_lock:
            self.state.last_twist = twist
        self.output.publish_twist(twist)

    def publish_servo(self, servo: ServoCommand) -> None:
        with self.state_lock:
            self.state.last_servo = servo
        self.output.publish_servo(servo)

    def publish_gripper(self, gripper: GripperCommand) -> None:
        with self.state_lock:
            self.state.gripper_target = gripper.position
            self.state.last_gripper = gripper
        self.output.publish_gripper(gripper)

    def handle_message(self, msg: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
        msg_type = str(msg.get("type", ""))
        seq = int(msg.get("seq", 0) or 0)
        self.events.emit("protocol", "message received", data={"type": msg_type, "seq": seq})

        if not msg_type:
            self.events.emit("protocol", "missing message type", level="error", data={"seq": seq})
            yield self.make_protocol_error(msg, 2101, "missing type")
            return

        if msg.get("protocol_version", PROTOCOL_VERSION) != PROTOCOL_VERSION:
            self.events.emit("protocol", "unsupported protocol version", level="error", data={"seq": seq})
            yield self.make_protocol_error(msg, 2103, "unsupported protocol_version")
            return

        if msg_type == "heartbeat":
            self.events.emit("protocol", "heartbeat ack", data={"seq": seq})
            yield {
                "type": "heartbeat_ack",
                "protocol_version": PROTOCOL_VERSION,
                "seq": seq,
                "timestamp_ms": now_ms(),
                "server_time_ms": now_ms(),
            }
            return

        if msg_type in ("sync_request", "system_snapshot_request"):
            self.events.emit("sync", "system snapshot requested", data={"seq": seq})
            yield self.make_system_snapshot(seq)
            return

        if msg_type == "camera_list_request":
            cameras = self.current_cameras()
            self.events.emit("camera", "camera list requested", data={"seq": seq, "count": len(cameras)})
            yield self.make_camera_list_response(seq)
            return

        if msg_type == "camera_stream_request":
            if self.camera_stream_handler is None:
                yield self.make_ack(msg, False, 2400, "video manager unavailable")
                return
            ok, code, message, camera = self.camera_stream_handler(msg.get("params", {}) or {})
            yield self.make_ack(msg, ok, code, message)
            if camera is not None:
                yield self.make_camera_info(camera)
            return

        if msg_type == "operator_input":
            for response in self.handle_operator_input(msg):
                yield response
            return

        if msg_type == "emergency_stop":
            params = msg.get("params", {}) or {}
            source = str(params.get("source", "unknown"))
            with self.state_lock:
                self.state.emergency_active = True
                self.state.emergency_source = source
                self.state.watchdog_active = False
            self.events.emit("emergency", "emergency stop accepted", level="warning", data={
                "seq": seq,
                "source": source,
            })
            self.publish_zero("emergency_stop")
            service_result = self.call_configured_service_result(msg, "emergency_stop", params)
            if service_result is None:
                yield self.make_ack(msg, True, 0, "emergency active")
            else:
                ok, code, message, service, duration_ms = service_result
                yield self.make_ack(msg, ok, code, message)
                yield self.make_service_call_result(
                    msg,
                    command="emergency_stop",
                    service=service,
                    ok=ok,
                    code=code,
                    message=message,
                    duration_ms=duration_ms,
                )
            yield self.make_emergency_state("emergency active")
            return

        if msg_type == "clear_emergency":
            with self.state_lock:
                self.state.emergency_active = False
                self.state.emergency_source = ""
            self.events.emit("emergency", "emergency cleared", data={"seq": seq})
            yield self.make_ack(msg, True, 0, "emergency cleared")
            yield self.make_emergency_state("emergency cleared")
            return

        if msg_type == "system_command":
            command = str(msg.get("command", ""))
            if command == "clear_emergency":
                with self.state_lock:
                    self.state.emergency_active = False
                    self.state.emergency_source = ""
                self.events.emit("emergency", "emergency cleared", data={"seq": seq, "via": "system_command"})
                yield self.make_ack(msg, True, 0, "emergency cleared")
                yield self.make_emergency_state("emergency cleared")
            elif command == "set_control_mode":
                params = msg.get("params", {}) or {}
                mode = self.normalize_mode(str(params.get("mode", "")))
                if not mode:
                    yield self.make_ack(msg, False, 2201, "invalid control mode")
                    return
                service_result = self.call_configured_service_result(msg, command, params)
                if service_result is not None:
                    ok, code, message, service, duration_ms = service_result
                    if ok:
                        with self.state_lock:
                            self.state.control_mode = mode
                            self.state.last_pressed_keys = []
                        self.publish_zero("set_control_mode")
                        self.events.emit("mode", "control mode set", data={"seq": seq, "mode": mode})
                    yield self.make_ack(msg, ok, code, message)
                    yield self.make_service_call_result(
                        msg,
                        command=command,
                        service=service,
                        ok=ok,
                        code=code,
                        message=message,
                        duration_ms=duration_ms,
                    )
                    return
                with self.state_lock:
                    self.state.control_mode = mode
                    self.state.last_pressed_keys = []
                self.publish_zero("set_control_mode")
                self.events.emit("mode", "control mode set", data={"seq": seq, "mode": mode})
                yield self.make_ack(msg, True, 0, f"control mode set to {mode}")
            elif command in self.LIFECYCLE_COMMANDS:
                service = self.output.lifecycle_service_name(command)
                started = time.monotonic()
                ok, code, message = self.output.call_lifecycle(command)
                duration_ms = int((time.monotonic() - started) * 1000)
                self.events.emit("lifecycle", "lifecycle command handled", data={
                    "seq": seq,
                    "command": command,
                    "service": service,
                    "ok": ok,
                    "code": code,
                    "message": message,
                    "duration_ms": duration_ms,
                }, level="info" if ok else "error")
                yield self.make_ack(msg, ok, code, message)
                yield self.make_service_call_result(
                    msg,
                    command=command,
                    service=service,
                    ok=ok,
                    code=code,
                    message=message,
                    duration_ms=duration_ms,
                )
            else:
                yield self.make_ack(msg, True, 0, f"system command {command} accepted")
            return

        self.events.emit("protocol", "unsupported message type", level="warning", data={
            "seq": seq,
            "type": msg_type,
        })
        yield self.make_protocol_error(msg, 1001, f"unsupported message type: {msg_type}")

    def handle_operator_input(self, msg: Dict[str, Any]) -> List[Dict[str, Any]]:
        seq = int(msg.get("seq", 0) or 0)
        timestamp = int(msg.get("timestamp_ms", 0) or 0)
        ttl_ms = int(msg.get("ttl_ms", 0) or 0)
        current_ms = now_ms()
        pressed = self.normalized_pressed_keys(msg)
        gamepad = msg.get("gamepad", {}) if isinstance(msg.get("gamepad", {}), dict) else {}
        current_buttons = self.active_gamepad_buttons(gamepad)

        status: Optional[str] = None
        with self.state_lock:
            if seq <= self.state.last_operator_seq:
                status = "dropped_old_seq"
            elif ttl_ms > 0 and timestamp > 0 and current_ms > timestamp + ttl_ms:
                self.state.last_operator_seq = seq
                status = "dropped_expired"
            elif self.state.emergency_active:
                self.state.last_operator_seq = seq
                self.state.last_pressed_keys = sorted(pressed)
                self.state.last_gamepad_buttons = sorted(current_buttons)
                status = "ignored_emergency"

        if status == "dropped_old_seq":
            self.events.emit("operator_input", "operator input dropped old seq", level="warning", data={"seq": seq})
            return [self.input_status(seq, status)]

        if status == "dropped_expired":
            self.events.emit("operator_input", "operator input dropped expired", level="warning", data={"seq": seq})
            return [self.input_status(seq, status)]

        if status == "ignored_emergency":
            self.publish_zero("emergency_active")
            with self.state_lock:
                emergency_source = self.state.emergency_source
            self.events.emit("operator_input", "operator input ignored emergency", level="warning", data={
                "seq": seq,
                "emergency_source": emergency_source,
                "pressed_keys": sorted(pressed),
                "active_gamepad_buttons": sorted(current_buttons),
            })
            return [self.input_status(seq, status)]

        key_edges: Set[str]
        button_edges: Set[str]
        with self.state_lock:
            previous_pressed = set(self.state.last_pressed_keys)
            key_edges = pressed - previous_pressed
            button_edges = current_buttons - set(self.state.last_gamepad_buttons)
            self.state.last_pressed_keys = sorted(pressed)
            self.state.last_gamepad_buttons = sorted(current_buttons)
            self.state.last_operator_seq = seq
            self.state.watchdog_active = False

        emergency_source = self.emergency_input_source(key_edges, current_buttons, button_edges)
        if emergency_source:
            with self.state_lock:
                self.state.emergency_active = True
                self.state.emergency_source = emergency_source
                self.state.watchdog_active = False
            self.publish_zero("operator_input_emergency")
            self.events.emit("emergency", "operator input emergency key accepted", level="warning", data={
                "seq": seq,
                "source": emergency_source,
                "pressed_keys": sorted(pressed),
                "key_edges": sorted(key_edges),
                "active_gamepad_buttons": sorted(current_buttons),
                "button_edges": sorted(button_edges),
            })
            return [
                self.input_status(seq, "emergency_key"),
                self.make_emergency_state("emergency active"),
            ]

        self.apply_mode_and_speed_edges(key_edges, button_edges, seq)
        twist, servo, gripper = self.operator_input_to_outputs(pressed, gamepad, current_ms)
        self.publish_twist(twist)
        self.publish_servo(servo)
        if gripper is not None:
            self.publish_gripper(gripper)

        with self.state_lock:
            self.state.last_valid_input_ms = current_ms
            mode = self.state.control_mode
            speed_level = self.state.speed_level
            pressed_snapshot = list(self.state.last_pressed_keys)
        self.events.emit("operator_input", "operator input accepted", data={
            "seq": seq,
            "mode": mode,
            "speed_level": speed_level,
            "cmd_vel": {
                "linear_x": twist.linear_x,
                "angular_z": twist.angular_z,
                "source": twist.source,
            },
            "servo": servo.to_dict(),
            "gripper": gripper.to_dict() if gripper else None,
            "pressed_keys": pressed_snapshot,
        })
        return [self.input_status(seq, "accepted")]

    def input_status(self, seq: int, status: str) -> Dict[str, Any]:
        with self.state_lock:
            twist = self.state.last_twist
            servo = self.state.last_servo
            gripper_target = self.state.gripper_target
            watchdog_active = self.state.watchdog_active
            emergency_active = self.state.emergency_active
            control_mode = self.state.control_mode
            speed_level = self.state.speed_level
        return {
            "type": "system_status",
            "protocol_version": PROTOCOL_VERSION,
            "seq": 0,
            "timestamp_ms": now_ms(),
            "last_operator_input_seq": seq,
            "operator_input_status": status,
            "watchdog_active": watchdog_active,
            "emergency_active": emergency_active,
            "control_mode": control_mode,
            "speed_level": speed_level,
            "cmd_vel": {
                "linear_x": twist.linear_x,
                "angular_z": twist.angular_z,
                "source": twist.source,
            },
            "servo": servo.to_dict(),
            "gripper": {
                "target": gripper_target,
            },
        }

    def operator_input_to_outputs(
        self,
        pressed: Set[str],
        gamepad: Dict[str, Any],
        current_ms: int,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand]]:
        axes = gamepad.get("axes", {}) if isinstance(gamepad.get("axes", {}), dict) else {}
        buttons = gamepad.get("buttons", {}) if isinstance(gamepad.get("buttons", {}), dict) else {}
        connected = bool(gamepad.get("connected", False))

        with self.state_lock:
            mode = self.state.control_mode
            speed_level = self.state.speed_level
            previous_input_ms = self.state.last_valid_input_ms
            gripper_target = self.state.gripper_target

        if mode == self.MODE_ARM:
            return self.operator_input_to_arm_outputs(
                pressed, axes, buttons, connected, speed_level, previous_input_ms, current_ms, gripper_target
            )

        return self.operator_input_to_vehicle_outputs(pressed, axes, connected, speed_level)

    def operator_input_to_vehicle_outputs(
        self,
        pressed: Set[str],
        axes: Dict[str, Any],
        connected: bool,
        speed_level: int,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand]]:
        linear_axis = self.axis_value(pressed, "w", "s")
        angular_axis = self.axis_value(pressed, "a", "d")

        if connected:
            linear_axis += self.float_value(axes.get("left_y", 0.0))
            angular_axis += self.float_value(axes.get("right_x", 0.0))

        twist = TwistCommand(
            linear_x=clamp(linear_axis, -1.0, 1.0) * self.base_linear[speed_level],
            angular_z=clamp(angular_axis, -1.0, 1.0) * self.base_angular[speed_level],
            source="operator_input.vehicle",
        )
        servo = ServoCommand(frame_id=self.servo_frame, source="operator_input.vehicle_zero")
        return twist, servo, None

    def operator_input_to_arm_outputs(
        self,
        pressed: Set[str],
        axes: Dict[str, Any],
        buttons: Dict[str, Any],
        connected: bool,
        speed_level: int,
        previous_input_ms: int,
        current_ms: int,
        gripper_target: float,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand]]:
        linear = self.arm_linear[speed_level]
        angular = self.arm_angular[speed_level]

        linear_x_axis = self.axis_value(pressed, "u", "o")
        linear_y_axis = self.axis_value(pressed, "a", "d")
        linear_z_axis = self.axis_value(pressed, "w", "s")
        angular_x_axis = self.axis_value(pressed, "q", "e")
        angular_y_axis = self.axis_value(pressed, "k", "i")
        angular_z_axis = self.axis_value(pressed, "j", "l")

        if connected:
            linear_y_axis += self.float_value(axes.get("left_x", 0.0))
            linear_z_axis += self.float_value(axes.get("left_y", 0.0))
            linear_x_axis += self.float_value(axes.get("lt", 0.0)) - self.float_value(axes.get("rt", 0.0))
            angular_x_axis += self.bool_value(buttons.get("lb", False)) - self.bool_value(buttons.get("rb", False))
            angular_y_axis += -self.float_value(axes.get("right_y", 0.0))
            angular_z_axis += self.float_value(axes.get("right_x", 0.0))

        twist = TwistCommand(0.0, 0.0, "operator_input.arm_base_zero")
        servo = ServoCommand(
            linear_x=clamp(linear_x_axis, -1.0, 1.0) * linear,
            linear_y=clamp(linear_y_axis, -1.0, 1.0) * linear,
            linear_z=clamp(linear_z_axis, -1.0, 1.0) * linear,
            angular_x=clamp(angular_x_axis, -1.0, 1.0) * angular,
            angular_y=clamp(angular_y_axis, -1.0, 1.0) * angular,
            angular_z=clamp(angular_z_axis, -1.0, 1.0) * angular,
            frame_id=self.servo_frame,
            source="operator_input.arm",
        )

        dt = 0.0
        if previous_input_ms > 0:
            dt = clamp((current_ms - previous_input_ms) / 1000.0, 0.0, 0.2)
        gripper_axis = self.axis_value(pressed, "f", "h")
        gripper = None
        if abs(gripper_axis) > 0.0 and dt > 0.0:
            target = self.clamp_gripper(
                gripper_target + gripper_axis * self.gripper_rates[speed_level] * dt
            )
            gripper = GripperCommand(position=target, source="operator_input.arm")

        return twist, servo, gripper

    def normalized_pressed_keys(self, msg: Dict[str, Any]) -> Set[str]:
        keyboard = msg.get("keyboard", {}) if isinstance(msg.get("keyboard", {}), dict) else {}
        raw_pressed = keyboard.get("pressed_keys", []) or []
        if not isinstance(raw_pressed, list):
            return set()

        pressed: Set[str] = set()
        for raw_key in raw_pressed:
            key = str(raw_key).strip().lower()
            if not key:
                continue
            pressed.add(key)
            if key.startswith("num_") and len(key) == 5 and key[-1].isdigit():
                pressed.add(key[-1])
        return pressed

    def active_gamepad_buttons(self, gamepad: Dict[str, Any]) -> Set[str]:
        buttons = gamepad.get("buttons", {}) if isinstance(gamepad.get("buttons", {}), dict) else {}
        return {name for name, value in buttons.items() if bool(value)}

    def emergency_input_source(
        self,
        key_edges: Set[str],
        active_buttons: Set[str],
        button_edges: Set[str],
    ) -> str:
        if self.EMERGENCY_KEYS & key_edges:
            return "keyboard_space"
        if {"l3", "r3"}.issubset(active_buttons) and ({"l3", "r3"} & button_edges):
            return "gamepad_l3_r3"
        return ""

    def apply_mode_and_speed_edges(self, key_edges: Set[str], button_edges: Set[str], seq: int) -> None:
        mode_changed = False
        speed_changed = False

        with self.state_lock:
            if self.MODE_SWITCH_KEYS & key_edges:
                self.state.control_mode = (
                    self.MODE_ARM if self.state.control_mode == self.MODE_VEHICLE else self.MODE_VEHICLE
                )
                mode_changed = True

            if "dpad_up" in button_edges:
                self.state.control_mode = self.MODE_VEHICLE
                mode_changed = True
            elif "dpad_down" in button_edges:
                self.state.control_mode = self.MODE_ARM
                mode_changed = True

            for key in sorted(key_edges):
                if key in self.SPEED_KEY_LEVELS:
                    self.state.speed_level = self.SPEED_KEY_LEVELS[key]
                    speed_changed = True

            mode = self.state.control_mode
            speed_level = self.state.speed_level

        if mode_changed:
            self.publish_zero("mode_switch")
            self.events.emit("mode", "control mode switched", data={"seq": seq, "mode": mode})
        if speed_changed:
            self.events.emit("speed", "speed level set", data={"seq": seq, "speed_level": speed_level})

    def normalize_mode(self, mode: str) -> str:
        normalized = mode.strip().lower()
        if normalized in ("vehicle", "base"):
            return self.MODE_VEHICLE
        if normalized == "arm":
            return self.MODE_ARM
        return ""

    def axis_value(self, pressed: Set[str], positive_key: str, negative_key: str) -> float:
        value = 0.0
        if positive_key in pressed:
            value += 1.0
        if negative_key in pressed:
            value -= 1.0
        return value

    @staticmethod
    def float_value(value: Any) -> float:
        try:
            return float(value or 0.0)
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def bool_value(value: Any) -> float:
        return 1.0 if bool(value) else 0.0

    def clamp_gripper(self, value: float) -> float:
        return clamp(value, self.gripper_min_position, self.gripper_max_position)

    def check_watchdog(self) -> List[Dict[str, Any]]:
        emit_status = False
        with self.state_lock:
            if (
                self.state.last_valid_input_ms > 0
                and not self.state.emergency_active
                and not self.state.watchdog_active
                and now_ms() - self.state.last_valid_input_ms > self.watchdog_ms
            ):
                self.state.watchdog_active = True
                emit_status = True
                seq = self.state.last_operator_seq
            else:
                seq = self.state.last_operator_seq
        if not emit_status:
            return []
        self.publish_zero("watchdog_timeout")
        self.events.emit("watchdog", "operator input watchdog timeout", level="warning", data={"seq": seq})
        return [self.input_status(seq, "watchdog_timeout")]


class BridgeRuntime:
    def __init__(self, core: BridgeCore, broadcast: Callable[[Dict[str, Any]], None]) -> None:
        self.core = core
        self.broadcast = broadcast
        self.stop_event = threading.Event()

    def start_watchdog(self) -> None:
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def _watchdog_loop(self) -> None:
        while not self.stop_event.wait(0.05):
            for event in self.core.check_watchdog():
                self.broadcast(event)
