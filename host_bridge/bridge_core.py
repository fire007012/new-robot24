import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple

from debug_events import EventSink, NullEventSink
from bridge_protocol import MAX_FRAME_BYTES, PROTOCOL_VERSION, clamp, now_ms
from bridge_state import BridgeState, FlipperCommand, GripperCommand, ServoCommand, TwistCommand
from output_adapters import OutputAdapter


class BridgeCore:
    MODE_VEHICLE = "vehicle"
    MODE_ARM = "arm"
    LIFECYCLE_COMMANDS = {"init", "enable", "disable", "halt", "resume", "recover", "shutdown"}
    MODE_SWITCH_KEYS = {"1", "num_1"}
    MODE_SWITCH_GAMEPAD_BUTTONS = {"menu", "options", "start"}
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
    GRIPPER_RATE_LEVELS = {1: 0.004, 2: 0.008, 3: 0.012, 4: 0.016, 5: 0.02}

    def __init__(
        self,
        output: OutputAdapter,
        watchdog_ms: int,
        linear_speed: float,
        angular_speed: float,
        servo_frame: str = "grasp_tcp",
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
        camera_provider: Optional[Callable[[], List[Dict[str, Any]]]] = None,
        camera_stream_handler: Optional[Callable[[Dict[str, Any]], Tuple[bool, int, str, Optional[Dict[str, Any]]]]] = None,
        events: Optional[EventSink] = None,
    ) -> None:
        self.output = output
        self.events = events or NullEventSink()
        self.watchdog_ms = watchdog_ms
        self.base_linear = self.normalized_levels(
            base_linear_levels,
            self.scaled_levels(self.BASE_LINEAR_LEVELS, linear_speed, self.BASE_LINEAR_LEVELS[5]),
        )
        self.base_angular = self.normalized_levels(
            base_angular_levels,
            self.scaled_levels(self.BASE_ANGULAR_LEVELS, angular_speed, self.BASE_ANGULAR_LEVELS[5]),
        )
        self.arm_linear = self.normalized_levels(arm_linear_levels, self.ARM_LINEAR_LEVELS)
        self.arm_angular = self.normalized_levels(arm_angular_levels, self.ARM_ANGULAR_LEVELS)
        self.servo_angular_direction_corrections = self.normalized_axis_direction_corrections(
            servo_angular_direction_corrections
        )
        self.gripper_rates = self.normalized_levels(gripper_rate_levels, self.GRIPPER_RATE_LEVELS)
        self.flipper_velocities = self.normalized_levels(
            flipper_velocity_levels,
            {1: 0.2, 2: 0.4, 3: 0.55, 4: 0.7, 5: 0.8},
        )
        self.servo_frame = servo_frame
        self.flipper_joint_names = flipper_joint_names or [
            "left_front_arm_joint",
            "right_front_arm_joint",
            "left_rear_arm_joint",
            "right_rear_arm_joint",
        ]
        self.flipper_direction_corrections = self.normalized_flipper_direction_corrections(
            flipper_direction_corrections
        )
        self.flipper_jog_duration = flipper_jog_duration
        self.flipper_target_profile = flipper_target_profile
        self.flipper_profile_retry_sec = flipper_profile_retry_sec
        self.last_flipper_profile_attempt = 0.0
        self.gamepad_deadzone = clamp(gamepad_deadzone_percent / 100.0, 0.0, 1.0)
        self.keyboard_service_bindings = self.normalized_service_bindings(keyboard_service_bindings)
        self.gamepad_service_bindings = self.normalized_service_bindings(gamepad_service_bindings)
        self.cameras = cameras if cameras is not None else self.default_cameras()
        self.camera_provider = camera_provider
        self.camera_stream_handler = camera_stream_handler
        self.state = BridgeState(
            control_mode=self.MODE_VEHICLE,
            speed_level=self.clamp_speed_level(default_speed_level),
            gripper_velocity=0.0,
            last_servo=ServoCommand(frame_id=servo_frame),
            last_gripper=GripperCommand(velocity=0.0, source="startup"),
            last_flipper=self.zero_flipper_command("startup"),
            flipper_profile_target=flipper_target_profile,
        )
        self.robot_id = "loopback_robot"
        self.state_lock = threading.Lock()
        self.last_valid_input_monotonic = 0.0
        self.operator_input_clock_offset_ms: Optional[int] = None

    @staticmethod
    def scaled_levels(levels: Dict[int, float], configured_max: float, default_max: float) -> Dict[int, float]:
        import bridge_mapping as _mapping
        return _mapping.scaled_levels(levels, configured_max, default_max)

    @staticmethod
    def normalized_levels(configured: Optional[Dict[int, float]], defaults: Dict[int, float]) -> Dict[int, float]:
        import bridge_mapping as _mapping
        return _mapping.normalized_levels(configured, defaults)

    @staticmethod
    def default_cameras() -> List[Dict[str, Any]]:
        import bridge_messages as _messages
        return _messages.default_cameras()

    def make_hello(self) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_hello(self.robot_id)

    def make_capabilities(self) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_capabilities(self)

    def make_system_snapshot(self, seq: int) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_system_snapshot(self, seq)

    def make_camera_list_response(self, seq: int) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_camera_list_response(seq, self.current_cameras())

    def make_camera_info(self, camera: Dict[str, Any]) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_camera_info(camera)

    def make_co2_data(self, ppm: int) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_co2_data(ppm)

    def make_arm_named_targets(
        self,
        seq: int,
        targets: List[Dict[str, str]],
        message: str = "",
    ) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_arm_named_targets(seq, targets, message)

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
        import bridge_messages as _messages
        return _messages.build_ack(msg, ok=ok, code=code, message=message)

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
        import bridge_messages as _messages
        with self.state_lock:
            active = self.state.emergency_active
            source = self.state.emergency_source
        return _messages.build_emergency_state(active, source, message)

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
        import bridge_messages as _messages
        return _messages.build_service_call_result(
            msg, command=command, service=service, ok=ok, code=code, message=message, duration_ms=duration_ms
        )

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

    def call_named_service_result(self, service: str) -> Tuple[bool, int, str, int]:
        started = time.monotonic()
        ok, code, message = self.output.call_named_service(service)
        duration_ms = int((time.monotonic() - started) * 1000)
        return ok, code, message, duration_ms

    def publish_zero(self, source: str) -> None:
        self.publish_twist(TwistCommand(0.0, 0.0, source))
        self.publish_servo(ServoCommand(frame_id=self.servo_frame, source=source))
        self.publish_gripper(GripperCommand(velocity=0.0, source=source))
        self.publish_flipper(self.zero_flipper_command(source))

    def reset_operator_input_session(self, source: str) -> None:
        with self.state_lock:
            self.state.last_operator_seq = 0
            self.state.last_valid_input_ms = 0
            self.last_valid_input_monotonic = 0.0
            self.state.watchdog_active = False
            self.state.last_pressed_keys = []
            self.state.last_gamepad_buttons = []
        self.operator_input_clock_offset_ms = None
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
            self.state.gripper_velocity = gripper.velocity
            self.state.last_gripper = gripper
        self.output.publish_gripper(gripper)

    def publish_flipper(self, flipper: FlipperCommand) -> None:
        with self.state_lock:
            self.state.last_flipper = flipper
        self.output.publish_flipper(flipper)

    def zero_flipper_command(self, source: str) -> FlipperCommand:
        return FlipperCommand(
            joint_names=list(self.flipper_joint_names),
            velocities=[0.0] * len(self.flipper_joint_names),
            duration=self.flipper_jog_duration,
            source=source,
        )

    def ensure_flipper_profile(self, force: bool = False) -> None:
        if not self.flipper_joint_names or not self.flipper_target_profile:
            return

        with self.state_lock:
            if not force and self.state.flipper_profile_result == "ok":
                return

        now = time.monotonic()
        if not force and now - self.last_flipper_profile_attempt < self.flipper_profile_retry_sec:
            return
        self.last_flipper_profile_attempt = now

        ok, _code, message, detail = self.output.call_flipper_profile(self.flipper_target_profile)
        result = "ok" if ok else "unavailable" if detail.get("service_unavailable") else "rejected"
        with self.state_lock:
            self.state.flipper_profile_result = result
            self.state.flipper_profile_message = message
            self.state.flipper_profile_target = self.flipper_target_profile

        event_level = "info" if ok else "warning"
        self.events.emit("flipper", "flipper profile checked", level=event_level, data={
            "target_profile": self.flipper_target_profile,
            "result": result,
            "message": message,
            **detail,
        })

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
                        if mode == self.MODE_VEHICLE:
                            self.ensure_flipper_profile(force=True)
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
                if mode == self.MODE_VEHICLE:
                    self.ensure_flipper_profile(force=True)
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
            elif command == "request_arm_named_targets":
                if not self.output.supports_arm_named_targets():
                    message = "MoveIt unavailable"
                    self.events.emit("moveit", "arm named targets unavailable", level="warning", data={
                        "seq": seq,
                        "reason": message,
                    })
                    yield self.make_ack(msg, False, 2303, message)
                    yield self.make_arm_named_targets(seq, [], message)
                    return
                ok, code, message, targets = self.output.list_arm_named_targets()
                self.events.emit("moveit", "arm named targets requested", data={
                    "seq": seq,
                    "ok": ok,
                    "code": code,
                    "count": len(targets),
                }, level="info" if ok else "error")
                yield self.make_ack(msg, ok, code, message)
                yield self.make_arm_named_targets(seq, targets, message)
            elif command == "move_arm_named_target":
                params = msg.get("params", {}) or {}
                target = str(params.get("target", "")).strip()
                started = time.monotonic()
                ok, code, message = self.output.move_arm_named_target(target)
                duration_ms = int((time.monotonic() - started) * 1000)
                self.events.emit("moveit", "move arm named target handled", data={
                    "seq": seq,
                    "target": target,
                    "ok": ok,
                    "code": code,
                    "duration_ms": duration_ms,
                }, level="info" if ok else "error")
                yield self.make_ack(msg, ok, code, message)
                yield self.make_service_call_result(
                    msg,
                    command=command,
                    service=f"moveit:{getattr(self.output, 'moveit_group_name', 'arm')}/{target}",
                    ok=ok,
                    code=code,
                    message=message,
                    duration_ms=duration_ms,
                )
            else:
                params = msg.get("params", {}) or {}
                service_result = self.call_configured_service_result(msg, command, params)
                if service_result is not None:
                    ok, code, message, service, duration_ms = service_result
                    self.events.emit("service", "system command service handled", data={
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
        raw_age_ms, effective_age_ms, clock_offset_ms = self.operator_input_age_ms(timestamp, current_ms)

        status: Optional[str] = None
        previous_seq = 0
        emergency_source = ""
        with self.state_lock:
            previous_seq = self.state.last_operator_seq
            if seq <= self.state.last_operator_seq:
                status = "dropped_old_seq"
            elif self.state.emergency_active:
                self.state.last_operator_seq = seq
                self.state.last_pressed_keys = sorted(pressed)
                self.state.last_gamepad_buttons = sorted(current_buttons)
                emergency_source = self.state.emergency_source
                status = "ignored_emergency"

        drop_debug = {
            "seq": seq,
            "previous_seq": previous_seq,
            "timestamp_ms": timestamp,
            "ttl_ms": ttl_ms,
            "current_ms": current_ms,
            "raw_age_ms": raw_age_ms,
            "effective_age_ms": effective_age_ms,
            "clock_offset_ms": clock_offset_ms,
            "mode": str(msg.get("mode", "")),
            "pressed_keys": sorted(pressed),
            "active_gamepad_buttons": sorted(current_buttons),
            "gamepad_connected": bool(gamepad.get("connected", False)),
        }

        if status == "dropped_old_seq":
            self.events.emit(
                "operator_input",
                "operator input dropped old seq",
                level="warning",
                data=drop_debug,
            )
            return [self.input_status(seq, status)]

        if status == "ignored_emergency":
            self.publish_zero("emergency_active")
            self.events.emit(
                "operator_input",
                "operator input ignored emergency",
                level="warning",
                data={
                    **drop_debug,
                    "emergency_source": emergency_source,
                },
            )
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

        service_results = self.handle_input_service_bindings(msg, key_edges, button_edges)
        self.apply_mode_and_speed_edges(key_edges, button_edges, seq)
        twist, servo, gripper, flipper = self.operator_input_to_outputs(pressed, gamepad, current_ms)
        self.publish_twist(twist)
        self.publish_servo(servo)
        if gripper is not None:
            self.publish_gripper(gripper)
        self.publish_flipper(flipper)
        if flipper.source == "operator_input.vehicle":
            self.ensure_flipper_profile(force=False)

        with self.state_lock:
            self.state.last_valid_input_ms = current_ms
            self.last_valid_input_monotonic = time.monotonic()
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
            "flipper": flipper.to_dict(),
            "pressed_keys": pressed_snapshot,
        })
        return [self.input_status(seq, "accepted"), *service_results]

    def input_status(self, seq: int, status: str) -> Dict[str, Any]:
        import bridge_messages as _messages
        return _messages.build_input_status(self, seq, status)

    def operator_input_to_outputs(
        self,
        pressed: Set[str],
        gamepad: Dict[str, Any],
        current_ms: int,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand], FlipperCommand]:
        axes = self.normalized_gamepad_mapping(gamepad.get("axes", {}))
        buttons = self.normalized_gamepad_mapping(gamepad.get("buttons", {}))
        connected = bool(gamepad.get("connected", False))

        with self.state_lock:
            mode = self.state.control_mode
            speed_level = self.state.speed_level
            previous_input_ms = self.state.last_valid_input_ms

        if mode == self.MODE_ARM:
            return self.operator_input_to_arm_outputs(
                pressed, axes, buttons, connected, speed_level, previous_input_ms, current_ms
            )

        return self.operator_input_to_vehicle_outputs(pressed, axes, buttons, connected, speed_level)

    def operator_input_to_vehicle_outputs(
        self,
        pressed: Set[str],
        axes: Dict[str, Any],
        buttons: Dict[str, Any],
        connected: bool,
        speed_level: int,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand], FlipperCommand]:
        import bridge_mapping as _mapping
        flipper = self.operator_input_to_flipper_outputs(pressed, buttons, axes, connected, speed_level)
        return _mapping.map_vehicle_outputs(
            pressed, axes, buttons, connected,
            linear_scale=self.base_linear[speed_level],
            angular_scale=self.base_angular[speed_level],
            servo_frame=self.servo_frame,
            deadzone=self.gamepad_deadzone,
            flipper_cmd=flipper,
        )

    def operator_input_to_arm_outputs(
        self,
        pressed: Set[str],
        axes: Dict[str, Any],
        buttons: Dict[str, Any],
        connected: bool,
        speed_level: int,
        previous_input_ms: int,
        current_ms: int,
    ) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand], FlipperCommand]:
        import bridge_mapping as _mapping
        return _mapping.map_arm_outputs(
            pressed, axes, buttons, connected,
            linear=self.arm_linear[speed_level],
            angular=self.arm_angular[speed_level],
            gripper_rate=self.gripper_rates[speed_level],
            previous_input_ms=previous_input_ms,
            servo_frame=self.servo_frame,
            angular_direction_corrections=self.servo_angular_direction_corrections,
            deadzone=self.gamepad_deadzone,
            zero_flipper=self.zero_flipper_command("operator_input.arm_zero"),
        )

    def operator_input_to_flipper_outputs(
        self,
        pressed: Set[str],
        buttons: Dict[str, Any],
        axes: Dict[str, Any],
        connected: bool,
        speed_level: int,
    ) -> FlipperCommand:
        import bridge_mapping as _mapping
        return _mapping.map_flipper_outputs(
            pressed, buttons, axes, connected,
            flipper_speed=self.flipper_velocities[speed_level],
            joint_names=self.flipper_joint_names,
            direction_corrections=self.flipper_direction_corrections,
            jog_duration=self.flipper_jog_duration,
            deadzone=self.gamepad_deadzone,
        )

    def normalized_pressed_keys(self, msg: Dict[str, Any]) -> Set[str]:
        import bridge_mapping as _mapping
        return _mapping.normalized_pressed_keys(msg)

    def active_gamepad_buttons(self, gamepad: Dict[str, Any]) -> Set[str]:
        import bridge_mapping as _mapping
        return _mapping.active_gamepad_buttons(gamepad)

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

            if self.MODE_SWITCH_GAMEPAD_BUTTONS & button_edges:
                self.state.control_mode = (
                    self.MODE_ARM if self.state.control_mode == self.MODE_VEHICLE else self.MODE_VEHICLE
                )
                mode_changed = True

            for key in sorted(key_edges):
                if key in self.SPEED_KEY_LEVELS:
                    self.state.speed_level = self.SPEED_KEY_LEVELS[key]
                    speed_changed = True

            mode = self.state.control_mode
            speed_level = self.state.speed_level

        if mode_changed:
            self.publish_zero("mode_switch")
            service=self.output.command_service_name('set_control_mode',{'mode':mode})
            if service:
                ok,code,message=self.output.call_command_service('set_control_mode',{'mode':mode})
                if not ok:
                    with self.state_lock:
                        self.state.control_mode=self.MODE_VEHICLE if mode==self.MODE_ARM else self.MODE_ARM
                    self.events.emit('mode','mode switch rejected',level='error',data={'code':code,'message':message})
                    return
            self.events.emit("mode", "control mode switched", data={"seq": seq, "mode": mode})
            if mode == self.MODE_VEHICLE:
                self.ensure_flipper_profile(force=True)
        if speed_changed:
            self.events.emit("speed", "speed level set", data={"seq": seq, "speed_level": speed_level})

    def normalize_mode(self, mode: str) -> str:
        normalized = mode.strip().lower()
        if normalized in ("vehicle", "base"):
            return self.MODE_VEHICLE
        if normalized == "arm":
            return self.MODE_ARM
        return ""

    @staticmethod
    def normalized_service_bindings(bindings: Optional[Dict[str, str]]) -> Dict[str, str]:
        import bridge_mapping as _mapping
        return _mapping.normalized_service_bindings(bindings)

    def handle_input_service_bindings(
        self,
        msg: Dict[str, Any],
        key_edges: Set[str],
        button_edges: Set[str],
    ) -> List[Dict[str, Any]]:
        results: List[Dict[str, Any]] = []
        bindings = []
        bindings.extend(
            ("keyboard", key_name, self.keyboard_service_bindings[key_name])
            for key_name in sorted(key_edges)
            if key_name in self.keyboard_service_bindings
        )
        bindings.extend(
            ("gamepad", button_name, self.gamepad_service_bindings[button_name])
            for button_name in sorted(button_edges)
            if button_name in self.gamepad_service_bindings
        )
        for input_type, input_name, service_name in bindings:
            ok, code, message, duration_ms = self.call_named_service_result(service_name)
            command = f"{input_type}:{input_name}"
            self.events.emit(
                "service",
                "input binding service handled",
                data={
                    "command": command,
                    "service": service_name,
                    "ok": ok,
                    "code": code,
                    "message": message,
                    "duration_ms": duration_ms,
                },
                level="info" if ok else "error",
            )
            results.append(
                self.make_service_call_result(
                    msg,
                    command=command,
                    service=service_name,
                    ok=ok,
                    code=code,
                    message=message,
                    duration_ms=duration_ms,
                )
            )
        return results

    def clamp_speed_level(self, level: int) -> int:
        return max(1, min(5, int(level)))

    def axis_value(self, pressed: Set[str], positive_key: str, negative_key: str) -> float:
        import bridge_mapping as _mapping
        return _mapping.axis_value(pressed, positive_key, negative_key)

    def normalized_flipper_direction_corrections(self, configured: Optional[List[float]]) -> List[float]:
        defaults = [1.0] * len(self.flipper_joint_names)
        if not configured:
            return defaults
        values = [float(value) for value in configured]
        return (values + defaults)[0: len(self.flipper_joint_names)]

    @staticmethod
    def normalized_axis_direction_corrections(configured: Optional[List[float]]) -> List[float]:
        import bridge_mapping as _mapping
        return _mapping.normalized_axis_direction_corrections(configured)

    @staticmethod
    def apply_axis_direction_corrections(
        x_value: float,
        y_value: float,
        z_value: float,
        corrections: List[float],
    ) -> Tuple[float, float, float]:
        import bridge_mapping as _mapping
        return _mapping.apply_axis_direction_corrections(x_value, y_value, z_value, corrections)

    @staticmethod
    def normalized_gamepad_mapping(raw: Any) -> Dict[str, Any]:
        import bridge_mapping as _mapping
        return _mapping.normalized_gamepad_mapping(raw)

    @staticmethod
    def strongest_axis_value(*values: float) -> float:
        import bridge_mapping as _mapping
        return _mapping.strongest_axis_value(*values)

    @staticmethod
    def float_value(value: Any) -> float:
        import bridge_mapping as _mapping
        return _mapping.float_value(value)

    def axis_float_value(self, value: Any) -> float:
        import bridge_mapping as _mapping
        return _mapping.axis_float_value(value, self.gamepad_deadzone)

    @staticmethod
    def bool_value(value: Any) -> float:
        import bridge_mapping as _mapping
        return _mapping.bool_value(value)

    def button_axis_value(
        self,
        buttons: Dict[str, Any],
        positive_names: Tuple[str, ...],
        negative_names: Tuple[str, ...],
    ) -> float:
        import bridge_mapping as _mapping
        return _mapping.button_axis_value(buttons, positive_names, negative_names)

    def gamepad_button_axis_value(
        self,
        buttons: Dict[str, Any],
        axes: Dict[str, Any],
        *,
        positive_button_names: Tuple[str, ...] = (),
        negative_button_names: Tuple[str, ...] = (),
        positive_axis_names: Tuple[str, ...] = (),
        negative_axis_names: Tuple[str, ...] = (),
    ) -> float:
        import bridge_mapping as _mapping
        return _mapping.gamepad_button_axis_value(
            buttons, axes,
            deadzone=self.gamepad_deadzone,
            positive_button_names=positive_button_names,
            negative_button_names=negative_button_names,
            positive_axis_names=positive_axis_names,
            negative_axis_names=negative_axis_names,
        )

    def gamepad_inputs_active(
        self,
        buttons: Dict[str, Any],
        axes: Dict[str, Any],
        button_names: Tuple[str, ...],
        axis_names: Tuple[str, ...],
    ) -> bool:
        import bridge_mapping as _mapping
        return _mapping.gamepad_inputs_active(buttons, axes, button_names, axis_names, self.gamepad_deadzone)

    def operator_input_age_ms(self, timestamp_ms: int, current_ms: int) -> Tuple[Optional[int], Optional[int], Optional[int]]:
        if timestamp_ms <= 0:
            return None, None, self.operator_input_clock_offset_ms

        raw_age_ms = current_ms - timestamp_ms
        if self.operator_input_clock_offset_ms is None or raw_age_ms < self.operator_input_clock_offset_ms:
            self.operator_input_clock_offset_ms = raw_age_ms

        clock_offset_ms = self.operator_input_clock_offset_ms
        effective_age_ms = raw_age_ms - clock_offset_ms if clock_offset_ms is not None else raw_age_ms
        return raw_age_ms, max(0, effective_age_ms), clock_offset_ms

    def check_watchdog(self) -> List[Dict[str, Any]]:
        emit_status = False
        with self.state_lock:
            if (
                self.last_valid_input_monotonic > 0
                and not self.state.emergency_active
                and not self.state.watchdog_active
                and (time.monotonic() - self.last_valid_input_monotonic) * 1000 > self.watchdog_ms
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
