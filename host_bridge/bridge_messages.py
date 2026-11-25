"""Protocol payload builders for host_bridge telemetry, responses, and capabilities."""
from typing import Any, Dict, List

from bridge_protocol import MAX_FRAME_BYTES, PROTOCOL_VERSION, now_ms


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


def build_hello(robot_id: str) -> Dict[str, Any]:
    return {
        "type": "hello",
        "protocol_version": PROTOCOL_VERSION,
        "seq": 0,
        "timestamp_ms": now_ms(),
        "bridge_name": "host_bridge_node",
        "bridge_version": "loopback-mvp",
        "robot_id": robot_id,
    }


def build_capabilities(core: Any) -> Dict[str, Any]:
    arm_named_targets_supported = bool(core.output.supports_arm_named_targets())
    supports = [
        "control_v2",
        "legacy_readonly",
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
        "monotonic_watchdog",
        "connection_control_owner",
        "readonly_monitor",
        "keyboard_base_arm_mapping",
        "arm_servo_output",
        "gripper_velocity_output",
        "flipper_jog_output",
        "joint_runtime_states",
        "co2_data",
        "hybrid_lifecycle_services",
        "service_call_result",
    ]
    if arm_named_targets_supported:
        supports.extend([
            "arm_named_targets",
            "move_arm_named_target",
        ])
    return {
        "type": "capabilities",
        "protocol_version": PROTOCOL_VERSION,
        "seq": 0,
        "timestamp_ms": now_ms(),
        "supports": supports,
        "arm_named_targets_supported": arm_named_targets_supported,
        "control_protocol_version": 2,
        "control_capabilities": __import__("control_v2").SUPPORTED_CAPABILITIES,
        "motion_presets": __import__("motion_presets").PRESETS,
        "speed_control": {
            "command": "set_speed_level",
            "parameter": "level",
            "levels": [1, 2, 3, 4, 5],
            "neutral_required": True,
            "reauthorize_required": True,
            "base_linear_m_s": dict(core.base_linear),
            "base_angular_rad_s": dict(core.base_angular),
            "arm_linear_m_s": dict(core.arm_linear),
            "arm_angular_rad_s": dict(core.arm_angular),
            "gripper_m_s": dict(core.gripper_rates),
            "flipper_rad_s": dict(core.flipper_velocities),
        },
        "legacy_control_allowed": False,
        "max_frame_bytes": MAX_FRAME_BYTES,
        "watchdog_ms": core.watchdog_ms,
        "gamepad_deadzone_percent": core.gamepad_deadzone * 100.0,
        "keyboard_service_bindings": dict(core.keyboard_service_bindings),
        "gamepad_service_bindings": dict(core.gamepad_service_bindings),
        "keyboard_mapping": {
            "mode_switch": "system_command set_control_mode",
            "emergency": "emergency_stop (client translates space or l3+r3)",
            "speed_levels": "system_command set_speed_level, params.level=1..5",
            "vehicle": {
                "linear_x": {"positive": "w", "negative": "s"},
                "angular_z": {"positive": "a", "negative": "d"},
                "flippers": {
                    "left_front_arm_joint": {"positive": "y", "negative": "h"},
                    "right_front_arm_joint": {"positive": "u", "negative": "j"},
                    "left_rear_arm_joint": {"positive": "i", "negative": "k"},
                    "right_rear_arm_joint": {"positive": "o", "negative": "l"},
                },
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
        "cameras": core.current_cameras(),
    }


def build_system_snapshot(core: Any, seq: int) -> Dict[str, Any]:
    cameras = core.current_cameras()
    with core.state_lock:
        return {
            "type": "system_snapshot",
            "protocol_version": PROTOCOL_VERSION,
            "seq": seq,
            "timestamp_ms": now_ms(),
            "control_mode": core.state.control_mode,
            "emergency": {
                "active": core.state.emergency_active,
                "source": core.state.emergency_source,
            },
            "motor": {
                "initialized": core.state.motor_initialized,
                "enabled": core.state.motor_enabled,
            },
            "modules": {
                "base": "online",
                "arm": "online",
                "camera": "online" if any(cam.get("online", False) for cam in cameras) else "offline",
            },
            "last_error": {
                "code": core.state.last_error_code,
                "message": core.state.last_error_message,
            },
            "last_operator_input_seq": core.state.last_operator_seq,
            "watchdog_active": core.state.watchdog_active,
            "speed_level": core.state.speed_level,
            "last_twist": core.state.last_twist.to_dict(),
            "last_servo": core.state.last_servo.to_dict(),
            "last_flipper": core.state.last_flipper.to_dict(),
            "gripper_velocity": core.state.gripper_velocity,
            "flipper_profile": {
                "target": core.state.flipper_profile_target,
                "result": core.state.flipper_profile_result,
                "message": core.state.flipper_profile_message,
            },
        }


def build_camera_list_response(seq: int, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "type": "camera_list_response",
        "protocol_version": PROTOCOL_VERSION,
        "seq": seq,
        "timestamp_ms": now_ms(),
        "cameras": cameras,
    }


def build_camera_info(camera: Dict[str, Any]) -> Dict[str, Any]:
    message = dict(camera)
    message["type"] = "camera_info"
    message["protocol_version"] = PROTOCOL_VERSION
    message["seq"] = 0
    message["timestamp_ms"] = now_ms()
    return message


def build_co2_data(ppm: int) -> Dict[str, Any]:
    return {
        "type": "co2_data",
        "ppm": int(ppm),
    }


def build_arm_named_targets(
    seq: int,
    targets: List[Dict[str, str]],
    message: str = "",
) -> Dict[str, Any]:
    return {
        "type": "arm_named_targets",
        "protocol_version": PROTOCOL_VERSION,
        "seq": seq,
        "timestamp_ms": now_ms(),
        "targets": targets,
        "message": message,
    }


def build_ack(msg: Dict[str, Any], ok: bool = True, code: int = 0, message: str = "ok") -> Dict[str, Any]:
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


def build_emergency_state(active: bool, source: str, message: str) -> Dict[str, Any]:
    return {
        "type": "emergency_state",
        "protocol_version": PROTOCOL_VERSION,
        "seq": 0,
        "timestamp_ms": now_ms(),
        "active": active,
        "source": source,
        "message": message,
    }


def build_service_call_result(
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


def build_input_status(core: Any, seq: int, status: str) -> Dict[str, Any]:
    with core.state_lock:
        twist = core.state.last_twist
        servo = core.state.last_servo
        flipper = core.state.last_flipper
        gripper_velocity = core.state.gripper_velocity
        watchdog_active = core.state.watchdog_active
        emergency_active = core.state.emergency_active
        control_mode = core.state.control_mode
        speed_level = core.state.speed_level
        flipper_profile_target = core.state.flipper_profile_target
        flipper_profile_result = core.state.flipper_profile_result
        flipper_profile_message = core.state.flipper_profile_message
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
            "velocity": gripper_velocity,
        },
        "flipper": flipper.to_dict(),
        "flipper_profile": {
            "target": flipper_profile_target,
            "result": flipper_profile_result,
            "message": flipper_profile_message,
        },
    }
