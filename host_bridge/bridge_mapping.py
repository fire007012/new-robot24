"""Stateless operator input normalization and teleoperation mapping helpers for host_bridge."""
from typing import Any, Dict, List, Optional, Set, Tuple

from bridge_protocol import clamp
from bridge_state import FlipperCommand, GripperCommand, ServoCommand, TwistCommand


def scaled_levels(levels: Dict[int, float], configured_max: float, default_max: float) -> Dict[int, float]:
    if configured_max <= 0.0:
        return dict(levels)
    scale = configured_max / default_max
    return {level: value * scale for level, value in levels.items()}


def normalized_levels(configured: Optional[Dict[int, float]], defaults: Dict[int, float]) -> Dict[int, float]:
    values = dict(defaults)
    if configured:
        values.update({int(level): float(value) for level, value in configured.items()})
    return values


def normalized_service_bindings(bindings: Optional[Dict[str, str]]) -> Dict[str, str]:
    if not bindings:
        return {}
    normalized: Dict[str, str] = {}
    for raw_name, raw_service in bindings.items():
        name = str(raw_name).strip().lower()
        service = str(raw_service).strip()
        if name and service:
            normalized[name] = service
    return normalized


def normalized_pressed_keys(msg: Dict[str, Any]) -> Set[str]:
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


def normalized_gamepad_mapping(raw: Any) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        return {}
    normalized: Dict[str, Any] = {}
    for name, value in raw.items():
        key = str(name).strip().lower()
        if key:
            normalized[key] = value
    return normalized


def active_gamepad_buttons(gamepad: Dict[str, Any]) -> Set[str]:
    buttons = normalized_gamepad_mapping(gamepad.get("buttons", {}))
    return {name for name, value in buttons.items() if bool(value)}


def normalized_axis_direction_corrections(configured: Optional[List[float]]) -> List[float]:
    defaults = [1.0, 1.0, 1.0]
    if not configured:
        return defaults
    values = [float(value) for value in configured]
    return (values + defaults)[0:3]


def apply_axis_direction_corrections(
    x_value: float,
    y_value: float,
    z_value: float,
    corrections: List[float],
) -> Tuple[float, float, float]:
    normalized = normalized_axis_direction_corrections(corrections)
    return (
        x_value * normalized[0],
        y_value * normalized[1],
        z_value * normalized[2],
    )


def strongest_axis_value(*values: float) -> float:
    strongest = 0.0
    for value in values:
        if abs(value) > abs(strongest):
            strongest = value
    return strongest


def float_value(value: Any) -> float:
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return 0.0


def axis_float_value(value: Any, deadzone: float) -> float:
    axis = float_value(value)
    return 0.0 if abs(axis) <= deadzone else axis


def bool_value(value: Any) -> float:
    return 1.0 if bool(value) else 0.0


def axis_value(pressed: Set[str], positive_key: str, negative_key: str) -> float:
    value = 0.0
    if positive_key in pressed:
        value += 1.0
    if negative_key in pressed:
        value -= 1.0
    return value


def button_axis_value(
    buttons: Dict[str, Any],
    positive_names: Tuple[str, ...],
    negative_names: Tuple[str, ...],
) -> float:
    value = 0.0
    if any(bool(buttons.get(name, False)) for name in positive_names):
        value += 1.0
    if any(bool(buttons.get(name, False)) for name in negative_names):
        value -= 1.0
    return value


def gamepad_inputs_active(
    buttons: Dict[str, Any],
    axes: Dict[str, Any],
    button_names: Tuple[str, ...],
    axis_names: Tuple[str, ...],
    deadzone: float,
) -> bool:
    if any(bool(buttons.get(name, False)) for name in button_names):
        return True
    return any(abs(axis_float_value(axes.get(name, 0.0), deadzone)) > 0.0 for name in axis_names)


def gamepad_button_axis_value(
    buttons: Dict[str, Any],
    axes: Dict[str, Any],
    *,
    deadzone: float,
    positive_button_names: Tuple[str, ...] = (),
    negative_button_names: Tuple[str, ...] = (),
    positive_axis_names: Tuple[str, ...] = (),
    negative_axis_names: Tuple[str, ...] = (),
) -> float:
    value = 0.0
    if gamepad_inputs_active(buttons, axes, positive_button_names, positive_axis_names, deadzone):
        value += 1.0
    if gamepad_inputs_active(buttons, axes, negative_button_names, negative_axis_names, deadzone):
        value -= 1.0
    return value


def map_flipper_outputs(
    pressed: Set[str],
    buttons: Dict[str, Any],
    axes: Dict[str, Any],
    connected: bool,
    *,
    flipper_speed: float,
    joint_names: Tuple[str, ...],
    direction_corrections: List[float],
    jog_duration: float,
    deadzone: float,
) -> FlipperCommand:
    logical_axes = {
        "left_front_arm_joint": axis_value(pressed, "y", "h"),
        "right_front_arm_joint": axis_value(pressed, "u", "j"),
        "left_rear_arm_joint": axis_value(pressed, "i", "k"),
        "right_rear_arm_joint": axis_value(pressed, "o", "l"),
    }
    if connected:
        logical_axes["left_front_arm_joint"] += gamepad_button_axis_value(
            buttons,
            axes,
            deadzone=deadzone,
            positive_button_names=("l1", "lb"),
            negative_button_names=("l2", "lt"),
            negative_axis_names=("lt",),
        )
        logical_axes["right_front_arm_joint"] += gamepad_button_axis_value(
            buttons,
            axes,
            deadzone=deadzone,
            positive_button_names=("r1", "rb"),
            negative_button_names=("r2", "rt"),
            negative_axis_names=("rt",),
        )
        logical_axes["left_rear_arm_joint"] += gamepad_button_axis_value(
            buttons,
            axes,
            deadzone=deadzone,
            positive_button_names=("dpad_up",),
            negative_button_names=("dpad_down",),
        )
        logical_axes["right_rear_arm_joint"] += gamepad_button_axis_value(
            buttons,
            axes,
            deadzone=deadzone,
            positive_button_names=("triangle", "y"),
            negative_button_names=("cross", "a"),
        )

    velocities = [
        clamp(logical_axes.get(joint_name, 0.0), -1.0, 1.0) * flipper_speed * correction
        for joint_name, correction in zip(joint_names, direction_corrections)
    ]
    velocities = (velocities + [0.0] * len(joint_names))[: len(joint_names)]

    return FlipperCommand(
        joint_names=list(joint_names),
        velocities=velocities,
        duration=jog_duration,
        source="operator_input.vehicle",
    )


def map_vehicle_outputs(
    pressed: Set[str],
    axes: Dict[str, Any],
    buttons: Dict[str, Any],
    connected: bool,
    *,
    linear_scale: float,
    angular_scale: float,
    servo_frame: str,
    deadzone: float,
    flipper_cmd: FlipperCommand,
) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand], FlipperCommand]:
    linear_axis = axis_value(pressed, "w", "s")
    angular_axis = axis_value(pressed, "a", "d")

    if connected:
        linear_axis += axis_float_value(axes.get("left_y", 0.0), deadzone)
        # Keep physical left aligned with keyboard "a": positive angular velocity.
        left_rotation_axis = -axis_float_value(axes.get("left_x", 0.0), deadzone)
        right_rotation_axis = -axis_float_value(axes.get("right_x", 0.0), deadzone)
        angular_axis += strongest_axis_value(left_rotation_axis, right_rotation_axis)

    twist = TwistCommand(
        linear_x=clamp(linear_axis, -1.0, 1.0) * linear_scale,
        angular_z=clamp(angular_axis, -1.0, 1.0) * angular_scale,
        source="operator_input.vehicle",
    )
    servo = ServoCommand(frame_id=servo_frame, source="operator_input.vehicle_zero")
    return twist, servo, None, flipper_cmd


def map_arm_outputs(
    pressed: Set[str],
    axes: Dict[str, Any],
    buttons: Dict[str, Any],
    connected: bool,
    *,
    linear: float,
    angular: float,
    gripper_rate: float,
    previous_input_ms: int,
    servo_frame: str,
    angular_direction_corrections: List[float],
    deadzone: float,
    zero_flipper: FlipperCommand,
) -> Tuple[TwistCommand, ServoCommand, Optional[GripperCommand], FlipperCommand]:
    linear_x_axis = axis_value(pressed, "u", "o")
    linear_y_axis = axis_value(pressed, "a", "d")
    linear_z_axis = axis_value(pressed, "w", "s")
    angular_x_axis = axis_value(pressed, "q", "e")
    angular_y_axis = axis_value(pressed, "k", "i")
    angular_z_axis = axis_value(pressed, "j", "l")

    if connected:
        # Keep physical left on the stick aligned with keyboard "a" / +linear.y.
        linear_y_axis += -axis_float_value(axes.get("left_x", 0.0), deadzone)
        linear_z_axis += axis_float_value(axes.get("left_y", 0.0), deadzone)
        # Use right trigger for forward and left trigger for backward motion.
        linear_x_axis += axis_float_value(axes.get("rt", 0.0), deadzone) - axis_float_value(axes.get("lt", 0.0), deadzone)
        angular_x_axis += bool_value(buttons.get("lb", False)) - bool_value(buttons.get("rb", False))
        angular_y_axis += -axis_float_value(axes.get("right_y", 0.0), deadzone)
        angular_z_axis += axis_float_value(axes.get("right_x", 0.0), deadzone)

    angular_x = clamp(angular_x_axis, -1.0, 1.0) * angular
    angular_y = clamp(angular_y_axis, -1.0, 1.0) * angular
    angular_z = clamp(angular_z_axis, -1.0, 1.0) * angular
    angular_x, angular_y, angular_z = apply_axis_direction_corrections(
        angular_x,
        angular_y,
        angular_z,
        angular_direction_corrections,
    )

    twist = TwistCommand(0.0, 0.0, "operator_input.arm_base_zero")
    servo = ServoCommand(
        linear_x=clamp(linear_x_axis, -1.0, 1.0) * linear,
        linear_y=clamp(linear_y_axis, -1.0, 1.0) * linear,
        linear_z=clamp(linear_z_axis, -1.0, 1.0) * linear,
        angular_x=angular_x,
        angular_y=angular_y,
        angular_z=angular_z,
        frame_id=servo_frame,
        source="operator_input.arm",
    )

    gripper_axis = axis_value(pressed, "f", "h")
    if connected:
        gripper_axis += button_axis_value(
            buttons,
            positive_names=("x", "square"),
            negative_names=("b", "circle"),
        )
    if previous_input_ms <= 0:
        gripper_velocity = 0.0
    else:
        # Velocity mode must receive explicit zero when keys are released.
        gripper_velocity = clamp(gripper_axis, -1.0, 1.0) * gripper_rate
    gripper = GripperCommand(velocity=gripper_velocity, source="operator_input.arm")

    return twist, servo, gripper, zero_flipper
