from typing import Any, Dict, Optional, Tuple

from bridge_state import GripperCommand, ServoCommand, TwistCommand
from debug_events import EventSink, NullEventSink


class OutputAdapter:
    def __init__(self, events: Optional[EventSink] = None) -> None:
        self.events = events or NullEventSink()

    def publish_twist(self, twist: TwistCommand) -> None:
        raise NotImplementedError

    def publish_servo(self, servo: ServoCommand) -> None:
        raise NotImplementedError

    def publish_gripper(self, gripper: GripperCommand) -> None:
        raise NotImplementedError

    def call_lifecycle(self, command: str) -> Tuple[bool, int, str]:
        raise NotImplementedError

    def lifecycle_service_name(self, command: str) -> str:
        return f"lifecycle/{command}"

    def command_service_name(self, command: str, params: Optional[Dict[str, Any]] = None) -> str:
        return ""

    def call_command_service(self, command: str, params: Optional[Dict[str, Any]] = None) -> Tuple[bool, int, str]:
        return False, 2302, f"no ROS service configured for {command}"


class DryRunOutput(OutputAdapter):
    def __init__(
        self,
        events: Optional[EventSink] = None,
        service_commands: Optional[Dict[str, str]] = None,
    ) -> None:
        super().__init__(events)
        self.service_commands = service_commands or {}

    def publish_twist(self, twist: TwistCommand) -> None:
        print(
            f"[bridge] cmd_vel source={twist.source} "
            f"linear_x={twist.linear_x:+.3f} angular_z={twist.angular_z:+.3f}",
            flush=True,
        )
        self.events.emit("output", "dry-run cmd_vel", data={
            "linear_x": twist.linear_x,
            "angular_z": twist.angular_z,
            "source": twist.source,
        })

    def publish_servo(self, servo: ServoCommand) -> None:
        print(
            f"[bridge] servo source={servo.source} frame={servo.frame_id} "
            f"lin=({servo.linear_x:+.3f},{servo.linear_y:+.3f},{servo.linear_z:+.3f}) "
            f"ang=({servo.angular_x:+.3f},{servo.angular_y:+.3f},{servo.angular_z:+.3f})",
            flush=True,
        )
        self.events.emit("output", "dry-run servo", data=servo.to_dict())

    def publish_gripper(self, gripper: GripperCommand) -> None:
        print(
            f"[bridge] gripper source={gripper.source} position={gripper.position:.4f}",
            flush=True,
        )
        self.events.emit("output", "dry-run gripper", data=gripper.to_dict())

    def call_lifecycle(self, command: str) -> Tuple[bool, int, str]:
        print(f"[bridge] lifecycle dry-run command={command}", flush=True)
        self.events.emit("lifecycle", "dry-run lifecycle service", data={"command": command})
        return True, 0, f"dry-run lifecycle {command} accepted"

    def lifecycle_service_name(self, command: str) -> str:
        return f"dry-run:lifecycle/{command}"

    def command_service_name(self, command: str, params: Optional[Dict[str, Any]] = None) -> str:
        template = self.service_commands.get(command, "")
        if not template:
            return ""
        return format_service_name(template, command, params)

    def call_command_service(self, command: str, params: Optional[Dict[str, Any]] = None) -> Tuple[bool, int, str]:
        service_name = self.command_service_name(command, params)
        print(f"[bridge] service dry-run command={command} service={service_name}", flush=True)
        self.events.emit("service", "dry-run command service", data={
            "command": command,
            "service": service_name,
            "params": params or {},
        })
        return True, 0, f"dry-run service {service_name} accepted"

class RosOutput(OutputAdapter):
    def __init__(
        self,
        node_name: str,
        cmd_vel_topic: str,
        servo_topic: str,
        gripper_position_topic: str,
        hybrid_service_ns: str,
        service_commands: Optional[Dict[str, str]] = None,
        events: Optional[EventSink] = None,
    ) -> None:
        super().__init__(events)
        import rospy
        from geometry_msgs.msg import Twist, TwistStamped
        from std_msgs.msg import Float64
        from std_srvs.srv import Trigger

        self._twist_type = Twist
        self._twist_stamped_type = TwistStamped
        self._float64_type = Float64
        self._trigger_type = Trigger
        self._rospy = rospy
        self.hybrid_service_ns = hybrid_service_ns.rstrip("/") or "/hybrid_motor_hw_node"
        self.service_commands = service_commands or {}
        rospy.init_node(node_name, anonymous=False)
        self._cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self._servo_pub = rospy.Publisher(servo_topic, TwistStamped, queue_size=1)
        self._gripper_pub = rospy.Publisher(gripper_position_topic, Float64, queue_size=1)
        rospy.loginfo("host_bridge_node publishing Twist to %s", cmd_vel_topic)
        rospy.loginfo("host_bridge_node publishing TwistStamped to %s", servo_topic)
        rospy.loginfo("host_bridge_node publishing Float64 to %s", gripper_position_topic)
        rospy.loginfo("host_bridge_node using hybrid service namespace %s", self.hybrid_service_ns)

    def publish_twist(self, twist: TwistCommand) -> None:
        msg = self._twist_type()
        msg.linear.x = twist.linear_x
        msg.angular.z = twist.angular_z
        self._cmd_vel_pub.publish(msg)
        self.events.emit("output", "ros cmd_vel published", data={
            "linear_x": twist.linear_x,
            "angular_z": twist.angular_z,
            "source": twist.source,
        })

    def publish_servo(self, servo: ServoCommand) -> None:
        msg = self._twist_stamped_type()
        msg.header.stamp = self._rospy.Time.now()
        msg.header.frame_id = servo.frame_id
        msg.twist.linear.x = servo.linear_x
        msg.twist.linear.y = servo.linear_y
        msg.twist.linear.z = servo.linear_z
        msg.twist.angular.x = servo.angular_x
        msg.twist.angular.y = servo.angular_y
        msg.twist.angular.z = servo.angular_z
        self._servo_pub.publish(msg)
        self.events.emit("output", "ros servo published", data=servo.to_dict())

    def publish_gripper(self, gripper: GripperCommand) -> None:
        msg = self._float64_type(data=gripper.position)
        self._gripper_pub.publish(msg)
        self.events.emit("output", "ros gripper published", data=gripper.to_dict())

    def call_lifecycle(self, command: str) -> Tuple[bool, int, str]:
        service_name = f"{self.hybrid_service_ns}/{command}"
        try:
            self._rospy.wait_for_service(service_name, timeout=2.0)
            response = self._rospy.ServiceProxy(service_name, self._trigger_type)()
        except Exception as exc:
            self.events.emit("lifecycle", "lifecycle service failed", level="error", data={
                "command": command,
                "service": service_name,
                "error": str(exc),
            })
            return False, 2301, f"{service_name} failed: {exc}"

        self.events.emit("lifecycle", "lifecycle service completed", data={
            "command": command,
            "service": service_name,
            "success": bool(response.success),
            "message": str(response.message),
        })
        return bool(response.success), 0 if response.success else 2301, str(response.message)

    def lifecycle_service_name(self, command: str) -> str:
        return f"{self.hybrid_service_ns}/{command}"

    def command_service_name(self, command: str, params: Optional[Dict[str, Any]] = None) -> str:
        template = self.service_commands.get(command, "")
        if not template:
            return ""
        return format_service_name(template, command, params)

    def call_command_service(self, command: str, params: Optional[Dict[str, Any]] = None) -> Tuple[bool, int, str]:
        service_name = self.command_service_name(command, params)
        try:
            self._rospy.wait_for_service(service_name, timeout=2.0)
            response = self._rospy.ServiceProxy(service_name, self._trigger_type)()
        except Exception as exc:
            self.events.emit("service", "command service failed", level="error", data={
                "command": command,
                "service": service_name,
                "params": params or {},
                "error": str(exc),
            })
            return False, 2301, f"{service_name} failed: {exc}"

        self.events.emit("service", "command service completed", data={
            "command": command,
            "service": service_name,
            "params": params or {},
            "success": bool(response.success),
            "message": str(response.message),
        })
        return bool(response.success), 0 if response.success else 2301, str(response.message)


def format_service_name(template: str, command: str, params: Optional[Dict[str, Any]] = None) -> str:
    params = params or {}
    mode = str(params.get("mode", ""))
    source = str(params.get("source", ""))
    return template.format(command=command, mode=mode, source=source)
