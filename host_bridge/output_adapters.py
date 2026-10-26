import sys
from typing import Any, Dict, List, Optional, Tuple

from bridge_state import FlipperCommand, GripperCommand, ServoCommand, TwistCommand
from debug_events import EventSink, NullEventSink


VISION_MONITOR_COMMANDS = {
    "thermal_monitor": {
        "enable_all": False,
        "enable_yolo": True,
        "enable_qrcode": False,
        "enable_motion": False,
        "enable_obstacle_warning": False,
        "enable_center_distance": False,
    },
    "danger_sign": {
        "enable_all": False,
        "enable_yolo": True,
        "enable_qrcode": False,
        "enable_motion": False,
        "enable_obstacle_warning": False,
        "enable_center_distance": False,
    },
    "qr_code_detect": {
        "enable_all": False,
        "enable_yolo": False,
        "enable_qrcode": True,
        "enable_motion": False,
        "enable_obstacle_warning": False,
        "enable_center_distance": False,
    },
    "dynamic_monitor": {
        "enable_all": False,
        "enable_yolo": False,
        "enable_qrcode": False,
        "enable_motion": True,
        "enable_obstacle_warning": False,
        "enable_center_distance": False,
    },
    "reset_monitor": {
        "enable_all": False,
        "enable_yolo": False,
        "enable_qrcode": False,
        "enable_motion": False,
        "enable_obstacle_warning": False,
        "enable_center_distance": False,
    },
}


class OutputAdapter:
    def __init__(self, events: Optional[EventSink] = None) -> None:
        self.events = events or NullEventSink()

    def publish_twist(self, twist: TwistCommand) -> None:
        raise NotImplementedError

    def publish_servo(self, servo: ServoCommand) -> None:
        raise NotImplementedError

    def publish_gripper(self, gripper: GripperCommand) -> None:
        raise NotImplementedError

    def publish_flipper(self, flipper: FlipperCommand) -> None:
        raise NotImplementedError

    def call_flipper_profile(self, profile: str) -> Tuple[bool, int, str, Dict[str, Any]]:
        return False, 2302, "no flipper profile service configured", {}

    def call_lifecycle(self, command: str) -> Tuple[bool, int, str]:
        raise NotImplementedError

    def lifecycle_service_name(self, command: str) -> str:
        return f"lifecycle/{command}"

    def command_service_name(self, command: str, params: Optional[Dict[str, Any]] = None) -> str:
        return ""

    def call_command_service(self, command: str, params: Optional[Dict[str, Any]] = None) -> Tuple[bool, int, str]:
        return False, 2302, f"no ROS service configured for {command}"

    def call_named_service(self, service_name: str) -> Tuple[bool, int, str]:
        return False, 2302, f"no ROS service configured for {service_name}"

    def list_arm_named_targets(self) -> Tuple[bool, int, str, List[Dict[str, str]]]:
        return False, 2302, "no MoveIt arm group configured", []

    def move_arm_named_target(self, target: str) -> Tuple[bool, int, str]:
        return False, 2302, f"no MoveIt arm group configured for target {target}"

    def supports_arm_named_targets(self) -> bool:
        return False


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
            f"[bridge] gripper source={gripper.source} velocity={gripper.velocity:+.4f}",
            flush=True,
        )
        self.events.emit("output", "dry-run gripper", data=gripper.to_dict())

    def publish_flipper(self, flipper: FlipperCommand) -> None:
        pairs = ", ".join(
            f"{name}={velocity:+.3f}"
            for name, velocity in zip(flipper.joint_names, flipper.velocities)
        )
        print(
            f"[bridge] flipper source={flipper.source} duration={flipper.duration:.3f} {pairs}",
            flush=True,
        )
        self.events.emit("output", "dry-run flipper", data=flipper.to_dict())

    def call_flipper_profile(self, profile: str) -> Tuple[bool, int, str, Dict[str, Any]]:
        print(f"[bridge] flipper profile dry-run profile={profile}", flush=True)
        detail = {"active_profile": profile, "active_controller": "dry-run"}
        self.events.emit("flipper", "dry-run flipper profile service", data=detail)
        return True, 0, f"dry-run flipper profile {profile} accepted", detail

    def call_lifecycle(self, command: str) -> Tuple[bool, int, str]:
        print(f"[bridge] lifecycle dry-run command={command}", flush=True)
        self.events.emit("lifecycle", "dry-run lifecycle service", data={"command": command})
        return True, 0, f"dry-run lifecycle {command} accepted"

    def lifecycle_service_name(self, command: str) -> str:
        return f"dry-run:lifecycle/{command}"

    def command_service_name(self, command: str, params: Optional[Dict[str, Any]] = None) -> str:
        if command in VISION_MONITOR_COMMANDS:
            return "/paw_vision/set_detection_enabled"
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

    def call_named_service(self, service_name: str) -> Tuple[bool, int, str]:
        print(f"[bridge] trigger service dry-run service={service_name}", flush=True)
        self.events.emit("service", "dry-run named trigger service", data={
            "service": service_name,
        })
        return True, 0, f"dry-run service {service_name} accepted"

    def list_arm_named_targets(self) -> Tuple[bool, int, str, List[Dict[str, str]]]:
        targets = [
            {"name": "home", "description": "dry-run"},
            {"name": "ready", "description": "dry-run"},
            {"name": "inspect", "description": "dry-run"},
        ]
        self.events.emit("moveit", "dry-run named targets listed", data={"count": len(targets)})
        return True, 0, "dry-run MoveIt named targets", targets

    def move_arm_named_target(self, target: str) -> Tuple[bool, int, str]:
        print(f"[bridge] moveit dry-run target={target}", flush=True)
        self.events.emit("moveit", "dry-run named target accepted", data={"target": target})
        return True, 0, f"dry-run MoveIt target {target} accepted"

    def supports_arm_named_targets(self) -> bool:
        return True

class RosOutput(OutputAdapter):
    """ROS2 transport for the unchanged TCP/JSON protocol core."""
    def __init__(self,node_name,cmd_vel_topic,servo_topic,gripper_velocity_topic,flipper_jog_topic,
                 flipper_profile_service,hybrid_service_ns,service_commands=None,
                 vision_detection_service='/paw_vision/set_detection_enabled',moveit_group='arm',events=None):
        super().__init__(events)
        import rclpy,threading
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist,TwistStamped
        from control_msgs.msg import JointJog
        from std_msgs.msg import Float64,String
        from robot_hardware.srv import SetProfile
        from std_srvs.srv import Trigger
        from arm_control.action import ExecuteArmGoal
        from rclpy.action import ActionClient
        if not rclpy.ok():rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=Node(node_name);self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin);self.thread.start()
        self._twist_type,self._twist_stamped_type,self._joint_jog_type,self._float64_type=Twist,TwistStamped,JointJog,Float64
        self._cmd_vel_pub=self.node.create_publisher(Twist,cmd_vel_topic,1)
        self._servo_pub=self.node.create_publisher(TwistStamped,servo_topic,1)
        self._gripper_pub=self.node.create_publisher(Float64,gripper_velocity_topic,1)
        self._flipper_pub=self.node.create_publisher(JointJog,flipper_jog_topic,1)
        self.flipper_profile_service=flipper_profile_service;self.hybrid_service_ns='/robot_runtime'
        self.service_commands=service_commands or {};self.vision_detection_service=vision_detection_service
        self.clients={};self.client_lock=threading.Lock();self.runtime_reset_callback=None;self.epoch=None
        self._arm_type=ExecuteArmGoal;self.arm=ActionClient(self.node,ExecuteArmGoal,'/arm_control/execute_goal')
        self.node.create_subscription(String,'/robot_runtime/state',self.state,1)

    def state(self,message):
        import json
        try:
            state=json.loads(message.data);epoch=json.dumps([state.get('groups',{}),state.get('operations',{})],sort_keys=True)
            if epoch!=self.epoch and self.runtime_reset_callback:self.runtime_reset_callback('runtime_epoch')
            self.epoch=epoch
        except ValueError:pass

    def close(self):
        import rclpy
        self.executor.shutdown();self.thread.join();self.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

    def wait(self,future,seconds=30):
        import threading
        done=threading.Event();future.add_done_callback(lambda _:done.set())
        if not done.wait(seconds):raise TimeoutError('ROS2 request timeout; outcome uncertain')
        return future.result()

    def request(self,kind,path,request,seconds=30):
        with self.client_lock:
            client=self.clients.get(path)
            if client is None:client=self.node.create_client(kind,path);self.clients[path]=client
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError(path+' unavailable')
        return self.wait(client.call_async(request),seconds)

    def publish_twist(self,twist):
        msg=self._twist_type();msg.linear.x=twist.linear_x;msg.angular.z=twist.angular_z;self._cmd_vel_pub.publish(msg)
    def publish_servo(self,servo):
        msg=self._twist_stamped_type();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id=servo.frame_id
        for axis in ('x','y','z'):
            setattr(msg.twist.linear,axis,getattr(servo,'linear_'+axis));setattr(msg.twist.angular,axis,getattr(servo,'angular_'+axis))
        self._servo_pub.publish(msg)
    def publish_gripper(self,gripper):
        import math
        value=gripper.velocity
        self._gripper_pub.publish(self._float64_type(data=max(-.02,min(.02,value)) if math.isfinite(value) else 0.0))
    def publish_flipper(self,flipper):
        msg=self._joint_jog_type();msg.header.stamp=self.node.get_clock().now().to_msg();msg.joint_names=flipper.joint_names;msg.velocities=flipper.velocities;msg.duration=flipper.duration;self._flipper_pub.publish(msg)
    def call_flipper_profile(self,profile):
        from robot_hardware.srv import SetProfile
        translated={'csv_velocity':'csv','csp_position':'csp','csp_jog':'csp'}.get(profile,profile)
        try:
            result=self.request(SetProfile,self.flipper_profile_service,SetProfile.Request(profile=translated))
            return result.success,0 if result.success else 2301,result.message,{'active_profile':translated}
        except Exception as error:return False,2301,str(error),{'service_unavailable':True}
    def call_lifecycle(self,command):
        return self._call_trigger_service(self.lifecycle_service_name(command))
    def lifecycle_service_name(self,command):return '/robot_runtime/'+command
    def command_service_name(self,command,params=None):
        if command=='set_control_mode':return '/arm_control/set_servo_mode'
        if command in VISION_MONITOR_COMMANDS:return self.vision_detection_service
        template=self.service_commands.get(command,'')
        return format_service_name(template,command,params) if template else ''
    def call_command_service(self,command,params=None):
        path=self.command_service_name(command,params)
        if command in VISION_MONITOR_COMMANDS:
            from vision_pkg.srv import SetDetectionEnabled
            try:
                result=self.request(SetDetectionEnabled,path,SetDetectionEnabled.Request(**VISION_MONITOR_COMMANDS[command]))
                return result.success,0 if result.success else 2301,result.message
            except Exception as error:return False,2301,str(error)
        if command=='set_control_mode':
            from std_srvs.srv import SetBool
            mode=(params or {}).get('mode','')
            if mode not in ('arm','base','vehicle','servo','trajectory'):return False,2101,'unknown control mode'
            try:
                result=self.request(SetBool,'/arm_control/set_servo_mode',SetBool.Request(data=mode in ('arm','servo')))
                return result.success,0 if result.success else 2301,result.message
            except Exception as error:return False,2301,str(error)
        return self._call_trigger_service(path)
    def call_named_service(self,service_name):return self._call_trigger_service(service_name)
    def _call_trigger_service(self,path,**kwargs):
        from std_srvs.srv import Trigger
        if not path:return False,2302,'no service configured'
        try:
            result=self.request(Trigger,path,Trigger.Request());return result.success,0 if result.success else 2301,result.message
        except Exception as error:return False,2301,str(error)
    def list_arm_named_targets(self):
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        import xml.etree.ElementTree as ET
        root=ET.parse(Path(get_package_share_directory('car_moveit_config'))/'config/car_urdf.srdf').getroot()
        return True,0,'ok',[{'name':x.get('name'),'description':''} for x in root.findall('group_state') if x.get('group')=='arm']
    def move_arm_named_target(self,target):
        if not self.arm.wait_for_server(timeout_sec=2):return False,2301,'arm action unavailable'
        goal=self._arm_type.Goal(target_type=self._arm_type.Goal.TARGET_NAMED,named_target=target)
        try:
            handle=self.wait(self.arm.send_goal_async(goal),5)
            if not handle.accepted:return False,2301,'arm busy or Servo mode active'
            try:result=self.wait(handle.get_result_async(),120).result
            except TimeoutError:
                self.wait(handle.cancel_goal_async(),3);return False,2301,'arm timeout; cancellation requested'
            return result.success,0 if result.success else 2301,result.message
        except Exception as error:return False,2301,str(error)
    def supports_arm_named_targets(self):return True


def format_service_name(template: str, command: str, params: Optional[Dict[str, Any]] = None) -> str:
    params = params or {}
    mode = str(params.get("mode", ""))
    source = str(params.get("source", ""))
    target = str(params.get("target", ""))
    return template.format(command=command, mode=mode, source=source, target=target)
