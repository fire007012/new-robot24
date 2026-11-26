import threading
import time
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
        self.operation_context = threading.local()
        self.late_operation_result = None

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

    def call_global_stop(self):
        return False, 2302, 'independent global stop unavailable'

    def reset_global_stop(self):
        return False, 2302, 'global stop reset unavailable'

    def call_group_operation(self, command, groups):
        return dict(success=False, message='group operations unavailable', operation_id='')

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

    def call_global_stop(self):
        return True, 0, 'dry-run global stop; no hardware operated'

    def reset_global_stop(self):
        return True, 0, 'dry-run global stop reset; no hardware operated'

    def call_group_operation(self, command, groups):
        return dict(success=True, operation_id='', message='dry-run; no hardware operated',
                    groups=list(groups), operation=command)

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
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist,TwistStamped
        from control_msgs.msg import JointJog
        from std_msgs.msg import Float64,String
        from robot_hardware.srv import GroupOperation,SetProfile
        from std_srvs.srv import SetBool,Trigger
        from arm_control.action import ExecuteArmGoal
        from rclpy.action import ActionClient
        if not rclpy.ok():rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=Node(node_name)
        self._safety_group=ReentrantCallbackGroup()
        self._service_group=ReentrantCallbackGroup()
        self._telemetry_group=ReentrantCallbackGroup()
        self.executor=MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()
        self._twist_type,self._twist_stamped_type,self._joint_jog_type,self._float64_type=Twist,TwistStamped,JointJog,Float64
        self._cmd_vel_pub=self.node.create_publisher(Twist,cmd_vel_topic,1)
        self._servo_pub=self.node.create_publisher(TwistStamped,servo_topic,1)
        self._gripper_pub=self.node.create_publisher(Float64,gripper_velocity_topic,1)
        self._flipper_pub=self.node.create_publisher(JointJog,flipper_jog_topic,1)
        self.flipper_profile_service=flipper_profile_service
        self.hybrid_service_ns=(hybrid_service_ns or '/robot_runtime').rstrip('/') or '/robot_runtime'
        self.service_commands=service_commands or {};self.vision_detection_service=vision_detection_service
        # Discover safety and business RPC endpoints up front to avoid DDS response
        # discovery races on immediate replies and eliminate dynamic create_client during spin.
        self.clients={}
        for path in (
            f'{self.hybrid_service_ns}/global_stop',
            f'{self.hybrid_service_ns}/reset_global_stop',
            f'{self.hybrid_service_ns}/halt',
        ):
            self.clients[path]=self.node.create_client(Trigger,path,callback_group=self._safety_group)
        business_clients=[
            (GroupOperation,f'{self.hybrid_service_ns}/group_operation'),
            (SetBool,'/arm_control/set_servo_mode'),
            (Trigger,'/arm_control/servo_transport_ready'),
        ]
        for cmd in ('init','enable','resume','recover','shutdown'):
            business_clients.append((Trigger,f'{self.hybrid_service_ns}/{cmd}'))
        if self.flipper_profile_service:
            business_clients.append((SetProfile,self.flipper_profile_service))
        for kind,path in business_clients:
            if path not in self.clients:
                self.clients[path]=self.node.create_client(kind,path,callback_group=self._service_group)
        self.client_lock=threading.Lock();self.runtime_reset_callback=None;self.runtime_state_callback=None;self.epoch=None
        self.runtime_state=None;self.runtime_received=None
        self._arm_type=ExecuteArmGoal
        self.arm=ActionClient(self.node,ExecuteArmGoal,'/arm_control/execute_goal',callback_group=self._service_group)
        self.node.create_subscription(String,'/robot_runtime/state',self.state,1,callback_group=self._telemetry_group)

    def state(self,message):
        import json
        try:
            state=json.loads(message.data)
            if not isinstance(state,dict):
                self.runtime_state=None;return
            previous=getattr(self,'runtime_state',None) or {}
            epoch=json.dumps([state.get('groups',{}),state.get('input_epochs',{}),
                              {k:state.get('diagnostics',{}).get(k) for k in
                               ('global_inhibit','owner_fresh','configuration_version')}],sort_keys=True)
            self.runtime_state=state;self.runtime_received=time.monotonic()
            if epoch!=self.epoch and self.runtime_reset_callback:
                changed={g for key in ('groups','input_epochs')
                         for g in set(previous.get(key,{})) | set(state.get(key,{}))
                         if previous.get(key,{}).get(g)!=state.get(key,{}).get(g)}
                global_change=any(previous.get('diagnostics',{}).get(k)!=state.get('diagnostics',{}).get(k)
                                  for k in ('global_inhibit','owner_fresh','configuration_version'))
                self.runtime_reset_callback(dict(type='runtime_epoch',groups=sorted(changed),global_change=global_change))
            self.epoch=epoch
            state_cb=getattr(self,'runtime_state_callback',None)
            if state_cb:
                state_cb(state)
        except ValueError:pass

    def close(self):
        import rclpy
        self.executor.shutdown();self.thread.join(2.0);self.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

    def wait(self,future,seconds=30,client=None):
        import threading
        done=threading.Event();future.add_done_callback(lambda _:done.set())
        if not done.wait(seconds):
            oid=getattr(self.operation_context,'operation_id',None)
            observer=self.late_operation_result
            if oid and observer:
                def completed(f):
                    try:response,error=f.result(),None
                    except Exception as exc:response,error=None,type(exc).__name__
                    observer(oid,response,error)
                future.add_done_callback(completed)
            elif client is not None and hasattr(client,'remove_pending_request'):
                try:client.remove_pending_request(future)
                except Exception:pass
            raise TimeoutError('ROS2 request timeout; outcome uncertain')
        return future.result()

    def request(self,kind,path,request,seconds=30):
        with self.client_lock:
            client=self.clients.get(path)
            if client is None:
                client=self.node.create_client(kind,path,callback_group=self._service_group)
                self.clients[path]=client
        wait_budget=min(2.0,max(0.05,float(seconds)))
        if not client.wait_for_service(timeout_sec=wait_budget):raise RuntimeError(path+' unavailable')
        return self.wait(client.call_async(request),seconds,client=client)

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

    def _hybrid_ns(self):
        return (getattr(self, 'hybrid_service_ns', '/robot_runtime') or '/robot_runtime').rstrip('/') or '/robot_runtime'

    def call_global_stop(self):
        # Independent C++ guard: never route an emergency through Python group locks.
        return self._call_trigger_service(f'{self._hybrid_ns()}/global_stop')

    def reset_global_stop(self):
        return self._call_trigger_service(f'{self._hybrid_ns()}/reset_global_stop')

    GROUP_OPERATION_TIMEOUTS = {
        'init': 55.0,
        'enable': 55.0,
        'recover': 55.0,
        'recover_resource': 55.0,
        'resume': 45.0,
        'halt': 30.0,
        'shutdown': 30.0,
    }

    def call_group_operation(self, command, groups):
        from robot_hardware.srv import GroupOperation
        # Let timeouts propagate: the control journal must retain result_unknown.
        # Multi-group init/enable/recover runs bounded backend prepare/verify steps
        # sequentially, so budget slightly below Qt's 60s stage timeout instead of 30s.
        timeout = float(self.GROUP_OPERATION_TIMEOUTS.get(command, 55.0))
        result = self.request(GroupOperation, f'{self._hybrid_ns()}/group_operation',
                              GroupOperation.Request(groups=list(groups), operation=command, resource=''),
                              seconds=timeout)
        return dict(success=result.success, operation_id=result.operation_id, message=result.message)
    def lifecycle_service_name(self,command):return f'{self._hybrid_ns()}/{command}'
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
                if result.success and mode in ('arm','servo'):
                    return self.wait_servo_transport()
                return result.success,0 if result.success else 2301,result.message
            except Exception as error:return False,2301,str(error)
        return self._call_trigger_service(path)
    def wait_servo_transport(self, timeout=15):
        # Mode completion means the gate can consume a fresh command, not just
        # that the velocity controller was selected. Never queue operator input.
        deadline = time.monotonic() + timeout
        message = 'Servo transport is preparing'
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            per_call = min(2.0, max(0.05, deadline - now))
            try:
                ok, code, message = self._call_trigger_service('/arm_control/servo_transport_ready', timeout=per_call)
            except TypeError:
                ok, code, message = self._call_trigger_service('/arm_control/servo_transport_ready')
            if ok:
                return True, 0, 'arm Servo ready; neutral input and explicit authorization required'
            time.sleep(.05)
        return False, 2301, 'Servo transport preparation timed out: ' + message

    def call_named_service(self,service_name):return self._call_trigger_service(service_name)
    def _call_trigger_service(self, path, timeout=30.0, **kwargs):
        from std_srvs.srv import Trigger
        if not path:return False,2302,'no service configured'
        try:
            result = self.request(Trigger, path, Trigger.Request(), seconds=timeout)
            return result.success, 0 if result.success else 2301, result.message
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
