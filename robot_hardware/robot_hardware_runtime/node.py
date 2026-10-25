import json
import math
import threading
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .flipper import Reference, NAMES, GROUPS
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController, ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State
from canopen_interfaces.srv import CORead, COWrite
from robot_hardware.srv import Maintain, SetProfile
from .policy import Policy
from .coordinator import Coordinator

class Adapter:
    def __init__(self, node):
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.clients = {}
        self.timeout = 30.0
    def call(self, service, kind, request):
        client = self.clients.get(service)
        if client is None:
            client = self.node.create_client(kind, service, callback_group=self.group)
            self.clients[service] = client
        if not client.wait_for_service(timeout_sec=3.0): raise RuntimeError(f'{service}: unavailable')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(self.timeout):
            future.cancel()
            raise RuntimeError(f'{service}: timeout; outcome unknown')
        result = future.result()
        if result is None: raise RuntimeError(f'{service}: no response')
        return result
    def read(self, joint, index, sub):
        request = CORead.Request(index=index, subindex=sub)
        response = self.call('/' + joint + '/sdo_read', CORead, request)
        if not response.success: raise RuntimeError(f'{joint}: SDO read {index:04X}:{sub:02X} failed')
        return response.text if index in (0x1008, 0x1009, 0x100A) else response.data
    def write(self, joint, index, sub, data):
        request = COWrite.Request(index=index, subindex=sub, data=data)
        if not self.call('/' + joint + '/sdo_write', COWrite, request).success:
            raise RuntimeError(f'{joint}: SDO write {index:04X}:{sub:02X} failed')
    def hardware(self, name, state):
        request = SetHardwareComponentState.Request(name=name)
        request.target_state = State(id={'unconfigured': 1, 'inactive': 2, 'active': 3}[state], label=state)
        response = self.call('/controller_manager/set_hardware_component_state', SetHardwareComponentState, request)
        if not response.ok or response.state.id != request.target_state.id:
            raise RuntimeError(f'{name}: transition to {state} failed; state={response.state.label}')
    def controllers(self, start, stop):
        request = SwitchController.Request(activate_controllers=start, deactivate_controllers=stop,
                                          strictness=SwitchController.Request.STRICT, activate_asap=False)
        request.timeout.sec = 15
        if not self.call('/controller_manager/switch_controller', SwitchController, request).ok:
            raise RuntimeError('controller switch failed')
    def active_controllers(self):
        response = self.call('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
        return [c.name for c in response.controller if c.state == 'active' and c.name != 'joint_state_broadcaster']
    def running(self, value):
        response = self.call('/robot_runtime/set_running', SetBool, SetBool.Request(data=value))
        if not response.success: raise RuntimeError(response.message)
    def arm(self):
        deadline = time.monotonic() + 3
        while True:
            result = self.call('/robot_runtime/arm', Trigger, Trigger.Request())
            if result.success: return
            if time.monotonic() >= deadline: raise RuntimeError('both hardware components not ready: ' + result.message)
            time.sleep(0.02)
    def disable(self, joint):
        result = self.call('/' + joint + '/disable', Trigger, Trigger.Request())
        if not result.success: raise RuntimeError(joint + ': disable failed')

class RuntimeNode(Node):
    def __init__(self):
        super().__init__('robot_runtime')
        self.declare_parameter('simulation', False)
        for name, default in [('bus_config', ''), ('baseline', ''), ('journal', '/tmp/robot-maintenance.json')]:
            self.declare_parameter(name, default)
        load = lambda name: yaml.safe_load(Path(self.get_parameter(name).value).read_text())
        self.io = Adapter(self)
        baseline = load('baseline')
        if self.get_parameter('simulation').value:
            from ament_index_python.packages import get_package_share_directory
            baseline = yaml.safe_load((Path(get_package_share_directory('robot_hardware'))/'config/mock_commissioning.yaml').read_text())
        self.policy = Policy(self.io, load('bus_config'), baseline, self.get_parameter('journal').value)
        self.coordinator = Coordinator(self.io, self.policy)
        from ament_index_python.packages import get_package_share_directory
        self.limits = yaml.safe_load((Path(get_package_share_directory('robot_hardware'))/'config/joint_limits.yaml').read_text())
        self.policy.limits = self.limits
        self.reference = Reference(self.limits)
        self.measured = None
        self.measured_at = 0.0
        self.last_profile = None
        self.jog_active = False
        self.csv_pub = self.create_publisher(Float64MultiArray, '/flipper_csv_forward_controller/command', 1)
        self.csp_pub = self.create_publisher(JointTrajectory, '/flipper_csp_controller/joint_trajectory', 1)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state, 1)
        self.jog_sub = self.create_subscription(JointJog, '/flipper_control/jog_cmd', self.jog, 1)
        self.trajectory_sub = self.create_subscription(JointTrajectory, '/flipper_control/command', self.trajectory, 1)
        self.flipper_timer = self.create_timer(0.02, self.flipper_step)

        self.runtime_services = []
        for operation in ('init', 'enable', 'resume', 'halt', 'recover', 'shutdown'):
            self.runtime_services.append(self.create_service(Trigger, '~/'+operation,
                lambda request, response, op=operation: self.trigger(op, response)))
        self.runtime_services.append(self.create_service(Trigger, '~/snapshot', self.snapshot))
        self.runtime_services.append(self.create_service(Maintain, '~/maintain', self.maintain))
        self.runtime_services.append(self.create_service(SetProfile, '~/set_flipper_profile', self.profile))
        self.runtime_services.append(self.create_service(SetProfile, '/flipper_control/set_control_profile', self.profile))
        self.runtime_services.append(self.create_service(SetProfile, '/flipper_control/set_linkage_mode', self.linkage))
        self.publisher = self.create_publisher(String, '~/state', 1)
        self.monitor_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.1, self.monitor, callback_group=self.monitor_group)
    def trigger(self, operation, response):
        try:
            getattr(self.coordinator, operation)()
            if operation == 'init':
                self.io.controllers(['joint_state_broadcaster'], [])
            response.success, response.message = True, self.coordinator.state
        except Exception as error: response.success, response.message = False, str(error)
        return response
    def snapshot(self, request, response):
        with self.coordinator.lock:
            try:
                if self.coordinator.state == 'Running': raise RuntimeError('halt before snapshot')
                self.io.hardware('RobotCanopenSystem', 'inactive')
                self.io.running(False)
                # Read-only export; operator must add physical labels/directions and acknowledge commissioning.
                response.message = json.dumps(self.policy.snapshot(), ensure_ascii=False)
                response.success = True
            except Exception as error: response.message = str(error); response.success = False
        return response
    def maintain(self, request, response):
        try:
            self.coordinator.maintain(request.joint, request.operation, request.lower, request.upper, request.save)
            response.success, response.message = True, 'readback verified; reboot persistence verification required if saved'
        except Exception as error: response.success, response.message = False, str(error)
        return response
    def profile(self, request, response):
        try:
            self.coordinator.switch_profile(request.profile)
            response.success, response.message = True, self.coordinator.profile
        except Exception as error: response.success, response.message = False, str(error)
        return response
    def linkage(self, request, response):
        if request.profile not in GROUPS:
            response.success=False;response.message='unknown linkage mode';return response
        self.reference.reset();self.reference.linkage=request.profile
        response.success=True;response.message=request.profile;return response
    def joint_state(self, message):
        values=dict(zip(message.name,message.position))
        if all(name in values for name in NAMES):
            self.measured=[values[name] for name in NAMES];self.measured_at=time.monotonic()
    def jog(self, message):
        if self.coordinator.state!='Running':return
        values=list(message.velocities)
        if not values and message.duration>0:values=[d/message.duration for d in message.displacements]
        try:
            self.reference.command(list(message.joint_names), values, time.monotonic());self.jog_active=True
        except ValueError as error:self.get_logger().error(str(error));self.jog_active=False
    def trajectory(self, message):
        if self.coordinator.state!='Running' or self.coordinator.profile!='csp':return
        try:
            if set(message.joint_names)!=set(NAMES) or len(message.joint_names)!=4 or not message.points:
                raise ValueError('CSP trajectory requires four unique flipper joints')
            previous=-1
            for point in message.points:
                stamp=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
                if stamp<=previous or len(point.positions)!=4:raise ValueError('invalid trajectory timing/size')
                previous=stamp
                for name,q in zip(message.joint_names,point.positions):
                    limit=self.limits[name]
                    if not math.isfinite(q) or not limit['lower']<=q<=limit['upper']:raise ValueError('trajectory outside limits')
                if any(not math.isfinite(v) for v in list(point.velocities)+list(point.accelerations)):
                    raise ValueError('non-finite trajectory derivative')
            self.reference.reset();self.jog_active=False;self.csp_pub.publish(message)
        except ValueError as error:self.get_logger().error(str(error))
    def flipper_step(self):
        state=(self.coordinator.state,self.coordinator.profile)
        if state!=self.last_profile:
            self.reference.reset();self.jog_active=False;self.last_profile=state
        if state[0]!='Running' or self.measured is None:return
        if time.monotonic()-self.measured_at>.1:
            self.csv_pub.publish(Float64MultiArray(data=[0.0]*4));self.reference.reset();return
        if not self.jog_active:return
        try:positions,velocities,expired=self.reference.step(self.measured,time.monotonic(),.02)
        except ValueError:return
        if self.coordinator.profile=='csv':self.csv_pub.publish(Float64MultiArray(data=velocities))
        else:
            point=JointTrajectoryPoint(positions=positions)
            point.time_from_start.nanosec=40000000
            self.csp_pub.publish(JointTrajectory(joint_names=list(NAMES),points=[point]))
        if expired:self.jog_active=False

    def monitor(self):
        if not self.coordinator.lock.acquire(blocking=False): return
        try:
            if self.coordinator.state == 'Running':
                try:
                    response = self.io.call('/robot_runtime/status', Trigger, Trigger.Request())
                    if not response.success: raise RuntimeError(response.message)
                    if len(self.io.active_controllers()) != 4: raise RuntimeError('expected motion controller stopped')
                except Exception as error:
                    self.get_logger().error(str(error)); self.coordinator.converge()
            self.publisher.publish(String(data=json.dumps({'state': self.coordinator.state,
                'profile': self.coordinator.profile, 'maintenance_dirty': self.policy.dirty})))
        finally: self.coordinator.lock.release()

def main():
    rclpy.init()
    node = RuntimeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()
