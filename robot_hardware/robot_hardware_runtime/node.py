import json
import threading
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, UInt64
from .flipper import FlipperController
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController, ListControllers, ListHardwareComponents
from lifecycle_msgs.msg import State
from canopen_interfaces.srv import CORead, COWrite
from robot_hardware.srv import Maintain, SetProfile, GroupOperation
from .policy import Policy

class Adapter:
    def __init__(self, node):
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.clients = {}
        self.timeout = 20.0
        self.sdo_timeout = 5.0
        self.scope = threading.local()
        self.pending = {}
        self.pending_lock = threading.Lock()
        self.client_lock = threading.Lock()
        self.sdo_lock = threading.RLock()
        self.heartbeat_publisher = None
    def prepare_clients(self, contract, groups):
        # Discover both request and response endpoints before the first operator
        # mutation. A just-created Fast DDS client can lose an immediate reply;
        # mutation replay is unsafe, so establish clients during software startup.
        triggers={'/robot_runtime/status', '/robot_runtime/owner_lost',
                  '/arm_control/check_servo_ready'}
        for group in groups:
            for operation in ('maintenance_clear','acknowledge','arm','run','halt','maintenance_lock'):
                triggers.add('/robot_runtime/groups/'+group+'/'+operation)
            backends={axis['backend'] for axis in contract['axes'].values() if axis['group']==group}
            for backend in backends:
                for operation in ('initialize','prepare'):
                    triggers.add('/robot_backend/'+backend+'/'+group+'/'+operation)
        requests=[(Trigger,path) for path in sorted(triggers)]
        requests += [(kind,'/controller_manager/'+name) for kind,name in (
            (ListControllers,'list_controllers'),(ListHardwareComponents,'list_hardware_components'),
            (SetHardwareComponentState,'set_hardware_component_state'),(SwitchController,'switch_controller'))]
        with self.client_lock:
            for kind,path in requests:
                if path not in self.clients:
                    self.clients[path]=self.node.create_client(kind,path,callback_group=self.group)

    def begin_scope(self, group):
        self.scope.group = group
    def ensure_settled(self, group):
        with self.pending_lock:
            pending = [f for f in self.pending.get(group, [])+self.pending.get(None, []) if not f.done()]
            self.pending[group] = pending
        if pending: raise RuntimeError(group + ': previous remote operation still pending; gate remains closed')
    def heartbeat(self):
        if self.heartbeat_publisher is None:
            self.heartbeat_publisher = self.node.create_publisher(UInt64, '/robot_runtime/owner_heartbeat', 1)
        self.heartbeat_publisher.publish(UInt64(data=time.monotonic_ns()))
    def owner_reset(self):
        result = self.call('/robot_runtime/owner_lost', Trigger, Trigger.Request(), mutation=True)
        if not result.success: raise RuntimeError(result.message)
    def group_operation(self, group, operation):
        result = self.call('/robot_runtime/groups/'+group+'/'+operation, Trigger, Trigger.Request(), mutation=True)
        if not result.success: raise RuntimeError(result.message)
    def check_servo_ready(self):
        result = self.call('/arm_control/check_servo_ready', Trigger, Trigger.Request(), timeout=2.0)
        if not result.success: raise RuntimeError(result.message)
    def initialize(self, backend, group):
        result = self.call('/robot_backend/'+backend+'/'+group+'/initialize', Trigger, Trigger.Request(), mutation=True)
        if not result.success: raise RuntimeError(result.message)
    def prepare(self, backend, group):
        result = self.call('/robot_backend/'+backend+'/'+group+'/prepare', Trigger, Trigger.Request(),
                           timeout=self.node.get_parameter('prepare_timeout').value, mutation=True)
        if not result.success: raise RuntimeError(result.message)
    def status(self):
        # Read-only discovery/status failures must not hold the startup monitor
        # for the 30-second mutation deadline; publish unreadable and retry on
        # its next tick. Never apply this retry policy to device mutations.
        result = self.call('/robot_runtime/status', Trigger, Trigger.Request(), timeout=2.0)
        if not result.success: raise RuntimeError(result.message)
        return json.loads(result.message)
    def hardware_state(self, name):
        result = self.call('/controller_manager/list_hardware_components', ListHardwareComponents, ListHardwareComponents.Request(), timeout=2.0)
        return next(c.state.label for c in result.component if c.name == name)
    def call(self, service, kind, request, timeout=None, resource=None, mutation=False):
        with self.client_lock:
            client = self.clients.get(service)
            if client is None:
                client = self.node.create_client(kind, service, callback_group=self.group)
                self.clients[service] = client
        if not client.wait_for_service(timeout_sec=3.0): raise RuntimeError(f'{service}: unavailable')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _, done=event: done.set())
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        retry_at = time.monotonic() + .1
        while not event.is_set() and time.monotonic() < deadline:
            if not rclpy.ok(context=self.node.context):
                raise RuntimeError(f'{service}: context closed; remote outcome unknown')
            event.wait(min(.1, max(0., deadline-time.monotonic())))
            if not mutation and not event.is_set() and time.monotonic() >= retry_at and time.monotonic() < deadline:
                # Retry observations within the existing deadline. Cancel uses
                # callbacks too, so give each attempt its own completion event.
                future.cancel()
                future = client.call_async(request)
                event = threading.Event()
                future.add_done_callback(lambda _, done=event: done.set())
                retry_at = time.monotonic() + .1
        if not event.is_set():
            # Local timeout cannot cancel remote effects. Keep the future and fence re-arming
            # until the server has actually completed the old operation.
            group = getattr(self.scope, 'group', None)
            if mutation:
                with self.pending_lock:
                    self.pending.setdefault(group, []).append(future)
                    if resource: self.pending.setdefault(resource, []).append(future)
            else:
                future.cancel()
                raise RuntimeError(f'{service}: read timeout; no fresh observation')
            raise RuntimeError(f'{service}: timeout; outcome unknown')
        result = future.result()
        if result is None: raise RuntimeError(f'{service}: no response')
        return result
    def read(self, joint, index, sub):
        request = CORead.Request(index=index, subindex=sub)
        with self.sdo_lock:
            response = self.call('/' + joint + '/sdo_read', CORead, request, timeout=self.sdo_timeout)
        if not response.success: raise RuntimeError(f'{joint}: SDO read {index:04X}:{sub:02X} failed')
        return response.text if index in (0x1008, 0x1009, 0x100A) else response.data
    def write(self, joint, index, sub, data):
        request = COWrite.Request(index=index, subindex=sub, data=data)
        with self.sdo_lock:
            if not self.call('/' + joint + '/sdo_write', COWrite, request, timeout=self.sdo_timeout, mutation=True).success:
                raise RuntimeError(f'{joint}: SDO write {index:04X}:{sub:02X} failed')
    def hardware(self, name, state):
        request = SetHardwareComponentState.Request(name=name)
        request.target_state = State(id={'unconfigured': 1, 'inactive': 2, 'active': 3}[state], label=state)
        response = self.call('/controller_manager/set_hardware_component_state', SetHardwareComponentState, request, resource='resource:'+name, mutation=True)
        if not response.ok or response.state.id != request.target_state.id:
            raise RuntimeError(f'{name}: transition to {state} failed; state={response.state.label}')
    def controllers(self, start, stop):
        # Controller spawners are launched concurrently with the runtime node.
        # A client can therefore reach init/resume after the runtime service is
        # ready but before one of the mutually exclusive profile controllers has
        # finished loading/configuring. A loaded controller is advertised while
        # still unconfigured; activation must also wait for configure to finish.
        # Keep the strict switch as the final authority after that startup wait.
        expected = set(start) | set(stop)
        if expected:
            deadline = time.monotonic() + self.timeout
            while True:
                try:
                    response = self.call('/controller_manager/list_controllers',
                                         ListControllers, ListControllers.Request(),
                                         timeout=min(2.0, max(0.1, deadline-time.monotonic())))
                    loaded = {controller.name: controller.state for controller in response.controller}
                    missing = expected - loaded.keys()
                    missing |= {name + '(' + loaded[name] + ')' for name in start
                                if name in loaded and loaded[name] not in ('inactive', 'active')}
                    if not missing:
                        break
                except Exception:
                    missing = expected
                if time.monotonic() >= deadline:
                    raise RuntimeError('controllers not ready: ' + ', '.join(sorted(missing)))
                time.sleep(.05)
        request = SwitchController.Request(activate_controllers=start, deactivate_controllers=stop,
                                          strictness=SwitchController.Request.STRICT, activate_asap=False)
        request.timeout.sec = 15
        if not self.call('/controller_manager/switch_controller', SwitchController, request, mutation=True).ok:
            raise RuntimeError('controller switch failed')
    def active_controllers(self):
        response = self.call('/controller_manager/list_controllers', ListControllers, ListControllers.Request(), timeout=2.0)
        return [c.name for c in response.controller if c.state == 'active' and c.name != 'joint_state_broadcaster']
    def active_resources(self):
        response = self.call('/controller_manager/list_controllers', ListControllers, ListControllers.Request(), timeout=2.0)
        return {c.name: set(c.claimed_interfaces) for c in response.controller if c.state == 'active'}
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
        result = self.call('/' + joint + '/disable', Trigger, Trigger.Request(), mutation=True)
        if not result.success: raise RuntimeError(joint + ': disable failed')

class RuntimeNode(Node):
    def __init__(self):
        super().__init__('robot_runtime')
        self.declare_parameter('selected_groups', ['arm', 'base', 'flippers', 'gripper'])
        self.declare_parameter('simulation', False)
        self.declare_parameter('group_policy', '')
        self.declare_parameter('enable_servo', False)
        self.declare_parameter('prepare_timeout', 20.0)
        self.grouped = bool(self.get_parameter('group_policy').value)
        for name, default in [('bus_config', ''), ('baseline', ''), ('journal', '/tmp/robot-maintenance.json')]:
            self.declare_parameter(name, default)
        load = lambda name: yaml.safe_load(Path(self.get_parameter(name).value).read_text())
        self.io = Adapter(self)
        baseline = load('baseline')
        if self.get_parameter('simulation').value:
            from ament_index_python.packages import get_package_share_directory
            baseline = yaml.safe_load((Path(get_package_share_directory('robot_hardware'))/'config/mock_commissioning.yaml').read_text())
        self.policy = Policy(self.io, load('bus_config'), baseline, self.get_parameter('journal').value)
        if self.grouped:
            from .groups import GroupCoordinator
            contract = yaml.safe_load(Path(self.get_parameter('group_policy').value).read_text())
            if self.get_parameter('enable_servo').value:
                for controller in contract['groups']['arm'].get('optional_controllers', ['arm_servo_controller']):
                    if controller not in contract['groups']['arm']['controllers']:
                        contract['groups']['arm']['controllers'].append(controller)
            self.coordinator = GroupCoordinator(self.io, self.policy, contract, self.get_parameter('selected_groups').value)
            self.io.prepare_clients(contract, self.coordinator.allowed_groups)
        else:
            from .coordinator import Coordinator
            self.coordinator = Coordinator(self.io, self.policy)
        from ament_index_python.packages import get_package_share_directory
        self.limits = yaml.safe_load((Path(get_package_share_directory('robot_hardware'))/'config/joint_limits.yaml').read_text())
        self.policy.limits = self.limits
        self.broadcaster_generation = -1
        self.broadcaster_lock = threading.Lock()
        FlipperController.attach(self, self.limits)

        self.runtime_services = []
        self.operation_callbacks = ReentrantCallbackGroup()
        for operation in ('init', 'enable', 'resume', 'halt', 'recover', 'shutdown'):
            self.runtime_services.append(self.create_service(Trigger, '~/'+operation,
                lambda request, response, op=operation: self.trigger(op, response), callback_group=self.operation_callbacks))
        self.runtime_services.append(self.create_service(Trigger, '~/snapshot', self.snapshot, callback_group=self.operation_callbacks))
        self.runtime_services.append(self.create_service(GroupOperation, '~/group_operation', self.group_operation, callback_group=self.operation_callbacks))
        self.runtime_services.append(self.create_service(Maintain, '~/maintain', self.maintain, callback_group=self.operation_callbacks))
        from std_srvs.srv import SetBool
        self.runtime_services.append(self.create_service(SetBool, '/arm_control/set_servo_mode', self.arm_mode, callback_group=self.operation_callbacks))
        self.runtime_services.append(self.create_service(SetProfile, '~/set_flipper_profile', self.profile, callback_group=self.operation_callbacks))
        self.runtime_services.append(self.create_service(SetProfile, '/flipper_control/set_control_profile', self.profile))
        self.runtime_services.append(self.create_service(SetProfile, '/flipper_control/set_linkage_mode', self.linkage))
        self.publisher = self.create_publisher(String, '~/state', 1)
        self.monitor_group = ReentrantCallbackGroup()
        self.monitor_lock = threading.Lock()
        self.timer = self.create_timer(0.1, self.monitor, callback_group=self.monitor_group)
        if self.grouped: self.heartbeat_timer = self.create_timer(.1, self.owner_heartbeat, callback_group=self.monitor_group)
    def arm_mode(self, request, response):
        try:
            if not self.grouped:raise RuntimeError('Servo mode requires grouped runtime')
            self.coordinator.switch_arm_profile('servo' if request.data else 'trajectory')
            response.success=True;response.message=self.coordinator.arm_profile
        except Exception as error:response.success=False;response.message=str(error)
        return response

    def owner_heartbeat(self):
        try:
            if self.coordinator.check_owner(): self.io.heartbeat()
        except Exception as error:
            self.coordinator.diagnostics = {'readable': False, 'error': str(error)}
    def group_operation(self, request, response):
        if self.grouped: self.coordinator.operation_context.id = ''
        try:
            if not self.grouped: raise RuntimeError('group policy is not enabled')
            if request.operation == 'recover_resource':
                if request.groups: raise ValueError('resource recovery takes no group selection')
                selected = self.coordinator.resource_groups(request.resource)
                request.groups = selected
                self.coordinator.recover_resource(request.resource)
                self.refresh_joint_states()
                response.success = True
            else:
                if request.resource: raise ValueError('resource is only valid for recover_resource')
                if request.operation not in ('init', 'enable', 'resume', 'halt', 'recover', 'shutdown'):
                    raise ValueError('unknown group operation')
                selected = self.coordinator.selected(request.groups)
                getattr(self.coordinator, request.operation)(selected)
                self.refresh_joint_states()
                response.success = True
        except Exception as error:
            response.success = False; response.message = str(error)
        if self.grouped:
            op_id = self.coordinator.operation_context.id
            records = {}
            if op_id:
                for g in request.groups:
                    rec = self.coordinator.operations.get(g, {})
                    if rec.get('id') == op_id:
                        records[g] = rec
            response.operation_id = op_id
            response.message = json.dumps({'reason': response.message, 'groups': self.coordinator.states,
                                           'operations': records}, ensure_ascii=False)
        return response
    def refresh_joint_states(self):
        # ALL state interfaces are selected when the broadcaster activates. A component
        # configured later adds feedback interfaces without changing motion controllers.
        with self.broadcaster_lock:
            generation = self.coordinator.topology_generation
            active = self.joint_broadcaster_active()
            if all(self.io.hardware_state(c) == 'unconfigured' for c in self.coordinator.COMPONENTS):
                if active: self.io.controllers([], ['joint_state_broadcaster'])
                self.broadcaster_generation = -1
                return
            if active and self.broadcaster_generation == generation: return
            if active: self.io.controllers([], ['joint_state_broadcaster'])
            self.io.controllers(['joint_state_broadcaster'], [])
            self.broadcaster_generation = generation
    def joint_broadcaster_active(self):
        result = self.io.call('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
        return any(c.name == 'joint_state_broadcaster' and c.state == 'active' for c in result.controller)
    def trigger(self, operation, response):
        try:
            getattr(self.coordinator, operation)()
            if self.grouped: self.refresh_joint_states()
            elif operation == 'init': self.io.controllers(['joint_state_broadcaster'], [])
            response.success, response.message = True, self.coordinator.state
        except Exception as error: response.success, response.message = False, str(error)
        return response
    def snapshot(self, request, response):
        with self.coordinator.lock:
            try:
                if not self.grouped:
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
        return FlipperController.set_linkage(self, request, response)
    def joint_state(self, message):
        FlipperController.update_joint_state(self, message)
    def sync_flipper_session(self):
        return FlipperController.sync_session(self)
    def jog(self, message, session):
        FlipperController.handle_jog(self, message, session)
    def trajectory(self, message, session):
        FlipperController.handle_trajectory(self, message, session)
    def flipper_step(self):
        FlipperController.step(self)

    def monitor(self):
        # A suspended process can resume with several ready timer callbacks. Keep
        # executor threads available for RPC completions and the owner heartbeat.
        if not self.monitor_lock.acquire(blocking=False): return
        try:
            self._monitor()
        finally:
            self.monitor_lock.release()

    def _monitor(self):
        if self.grouped:
            try: self.coordinator.monitor()
            except Exception as error:
                self.get_logger().error(str(error))
                if self.coordinator.owner_checked:
                    for g in self.coordinator.allowed_groups:
                        self.coordinator.fault_reasons[g] = str(error)
                    self.coordinator.converge()
                self.coordinator.diagnostics = {'readable': False, 'error': str(error)}
            self.publisher.publish(String(data=json.dumps({'state': self.coordinator.state,
                'input_epochs': dict(self.coordinator.generations),
                'groups': dict(self.coordinator.states), 'fault_reasons': dict(self.coordinator.fault_reasons), 'diagnostics': self.coordinator.diagnostics,
                'operations': dict(self.coordinator.operations), 'profile': self.coordinator.profile, 'arm_profile': self.coordinator.arm_profile,
                'maintenance_dirty_axes': sorted(self.policy.dirty_axes)})))
            return
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

def spin_runtime(executor, node):
    from rclpy._rclpy_pybind11 import InvalidHandle
    while rclpy.ok():
        try:
            executor.spin()
            return
        except InvalidHandle:
            # A take queued before intentional subscription retirement can fail
            # outside its callback. Rebuild the wait set on the next spin.
            # All unrelated handle failures still propagate and fail closed.
            if (not rclpy.ok() or node.retired_subscription_errors <= 0 or
                    time.monotonic() - node.last_subscription_retirement > 1.0):
                raise
            node.retired_subscription_errors -= 1


def main():
    rclpy.init()
    node = RuntimeNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try: spin_runtime(executor, node)
    except KeyboardInterrupt: pass
    except Exception:
        if rclpy.ok(): raise
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
