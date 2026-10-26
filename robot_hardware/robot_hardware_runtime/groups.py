"""Group operations; the shared C++ guard remains the authority for write permission."""
from contextlib import contextmanager
import json
import threading
import time
import uuid
from .policy import counts


class GroupCoordinator:
    COMPONENTS = ('RobotCanopenSystem', 'CanDriverSystem')

    def __init__(self, io, policy, contract):
        self.io, self.policy, self.contract = io, policy, contract
        self.profile = 'csv'
        self.arm_profile = 'trajectory'
        self.topology_generation = 0
        self.lock = threading.RLock()  # Short metadata/snapshot lock, never held over device I/O.
        self.locks = {g: threading.RLock() for g in contract['groups']}
        self.component_locks = {c: threading.Lock() for c in self.COMPONENTS}
        self.states = {g: 'Stopped' for g in self.locks}
        self.generations = {g: 0 for g in self.locks}
        self.operations = {}
        self.operation_context = threading.local()
        self.diagnostics = {}
        self.requested = set()
        self.owner_checked = False
        self.owner_lock = threading.Lock()
        owners = {}
        for group, definition in contract['groups'].items():
            for controller in definition['controllers']:
                if controller in owners:
                    raise ValueError('controller resource ownership conflict')
                owners[controller] = group
        if set(self.states) != {'arm', 'base', 'flippers', 'gripper'}:
            raise ValueError('four motion groups required')
        for axis in contract['axes'].values():
            if axis['group'] not in self.states or axis['backend'] not in self.COMPONENTS:
                raise ValueError('invalid axis ownership')

    def check_owner(self):
        if self.owner_checked: return True
        if not self.owner_lock.acquire(blocking=False): return False
        try:
            status = self.io.status()
            if set(status.get('groups', {})) != set(self.states): raise RuntimeError('group guard not ready')
            previous = [g for g in self.states if status['groups'][g]['state'] in (1, 2)]
            if previous:
                self.io.owner_reset()
                active = set(self.io.active_controllers())
                owned = {c for g in self.states for c in self.controllers(g, True)}
                if active & owned: self.io.controllers([], sorted(active & owned))
                for g in previous:
                    self.states[g] = 'Fault'; self.generations[g] += 1; self.requested.add(g)
            self.owner_checked = True
            return True
        finally:
            self.owner_lock.release()

    @property
    def state(self):
        values = list(self.states.values())
        if 'Running' in values:
            return 'Running' if all(v == 'Running' for v in values) else 'PartialRunning'
        for state in ('Fault', 'Armed', 'Initialized', 'Halted'):
            if state in values:
                return state
        return 'Stopped'

    def generation(self, group):
        return self.generations[group]

    def group_state(self, group):
        return self.states[group]

    def selected(self, groups):
        result = list(self.states) if groups is None else [groups] if isinstance(groups, str) else list(groups)
        if not result or len(set(result)) != len(result) or not set(result) <= set(self.states):
            raise ValueError('select unique known groups')
        return sorted(result)

    def axes(self, group):
        return [a for a, definition in self.contract['axes'].items() if definition['group'] == group]

    def backends(self, group):
        return sorted({self.contract['axes'][a]['backend'] for a in self.axes(group)})

    def controllers(self, group, all_profiles=False):
        names = self.contract['groups'][group]['controllers']
        if group == 'arm' and not all_profiles and 'arm_servo_controller' in names:
            return ['arm_servo_controller' if self.arm_profile=='servo' else 'arm_position_controller']
        if group == 'flippers' and not all_profiles:
            return [next(n for n in names if ('csv' if self.profile == 'csv' else 'csp') in n)]
        return names

    @contextmanager
    def operation(self, groups, action):
        self.operation_context.id = ''
        selected = self.selected(groups)
        if not self.check_owner(): raise RuntimeError('runtime ownership reconciliation in progress')
        acquired = []
        try:
            for group in selected:
                if not self.locks[group].acquire(blocking=False):
                    raise RuntimeError(group + ': another operation is in progress')
                acquired.append(group)
            operation_id = uuid.uuid4().hex
            self.operation_context.id = operation_id
            with self.lock:
                self.requested.update(selected)
                for group in selected:
                    self.operations[group] = {'id': operation_id, 'action': action, 'state': 'in_progress'}
            if len(selected) == 1: self.io.begin_scope(selected[0])
            yield selected
            with self.lock:
                for group in selected:
                    if self.operations[group]['state'] == 'in_progress':
                        self.operations[group]['state'] = 'complete'
        except Exception as error:
            with self.lock:
                for group in acquired:
                    if group in self.operations and self.operations[group]['state'] == 'in_progress':
                        self.operations[group].update(state='failed', reason=str(error))
            raise
        finally:
            self.io.begin_scope(None)
            for group in reversed(acquired):
                self.locks[group].release()

    def transition(self, component, target):
        with self.component_locks[component]:
            self.io.ensure_settled('resource:'+component)
            current = self.io.hardware_state(component)
            if current == target:
                return
            if target == 'active' and current == 'unconfigured':
                self.io.hardware(component, 'inactive')
                with self.lock: self.topology_generation += 1
            self.io.hardware(component, target)
            if target == 'inactive' and current == 'unconfigured':
                with self.lock: self.topology_generation += 1

    def _init(self, group):
        if self.states[group] in ('Running', 'Armed'):
            raise RuntimeError(group + ': halt before init')
        for backend in self.backends(group):
            with self.component_locks[backend]:
                self.io.ensure_settled('resource:'+backend)
                if self.io.hardware_state(backend) == 'unconfigured':
                    self.io.hardware(backend, 'inactive')
                    with self.lock: self.topology_generation += 1
        canopen_axes = [a for a in self.axes(group) if self.contract['axes'][a]['backend'] == 'RobotCanopenSystem']
        if canopen_axes:
            self.io.initialize('RobotCanopenSystem', group)
            self.policy.verify(canopen_axes)
        self.states[group] = 'Initialized'

    def _prepare(self, group):
        self.io.ensure_settled(group)
        if self.states[group] not in ('Initialized', 'Halted'):
            raise RuntimeError(group + ': init/recover required')
        active = set(self.io.active_controllers())
        lingering = [c for c in self.controllers(group, True) if c in active]
        if lingering: self.io.controllers([], lingering)
        for backend in self.backends(group):
            self.transition(backend, 'active')
            self.io.prepare(backend, group)
        deadline = time.monotonic() + 3
        while True:
            status = self.io.status()['groups'][group]
            if status['ready']:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(group + ': feedback readiness timeout: ' + json.dumps(status))
            time.sleep(.02)

    def _acknowledge(self, group):
        if set(self.axes(group)) & self.policy.dirty_axes:
            raise RuntimeError(group + ': unresolved maintenance journal')
        self.io.group_operation(group, 'maintenance_clear')
        self.io.group_operation(group, 'acknowledge')

    def _arm(self, group):
        self.io.ensure_settled(group)
        self.io.heartbeat()
        self.io.group_operation(group, 'arm')
        self.generations[group] += 1
        self.states[group] = 'Armed'

    def _enable(self, group):
        self._prepare(group)
        self._acknowledge(group)
        self._arm(group)

    def _stop(self, group):
        self.generations[group] += 1
        errors = []
        try:
            self.io.group_operation(group, 'halt')  # Invalidates even an in-flight preparation.
        except Exception as error:
            errors.append(str(error))
        try:
            active = set(self.io.active_controllers())
            stop = [c for c in self.controllers(group, True) if c in active]
            if stop:
                self.io.controllers([], stop)
        except Exception as error:
            errors.append(str(error))
        self.states[group] = 'Fault' if errors else 'Halted'
        if errors:
            raise RuntimeError('; '.join(errors))

    def _apply(self, groups, action, method, allowed=None, phases=None):
        with self.operation(groups, action) as selected:
            if allowed and any(self.states[g] not in allowed for g in selected):
                raise RuntimeError(action + ': invalid group state; request rejected')
            errors, failed = [], set()
            steps = phases or [method]
            for step in steps:
                for group in selected:
                    if group in failed:
                        continue
                    try:
                        self.io.begin_scope(group)
                        step(group)
                    except Exception as error:
                        try:
                            self._stop(group)
                        except Exception:
                            pass
                        self.states[group] = 'Fault'
                        self.operations[group].update(state='failed', reason=str(error))
                        failed.add(group)
                        errors.append(group + ': ' + str(error))
                    finally:
                        self.io.begin_scope(None)
            for group in set(selected) - failed:
                self.operations[group]['state'] = 'complete'
            if errors:
                raise RuntimeError('; '.join(errors))

    def init(self, groups=None):
        self._apply(groups, 'init', self._init, {'Stopped', 'Fault', 'Halted', 'Initialized'})

    def enable(self, groups=None):
        self._apply(groups, 'enable', None, {'Initialized', 'Halted'},
                    [self._prepare, self._acknowledge, self._arm])

    def resume(self, groups=None):
        def prepare(group):
            if self.states[group] == 'Halted':
                self._init(group); self._prepare(group)
        def acknowledge(group):
            if self.states[group] != 'Armed':
                self._acknowledge(group)
        def arm(group):
            if self.states[group] != 'Armed': self._arm(group)
        def start(group):
            self.io.controllers(self.controllers(group), [])
            self.io.group_operation(group, 'run')
            self.states[group] = 'Running'
        self._apply(groups, 'resume', None, {'Armed', 'Halted'}, [prepare, acknowledge, arm, start])

    def halt(self, groups=None):
        self._apply(groups, 'halt', self._stop)

    def recover(self, groups=None):
        self._apply(groups, 'recover', None, phases=[self._stop, self._init, self._prepare,
            self._acknowledge, self._arm])

    def resource_groups(self, component):
        if component not in self.COMPONENTS: raise ValueError('unknown shared hardware resource')
        return sorted(g for g in self.states if component in self.backends(g))

    def recover_resource(self, component):
        users = self.resource_groups(component)
        rebuilt = False
        def rebuild(group):
            nonlocal rebuilt
            if rebuilt: return
            self.transition(component, 'unconfigured')
            self.transition(component, 'inactive')
            rebuilt = True
        self._apply(users, 'recover_resource', None, phases=[self._stop, rebuild, self._init,
            self._prepare, self._acknowledge, self._arm])

    def shutdown(self, groups=None):
        with self.operation(groups, 'shutdown') as selected:
            for group in selected:
                self._stop(group)
                self.states[group] = 'Stopped'
            for component in self.COMPONENTS:
                users = {g for g in self.states if component in self.backends(g)}
                if users <= set(selected):
                    self.transition(component, 'unconfigured')

    def converge(self, groups=None):
        for group in self.selected(groups):
            self.io.begin_scope(group)
            try:
                self._stop(group)
            except Exception:
                pass
            finally:
                self.io.begin_scope(None)
            self.states[group] = 'Fault'

    def switch_profile(self, profile):
        if profile not in ('csv', 'csp'):
            raise ValueError('profile must be csv or csp')
        if profile == self.profile:
            return
        with self.operation(['flippers'], 'profile'):
            was_running = self.states['flippers'] == 'Running'
            self._stop('flippers')
            self.profile = profile
            if was_running:
                try:
                    self._init('flippers'); self._enable('flippers')
                    self.io.controllers(self.controllers('flippers'), [])
                    self.io.group_operation('flippers', 'run')
                    self.states['flippers'] = 'Running'
                except Exception:
                    self.converge(['flippers'])
                    raise

    def switch_arm_profile(self, profile):
        if profile not in ('trajectory','servo'):raise ValueError('arm profile must be trajectory or servo')
        if profile == 'servo' and 'arm_servo_controller' not in self.contract['groups']['arm']['controllers']:
            raise RuntimeError('Servo controller not configured; launch with enable_servo:=true')
        if profile == self.arm_profile:return
        with self.operation(['arm'], 'arm_profile'):
            was_running=self.states['arm']=='Running'
            self._stop('arm');self.arm_profile=profile
            if was_running:
                try:
                    self._init('arm');self._enable('arm')
                    self.io.controllers(self.controllers('arm'),[])
                    self.io.group_operation('arm','run');self.states['arm']='Running'
                except Exception:
                    self.converge(['arm']);raise

    def maintain(self, joint, operation, lower, upper, save):
        if joint not in self.contract['axes'] or self.contract['axes'][joint]['backend'] != 'RobotCanopenSystem':
            raise ValueError('maintenance requires a known CANopen axis')
        if operation not in ('restore', 'set_zero', 'set_offset', 'limits', 'limits_from_urdf', 'disable_limits'):
            raise ValueError('unknown maintenance operation')
        if operation == 'set_offset': counts(lower, joint)
        if operation == 'limits' and counts(lower, joint) >= counts(upper, joint):
            raise ValueError('limits must be strictly ordered')
        if operation == 'limits_from_urdf' and joint not in self.policy.limits:
            raise ValueError('URDF limits unavailable')
        if operation == 'restore' and self.policy.journal_axis != joint:
            raise ValueError('restore target does not match unresolved journal')
        group = self.contract['axes'][joint]['group']
        with self.operation([group], 'maintain'):
            self._stop(group)
            self.io.disable(joint)
            self.policy.verify([joint], allow_dirty=operation == 'restore')
            try:
                if operation == 'restore':
                    if self.policy.journal_axis != joint:
                        raise RuntimeError('restore target does not match unresolved journal')
                    self.policy.restore()
                else:
                    self.policy.maintain(joint, operation, lower, upper, save)
            finally:
                self.states[group] = 'Fault' if joint in self.policy.dirty_axes else 'Initialized'
                if joint in self.policy.dirty_axes: self.io.group_operation(group, 'maintenance_lock')

    def monitor(self):
        if not self.check_owner(): return
        observed = {g: (self.generations[g], self.states[g]) for g in self.states}
        for group in self.states:
            if set(self.axes(group)) & self.policy.dirty_axes:
                self.io.group_operation(group, 'maintenance_lock')
        status = self.io.status()
        resources = self.io.active_resources()
        active = set(resources)
        status['readable'] = True
        status['observed_monotonic'] = time.monotonic()
        self.diagnostics = status
        for group in list(self.requested):
            if self.states[group] not in ('Running', 'Armed'):
                continue
            expected = set(self.controllers(group))
            actual = active.intersection(self.controllers(group, True))
            interface = 'velocity' if group == 'base' or (group == 'flippers' and self.profile == 'csv') else 'position'
            required_resources = {a+'/'+interface for a in self.axes(group)}
            valid_resources = all(resources.get(c, set()) == required_resources for c in expected)
            expected_state = 2 if self.states[group] == 'Running' else 1
            bad_resources = self.states[group] == 'Running' and (actual != expected or not valid_resources)
            if status['groups'][group]['state'] != expected_state or bad_resources:
                if self.locks[group].acquire(blocking=False):
                    try:
                        # The RPC snapshot may precede a completed halt/rearm/start.
                        # Never revoke a newer session using an older observation.
                        if observed[group] != (self.generations[group], self.states[group]):
                            continue
                        self.io.begin_scope(group)
                        self._stop(group)
                        self.states[group] = 'Fault'
                    finally:
                        self.io.begin_scope(None)
                        self.locks[group].release()
