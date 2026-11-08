"""Versioned admission/execution boundary for the existing BridgeCore (no ROS imports)."""
import copy
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import queue
import secrets
import threading
import time
import uuid

CAPABILITIES = ['session_v2', 'freshness_v2', 'explicit_authorization',
                'bounded_operations', 'isolated_output', 'runtime_permission']
BUILD = 'robot24-p0-config-2'


class Rejected(Exception):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


class ControlV2:
    TOKEN_SEC = .300
    REFRESH_SEC = .100
    LEASE_SEC = 1.5
    RUNTIME_SEC = 1.0
    CACHE_SEC = 86400
    MAX_RECORDS = 512
    MOTION_KEYS = set('wasduoqekijlfhy')

    def __init__(self, core, clock=time.monotonic, state_dir=None):
        self.core, self.clock = core, clock
        self.lock = threading.RLock()
        self.output_lock = threading.RLock()
        self.latch_lock = threading.Lock()
        self.disk_lock = threading.Lock()
        self.journal_revision = 0
        self.persisted_revision = 0
        self.durable = {}
        self.reserved_waiting = 0
        self.emergency_pending = threading.Event()
        self.sessions, self.owner, self.latest = {}, None, None
        self.epoch = str(uuid.uuid4())
        self.counts = Counter()
        self.core.output.late_operation_result = self._late_result
        self.operations, self.requests = {}, {}
        self.jobs = queue.Queue(16)
        self.safety = queue.Queue(1)
        self.safety_status = dict(state='idle',physical_stop_confirmed=False)
        self.closed = threading.Event()
        self.path = None
        self.journal_lock = None
        if state_dir:
            folder = Path(state_dir)
            folder.mkdir(mode=0o700, parents=True, exist_ok=True)
            # The bridge process owns this journal; deployment uses a private directory.
            import fcntl
            self.journal_lock = (folder / 'control.lock').open('a')
            fcntl.flock(self.journal_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.path = folder / 'control-operations.json'
            latch = folder / 'emergency.json'
            if latch.exists():
                self.core.state.emergency_active = json.loads(latch.read_text())['active']
            if self.path.exists():
                data = json.loads(self.path.read_text())
                self.operations, self.requests = data['operations'], data['requests']
                if not latch.exists():
                    self.core.state.emergency_active = data.get('emergency_active', False)
                for op in self.operations.values():
                    if op['state'] in ('accepted', 'running'):
                        op.update(state='result_unknown', error='BRIDGE_RESTART')
                self._save()
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()
        self.safety_worker = threading.Thread(target=self._safety_work, daemon=True)
        self.safety_worker.start()

    def close(self):
        self.closed.set()
        self.worker.join(1)
        self.safety_worker.join(1)
        if self.journal_lock and not self.worker.is_alive() and not self.safety_worker.is_alive() and not self.reserved_waiting and not self.disk_lock.locked():
            self.journal_lock.close()
            self.journal_lock = None

    def _save(self):
        if not self.path:
            return
        import os
        with self.lock:
            self.journal_revision += 1
            revision = self.journal_revision
            snapshot = copy.deepcopy(dict(operations=self.operations, requests=self.requests))
        # No admission/watchdog lock across encoding, disk I/O, rename or fsync.
        with self.disk_lock:
            if revision <= self.persisted_revision:
                return
            tmp = self.path.with_suffix('.tmp')
            with tmp.open('w') as out:
                json.dump(snapshot, out, allow_nan=False)
                out.flush();os.fsync(out.fileno())
            tmp.replace(self.path)
            fd = os.open(self.path.parent, os.O_DIRECTORY)
            try:os.fsync(fd)
            finally:os.close(fd)
            self.persisted_revision = revision

    def _revoke(self, reason):
        had_motion = bool(self.owner in self.sessions and self.sessions[self.owner]['authorized'])
        had_motion = had_motion or any(op.get('state')=='running' and op.get('command')=='move_arm_named_target' for op in self.operations.values())
        if self.owner in self.sessions:
            s = self.sessions[self.owner]
            s.update(authorized=False, operations_authorized=False, neutral_at=None, tokens={})
            s['authorization_generation'] = s.get('authorization_generation',0)+1
        self.latest = None
        self.core.reset_operator_input_session(reason)
        self.counts['revocations'] += 1
        if had_motion:
            self._queue_halt()

    def runtime_changed(self, reason='runtime_epoch'):
        with self.lock:
            self.epoch = str(uuid.uuid4())
            self._revoke(reason)

    def disconnect(self, connection):
        with self.lock:
            if connection == self.owner:
                self._revoke('disconnect')
                self.owner = None
            self.sessions.pop(connection, None)

    def _session(self, connection, m, control=True):
        s = self.sessions.get(connection)
        if not s or m.get('session_id') != s['session_id']:
            raise Rejected('SESSION')
        if m.get('epoch') != self.epoch:
            raise Rejected('EPOCH')
        if control and (self.owner != connection or s['role'] != 'control' or self.clock() >= s['lease_until']):
            raise Rejected('LEASE')
        return s

    def _fresh(self, connection, m):
        s = self._session(connection, m)
        token = s['tokens'].get(m.get('freshness_token'))
        ttl = m.get('ttl_ms')
        if type(ttl) is not int or not 1 <= ttl <= 300:
            raise Rejected('TTL')
        if token is None or self.clock() >= token + ttl / 1000:
            raise Rejected('STALE_TOKEN')
        seq = m.get('seq')
        if type(seq) is not int or not 1 <= seq <= 2**53-1:
            raise Rejected('SEQ')
        if seq <= s['seq']:
            raise Rejected('REPLAY')
        s['seq'] = seq
        return s, token + ttl / 1000

    def _token(self, s):
        now = self.clock()
        s['tokens'] = {k: v for k, v in s['tokens'].items() if now < v + self.TOKEN_SEC}
        if not s['tokens'] or now - max(s['tokens'].values()) >= self.REFRESH_SEC:
            s['tokens'][secrets.token_urlsafe(24)] = now
        token = max(s['tokens'], key=s['tokens'].get)
        s['lease_until'] = now + self.LEASE_SEC
        return dict(freshness_token=token, valid_for_ms=max(0, int((s['tokens'][token]+self.TOKEN_SEC-now)*1000)),
                    refresh_ms=100, input_ttl_ms=300, lease_ms=1500, watchdog_ms=min(500, self.core.watchdog_ms))

    def permission(self):
        output = self.core.output
        value = getattr(output, 'runtime_state', None)
        at = getattr(output, 'runtime_received', None)
        if not isinstance(value, dict) or at is None or self.clock() - at >= self.RUNTIME_SEC:
            return False
        d = value.get('diagnostics', {})
        if not isinstance(d,dict) or not isinstance(d.get('groups'),dict) or not isinstance(value.get('groups'),dict):
            return False
        if d.get('readable') is not True or d.get('global_inhibit') is not False or d.get('owner_fresh') is not True:
            return False
        # Conservative whole-robot permission; partial-group driving is not negotiated in v2.
        for group in ('base', 'arm', 'flippers', 'gripper'):
            g = d.get('groups', {}).get(group, {})
            if not isinstance(g,dict):
                return False
            if (value.get('groups', {}).get(group) != 'Running' or g.get('state') != 2 or
                    g.get('ready') is not True or g.get('blockers') != []):
                return False
        return True

    def _input(self, m):
        keyboard, gamepad = m.get('keyboard', {}), m.get('gamepad', {})
        if not isinstance(keyboard, dict) or not isinstance(gamepad, dict):
            raise Rejected('INPUT_SCHEMA')
        keys = keyboard.get('pressed_keys', [])
        if not isinstance(keys, list) or any(not isinstance(k, str) or k not in self.MOTION_KEYS for k in keys):
            raise Rejected('INPUT_KEYS')
        axes, buttons = gamepad.get('axes', {}), gamepad.get('buttons', {})
        if not isinstance(axes, dict) or not isinstance(buttons, dict) or type(gamepad.get('connected', False)) is not bool:
            raise Rejected('INPUT_SCHEMA')
        if set(axes) - {'left_x', 'left_y', 'right_x', 'right_y', 'lt', 'rt'}:
            raise Rejected('INPUT_SCHEMA')
        if set(buttons) - {'lb','rb','l1','r1','l2','r2','dpad_up','dpad_down','triangle','cross','x','square','b','circle','y','a'}:
            raise Rejected('INPUT_SCHEMA')
        if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1 for v in axes.values()):
            raise Rejected('INPUT_SCHEMA')
        if any(type(v) is not bool for v in buttons.values()):
            raise Rejected('INPUT_SCHEMA')
        neutral = not keys and not any(abs(v) > self.core.gamepad_deadzone for v in axes.values()) and not any(buttons.values())
        return neutral

    def handle(self, connection, m):
        if m.get('type') == 'emergency_stop':
            self.emergency()
            return dict(type='control_result',protocol_version=2,request_id=m.get('request_id'),seq=m.get('seq'),ok=True,
                        result=dict(emergency_active=True,physical_stop_confirmed=False))
        try:
            if m.get('protocol_version') == 2 and m.get('type') in ('system_command','camera_stream_request'):
                result = self._submit(connection, m)
            else:
                with self.lock:
                    result = copy.deepcopy(self._handle(connection, m))
            self.counts['accepted'] += 1
            return dict(type='control_result', protocol_version=2, request_id=m.get('request_id'), seq=m.get('seq'), ok=True, result=result)
        except Rejected as exc:
            self.counts[exc.code] += 1
            return dict(type='control_result', protocol_version=2, request_id=m.get('request_id'), seq=m.get('seq'), ok=False, error=dict(code=exc.code))

    def _handle(self, connection, m):
        kind = m.get('type')
        if kind == 'emergency_stop':
            self.emergency()
            return dict(emergency_active=True, physical_stop_confirmed=False)
        if m.get('protocol_version') != 2:
            raise Rejected('UPGRADE_REQUIRED')
        if kind == 'control_hello':
            if connection in self.sessions:
                raise Rejected('ALREADY_NEGOTIATED')
            if m.get('robot_id') != self.core.robot_id or m.get('role') not in ('control','monitor'):
                raise Rejected('IDENTITY_ROLE')
            if not isinstance(m.get('capabilities'), list) or not set(CAPABILITIES) <= set(m['capabilities']):
                raise Rejected('CAPABILITIES')
            if len(self.sessions) >= 16:
                raise Rejected('BUSY')
            if m['role'] == 'control' and self.owner is not None:
                old = self.sessions[self.owner]
                if self.clock() < old['lease_until']:
                    raise Rejected('OWNER_BUSY')
                self._revoke('lease_expired')
                self.owner = None
            s = dict(session_id=secrets.token_urlsafe(24), role=m['role'], lease_until=0, tokens={}, seq=0,
                     authorized=False, operations_authorized=False, neutral_at=None)
            self.sessions[connection] = s
            if s['role'] == 'control':
                self.owner = connection
                self._revoke('acquired')
            result = dict(session_id=s['session_id'], epoch=self.epoch, robot_id=self.core.robot_id,
                          capabilities=CAPABILITIES, role=s['role'], build_version=BUILD, authorized=False)
            if s['role'] == 'control':
                result.update(self._token(s))
            return result
        if kind == 'control_status':
            s = self.sessions.get(connection)
            if not s or s['session_id'] != m.get('session_id'):
                raise Rejected('SESSION')
            return dict(epoch=self.epoch, counts=dict(self.counts), motion_permitted=self.permission(),
                        emergency_active=self.core.state.emergency_active, queue_depth=self.jobs.qsize(), safety_halt=self.safety_status,
                        authorized=self.sessions[connection]['authorized'])
        if kind == 'operation_query':
            self._session(connection, m, False)
            op = self.operations.get(m.get('operation_id'))
            if op is None or op.get('robot_id') != self.core.robot_id:
                raise Rejected('RESULT_UNKNOWN')
            return op
        if kind == 'freshness_request':
            s = self._session(connection, m)
            return dict(epoch=self.epoch, **self._token(s))
        # Idempotent lookup precedes fresh/sequence checks, but stays robot/session protected.
        if kind == 'system_command' or kind == 'camera_stream_request':
            self._session(connection, m)
            return self._submit(connection, m)
        s, deadline = self._fresh(connection, m)
        if kind == 'operator_input':
            neutral = self._input(m)
            if neutral:
                s['neutral_at'] = self.clock()
            if not s['authorized']:
                if not neutral:
                    raise Rejected('NOT_AUTHORIZED')
                self.core.publish_zero('neutral')
                return dict(input_status='neutral', authorized=False)
            if not self.permission():
                self._revoke('permission_lost')
                raise Rejected('RUNTIME_PERMISSION')
            self.latest = (connection, copy.deepcopy(m), deadline)
            return dict(input_status='staged')
        if kind == 'authorize_operations':
            if s['neutral_at'] is None or self.clock()-s['neutral_at'] >= .300:
                raise Rejected('NEUTRAL_REQUIRED')
            if self.core.state.emergency_active:
                raise Rejected('EMERGENCY_ACTIVE')
            s['operations_authorized'] = True
            return dict(operations_authorized=True, authorized=s['authorized'])
        if kind == 'authorize':
            if s['neutral_at'] is None or self.clock()-s['neutral_at'] >= .300:
                raise Rejected('NEUTRAL_REQUIRED')
            if self.core.state.emergency_active or not self.permission():
                raise Rejected('RUNTIME_PERMISSION')
            s['authorized'] = True
            s['last_input'] = self.clock()
            return dict(authorized=True)
        if kind == 'clear_emergency':
            if s['neutral_at'] is None or self.clock()-s['neutral_at'] >= .300:
                raise Rejected('NEUTRAL_REQUIRED')
            self._revoke('clear_emergency')
            with self.output_lock:
                self.emergency_pending.clear()
                self.core.state.emergency_active = False
                self.core.state.emergency_source = ''
                self._save_latch(False)
            return dict(emergency_active=False, authorized=False, runtime_release_required=True)
        if kind == 'release_control':
            self._revoke('released')
            self.owner = None
            return dict(authorized=False)
        raise Rejected('METHOD')

    def _save_latch(self, active):
        if not self.path:
            return
        import os
        with self.latch_lock:
            path = self.path.parent / 'emergency.json'
            tmp = path.with_suffix('.tmp')
            with tmp.open('w') as out:
                json.dump(dict(active=active),out)
                out.flush();os.fsync(out.fileno())
            tmp.replace(path)
            fd=os.open(path.parent,os.O_DIRECTORY)
            try:os.fsync(fd)
            finally:os.close(fd)

    def emergency(self):
        # No ordinary admission/journal lock before local stop or ROS safety enqueue.
        with self.output_lock:
            self.core.state.emergency_active = True
            self.core.state.emergency_source = 'control_v2'
            self.emergency_pending.set()
            self.core.publish_zero('emergency_stop')
            self._queue_halt()
            self._save_latch(True)

    def _submit(self, connection, m):
        with self.lock:
            self._session(connection, m)
            rid = m.get('request_id')
            if not isinstance(rid, str) or not 1 <= len(rid) <= 128:
                raise Rejected('REQUEST_ID')
            payload = {k: m.get(k) for k in ('type','command','params')}
            payload['robot_id'] = self.core.robot_id
            if not isinstance(m.get('params', {}), dict):
                raise Rejected('PARAMS')
            # Arbitrary named services and implicit input-edge operations are excluded.
            if m['type'] == 'system_command' and m.get('command') not in self.core.LIFECYCLE_COMMANDS | {'set_control_mode','request_arm_named_targets','move_arm_named_target'}:
                raise Rejected('COMMAND')
            digest = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()
            if rid in self.requests:
                old_digest, oid = self.requests[rid]
                if old_digest != digest:
                    raise Rejected('REQUEST_CONFLICT')
                event = self.durable.get(oid)
                new = False
            else:
                s, deadline = self._fresh(connection, m)
                if not s['operations_authorized']:
                    raise Rejected('OPERATIONS_NOT_AUTHORIZED')
                if m.get('command') == 'move_arm_named_target' and (not s['authorized'] or not self.permission()):
                    raise Rejected('NOT_AUTHORIZED')
                if self.reserved_waiting >= 16:
                    raise Rejected('QUEUE_FULL')
                # A bounded journal rejects new work when 24h retention cannot be honored.
                now = time.time()
                # Never evict idempotency under wall-clock jumps; offline archive is explicit.
                if len(self.operations) >= self.MAX_RECORDS:
                    raise Rejected('CACHE_FULL')
                oid = str(uuid.uuid4())
                op = dict(operation_id=oid, request_id=rid, robot_id=self.core.robot_id, command=m.get('command'), state='accepted', error=None, created_at=now,
                          epoch=self.epoch, authorization_generation=s.get('authorization_generation',0), invalidated=False, result=None)
                self.operations[oid] = op
                self.requests[rid] = [digest, oid]
                self.reserved_waiting += 1
                event = threading.Event()
                self.durable[oid] = event
                new = True
        if new:
            try:
                self._save()  # Durable intent before enqueue and acceptance response.
                with self.lock:
                    self.jobs.put_nowait((oid, connection, copy.deepcopy(m), deadline))
            except Exception:
                with self.lock:
                    self.reserved_waiting -= 1
                    self.operations[oid].update(state='result_unknown',error='PERSISTENCE')
                raise Rejected('PERSISTENCE')
            finally:
                event.set()
        elif event:
            event.wait()  # Only this receiver waits; watchdog and other connections run.
        with self.lock:
            if self.operations[oid].get('error') == 'PERSISTENCE':
                raise Rejected('PERSISTENCE')
            return copy.deepcopy(self.operations[oid])

    def _validate_job(self, connection, m, deadline):
        s = self._session(connection, m)
        if self.clock() >= deadline:
            raise Rejected('EXPIRED_IN_QUEUE')
        if not s['operations_authorized']:
            raise Rejected('OPERATIONS_NOT_AUTHORIZED')
        if m.get('command') == 'move_arm_named_target' and (not s['authorized'] or not self.permission()):
            raise Rejected('NOT_AUTHORIZED')
        if self.core.state.emergency_active and m.get('command') not in ('halt','disable','shutdown'):
            raise Rejected('EMERGENCY_ACTIVE')

    def _persist_operation(self, oid):
        try:
            self._save()
            return True
        except Exception:
            with self.lock:
                self.operations[oid].update(state='result_unknown',error='PERSISTENCE')
                self.counts['PERSISTENCE'] += 1
            return False

    def _work(self):
        while not self.closed.is_set():
            try:
                oid, connection, m, deadline = self.jobs.get(timeout=.1)
            except queue.Empty:
                continue
            with self.lock:
                self.reserved_waiting -= 1
                op = self.operations[oid]
                try:
                    self._validate_job(connection,m,deadline)
                    op['state'] = 'running'
                except Rejected as exc:
                    op.update(state='failed',error=exc.code)
            if not self._persist_operation(oid):
                self.jobs.task_done()
                continue
            with self.lock:
                # Persistence may itself be slow: validate again at the execution boundary.
                if op['state'] == 'running':
                    try:self._validate_job(connection,m,deadline)
                    except Rejected as exc:op.update(state='failed',error=exc.code)
                rejected = op['state'] != 'running'
            if rejected:
                self._persist_operation(oid)
                self.jobs.task_done()
                continue
            # No admission/watchdog lock across a ROS, action or video IPC call.
            try:
                self.core.output.operation_context.operation_id = oid
                request = dict(m, protocol_version=1)
                result = list(self.core.handle_message(request))
                failed = any(r.get('ok') is False or r.get('success') is False for r in result)
                uncertain = any('timeout' in str(r).lower() or 'uncertain' in str(r).lower() for r in result)
                if len(json.dumps(result).encode()) > 60000:
                    result = [dict(error='RESULT_TOO_LARGE')]
                    uncertain = True
                state = 'result_unknown' if uncertain else 'failed' if failed else 'succeeded'
                error = 'BACKEND_TIMEOUT' if uncertain else 'BACKEND_REJECTED' if failed else None
            except Exception as exc:
                state, error, result = 'result_unknown', type(exc).__name__, None
            with self.lock:
                s = self.sessions.get(connection)
                invalidated = (op['epoch'] != self.epoch or not s or self.owner != connection or self.clock() >= s['lease_until'] or not s['operations_authorized'] or s.get('authorization_generation',0) != op['authorization_generation'])
                op.update(state=state, error=error, result=result, invalidated=invalidated)
            self._persist_operation(oid)
            self.jobs.task_done()

    def _late_result(self, oid, response, error):
        if self.closed.is_set():
            return
        with self.lock:
            op = self.operations.get(oid)
            if op is None:
                return
            op.setdefault('late_backend_results', []).append(dict(
                success=getattr(response, 'success', None),
                message=str(getattr(response, 'message', ''))[:1024], error=error))
            # An observed RPC completion cannot prove the complete physical operation.
            # Preserve result_unknown until runtime/device reconciliation by the operator.
        self._persist_operation(oid)

    def _queue_halt(self):
        try:
            self.safety.put_nowait(True)
        except queue.Full:
            pass

    def _safety_work(self):
        while not self.closed.is_set():
            try:
                self.safety.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                self.safety_status = dict(state='running',physical_stop_confirmed=False)
                ok,code,message = self.core.output.call_lifecycle('halt')
                self.safety_status = dict(state='succeeded' if ok else 'result_unknown',ok=ok,code=code,message=message,physical_stop_confirmed=False)
                if not ok:self.counts['safety_backend_error'] += 1
            except Exception as exc:
                self.safety_status = dict(state='result_unknown',error=type(exc).__name__,physical_stop_confirmed=False)
                self.counts['safety_backend_error'] += 1
            self.safety.task_done()

    def tick(self):
        with self.lock:
            now = self.clock()
            if self.emergency_pending.is_set():
                self._revoke('emergency_stop')
                self.emergency_pending.clear()
            if self.owner is not None:
                s = self.sessions[self.owner]
                if now >= s['lease_until']:
                    self._revoke('lease_expired')
                    self.owner = None
                elif s['authorized'] and (not self.permission() or now - s.get('last_input', now) >= min(.500,self.core.watchdog_ms/1000)):
                    self._revoke('watchdog_or_permission')
            latest, self.latest = self.latest, None
            if latest:
                connection, m, deadline = latest
                s = self.sessions.get(connection)
                if now >= deadline:
                    self.counts['EXPIRED_BEFORE_OUTPUT'] += 1
                elif self.owner == connection and s and s['authorized'] and self.permission():
                    # Continuous mapping only: no mode/service/profile edge handlers.
                    pressed = self.core.normalized_pressed_keys(m)
                    twist, servo, gripper, flipper = self.core.operator_input_to_outputs(pressed, m.get('gamepad',{}), int(now*1000))
                    with self.output_lock:
                        if self.core.state.emergency_active:
                            self.core.publish_zero('emergency_stop')
                            return
                        self.core.publish_twist(twist)
                        self.core.publish_servo(servo)
                        if gripper:
                            self.core.publish_gripper(gripper)
                        self.core.publish_flipper(flipper)
                        self.core.state.last_operator_seq = m['seq']
                        self.core.state.last_valid_input_ms = int(now*1000)
                        s['last_input'] = now
                        self.counts['outputs'] += 1
            # Repeated zero publication; ROS guard and hardware remain independent.
            if self.owner is None or not self.sessions[self.owner]['authorized'] or self.core.state.emergency_active:
                self.core.publish_zero('control_v2_inhibited')
