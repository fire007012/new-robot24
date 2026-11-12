#!/usr/bin/env python3
"""Independent, bounded TCP management daemon. No ROS imports in this process."""
import argparse
import copy
import fcntl
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
import uuid
from config_transactions import ConfigTransactions, METHODS as CONFIG_METHODS

VERSION = '2.0.0'
METHODS = ['profiles', 'preflight', 'start', 'stop', 'operation', 'snapshot', 'logs'] + CONFIG_METHODS

class Fault(Exception):
    def __init__(self, code, message, retryable=False, hint='检查配置及操作日志'):
        self.data = dict(code=code, message=message, retryable=retryable, hint=hint)
        super().__init__(message)

def identity(pid, include_zombie=False):
    try:
        raw = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if raw[0] == 'Z' and not include_zombie:
            return None
        return [Path('/proc/sys/kernel/random/boot_id').read_text().strip(), raw[19]]
    except (OSError, IndexError):
        return None

def wire(obj):
    return (json.dumps(obj, ensure_ascii=False, allow_nan=False) + '\n').encode()

class Manager:
    def __init__(self, config, backend=None):
        self.config = json.loads(Path(config).read_text())
        self.config['control_endpoint']['protocol_version'] = 2
        self.config['control_endpoint']['legacy_readonly_version'] = 1
        self.root = Path(self.config['config_root']).resolve(strict=True)
        self.state_dir = Path(self.config['state_dir'])
        self.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock_file = (self.state_dir / 'lock').open('w')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.token = Path(self.config['token_file']).read_text().strip()
        if len(self.token) < 32:
            raise ValueError('token must have at least 32 characters')
        self.lock = threading.RLock()
        self.backend = backend or Backend(self.config, self.root, self.state_dir)
        self.path = self.state_dir / 'state.json'
        self.data = json.loads(self.path.read_text()) if self.path.exists() else dict(operations={}, requests={}, resources=[], software_state='stopped', profile=None, health={})
        self.last_health_monotonic = None
        self.recovery = bool(self.data['resources'])
        for op in self.data['operations'].values():
            if op['state'] in ('accepted', 'running'):
                op.update(state='failed', phase='reconcile', error=Fault('RECOVERY_REQUIRED', 'service interrupted; result unknown').data, retryable=False, updated_at=time.time())
                self.recovery = True
        if self.recovery:
            self.data['software_state'] = 'failed'
            self.data['health'] = {'modules': {r['module']: {'ok': False, 'identity_matches': bool(r.get('pid') and identity(r['pid']) == r.get('identity'))} for r in self.data['resources']}, 'runtime': None}
        self.save()
        self.config_transactions = ConfigTransactions(self, Fault) if self.config.get('enable_config_transactions', False) else None
        self.closed = threading.Event()
        threading.Thread(target=self.monitor, daemon=True).start()

    def save(self):
        tmp = self.path.with_suffix('.tmp')
        with tmp.open('w') as out:
            json.dump(self.data, out, ensure_ascii=False)
            out.flush()
            os.fsync(out.fileno())
        tmp.replace(self.path)
        fd = os.open(self.state_dir, os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def profile(self, p):
        if p.get('robot_id') != self.config['robot_id']:
            raise Fault('PROFILE', 'robot_id mismatch')
        name = p.get('profile_id', '')
        if not isinstance(name, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', name):
            raise Fault('PROFILE', 'invalid profile_id')
        if self.config_transactions:
            self.config_transactions._read(self.config_transactions.current())
        source_root = self.config_transactions.profiles_root() if self.config_transactions else self.root
        path = (source_root / (name + '.json')).resolve()
        if path.parent != source_root or not path.is_file():
            raise Fault('PROFILE', 'unknown profile')
        try:
            profile = json.loads(path.read_text())
            if not isinstance(profile, dict) or not all(isinstance(profile.get(k), str) for k in ('profile_id','config_revision','mode','build_version')):
                raise ValueError('profile schema')
        except (OSError, ValueError):
            raise Fault('CONFIG', 'invalid deployed profile JSON/schema', False, '修正部署档案后使用新请求 ID')
        if profile['profile_id'] != name or p.get('config_revision') != profile['config_revision']:
            raise Fault('REVISION', 'config_revision mismatch')
        self.backend.validate(profile)
        return profile

    def log(self, oid, line):
        path = self.state_dir / (oid + '.log')
        if not path.exists() or path.stat().st_size < 1048576:
            with path.open('a') as out:
                out.write(line[:4096] + '\n')

    def active(self):
        return next((o for o in reversed(list(self.data['operations'].values())) if o['state'] in ('accepted', 'running')), None)

    def dispatch(self, req):
        if not isinstance(req, dict):
            raise Fault('BAD_REQUEST', 'object required')
        if not isinstance(req.get('token'), str) or not hmac.compare_digest(req['token'], self.token):
            raise Fault('AUTH', 'authentication failed')
        if req.get('version') != 1:
            raise Fault('VERSION', 'version 1 required')
        rid, method, p = req.get('request_id'), req.get('method'), req.get('params', {})
        if not isinstance(rid, str) or not 1 <= len(rid) <= 128 or not isinstance(p, dict):
            raise Fault('BAD_REQUEST', 'invalid request_id or params')
        if method in CONFIG_METHODS:
            if self.config_transactions is None:
                raise Fault('CAPABILITY_DISABLED', 'configuration maintenance is not enabled in deployment')
            return self.config_transactions.dispatch(method, rid, p)
        with self.lock:
            if method in ('start', 'stop', 'preflight') and self.config_transactions and self.config_transactions.busy:
                raise Fault('BUSY', 'configuration transaction running', True)
            if method == 'hello':
                return dict(protocol_version=1, service_version=VERSION, robot_id=self.config['robot_id'], auth='shared_token', capabilities=(METHODS if self.config_transactions else [m for m in METHODS if m not in CONFIG_METHODS]))
            if method == 'profiles':
                return [json.loads(x.read_text()) for x in sorted((self.config_transactions.profiles_root() if self.config_transactions else self.root).glob('*.json'))]
            if method == 'snapshot':
                profile = self.data['profile'] or {}
                return copy.deepcopy(dict(software_state=self.data['software_state'], profile_id=profile.get('profile_id'), config_revision=profile.get('config_revision'), build_version=self.config['build_version'], control_endpoint=self.config['control_endpoint'], runtime=self.data['health'].get('runtime'), safety=self.data['health'].get('safety'), modules=self.data['health'].get('modules', {}), observed_at=self.data['health'].get('observed_at'), operation_id=self.data.get('current'), recovery_required=self.recovery))
            if method in ('operation', 'logs'):
                oid = p.get('operation_id', self.data.get('current'))
                if not isinstance(oid, str) or oid not in self.data['operations']:
                    raise Fault('NOT_FOUND', 'unknown operation')
                if method == 'operation':
                    return copy.deepcopy(self.data['operations'][oid])
                cursor, limit = p.get('cursor', 0), p.get('limit', 100)
                if type(cursor) is not int or type(limit) is not int or cursor < 0 or not 1 <= limit <= 200:
                    raise Fault('BAD_REQUEST', 'invalid cursor/limit')
                path = self.state_dir / (oid + '.log')
                lines = path.read_text().splitlines() if path.exists() else []
                if cursor > len(lines):
                    raise Fault('CURSOR', 'cursor beyond end')
                selected, budget = [], 60000
                for line in lines[cursor:cursor+limit]:
                    cost = len(wire(line))
                    if cost > budget: break
                    selected.append(line)
                    budget -= cost
                return dict(lines=selected, next_cursor=cursor+len(selected), expired=path.exists() and path.stat().st_size >= 1048576)
            if method == 'preflight':
                profile = self.profile(p)
            elif method in ('start', 'stop'):
                fingerprint = hashlib.sha256(json.dumps([method,p],sort_keys=True,separators=(',',':')).encode()).hexdigest()
                old = self.data['requests'].get(rid)
                if old:
                    if old[0] != fingerprint:
                        raise Fault('REQUEST_CONFLICT', 'request_id reused with different content')
                    return copy.deepcopy(self.data['operations'][old[1]])
                profile = self.profile(p) if method == 'start' else self.data['profile']
                active = self.active()
                if active:
                    if method == 'start' and active['method'] == 'start' and profile == self.data['profile']:
                        self.data['requests'][rid] = [fingerprint, active['operation_id']]
                        self.save()
                        return copy.deepcopy(active)
                    raise Fault('BUSY', 'operation running', True, '查询当前操作后再试')
                if method == 'start' and self.recovery:
                    raise Fault('RECOVERY_REQUIRED', 'stop required before restart')
                if method == 'start' and self.data['software_state'] in ('ready', 'degraded'):
                    if profile != self.data['profile']:
                        raise Fault('BUSY', 'stop current profile first')
                    oid = self.data['current']
                    self.data['requests'][rid] = [fingerprint, oid]
                    self.save()
                    return copy.deepcopy(self.data['operations'][oid])
                if method == 'start' and self.data['resources']:
                    raise Fault('RECOVERY_REQUIRED', 'owned resources require stop')
                oid = str(uuid.uuid4())
                op = dict(operation_id=oid, method=method, state='accepted', phase='preflight' if method == 'start' else 'stop', robot_id=self.config['robot_id'], profile_id=(profile or {}).get('profile_id'), config_revision=(profile or {}).get('config_revision'), modules={}, error=None, retryable=False, created_at=time.time(), updated_at=time.time())
                self.data['requests'][rid] = [fingerprint, oid]
                self.data['operations'][oid] = op
                self.data['current'] = oid
                if method == 'start':
                    self.data['profile'] = profile
                self.data['software_state'] = 'starting' if method == 'start' else 'stopping'
                self.save()
                threading.Thread(target=self.work, args=(oid, method, profile), daemon=True).start()
                return copy.deepcopy(op)
            else:
                raise Fault('BAD_REQUEST', 'unknown method')
        return self.backend.preflight(profile)

    def update(self, oid, **values):
        with self.lock:
            self.data['operations'][oid].update(updated_at=time.time(), **values)
            self.save()

    def work(self, oid, method, profile):
        created = []
        try:
            self.update(oid, state='running')
            if method == 'stop':
                self.stop_resources(profile)
                with self.lock:
                    self.data.update(software_state='stopped', health={}, profile=None)
                    self.recovery = False
            else:
                checks = self.backend.preflight(profile)
                self.log(oid, json.dumps(checks, ensure_ascii=False))
                if not checks['passed']:
                    first = next((x for x in checks['checks'] if not x['ok']), {})
                    error = Fault(first.get('code') or 'CONFIG', 'preflight failed: '+first.get('name','configuration'), False, first.get('hint') or '检查档案及部署环境')
                    error.data['checks'] = checks['checks']
                    raise error
                self.update(oid, phase='launch')
                for module in self.backend.modules(profile):
                    # Persist spawn intent before process creation. A crash in this small
                    # window is an unknown result and blocks future starts.
                    with self.lock:
                        intent = dict(module=module, pending=True)
                        self.data['resources'].append(intent)
                        self.save()
                        resource = self.backend.start(module, profile, oid)
                        intent.update(resource, pending=False)
                        created.append(intent)
                        self.save()
                    self.log(oid, 'launch ' + module)
                self.update(oid, phase='health')
                deadline = time.monotonic() + profile.get('startup_timeout', 90)
                while True:
                    health = self.probe(profile)
                    with self.lock:
                        self.data['health'] = health
                    self.update(oid, modules=health['modules'])
                    if all(v['ok'] for k,v in health['modules'].items() if k in self.backend.required(profile)):
                        with self.lock:
                            self.data['software_state'] = 'ready' if all(v['ok'] for v in health['modules'].values()) else 'degraded'
                        break
                    if time.monotonic() >= deadline:
                        raise Fault('HEALTH', 'required module readiness timed out', True)
                    time.sleep(0.2)
            self.update(oid, state='succeeded', phase='complete')
        except Exception as exc:
            error = exc.data if isinstance(exc, Fault) else Fault('INTERNAL', type(exc).__name__).data
            if method == 'start':
                # This launch never grants motion. Nonetheless request halt before cleanup.
                try:
                    self.stop_resources(profile, created)
                except Exception:
                    error = Fault('STOP_UNCONFIRMED', 'cleanup uncertain; explicit recovery required').data
                    self.recovery = True
            with self.lock:
                self.data['software_state'] = 'failed'
            self.log(oid, error['code'] + ': ' + error['message'])
            self.update(oid, state='failed', error=error, retryable=error['retryable'])

    def stop_resources(self, profile, subset=None):
        resources = list(self.data['resources'] if subset is None else subset)
        if resources:
            try:
                self.backend.halt(profile)
            except Exception as exc:
                error = Fault('STOP_UNCONFIRMED', 'halt not confirmed; resources retained')
                if isinstance(exc, Fault) and 'diagnostic' in exc.data:
                    error.data['diagnostic'] = exc.data['diagnostic']
                raise error
        for resource in reversed(resources):
            self.backend.stop(resource)
            with self.lock:
                self.data['resources'].remove(resource)
                self.save()

    def probe(self, profile):
        health = self.backend.health(profile)
        self.last_health_monotonic = time.monotonic()
        if hasattr(self.backend, 'alive'):
            with self.lock:
                resources = copy.deepcopy(self.data['resources'])
            for resource in resources:
                if not self.backend.alive(resource):
                    names = ['bridge', 'video'] if resource['module'] == 'video_stack' else [resource['module']]
                    for name in names:
                        health['modules'][name] = dict(ok=False, reason='owned_process_missing')
        return health

    def monitor(self):
        while not self.closed.wait(2):
            with self.lock:
                if self.active() or self.data['software_state'] not in ('ready', 'degraded'):
                    continue
                profile = copy.deepcopy(self.data['profile'])
            try:
                health = self.probe(profile)
            except Exception:
                health = dict(modules={k:dict(ok=False) for k in self.backend.required(profile)}, runtime=None, observed_at=time.time())
            with self.lock:
                if self.active() or self.data['profile'] != profile:
                    continue
                self.data['health'] = health
                self.data['software_state'] = ('failed' if any(not health['modules'].get(k, {}).get('ok') for k in self.backend.required(profile)) else 'ready' if all(v['ok'] for v in health['modules'].values()) else 'degraded')
                self.save()

class Backend:
    def __init__(self, config, root, state):
        self.config, self.root, self.state = config, root, state
        self.children = {}

    def validate(self, p):
        if p.get('mode') not in ('mock', 'hardware') or type(p.get('video')) is not bool:
            raise Fault('CONFIG', 'invalid mode/video')
        if type(p.get('enable_servo', False)) is not bool:
            raise Fault('CONFIG', 'enable_servo must be boolean')
        if type(p.get('startup_timeout',90)) not in (int,float) or not 1 <= p.get('startup_timeout', 90) <= 180:
            raise Fault('CONFIG', 'invalid startup_timeout')
        for key in ('baseline', 'video_config'):
            if p.get(key):
                path = Path(p[key]).resolve()
                if not path.is_relative_to(self.root) or not path.is_file():
                    raise Fault('CONFIG', key + ' must be a deployed file inside config_root')
        if p['mode'] == 'hardware' and (not p.get('baseline') or not p.get('devices')):
            raise Fault('CONFIG', 'hardware requires baseline and devices; no mock fallback')
        expected_required = self.required(p)
        expected_optional = ['video'] if p['video'] and not p.get('video_required') else []
        if p.get('required_modules', expected_required) != expected_required or p.get('optional_modules', expected_optional) != expected_optional:
            raise Fault('CONFIG', 'module declarations disagree with mode/video dependencies')
        if p['video'] and not p.get('video_config'):
            raise Fault('CONFIG', 'video_config required')

    @staticmethod
    def digest(path):
        value = hashlib.sha256()
        deadline = time.monotonic() + 30
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                if time.monotonic() > deadline:
                    raise Fault('TIMEOUT', 'release artifact verification timed out', True)
                value.update(chunk)
        return value.hexdigest()

    def required(self, p):
        return ['core', 'bridge'] + (['video'] if p.get('video_required') else [])

    def modules(self, p):
        # Video stack includes host_bridge: never also launch standalone bridge.
        return ['core', 'video_stack' if p['video'] else 'bridge']

    def preflight(self, p):
        self.validate(p)
        checks = []
        def add(name, ok, code, hint):
            checks.append(dict(name=name, ok=bool(ok), code=None if ok else code, hint='' if ok else hint))
        add('build_version', p.get('build_version') == self.config['build_version'], 'DEPENDENCY', '部署匹配的软件及档案版本')
        add('setup', Path(self.config['setup']).is_file(), 'DEPENDENCY', '安装预构建 ROS2 产物')
        try:
            manifest = json.loads(Path(self.config['build_manifest']).read_text())
            version_ok = (manifest['build_version'] == self.config['build_version']
                          and manifest['management_protocol'] == 1 and manifest['control_protocol'] == 2
                          and bool(manifest['files']))
            base = Path(self.config['build_manifest']).parent.resolve()
            for rel, digest in manifest['files'].items():
                artifact = (base / rel).resolve()
                version_ok = version_ok and artifact.is_relative_to(base) and artifact.is_file()
                if version_ok:
                    version_ok = self.digest(artifact) == digest
        except (OSError, ValueError, KeyError, TypeError):
            version_ok = False
        add('release_manifest', version_ok, 'DEPENDENCY', '部署匹配版本及文件 SHA256 清单')
        for dev in p.get('devices', []):
            path = Path('/sys/class/net') / dev if not dev.startswith('/') else Path(dev)
            exists = path.exists()
            if not dev.startswith('/') and exists:
                exists = ((path/'type').read_text().strip() == '280' and
                          bool(int((path/'flags').read_text().strip(), 16) & 1))  # CAN + IFF_UP
            elif dev.startswith('/') and exists:
                exists = os.access(path, os.R_OK | os.W_OK)
            add('device:' + dev, exists, 'DEVICE', '核对 CAN 接口类型、设备、权限及部署档案')
        for port in [self.config['control_endpoint']['port'],18080] + ([18081] if p['video'] else []):
            try:
                with socket.socket() as s:
                    s.bind(('0.0.0.0', port))
                ok = True
            except OSError:
                ok = False
            add('port:' + str(port), ok, 'PORT', '停止占用端口的非受管程序')
        try:
            self.run(['ros2', 'pkg', 'prefix', 'robot_bringup'], 5)
            self.run(['ros2', 'pkg', 'prefix', 'host_bridge'], 5)
            self.run([sys.executable, str(Path(__file__).with_name('ros_health.py')), 'preflight', json.dumps(p)], 5)
            ok = True
        except Exception:
            ok = False
        add('ros_packages', ok, 'DEPENDENCY', '核对 setup 及安装包，启动按钮不会构建或下载')
        return dict(passed=all(x['ok'] for x in checks), checks=checks)

    def command(self, args):
        return ['/bin/bash', '-c', 'source "$1" && shift && exec "$@"', 'robot-manager', self.config['setup'], *args]

    def run(self, args, timeout):
        # Helpers get their own process group so a timeout also kills descendants.
        proc = subprocess.Popen(self.command(args), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            out = proc.communicate(timeout=timeout)[0]
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate()
            raise Fault('TIMEOUT', 'helper timed out', True)
        if proc.returncode:
            error = Fault('PROCESS', 'helper failed', True)
            if args[-1] == 'halt':
                # The halt probe only emits bounded runtime diagnostics, never
                # credentials or video config contents.
                error.data['diagnostic'] = out.decode(errors='replace')[-2000:]
            raise error
        return out.decode(errors='replace')

    def start(self, module, p, oid):
        port = str(self.config['control_endpoint']['port'])
        if module == 'core':
            args = ['ros2','launch','robot_bringup','full_system.launch.py','mock:='+str(p['mode']=='mock').lower(),'enable_host:=false','enable_moveit:=true','enable_servo:='+str(p.get('enable_servo',False)).lower(),'journal:='+str(self.state/'maintenance.json')]
            if p.get('baseline'):
                args += ['baseline:='+p['baseline']]
        elif module == 'video_stack':
            args = ['ros2','launch','host_bridge','video_stack.launch.py','video_config:='+p['video_config'],'host:='+self.config.get('bridge_bind','127.0.0.1'),'robot_id:='+self.config['robot_id'],'port:='+port,'control_state_dir:='+str(self.state/'control'),'log_dir:='+str(self.state/'video')]
        else:
            args = ['ros2','launch','host_bridge','host_bridge.launch.py','host:='+self.config.get('bridge_bind','127.0.0.1'),'robot_id:='+self.config['robot_id'],'port:='+port,'control_state_dir:='+str(self.state/'control')]
        with (self.state/(oid+'-'+module+'.process.log')).open('ab') as log:
            proc = subprocess.Popen(self.command(args), stdout=log, stderr=log, start_new_session=True, cwd=self.state)
        self.children[proc.pid] = proc
        return dict(pid=proc.pid, identity=identity(proc.pid), module=module)

    def alive(self, resource):
        child = self.children.get(resource.get('pid'))
        if child is not None:
            child.poll()  # Reap an exited launch process.
        return not resource.get('pending') and bool(resource.get('identity')) and identity(resource['pid']) == resource['identity']

    def health(self, p):
        try:
            output = self.run([sys.executable, str(Path(__file__).with_name('ros_health.py')), 'health', str(self.config['control_endpoint']['port']), str(p['video']).lower()], 12)
            return json.loads(output.strip().splitlines()[-1])
        except Exception:
            return dict(modules={k:dict(ok=False) for k in self.required(p)}, runtime=None, observed_at=time.time())

    def halt(self, p):
        # If no runtime exists, cleanup is safe only for never-enabled startup;
        # the helper must otherwise confirm halt via the runtime service/state.
        self.run([sys.executable, str(Path(__file__).with_name('ros_health.py')), 'halt'], 30)

    def stop(self, r):
        if r.get('pending') or not r.get('identity'):
            raise Fault('RECOVERY_REQUIRED', 'spawn identity unknown; inspect service cgroup')
        pid = r['pid']
        child = self.children.get(pid)
        if child is not None:
            child.poll()
        if identity(pid) != r['identity']:
            # A gone group leader can leave descendants; do not signal a reused PID.
            if Path(f'/proc/{pid}').exists() and identity(pid, include_zombie=True) != r['identity']:
                raise Fault('RECOVERY_REQUIRED', 'process identity changed')
            # Only signal a group while its verified leader remains alive.
            # Unknown orphan groups require operator reconciliation.
            members = []
            for stat in Path('/proc').glob('[0-9]*/stat'):
                try:
                    fields = stat.read_text().rsplit(')',1)[1].split()
                    if int(fields[2]) == pid and fields[0] != 'Z': members.append(stat)
                except (OSError, ValueError, IndexError): pass
            if members:
                raise Fault('RECOVERY_REQUIRED', 'orphan group requires reconciliation')
            return
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic() + 8
        while identity(pid) == r['identity'] and time.monotonic() < deadline:
            time.sleep(.1)
        if identity(pid) == r['identity']:
            os.killpg(pid, signal.SIGKILL)
        child = self.children.pop(pid, None)
        if child:
            child.wait(timeout=3)
        # Verify descendants were removed even if launch exited first.
        for stat in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = stat.read_text().rsplit(')',1)[1].split()
                if int(fields[2]) == pid and fields[0] != 'Z':
                    raise Fault('RECOVERY_REQUIRED', 'descendant remains after stop')
            except (OSError, ValueError, IndexError): pass

class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(10)
        while True:
            req = None
            try:
                line = self.rfile.readline(65537)
                if not line or len(line) > 65536: return
                req = json.loads(line)
                result = self.server.manager.dispatch(req)
                response = dict(ok=True, result=result)
            except Fault as exc:
                response = dict(ok=False, error=exc.data)
            except (ValueError, TypeError, KeyError):
                response = dict(ok=False, error=Fault('BAD_REQUEST', 'invalid frame').data)
            except (OSError, TimeoutError):
                return
            except Exception:
                response = dict(ok=False, error=Fault('INTERNAL', 'request failed').data)
            response.update(version=1, request_id=req.get('request_id') if isinstance(req, dict) else None)
            frame = wire(response)
            if len(frame) > 65536:
                frame = wire(dict(version=1,request_id=response['request_id'],ok=False,error=Fault('RESPONSE_TOO_LARGE','response exceeds frame limit; inspect local configuration').data))
            self.wfile.write(frame)

class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    request_queue_size = 16
    def __init__(self, manager):
        self.manager = manager
        self.slots = threading.BoundedSemaphore(16)
        super().__init__((manager.config.get('bind', '127.0.0.1'), manager.config.get('port',9091)), Handler)
    def process_request(self, request, address):
        if not self.slots.acquire(False):
            request.close()
            return
        try:
            super().process_request(request,address)
        except Exception:
            self.slots.release()
            raise
    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request,address)
        finally:
            self.slots.release()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    manager = Manager(args.config)
    with Server(manager) as server:
        server.serve_forever()

if __name__ == '__main__': main()
