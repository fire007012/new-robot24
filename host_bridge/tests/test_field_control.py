"""Field regressions: independent emergency gate and explicit speed control."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent)]
from test_control_v2 import Capture, Clock
from bridge_core import BridgeCore
from control_v2 import ControlV2, SUPPORTED_CAPABILITIES
from host_bridge_node import load_bridge_control_config


class Tests(unittest.TestCase):
    def test_reset_readiness_waits_for_ordinary_halt_completion(self):
        self.authorize()
        self.output.runtime_state['diagnostics']['global_inhibit'] = True
        for group in self.output.runtime_state['diagnostics']['groups'].values():
            group['state'] = 3
        entered, release = threading.Event(), threading.Event()
        def halt(command):
            entered.set(); release.wait(2)
            return True, 0, 'cleanup completed'
        self.output.call_lifecycle = halt
        self.c._queue_halt()
        try:
            self.assertTrue(entered.wait(1))
            self.assertFalse(self.call('control_status')['result']['global_stop_reset_ready'])
            rejected = self.call('system_command',command='reset_global_stop',params={},request_id='cleanup-busy')
            self.assertEqual(rejected['error']['code'],'RUNTIME_NOT_SETTLED')
        finally:
            release.set()
        self.wait(lambda:self.c.safety_status['state']=='succeeded')
        self.assertTrue(self.call('control_status')['result']['global_stop_reset_ready'])

    def setUp(self):
        self.clock = Clock()
        self.output = Capture(self.clock)
        self.core = BridgeCore(self.output, 500, .8, 1.5)
        self.c = ControlV2(self.core, self.clock)
        self.seq = 0
        self.session = self.c.handle('owner', dict(type='control_hello', protocol_version=2,
            robot_id=self.core.robot_id, role='control', capabilities=SUPPORTED_CAPABILITIES))['result']

    def tearDown(self):
        self.c.close()

    def call(self, kind, **fields):
        self.seq += 1
        return self.c.handle('owner', dict(type=kind, protocol_version=2,
            seq=self.seq, session_id=self.session['session_id'], epoch=self.session['epoch'],
            freshness_token=self.session['freshness_token'], ttl_ms=300, **fields))

    def wait(self, predicate):
        deadline = time.monotonic()+2
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(.005)

    def authorize(self):
        self.assertTrue(self.call('operator_input', keyboard={'pressed_keys': []})['ok'])
        self.assertTrue(self.call('authorize_operations')['ok'])
        self.assertTrue(self.call('authorize')['ok'])

    def test_emergency_gate_bypasses_busy_halt_and_admission_lock(self):
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        def busy_halt(command):
            entered.set(); release.wait(3)
            return False, 2301, 'arm: another operation is in progress'
        self.output.call_lifecycle = busy_halt
        self.output.call_global_stop = lambda: (stopped.set() or True, 0, 'C++ gate closed')
        self.c._queue_halt(); self.assertTrue(entered.wait(1))
        acquired = threading.Event()
        def lock_admission():
            with self.c.lock: acquired.set(); release.wait(3)
        thread=threading.Thread(target=lock_admission); thread.start(); self.assertTrue(acquired.wait(1))
        try:
            self.c.emergency()
            self.assertTrue(stopped.wait(1), 'global stop waited for ordinary halt/admission')
            self.assertEqual(self.core.state.last_twist.linear_x, 0.)
        finally:
            release.set(); thread.join(2)
        self.wait(lambda:self.c.global_stop_status['state']=='succeeded')
        self.assertFalse(self.c.global_stop_status['physical_stop_confirmed'])

    def test_failed_global_stop_retries_and_cannot_be_cleared(self):
        self.authorize()
        release=threading.Event(); calls=[]
        def global_stop():
            calls.append(True)
            return release.is_set(), 0, 'guard outcome'
        self.output.call_global_stop=global_stop
        self.c.emergency(); self.c.tick()
        self.wait(lambda:len(calls)>=1)
        self.session.update(self.call('freshness_request')['result'])
        self.call('operator_input',keyboard={'pressed_keys':[]})
        self.assertEqual(self.call('clear_emergency')['error']['code'],'GLOBAL_STOP_UNCONFIRMED')
        release.set();self.wait(lambda:self.c.global_stop_status['state']=='succeeded')
        self.assertTrue(self.call('clear_emergency')['ok'])
        self.assertFalse(self.c.sessions['owner']['authorized'])

    def test_persisted_emergency_reasserts_cpp_gate_after_bridge_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'emergency.json').write_text(json.dumps({'active':True}))
            output=Capture(self.clock);stopped=threading.Event()
            output.call_global_stop=lambda:(stopped.set() or True,0,'latched')
            control=ControlV2(BridgeCore(output,500,.8,1.5),self.clock,folder)
            try:
                self.assertTrue(stopped.wait(1))
                self.assertTrue(control.core.state.emergency_active)
            finally:control.close()

    def test_speed_change_zeros_input_and_requires_new_epoch_and_authorization(self):
        self.authorize()
        self.assertTrue(self.call('operator_input',keyboard={'pressed_keys':['w']})['ok']);self.c.tick()
        self.assertGreater(self.output.outputs[-1][1],0)
        old_epoch=self.session['epoch']
        self.call('operator_input',keyboard={'pressed_keys':[]});self.c.tick()
        request=self.call('system_command',command='set_speed_level',params={'level':1},request_id='speed-one')
        self.assertTrue(request['ok'],request)
        oid=request['result']['operation_id'];self.wait(lambda:self.c.operations[oid]['state'] not in ('accepted','running'))
        self.assertEqual(self.c.operations[oid]['state'],'succeeded')
        self.assertEqual(self.core.state.speed_level,1)
        self.assertEqual(self.output.outputs[-1][1],0)
        self.assertNotEqual(old_epoch,self.c.epoch)
        self.assertFalse(self.c.sessions['owner']['authorized'])
        self.assertFalse(self.c.sessions['owner']['operations_authorized'])
        self.assertEqual(self.call('operator_input',keyboard={'pressed_keys':['w']})['error']['code'],'EPOCH')
        self.session['epoch']=self.call('control_status')['result']['epoch']
        self.session.update(self.call('freshness_request')['result']);self.authorize()
        self.call('operator_input',keyboard={'pressed_keys':['w']});self.c.tick()
        self.assertAlmostEqual(self.output.outputs[-1][1],self.core.base_linear[1])
        self.assertEqual(self.call('control_status')['result']['speed_level'],1)
        # A lost acceptance response can be retried with the same id, without changing speed again.
        replay=self.call('system_command',command='set_speed_level',params={'level':1},request_id='speed-one')
        self.assertTrue(replay['ok']);self.assertEqual(replay['result']['operation_id'],oid)

    def test_invalid_speed_and_ignored_field_are_rejected(self):
        self.authorize()
        for index, level in enumerate((0,6,True,1.5,'1',None)):
            r=self.call('system_command',command='set_speed_level',params={'level':level},request_id='bad-'+str(index))
            self.assertEqual(r['error']['code'],'SPEED_LEVEL')
        self.assertEqual(self.call('operator_input',keyboard={'pressed_keys':[]},speed_level=1)['error']['code'],'USE_SET_SPEED_LEVEL')
        self.clock.step(.301);self.output.runtime_received=self.clock()
        self.session.update(self.call('freshness_request')['result'])
        self.assertEqual(self.call('system_command',command='set_speed_level',params={'level':1},request_id='not-neutral')['error']['code'],'NEUTRAL_REQUIRED')

    def test_reset_requires_fresh_settled_runtime_and_stays_unauthorized(self):
        self.authorize()
        self.output.runtime_state['diagnostics']['global_inhibit']=True
        for group in self.output.runtime_state['diagnostics']['groups'].values():group['state']=3
        self.output.runtime_state['operations']={'arm':{'state':'in_progress'}}
        r=self.call('system_command',command='reset_global_stop',params={},request_id='busy-reset')
        self.assertEqual(r['error']['code'],'RUNTIME_NOT_SETTLED')
        self.output.runtime_state['operations']={}
        called=[];self.output.reset_global_stop=lambda:(called.append(True) or True,0,'reset')
        r=self.call('system_command',command='reset_global_stop',params={},request_id='reset')
        self.assertTrue(r['ok']);oid=r['result']['operation_id']
        self.wait(lambda:self.c.operations[oid]['state']=='succeeded')
        self.assertTrue(called);self.assertFalse(self.c.sessions['owner']['authorized'])

    def test_deployed_arm_levels_match_confirmed_ros1_keyboard_parameters(self):
        import yaml
        root=Path(__file__).resolve().parents[2]
        source=yaml.safe_load((root/'reference/ros1/car_control/config/keyboard_teleop.yaml').read_text())['keyboard_teleop_node']
        current=load_bridge_control_config(str(root/'host_bridge/bridge_control.yaml'))
        self.assertEqual(current['default_speed_level'],1)
        for axis in ('linear','angular'):
            for level in range(1,6):
                self.assertEqual(current['arm_'+axis+'_levels'][level],source[f'arm_{axis}_level_{level}'])
        caps=self.core.make_capabilities()
        self.assertIn('speed_level_v1',caps['control_capabilities'])
        self.assertIn('latched_global_stop_v1',caps['control_capabilities'])
        self.assertEqual(caps['speed_control']['arm_linear_m_s'][1],self.core.arm_linear[1])

    def test_absent_physical_evidence_is_unknown_and_never_inferred_from_ready_axes(self):
        associations=[dict(condition='arm_secured',source_group='arm',dependent_group='base',
            required_source='commissioned_arm_pose_and_brake',satisfied=False,evidence=0,valid_until_ns=0)]
        self.output.runtime_state['diagnostics']['associations']=associations
        self.assertTrue(self.c.permission())  # All groups ready; no failed source requires isolation evidence.
        status=self.call('control_status')['result']['safety_conditions'][0]
        self.assertEqual(status['status'],'unknown');self.assertFalse(status['satisfied'])
        associations[0].update(satisfied=True,evidence=1,valid_until_ns=int((self.clock()+.1)*1e9))
        self.assertEqual(self.call('control_status')['result']['safety_conditions'][0]['status'],'satisfied')
        self.clock.step(.2)
        self.assertEqual(self.call('control_status')['result']['safety_conditions'][0]['status'],'expired')
        self.clock.step(1)
        status=self.call('control_status')['result']['safety_conditions'][0]
        self.assertEqual(status['status'],'stale');self.assertFalse(status['satisfied'])

    def test_joint_runtime_states_fault_semantics_and_throttling(self):
        from host_bridge_node import HostBridgeServer
        server = HostBridgeServer('127.0.0.1', 0, self.output, joint_runtime_topic='/robot_runtime/state')
        sent = []
        server.broadcast = sent.append
        try:
            server.start_joint_runtime_forwarder()
            stopped_state = {
                'groups': {'base': 'Stopped', 'arm': 'Fault'},
                'diagnostics': {
                    'readable': True,
                    'axes': {
                        'left_front_wheel_joint': {'group': 'base', 'backend': 'CanDriverSystem', 'ready': False},
                        'shoulder_yaw_joint': {'group': 'arm', 'backend': 'RobotCanopenSystem', 'ready': False},
                    },
                    'events': [],
                },
            }
            server._on_runtime_state(stopped_state)
            server._on_runtime_state(stopped_state)  # Identical within 1s -> throttled
            self.assertEqual(len(sent), 1)
            by_name = {item['joint_name']: item for item in sent[0]['states']}
            self.assertFalse(by_name['left_front_wheel_joint']['fault'])
            self.assertFalse(by_name['left_front_wheel_joint']['online'])
            self.assertTrue(by_name['shoulder_yaw_joint']['fault'])
        finally:
            server.control.close()

    def test_detect_publish_host_offline_lan_and_hostname_fallback(self):
        import socket
        from unittest import mock
        from host_bridge_node import detect_publish_host

        class FakeUdpSocket:
            def __init__(self):
                self.target = None
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False
            def connect(self, addr):
                if addr[0] == '8.8.8.8':
                    raise OSError(101, 'Network is unreachable')
                self.target = addr[0]
            def getsockname(self):
                return ('192.168.1.50', 0)

        with mock.patch('host_bridge_node.socket.socket', return_value=FakeUdpSocket()):
            self.assertEqual(detect_publish_host(), '192.168.1.50')

        class AlwaysFailSocket:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False
            def connect(self, _addr):
                raise OSError(101, 'Network is unreachable')

        fake_addrinfo = [
            (socket.AF_INET, socket.SOCK_DGRAM, 0, '', ('127.0.1.1', 0)),
            (socket.AF_INET, socket.SOCK_DGRAM, 0, '', ('10.20.30.40', 0)),
        ]
        with mock.patch('host_bridge_node.socket.socket', return_value=AlwaysFailSocket()), \
             mock.patch('host_bridge_node.socket.getaddrinfo', return_value=fake_addrinfo):
            self.assertEqual(detect_publish_host(), '10.20.30.40')

        with mock.patch('host_bridge_node.socket.socket', return_value=AlwaysFailSocket()), \
             mock.patch('host_bridge_node.socket.getaddrinfo', side_effect=OSError('no host')):
            self.assertEqual(detect_publish_host(), '127.0.0.1')

    def test_build_co2_data_and_unique_example_cameras(self):
        import yaml
        from bridge_messages import build_co2_data
        from bridge_protocol import PROTOCOL_VERSION
        from host_bridge_node import load_cameras_from_yaml

        msg = build_co2_data(615, seq=3)
        self.assertEqual(msg['type'], 'co2_data')
        self.assertEqual(msg['protocol_version'], PROTOCOL_VERSION)
        self.assertEqual(msg['seq'], 3)
        self.assertEqual(msg['ppm'], 615)
        self.assertIsInstance(msg['timestamp_ms'], int)
        self.assertGreater(msg['timestamp_ms'], 0)

        example_yaml = Path(__file__).resolve().parents[1] / 'video_sources.example.yaml'
        raw = yaml.safe_load(example_yaml.read_text(encoding='utf-8'))
        all_sources = list(raw.get('direct_sources', [])) + list(raw.get('streams', []))
        raw_ids = [int(item['camera_id']) for item in all_sources]
        self.assertGreaterEqual(len(raw_ids), 3)
        self.assertEqual(len(raw_ids), len(set(raw_ids)))
        cams = load_cameras_from_yaml(str(example_yaml), '127.0.0.1')
        ids = [cam['camera_id'] for cam in cams]
        self.assertGreaterEqual(len(ids), 1)
        self.assertEqual(len(ids), len(set(ids)))

    def test_video_manager_snapshot_fallback_and_stream_timeout(self):
        from unittest import mock
        from video_manager_ipc import VideoManagerClient
        from video_manager_node import VideoManagerNode

        example_yaml = Path(__file__).resolve().parents[1] / 'video_sources.example.yaml'
        node = VideoManagerNode(str(example_yaml), '127.0.0.1', 0, dry_run=True)
        release_evt = threading.Event()
        # Hold _manager_lock from another thread so list_cameras hits the non-blocking fallback.
        hold_ready = threading.Event()
        def hold_lock():
            node._manager_lock.acquire()
            hold_ready.set()
            release_evt.wait(2.0)
            node._manager_lock.release()
        holder = threading.Thread(target=hold_lock, daemon=True)
        holder.start()
        self.assertTrue(hold_ready.wait(1.0))
        try:
            resp = node._handle_request({'type': 'list_cameras', 'seq': 9})
            self.assertTrue(resp['ok'])
            self.assertEqual(resp['type'], 'camera_list')
            self.assertGreaterEqual(len(resp['cameras']), 1)
        finally:
            release_evt.set()
            holder.join(1.0)

        client = VideoManagerClient('127.0.0.1', 18081, timeout=0.6)
        with mock.patch.object(client, 'request', return_value={'ok': True}) as mock_req:
            client.stream_request(2, 'start')
            self.assertGreaterEqual(mock_req.call_args.kwargs['timeout'], 8.0)

    def test_loopback_probe_disconnect_logged_as_debug(self):
        import socket
        from host_bridge_node import BridgeClient, HostBridgeServer
        from output_adapters import DryRunOutput

        server = HostBridgeServer('127.0.0.1', 0, DryRunOutput())
        a, b = socket.socketpair()
        b.settimeout(2.0)
        client = BridgeClient(server, a, ('127.0.0.1', 55555))
        server.active_clients.append(client)
        thread = threading.Thread(target=client.run, daemon=True)
        thread.start()
        try:
            with b.makefile('rb') as stream:
                stream.readline()
                stream.readline()
                b.sendall(b'{"type":"heartbeat","protocol_version":1,"seq":1}\n')
                ack = json.loads(stream.readline())
                self.assertEqual(ack['type'], 'heartbeat_ack')
            b.close()
            thread.join(2.0)
            events = [e for e in server.events.recent(20) if e['category'] == 'tcp']
            connected = [e for e in events if e['message'] == 'client connected'][-1]
            disconnected = [e for e in events if e['message'] == 'client disconnected'][-1]
            self.assertEqual(connected['level'], 'debug')
            self.assertEqual(disconnected['level'], 'debug')
        finally:
            try:
                b.close()
            except OSError:
                pass
            server.stop_event.set()
            server.control.close()

    def test_co2_data_seq_and_status_forwarding(self):
        import builtins
        from unittest import mock
        from bridge_protocol import PROTOCOL_VERSION
        from host_bridge_node import HostBridgeServer, parse_node_args
        from output_adapters import DryRunOutput

        d1 = self.core.make_co2_data(410)
        d2 = self.core.make_co2_data(415)
        st = self.core.make_co2_status(online=False, ppm=None, error='timeout', timestamp_ms=123456)
        self.assertEqual(d1['seq'], 1)
        self.assertEqual(d2['seq'], 2)
        self.assertEqual(st['type'], 'co2_status')
        self.assertEqual(st['protocol_version'], PROTOCOL_VERSION)
        self.assertEqual(st['seq'], 3)
        self.assertFalse(st['online'])
        self.assertIsNone(st['ppm'])
        self.assertEqual(st['error'], 'timeout')
        self.assertEqual(st['timestamp_ms'], 123456)

        server = HostBridgeServer('127.0.0.1', 0, DryRunOutput(), co2_status_topic='/co2_status')
        sent = []
        server.broadcast = sent.append
        try:
            self.assertIn('co2_status', server.core.make_capabilities()['supports'])
            server._on_co2_status({'online': True, 'ppm': 450, 'error': ''})
            server._on_co2_status({'online': True, 'ppm': 450, 'error': ''})
            self.assertEqual(len(sent), 1)
            server._on_co2_status({'online': False, 'ppm': None, 'error': 'conn reset'})
            self.assertEqual(len(sent), 2)
            self.assertFalse(sent[-1]['online'])
            self.assertIsNone(sent[-1]['ppm'])
            self.assertEqual(sent[-1]['error'], 'conn reset')
        finally:
            server.control.close()

        orig_import = builtins.__import__
        def fake_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == 'rclpy.utilities' or name == 'rclpy':
                raise ImportError('no rclpy')
            return orig_import(name, globals, locals, fromlist, level)
        with mock.patch('sys.argv', ['host_bridge_node.py', '--dry-run', '--port', '19090', '--ros-args', '-r', '__node:=x']), \
             mock.patch('builtins.__import__', side_effect=fake_import):
            _, args = parse_node_args()
            self.assertTrue(args.dry_run)
            self.assertEqual(args.port, 19090)

if __name__=='__main__':unittest.main(verbosity=2)
