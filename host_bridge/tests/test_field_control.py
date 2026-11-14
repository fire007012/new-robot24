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

if __name__=='__main__':unittest.main(verbosity=2)
