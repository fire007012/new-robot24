import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge_core import BridgeCore
from control_v2 import ControlV2, CAPABILITIES
from output_adapters import DryRunOutput


class Clock:
    def __init__(self): self.value = 10.0
    def __call__(self): return self.value
    def step(self, sec): self.value += sec


class Capture(DryRunOutput):
    def __init__(self, clock):
        super().__init__()
        self.clock = clock
        self.outputs = []
        self.calls = []
        self.gate = threading.Event()
        self.entered = threading.Event()
        self.block = False
        self.runtime_received = clock()
        self.runtime_state = dict(groups={g:'Running' for g in ('base','arm','flippers','gripper')},
            diagnostics=dict(readable=True,global_inhibit=False,owner_fresh=True,
                groups={g:dict(state=2,ready=True,blockers=[]) for g in ('base','arm','flippers','gripper')}))
    def publish_twist(self, value): self.outputs.append((self.clock(),value.linear_x,value.source))
    def publish_servo(self, value): pass
    def publish_gripper(self, value): pass
    def publish_flipper(self, value): pass
    def call_lifecycle(self, command):
        self.calls.append(command)
        if self.block and command != 'halt':
            self.entered.set()
            self.gate.wait(35)
        return True,0,'ok'


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.output = Capture(self.clock)
        self.core = BridgeCore(self.output,500,.8,1.5)
        self.tmp = tempfile.TemporaryDirectory()
        self.c = ControlV2(self.core,self.clock,self.tmp.name)
        self.seq = 0
        self.hello()
    def tearDown(self):
        self.output.gate.set()
        self.c.close()
        self.tmp.cleanup()
    def hello(self, conn='owner', role='control'):
        r = self.c.handle(conn,dict(type='control_hello',protocol_version=2,robot_id=self.core.robot_id,role=role,capabilities=CAPABILITIES))
        if conn == 'owner' and r['ok']:
            self.session = r['result']
            self.seq = 0
        return r
    def msg(self, kind, **fields):
        self.seq += 1
        return dict(type=kind,protocol_version=2,session_id=self.session['session_id'],epoch=self.session['epoch'],
                    freshness_token=self.session['freshness_token'],ttl_ms=300,seq=self.seq,**fields)
    def call(self, kind, **fields): return self.c.handle('owner',self.msg(kind,**fields))
    def refresh(self):
        self.output.runtime_received=self.clock()
        self.session['epoch']=self.c.epoch
        r=self.call('freshness_request')
        self.assertTrue(r['ok'],r)
        self.session.update(r['result'])
    def arm(self):
        self.assertTrue(self.call('operator_input',keyboard={'pressed_keys':[]})['ok'])
        self.assertTrue(self.call('authorize')['ok'])
        self.assertTrue(self.call('authorize_operations')['ok'])
    def move(self):
        r=self.call('operator_input',keyboard={'pressed_keys':['w']})
        self.c.tick()
        return r
    def wait_op(self, oid):
        until=time.monotonic()+3
        while self.c.operations[oid]['state'] in ('accepted','running'):
            self.assertLess(time.monotonic(),until)
            time.sleep(.005)
        return self.c.operations[oid]
    def test_wall_clock_and_stale_watchdog(self):
        self.arm()
        for wall in (time.time()+86400,time.time()-86400,0):
            with patch('time.time',return_value=wall):
                self.assertTrue(self.move()['ok'])
                self.assertGreater(self.output.outputs[-1][1],0)
            self.clock.step(.1);self.refresh()
        stale=self.msg('operator_input',keyboard={'pressed_keys':['w']})
        self.clock.step(.301)
        self.assertEqual(self.c.handle('owner',stale)['error']['code'],'STALE_TOKEN')
        self.clock.step(.201);self.c.tick()
        self.assertEqual(self.output.outputs[-1][1],0)
        self.assertFalse(self.c.sessions['owner']['authorized'])
    def test_sequence_session_epoch(self):
        self.arm()
        m=self.msg('operator_input',keyboard={'pressed_keys':['w']})
        self.assertTrue(self.c.handle('owner',m)['ok'])
        self.assertEqual(self.c.handle('owner',m)['error']['code'],'REPLAY')
        self.c.runtime_changed()
        self.assertEqual(self.c.handle('owner',m)['error']['code'],'EPOCH')
        status=self.call('control_status');self.assertTrue(status['ok'])
        self.refresh()
        self.assertEqual(self.move()['error']['code'],'NOT_AUTHORIZED')
        self.c.disconnect('owner');self.hello()
        self.assertEqual(self.c.handle('owner',m)['error']['code'],'SESSION')
    def test_monitor_and_two_owners(self):
        self.arm();self.move()
        seq=self.c.sessions['owner']['seq']
        self.assertEqual(self.hello('second')['error']['code'],'OWNER_BUSY')
        r=self.hello('monitor','monitor')['result']
        q=dict(type='operator_input',protocol_version=2,session_id=r['session_id'],epoch=r['epoch'],seq=99,ttl_ms=300)
        self.assertEqual(self.c.handle('monitor',q)['error']['code'],'LEASE')
        self.c.disconnect('monitor')
        self.assertEqual(self.c.sessions['owner']['seq'],seq)
        self.c.emergency()
        self.c.disconnect('owner');self.hello()
        self.assertTrue(self.core.state.emergency_active)
        self.c.tick();self.assertEqual(self.output.outputs[-1][1],0)
    def test_latest_only_and_expired_before_output(self):
        self.arm()
        self.call('operator_input',keyboard={'pressed_keys':['w']})
        self.call('operator_input',keyboard={'pressed_keys':[]})
        self.c.tick();self.assertEqual(self.output.outputs[-1][1],0)
        self.call('operator_input',keyboard={'pressed_keys':['w']})
        self.clock.step(.301);self.c.tick()
        self.assertEqual(self.c.counts['EXPIRED_BEFORE_OUTPUT'],1)
        self.assertFalse(any(v[1] for v in self.output.outputs))
    def test_unknown_and_expired_runtime(self):
        self.output.runtime_state['diagnostics'].pop('owner_fresh')
        self.call('operator_input',keyboard={'pressed_keys':[]})
        self.assertEqual(self.call('authorize')['error']['code'],'RUNTIME_PERMISSION')
        self.output.runtime_state['diagnostics']['owner_fresh']=True
        self.arm();self.move()
        self.output.runtime_received=0
        self.c.tick();self.assertEqual(self.output.outputs[-1][1],0)
    def test_schema_and_legacy(self):
        for keyboard,gamepad in [({'pressed_keys':['1']},{}),({},dict(axes={'left_x':float('nan')})),({},dict(buttons={'menu':True})),({},dict(axes={'unknown':0}))]:
            self.assertFalse(self.call('operator_input',keyboard=keyboard,gamepad=gamepad)['ok'])
        self.assertEqual(self.c.handle('old',dict(type='operator_input'))['error']['code'],'UPGRADE_REQUIRED')
    def test_tasks_dedup_queue_revoke(self):
        self.call('operator_input',keyboard={'pressed_keys':[]})
        self.call('authorize_operations')
        self.output.block=True
        first=self.msg('system_command',request_id='init-1',command='init')
        accepted=self.c.handle('owner',first)['result']
        self.assertTrue(self.output.entered.wait(1))
        self.assertEqual(self.c.handle('owner',first)['result']['operation_id'],accepted['operation_id'])
        changed=dict(first,command='enable')
        self.assertEqual(self.c.handle('owner',changed)['error']['code'],'REQUEST_CONFLICT')
        pending=self.call('system_command',request_id='enable-1',command='enable')['result']
        for n in range(15):
            self.assertTrue(self.call('system_command',request_id='q'+str(n),command='init')['ok'])
        self.assertEqual(self.call('system_command',request_id='overflow',command='resume')['error']['code'],'QUEUE_FULL')
        self.c.runtime_changed();self.output.gate.set()
        self.assertEqual(self.wait_op(pending['operation_id'])['state'],'failed')
        self.assertEqual(self.output.calls,['init'])
        self.assertTrue(self.c.operations[accepted['operation_id']]['invalidated'])
    def test_restart_unknown_and_emergency_persistence(self):
        self.c.emergency()
        self.c.operations['unknown']=dict(state='running')
        self.c._save()
        self.c.close()
        other_core=BridgeCore(Capture(self.clock),500,.8,1.5)
        other=ControlV2(other_core,self.clock,self.tmp.name)
        try:
            self.assertTrue(other_core.state.emergency_active)
            self.assertEqual(other.operations['unknown']['state'],'result_unknown')
        finally:
            other.close()
    def test_100_recoveries(self):
        for _ in range(100):
            self.arm();self.assertTrue(self.move()['ok'])
            self.c.disconnect('owner')
            self.assertEqual(self.output.outputs[-1][1],0)
            self.clock.step(.01);self.hello()
            self.assertEqual(self.move()['error']['code'],'NOT_AUTHORIZED')
            self.refresh()
    def test_lease_expiry_and_heartbeat_does_not_extend_input(self):
        self.arm();self.move()
        for n in range(6):
            self.clock.step(.1)
            list(self.core.handle_message(dict(type='heartbeat',seq=n)))
            self.c.tick()
        self.assertFalse(self.c.sessions['owner']['authorized'])
        self.assertEqual(self.output.outputs[-1][1],0)
        self.clock.step(1);self.c.tick()
        self.assertIsNone(self.c.owner)
        self.assertEqual(self.call('freshness_request')['error']['code'],'LEASE')
        self.assertTrue(self.hello('replacement')['ok'])

    def test_expired_queued_job_never_executes(self):
        self.call('operator_input',keyboard={'pressed_keys':[]})
        self.call('authorize_operations')
        self.output.block=True
        op=self.call('system_command',request_id='blocking',command='init')['result']
        self.assertTrue(self.output.entered.wait(1))
        pending=self.call('system_command',request_id='expired',command='enable')['result']
        self.clock.step(.301);self.output.gate.set()
        self.assertEqual(self.wait_op(op['operation_id'])['state'],'succeeded')
        result=self.wait_op(pending['operation_id'])
        self.assertEqual(result['error'],'EXPIRED_IN_QUEUE')
        self.assertEqual(self.output.calls,['init'])

    def test_emergency_bypasses_admission_lock(self):
        entered=threading.Event();release=threading.Event()
        def locked():
            with self.c.lock:
                entered.set();release.wait(2)
        worker=threading.Thread(target=locked);worker.start()
        self.assertTrue(entered.wait(1))
        try:
            begin=time.monotonic()
            self.c.emergency()
            self.assertLess(time.monotonic()-begin,.1)
            self.assertTrue(self.core.state.emergency_active)
            self.assertEqual(self.output.outputs[-1][1],0)
        finally:
            release.set();worker.join(2)

    def test_slow_journal_does_not_block_watchdog_or_emergency(self):
        self.arm();self.move()
        entered=threading.Event();gate=threading.Event();responses=[]
        original=self.c._save
        def save():entered.set();gate.wait(2);original()
        self.c._save=save
        request=self.msg('system_command',request_id='disk-slow',command='init')
        worker=threading.Thread(target=lambda:responses.append(self.c.handle('owner',request)))
        worker.start();self.assertTrue(entered.wait(1))
        try:
            begin=time.monotonic();self.clock.step(.501);self.c.tick()
            self.assertEqual(self.output.outputs[-1][1],0)
            self.assertTrue(self.hello('monitor','monitor')['ok'])
            self.c.emergency()
            self.assertLess(time.monotonic()-begin,.1)
            self.assertNotIn('init',self.output.calls)
        finally:
            gate.set();worker.join(3)
        self.assertFalse(worker.is_alive())
        result=self.wait_op(responses[0]['result']['operation_id'])
        self.assertEqual(result['state'],'failed')
        self.assertNotIn('init',self.output.calls)

    def test_persistence_failure_rejects_without_effect(self):
        self.arm()
        def failed():raise OSError('injected disk failure')
        self.c._save=failed
        response=self.call('system_command',request_id='disk-failed',command='init')
        self.assertEqual(response['error']['code'],'PERSISTENCE')
        self.assertTrue(self.c.jobs.empty())
        self.assertNotIn('init',self.output.calls)

    def test_worker_persistence_failure_is_queryable(self):
        self.arm();original=self.c._save;count=[0]
        def failed():
            count[0]+=1
            if count[0]>1:raise OSError('injected worker disk failure')
            original()
        self.c._save=failed
        response=self.call('system_command',request_id='disk-worker',command='init')
        result=self.wait_op(response['result']['operation_id'])
        self.assertEqual(result['state'],'result_unknown')
        self.assertEqual(result['error'],'PERSISTENCE')
        self.assertTrue(self.c.worker.is_alive())
        self.assertNotIn('init',self.output.calls)

    def test_runtime_progress_does_not_change_epoch(self):
        from types import SimpleNamespace
        from output_adapters import RosOutput
        output=RosOutput.__new__(RosOutput)
        output.epoch=None;changes=[];output.runtime_reset_callback=changes.append
        value=dict(groups={'base':'Stopped'},input_epochs={'base':1},operations={})
        output.state(SimpleNamespace(data=json.dumps(value)))
        value['operations']={'init':{'progress':50}}
        output.state(SimpleNamespace(data=json.dumps(value)))
        self.assertEqual(len(changes),1)
        value['input_epochs']['base']=2
        output.state(SimpleNamespace(data=json.dumps(value)))
        self.assertEqual(len(changes),2)

    def test_late_backend_success_stays_unknown(self):
        from concurrent.futures import Future
        from types import SimpleNamespace
        from output_adapters import RosOutput
        future=Future()
        self.c.operations['late']=dict(state='result_unknown',created_at=time.time())
        self.output.operation_context.operation_id='late'
        with self.assertRaises(TimeoutError):
            RosOutput.wait(self.output,future,.01)
        future.set_result(SimpleNamespace(success=True,message='eventually executed'))
        self.assertTrue(self.c.operations['late']['late_backend_results'][0]['success'])
        self.assertEqual(self.c.operations['late']['state'],'result_unknown')

    def test_30_second_service_isolation(self):
        # Real elapsed stall, not a simulated 30s clock advance.
        self.output.block=True
        self.arm();self.move()
        op=self.call('system_command',request_id='slow-init',command='init')['result']
        self.assertTrue(self.output.entered.wait(1))
        begin=time.monotonic()
        self.clock.step(.501)
        self.c.tick()
        self.assertEqual(self.output.outputs[-1][1],0)
        r=self.hello('monitor','monitor')['result']
        for _ in range(300):
            result=self.c.handle('monitor',dict(type='control_status',protocol_version=2,session_id=r['session_id']))
            self.assertTrue(result['ok'])
            time.sleep(.1)
        self.c.emergency()
        self.assertEqual(self.output.outputs[-1][1],0)
        self.assertLess(time.monotonic()-begin,32)
        self.output.gate.set()
        self.assertEqual(self.wait_op(op['operation_id'])['state'],'succeeded')
        print(json.dumps(dict(evidence='30s_service',elapsed=time.monotonic()-begin,outputs=self.output.outputs[-6:],calls=self.output.calls)))

if __name__=='__main__':unittest.main(verbosity=2)
