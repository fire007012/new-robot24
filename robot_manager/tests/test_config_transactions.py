import concurrent.futures
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from manager import Manager, Fault
from test_manager import Backend


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        (self.root/'profiles').mkdir()
        for name in ('mock','second'):
            (self.root/'profiles'/(name+'.json')).write_text(json.dumps(dict(profile_id=name,config_revision='1',mode='mock',build_version='test',video=False,startup_timeout=90)))
        (self.root/'token').write_text('x'*32)
        self.config=self.root/'config.json'
        self.config.write_text(json.dumps(dict(enable_config_transactions=True,config_root=str(self.root/'profiles'),state_dir=str(self.root/'state'),token_file=str(self.root/'token'),robot_id='robot24',build_version='test',control_endpoint=dict(host='127.0.0.1',port=9090))))
        self.backend=Backend();self.m=Manager(self.config,self.backend)
        self.n=0
    def tearDown(self):
        c=self.m.config_transactions
        if c.worker:c.worker.join(3)
        self.m.closed.set();self.m.lock_file.close();self.tmp.cleanup()
    def call(self,method,rid=None,**params):
        self.n+=1
        return self.m.dispatch(dict(version=1,token='x'*32,request_id=rid or str(self.n),method=method,params=params))
    def status(self):return self.call('config.status')
    def candidate(self,value=100,base=None,rid=None):
        return self.call('config.candidate',rid,base_revision=base or self.status()['source_revision'],patches=[dict(profile_id='mock',field='startup_timeout',value=value)])
    def apply(self,candidate,rid=None):
        return self.call('config.apply',rid,base_revision=self.status()['source_revision'],candidate_revision=candidate['candidate_revision'])
    def wait(self,op):
        until=time.monotonic()+3
        while True:
            result=self.call('config.operation',operation_id=op['operation_id'])
            if result['state'] not in ('accepted','running'):return result
            self.assertLess(time.monotonic(),until);time.sleep(.005)
    def test_schema_candidate_not_effective_and_conflicts(self):
        self.assertEqual(self.call('config.schema')['fields']['startup_timeout']['unit'],'s')
        source=self.status()['source_revision']
        self.assertEqual(self.status()['source_summary']['revision'],source)
        self.assertEqual(self.status()['source_summary']['profile_count'],2)
        candidate=self.candidate()
        self.assertEqual(candidate['summary']['revision'],candidate['candidate_revision'])
        self.assertEqual(self.status()['source_revision'],source)
        source_result=self.call('config.source',profile_id='mock')
        self.assertEqual(source_result['summary']['revision'],source)
        self.assertEqual(source_result['profile']['startup_timeout'],90)
        self.assertEqual(self.call('config.preview',candidate_revision=candidate['candidate_revision'])['diff'][0]['after'],100)
        with self.assertRaises(Fault):self.candidate(0)
        with self.assertRaises(Fault):self.candidate(181)
        with self.assertRaises(Fault):self.candidate(True)
        with self.assertRaises(Fault):self.candidate(base='0'*64)
        with self.assertRaises(Fault):self.call('config.candidate',base_revision=source,patches=[dict(profile_id='mock',field='baseline',value='/etc/passwd')])
        with self.assertRaises(Fault):self.call('config.candidate',base_revision=source,patches=[dict(profile_id='mock',field='video_required',value=True)])

    def test_non_finite_config_values_are_rejected_as_protocol_errors(self):
        source=self.status()['source_revision']
        with self.assertRaises(Fault) as caught:
            self.call('config.candidate',base_revision=source,patches=[dict(profile_id='mock',field='startup_timeout',value=float('nan'))])
        self.assertEqual(caught.exception.data['code'],'BAD_REQUEST')
    def test_apply_consumer_readback_pending_and_explicit_rollback(self):
        old=self.status()['source_revision']
        candidate=self.candidate()
        result=self.wait(self.apply(candidate))
        self.assertEqual(result['state'],'succeeded')
        self.assertEqual(result['verification'],'manager_source_readback')
        status=self.status()
        self.assertEqual(status['source_revision'],candidate['candidate_revision'])
        self.assertTrue(status['pending_restart']);self.assertIsNone(status['actual_runtime_revision'])
        profile=self.m.profile(dict(robot_id='robot24',profile_id='mock',config_revision=candidate['candidate_revision']))
        self.assertEqual(profile['startup_timeout'],100)
        self.m.data.update(profile=profile,software_state='ready',health={'observed_at':time.time()})
        self.m.last_health_monotonic=time.monotonic()
        self.assertEqual(self.status()['actual_runtime_revision'],candidate['candidate_revision'])
        self.assertFalse(self.status()['pending_restart'])
        self.m.data.update(profile=None,software_state='stopped',health={})
        rollback=self.call('config.rollback',base_revision=candidate['candidate_revision'],target_revision=old)
        self.assertEqual(self.wait(rollback)['state'],'succeeded')
        self.assertEqual(self.status()['source_revision'],old)
    def test_apply_failure_after_switch_rolls_back(self):
        old=self.status()['source_revision'];candidate=self.candidate()
        def fail(phase):
            if phase=='after_switch':raise RuntimeError('injected failure')
        self.m.config_transactions.hook=fail
        op=self.wait(self.apply(candidate))
        self.assertEqual(op['state'],'failed')
        self.assertEqual(op['phase'],'rolled_back')
        self.assertEqual(self.status()['source_revision'],old)
        self.assertEqual(self.call('config.source',profile_id='mock')['profile']['startup_timeout'],90)
    def test_maintenance_and_preflight_rechecked(self):
        candidate=self.candidate()
        self.m.data['software_state']='ready'
        with self.assertRaises(Fault):self.apply(candidate)
        self.m.data['software_state']='stopped';self.backend.bad=True
        op=self.wait(self.apply(candidate))
        self.assertEqual(op['error']['code'],'CONFIG_PREFLIGHT')
        self.assertEqual(op['phase'],'rolled_back')
    def test_duplicate_and_concurrent_apply(self):
        candidate=self.candidate(rid='save')
        self.assertEqual(self.candidate(rid='save'),candidate)
        with self.assertRaises(Fault):self.candidate(101,rid='save')
        gate=threading.Event();entered=threading.Event()
        def preflight(profile):entered.set();gate.wait(2);return dict(passed=True,checks=[])
        self.backend.preflight=preflight
        try:
            op=self.apply(candidate,rid='apply')
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.apply(candidate,rid='apply')['operation_id'],op['operation_id'])
            with self.assertRaises(Fault):self.apply(candidate,rid='different')
            with self.assertRaises(Fault):self.call('start',robot_id='robot24',profile_id='mock',config_revision='1')
            begin=time.monotonic();self.call('snapshot');self.assertLess(time.monotonic()-begin,.1)
        finally:gate.set()
        self.assertEqual(self.wait(op)['state'],'succeeded')
    def test_multiple_profiles_generation_interruption(self):
        old=self.status()['source_revision']
        seen=[]
        def fail(phase):
            if phase=='generated_file':
                seen.append(phase)
                if len(seen)==2:raise RuntimeError('power loss during generation')
        self.m.config_transactions.hook=fail
        with self.assertRaises(RuntimeError):self.candidate()
        self.assertEqual(self.status()['source_revision'],old)
        self.assertEqual(len(self.call('profiles')),2)
        self.m.config_transactions.hook=lambda _:None
        self.assertEqual(self.wait(self.apply(self.candidate()))['state'],'succeeded')
    def test_restart_unknown_no_replay_and_retained_source(self):
        candidate=self.candidate()
        c=self.m.config_transactions
        c.data['operations']['interrupted']=dict(operation_id='interrupted',state='running',phase='activate')
        c.save()
        self.m.closed.set();self.m.lock_file.close()
        self.m=Manager(self.config,self.backend)
        result=self.call('config.operation',operation_id='interrupted')
        self.assertEqual(result['state'],'result_unknown')
        self.assertEqual(result['error'],'MANAGER_RESTART')
        self.assertNotEqual(self.status()['source_revision'],candidate['candidate_revision'])
        self.assertFalse(self.backend.started)
    def test_integrity_mismatch_and_page_limits(self):
        source=self.status()['source_revision']
        with self.assertRaises(Fault):self.call('config.audit',limit=51)
        with self.assertRaises(Fault):self.call('config.source',revision='../token',profile_id='mock')
        c=self.m.config_transactions
        path=c.bundles/source/'profiles/mock.json'
        path.write_text('{}')
        with self.assertRaises(Fault):self.call('config.source',profile_id='mock')
    def test_effective_value_mismatch_rolls_back(self):
        old=self.status()['source_revision'];candidate=self.candidate()
        c=self.m.config_transactions
        def mismatch(phase):
            if phase=='after_switch':
                path=c.profiles_root()/'mock.json'
                value=json.loads(path.read_text());value['startup_timeout']=179;path.write_text(json.dumps(value))
        c.hook=mismatch
        result=self.wait(self.apply(candidate))
        self.assertEqual(result['state'],'failed')
        self.assertEqual(result['error']['code'],'CONFIG_DIGEST')
        self.assertEqual(self.status()['source_revision'],old)

if __name__=='__main__':unittest.main(verbosity=2)
