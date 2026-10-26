import copy
import math
from pathlib import Path
import tempfile
import unittest
import yaml
from robot_hardware_runtime.policy import Policy, AXES, IDENTITY, MAINTENANCE, counts, key, expected_pdos
from robot_hardware_runtime.coordinator import Coordinator

ROOT = Path(__file__).resolve().parents[2]
# Generate using the actual project converter, never emulate the expected SI map in a second converter.
import importlib.util
spec = importlib.util.spec_from_file_location('generate_bus', ROOT/'robot_canopen_config/scripts/generate_bus.py')
generator = importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
BUS = generator.build_bus(yaml.safe_load((ROOT/'robot_canopen_config/config/legacy_master.yaml').read_text()),
                          yaml.safe_load((ROOT/'robot_canopen_config/config/legacy_joints.yaml').read_text()), '.')
BASELINE = yaml.safe_load((ROOT/'robot_hardware/config/mock_commissioning.yaml').read_text())
class IO:
    def __init__(self):
        self.values = {}; self.writes = []; self.fail_at = None
        for joint in AXES:
            self.values[joint] = {**expected_pdos(BUS, joint), **BASELINE['axes'][joint]['identity'],
                                  **{key(i,s):0 for i,s in MAINTENANCE},key(0x6064,0):100,key(0x1010,1):1}
    def read(self, joint, index, sub): return self.values[joint][key(index,sub)]
    def write(self, joint, index, sub, value):
        self.writes.append((joint,index,sub,value))
        if self.fail_at == len(self.writes): raise TimeoutError('injected SDO timeout')
        self.values[joint][key(index,sub)] = value

class DevicePolicy(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.io=IO();self.baseline=copy.deepcopy(BASELINE)
        self.policy=Policy(self.io,BUS,self.baseline,Path(self.tmp.name)/'journal.json')
        self.joint='shoulder_yaw_joint'
    def test_identity_firmware_and_pdo_fail_closed(self):
        self.assertTrue(self.policy.verify())
        self.io.values[self.joint]['100A:00']='wrong firmware'
        with self.assertRaisesRegex(RuntimeError,'identity/firmware'):self.policy.verify()
        self.io.values[self.joint]['100A:00']='MOCK'
        self.io.values[self.joint]['1600:01'] ^= 1
        with self.assertRaisesRegex(RuntimeError,'PDO mismatch'):self.policy.verify()
    def test_commissioning_is_required(self):
        self.baseline['commissioned']=False
        with self.assertRaises(RuntimeError):self.policy.verify()
        with self.assertRaises(RuntimeError):self.policy.maintain(self.joint,'set_offset',0)
        self.assertEqual(self.io.writes,[])
    def test_conversion_and_invalid_range_never_write(self):
        self.assertEqual(counts(2*math.pi,self.joint),6619136)
        for args in [('limits',1,-1),('set_offset',math.nan,0),('set_offset',1e100,0)]:
            with self.assertRaises(ValueError):self.policy.maintain(self.joint,*args)
        self.assertEqual(self.io.writes,[])
    def test_zero_removes_existing_device_offset_once(self):
        self.io.values[self.joint]['607C:00']=50
        self.policy.maintain(self.joint,'set_zero')
        self.assertEqual(self.io.values[self.joint]['607C:00'],(-50)&0xFFFFFFFF)
        self.assertFalse(self.policy.dirty)
    def test_partial_write_restores_and_reads_back_baseline(self):
        before=copy.deepcopy(self.io.values[self.joint]);self.io.fail_at=2
        with self.assertRaisesRegex(RuntimeError,'RAM rollback=True'):self.policy.maintain(self.joint,'limits',-.5,.5)
        for i,s in MAINTENANCE:self.assertEqual(self.io.read(self.joint,i,s),before[key(i,s)])
        self.assertFalse(self.policy.dirty)
    def test_unknown_save_is_persistently_interlocked(self):
        self.io.fail_at=2 # Home write succeeds; save times out and may already have reached the device.
        with self.assertRaises(RuntimeError):self.policy.maintain(self.joint,'set_offset',.1,save=True)
        self.assertTrue(self.policy.dirty)
        reloaded=Policy(self.io,BUS,self.baseline,self.policy.journal)
        with self.assertRaisesRegex(RuntimeError,'unresolved'):reloaded.verify()
        self.io.fail_at=None;reloaded.restore();self.assertFalse(reloaded.dirty)
        self.assertTrue(any(w[1]==0x1010 for w in self.io.writes))
    def test_snapshot_is_read_only_and_not_commissioned(self):
        snap=self.policy.snapshot();self.assertFalse(snap['commissioned']);self.assertEqual(self.io.writes,[])

class RuntimeIO:
    def __init__(self):self.events=[];self.fail=None;self.active=[]
    def event(self,*event):
        self.events.append(event)
        if self.fail==event:raise RuntimeError('injected backend failure')
    def hardware(self,*args):self.event('hardware',*args)
    def arm(self):self.event('arm')
    def running(self,v):self.event('running',v)
    def controllers(self,start,stop):self.event('controllers',tuple(start),tuple(stop));self.active=start
    def active_controllers(self):return self.active
class RuntimePolicy(unittest.TestCase):
    def setUp(self):
        self.io=RuntimeIO()
        class PolicyStub:
            dirty=False
            def verify(self):return True
        self.runtime=Coordinator(self.io,PolicyStub())
    def test_enable_is_armed_resume_is_separate_and_recovery_does_not_run(self):
        self.runtime.init();self.runtime.enable();self.assertEqual(self.runtime.state,'Armed')
        self.assertNotIn(('running',True),self.io.events)
        self.runtime.resume();self.assertEqual(self.runtime.state,'Running')
        self.runtime.halt();self.runtime.recover();self.assertEqual(self.runtime.state,'Armed')
        self.assertEqual(self.io.events.count(('running',True)),1)
    def test_either_backend_activation_failure_converges_both(self):
        for name in self.runtime.COMPONENTS:
            self.setUp();self.runtime.init();self.io.fail=('hardware',name,'active')
            with self.assertRaises(RuntimeError):self.runtime.enable()
            self.assertEqual(self.runtime.state,'Fault')
            for backend in self.runtime.COMPONENTS:self.assertIn(('hardware',backend,'inactive'),self.io.events)
            self.assertNotIn(('running',True),self.io.events)
    def test_profiles_are_serialized_through_stop_and_fresh_controllers(self):
        self.runtime.init();self.runtime.enable();self.runtime.resume();self.io.events=[]
        self.runtime.switch_profile('csp')
        self.assertEqual(self.io.events[0],('running',False))
        self.assertEqual(self.runtime.state,'Running')
        self.assertIn('flipper_csp_controller',self.io.active)
        self.assertNotIn('flipper_csv_forward_controller',self.io.active)
    def test_failed_switch_never_restores_running_intent(self):
        self.runtime.init();self.runtime.enable();self.runtime.resume()
        self.io.fail=('arm',)
        with self.assertRaises(RuntimeError):self.runtime.switch_profile('csp')
        self.assertEqual(self.runtime.state,'Fault')
if __name__=='__main__':unittest.main()

class FlipperReference(unittest.TestCase):
    def test_linkage_conflicts_timeout_and_bounds(self):
        from robot_hardware_runtime.flipper import Reference, NAMES
        limits={n:{'lower':-1,'upper':1} for n in NAMES}
        r=Reference(limits);r.linkage='side_pair';r.command([NAMES[0]],[.2],1)
        positions,v,_=r.step([0]*4,1.01,.02)
        self.assertGreater(v[0],0);self.assertEqual(v[0],v[2]);self.assertEqual(v[1],0)
        _,v,expired=r.step([0]*4,1.3,.02);self.assertTrue(expired);self.assertEqual(v,[0]*4)
        with self.assertRaises(ValueError):r.command([NAMES[0],NAMES[2]],[.2,-.2],2)
        r.command([NAMES[0]],[.2],3);_,v,_=r.step([1,0,1,0],3.01,.02)
        self.assertEqual(v[0],0);self.assertEqual(v[2],0)

class MaintenanceRecovery(unittest.TestCase):
    def test_restore_from_fault_requires_identity_and_closes_dirty_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            transport=IO();policy=Policy(transport,BUS,copy.deepcopy(BASELINE),Path(directory)/'journal.json')
            transport.fail_at=2
            with self.assertRaises(RuntimeError):policy.maintain('shoulder_yaw_joint','set_offset',.1,save=True)
            self.assertTrue(policy.dirty)
            adapter=RuntimeIO();adapter.disable=lambda name:None
            runtime=Coordinator(adapter,policy);runtime.state='Fault';transport.fail_at=None
            runtime.maintain('shoulder_yaw_joint','restore',0,0,False)
            self.assertFalse(policy.dirty);self.assertEqual(runtime.state,'Initialized')
            self.assertNotIn(('running',True),adapter.events)
    def test_failed_restore_keeps_a_durable_interlock(self):
        with tempfile.TemporaryDirectory() as directory:
            transport=IO();baseline=copy.deepcopy(BASELINE)
            policy=Policy(transport,BUS,baseline,Path(directory)/'journal.json')
            policy.maintain('shoulder_yaw_joint','set_offset',.1)
            transport.fail_at=len(transport.writes)+2
            with self.assertRaises(TimeoutError):policy.restore()
            self.assertTrue(policy.dirty)
            self.assertTrue(Policy(transport,BUS,baseline,policy.journal).dirty)

class ScopedMaintenance(unittest.TestCase):
    def test_uncertain_target_does_not_block_other_group_identity_reads(self):
        with tempfile.TemporaryDirectory() as directory:
            transport=IO();policy=Policy(transport,BUS,copy.deepcopy(BASELINE),Path(directory)/'journal.json')
            transport.fail_at=2
            with self.assertRaises(RuntimeError):policy.maintain('shoulder_yaw_joint','set_offset',.1,save=True)
            self.assertEqual(policy.dirty_axes,{'shoulder_yaw_joint'})
            self.assertTrue(policy.verify(['left_front_arm_joint']))
            with self.assertRaisesRegex(RuntimeError,'unresolved'):policy.verify(['shoulder_yaw_joint'])
    def test_unscoped_legacy_dirty_journal_remains_conservative(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            journal=Path(directory)/'journal.json';journal.write_text(json.dumps({'dirty':True}))
            policy=Policy(IO(),BUS,copy.deepcopy(BASELINE),journal)
            self.assertEqual(policy.dirty_axes,set(AXES))
            with self.assertRaisesRegex(RuntimeError,'unresolved'):policy.verify(['left_front_arm_joint'])
