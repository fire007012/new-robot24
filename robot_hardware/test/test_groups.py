import copy
from pathlib import Path
import threading
import unittest
import yaml
from robot_hardware_runtime.groups import GroupCoordinator

CONTRACT = yaml.safe_load((Path(__file__).resolve().parents[2]/'robot_safety/config/group_policy_v1.yaml').read_text())

class IO:
    def __init__(self):
        self.events=[];self.active=set();self.hardware_states={'RobotCanopenSystem':'unconfigured','CanDriverSystem':'unconfigured'}
        self.fail=None;self.block=None;self.guard_states={g:0 for g in CONTRACT['groups']}
    def event(self,*args):
        self.events.append(args)
        if self.fail==args:raise RuntimeError('injected operation failure')
    def begin_scope(self,group):pass
    def ensure_settled(self,group):self.event('settled',group)
    def hardware_state(self,name):return self.hardware_states[name]
    def hardware(self,name,state):self.event('hardware',name,state);self.hardware_states[name]=state
    def initialize(self,backend,group):self.event('initialize',backend,group)
    def prepare(self,backend,group):
        self.event('prepare',backend,group)
        if self.block and self.block[0]==group:self.block[1].set();self.block[2].wait(5)
    def group_operation(self,group,operation):
        self.event('guard',group,operation)
        if operation in ('halt','arm','run'):self.guard_states[group]={'halt':0,'arm':1,'run':2}[operation]
    def heartbeat(self):self.event('heartbeat')
    def owner_reset(self):self.event('owner_reset');self.guard_states={g:3 for g in CONTRACT['groups']}
    def active_controllers(self):return list(self.active)
    def active_resources(self):
        out={}
        for c in self.active:
            group=next(g for g,d in CONTRACT['groups'].items() if c in d['controllers'])
            interface='velocity' if group=='base' or 'csv' in c else 'position'
            out[c]={a+'/'+interface for a,d in CONTRACT['axes'].items() if d['group']==group}
        return out
    def controllers(self,start,stop):
        self.event('controllers',tuple(start),tuple(stop));self.active.difference_update(stop);self.active.update(start)
    def status(self):return {'groups':{g:{'ready':True,'state':self.guard_states[g]} for g in CONTRACT['groups']}}
    def disable(self,joint):self.event('disable',joint)
class Policy:
    dirty_axes=set()
    def __init__(self):self.verified=[]
    def verify(self,joints,**kwargs):self.verified.append(list(joints))

class GroupOperations(unittest.TestCase):
    def setUp(self):
        self.io=IO();self.policy=Policy();self.c=GroupCoordinator(self.io,self.policy,copy.deepcopy(CONTRACT))
    def start(self,groups):self.c.init(groups);self.c.enable(groups);self.c.resume(groups)
    def test_base_only_does_not_prepare_canopen_or_other_private_axes(self):
        self.start(['base'])
        self.assertEqual(self.c.states['base'],'Running');self.assertEqual(self.policy.verified,[])
        self.assertNotIn(('hardware','RobotCanopenSystem','inactive'),self.io.events)
        self.assertEqual([e for e in self.io.events if e[0]=='prepare'],[('prepare','CanDriverSystem','base')])
        self.assertEqual(self.io.active,{'wheel_controller'})
    def test_group_recovery_keeps_other_controllers_and_component_lifecycle(self):
        self.start(['base','arm','gripper']);self.io.events=[]
        before_base=self.c.generation('base');before_arm=self.c.generation('arm')
        self.c.recover(['arm'])
        self.assertEqual(self.c.generation('base'),before_base);self.assertGreater(self.c.generation('arm'),before_arm)
        self.assertEqual(self.c.states['arm'],'Armed');self.assertEqual(self.c.states['base'],'Running')
        self.assertEqual(self.io.active,{'wheel_controller','gripper_controller'})
        self.assertFalse(any(e[0]=='hardware' for e in self.io.events))
        self.assertFalse(any(e[:2]==('guard','base') for e in self.io.events))
    def test_prepare_all_selected_groups_precedes_any_arm(self):
        self.c.init();self.io.events=[];self.c.enable()
        last_prepare=max(i for i,e in enumerate(self.io.events) if e[0]=='prepare')
        first_arm=min(i for i,e in enumerate(self.io.events) if e[0]=='guard' and e[2]=='arm')
        self.assertLess(last_prepare,first_arm)
    def test_invalid_request_preserves_running_session(self):
        self.start(['base']);before=list(self.io.events)
        with self.assertRaises(RuntimeError):self.c.enable(['base'])
        self.assertEqual(self.io.events,before);self.assertEqual(self.c.states['base'],'Running')
        with self.assertRaises(ValueError):self.c.halt(['typo'])
        self.assertEqual(self.io.events,before)
    def test_partial_failure_is_scoped_and_locatable(self):
        self.c.init(['base','arm']);self.io.fail=('prepare','RobotCanopenSystem','arm')
        with self.assertRaises(RuntimeError):self.c.enable(['base','arm'])
        self.assertEqual(self.c.states['base'],'Armed');self.assertEqual(self.c.states['arm'],'Fault')
        self.assertEqual(self.c.operations['arm']['state'],'failed');self.assertEqual(self.c.operations['base']['state'],'complete')
        self.assertEqual(self.c.operations['base']['id'],self.c.operations['arm']['id'])
    def test_profile_switch_rebuilds_only_flippers(self):
        self.start(['base','arm','flippers']);self.io.events=[];self.c.switch_profile('csp')
        self.assertEqual(self.c.states['base'],'Running');self.assertEqual(self.c.states['arm'],'Running')
        self.assertIn('flipper_csp_controller',self.io.active)
        self.assertFalse(any(e[0]=='guard' and e[1] in ('base','arm') for e in self.io.events))
    def test_long_arm_preparation_does_not_block_base_halt_or_diagnostics(self):
        self.start(['base']);self.c.init(['arm'])
        entered,released=threading.Event(),threading.Event();self.io.block=('arm',entered,released)
        failures=[]
        def enable():
            try:self.c.enable(['arm'])
            except Exception as e:failures.append(e)
        worker=threading.Thread(target=enable);worker.start();self.addCleanup(lambda:released.set())
        self.assertTrue(entered.wait(2));self.c.halt(['base']);self.c.monitor()
        with self.assertRaisesRegex(RuntimeError,'in progress'):self.c.recover(['arm'])
        released.set();worker.join(3);self.assertFalse(worker.is_alive());self.assertFalse(failures)
    def test_foreign_controller_stopping_does_not_require_four_active(self):
        self.start(['base']);self.c.monitor();self.assertEqual(self.c.states['base'],'Running')
        self.io.active.clear();self.c.monitor();self.assertEqual(self.c.states['base'],'Fault')
    def test_shared_canopen_rebuild_does_not_reconfigure_private_component(self):
        self.start(['arm','base','flippers','gripper']);self.io.events=[]
        self.c.recover_resource('RobotCanopenSystem')
        self.assertEqual(self.c.states['base'],'Running');self.assertEqual(self.c.states['gripper'],'Running')
        self.assertEqual(self.c.states['arm'],'Armed');self.assertEqual(self.c.states['flippers'],'Armed')
        changes=[e for e in self.io.events if e[0]=='hardware']
        self.assertTrue(changes);self.assertTrue(all(e[1]=='RobotCanopenSystem' for e in changes))
        self.assertIn(('hardware','RobotCanopenSystem','unconfigured'),changes)
    def test_conflicting_controller_ownership_rejected(self):
        bad=copy.deepcopy(CONTRACT);bad['groups']['gripper']['controllers']=['wheel_controller']
        with self.assertRaises(ValueError):GroupCoordinator(self.io,self.policy,bad)
    def test_new_owner_fences_previous_controller_sessions(self):
        self.start(['base','arm'])
        replacement=GroupCoordinator(self.io,self.policy,copy.deepcopy(CONTRACT))
        self.assertTrue(replacement.check_owner())
        self.assertIn(('owner_reset',),self.io.events)
        self.assertFalse(self.io.active)
        self.assertEqual(replacement.states['base'],'Fault')
        self.assertEqual(replacement.states['arm'],'Fault')
        before=list(self.io.events);self.assertTrue(replacement.check_owner())
        self.assertEqual(self.io.events,before)
    def test_monitor_discards_observations_from_previous_session(self):
        self.start(['base']);status=self.io.status
        def stale():
            snapshot=status()
            snapshot['groups']['base']['state']=1
            self.c.generations['base']+=1
            return snapshot
        self.io.status=stale
        self.c.monitor()
        self.assertEqual(self.c.states['base'],'Running')
        self.assertIn('wheel_controller',self.io.active)
    def test_invalid_maintenance_does_not_cancel_motion(self):
        self.start(['arm','base']);before=list(self.io.events)
        for operation,lower,upper in [('typo',0.,0.),('set_offset',float('nan'),0.),('limits',1.,-1.)]:
            with self.assertRaises(ValueError):self.c.maintain('shoulder_yaw_joint',operation,lower,upper,False)
        self.assertEqual(self.io.events,before)
        self.assertEqual(self.c.states['arm'],'Running')

if __name__=='__main__':unittest.main()
