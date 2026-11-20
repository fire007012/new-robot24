import copy
import importlib.util
from pathlib import Path
import sys,tempfile,unittest
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'robot_hardware')]
from register_arm_baseline import register,ARM
from robot_hardware_runtime.policy import Policy,IDENTITY,MAINTENANCE,key,expected_pdos
spec=importlib.util.spec_from_file_location('generate_bus',ROOT/'robot_canopen_config/scripts/generate_bus.py')
generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
load=lambda p:yaml.safe_load((ROOT/p).read_text())
BUS=generator.build_bus(load('robot_canopen_config/config/legacy_master.yaml'),load('robot_canopen_config/config/legacy_joints.yaml'),'.')

class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.original=load('robot_hardware/config/commissioning.yaml')
        self.registration=load('robot_hardware/config/arm_canopen_registration.yaml')
    def test_registered_unknown_identity_is_accepted_by_existing_runtime_policy(self):
        baseline=register(self.original,self.registration,BUS)
        self.assertFalse(baseline['commissioned'])
        self.assertIn('arm',baseline['commissioned_groups'])
        self.assertNotIn('flippers',baseline['commissioned_groups'])
        for joint in ARM:
            self.assertEqual(baseline['axes'][joint]['identity'],self.original['axes'][joint]['identity'])
            self.assertEqual(baseline['axes'][joint]['persistent'],self.original['axes'][joint]['persistent'])
        class FixtureIO:
            def __init__(self):self.reads=[];self.writes=[]
            def read(self,joint,index,sub):
                self.reads.append((joint,index,sub))
                values={**expected_pdos(BUS,joint),**{key(i,s):0 for i,s in MAINTENANCE},
                        **{key(i,s):1 for i,s in IDENTITY},'100A:00':'fixture-firmware'}
                return values[key(index,sub)]
            def write(self,*args):self.writes.append(args)
        io=FixtureIO()
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(Policy(io,BUS,baseline,Path(directory)/'journal').verify(list(ARM)))
        self.assertEqual(io.writes,[])
        self.assertIn(('shoulder_yaw_joint',0x1018,4),io.reads)
    def test_wrong_node_is_not_registered(self):
        bus=copy.deepcopy(BUS);bus['shoulder_yaw_joint']['node_id']=1
        with self.assertRaisesRegex(ValueError,'Node-ID mismatch'):register(self.original,self.registration,bus)
    def test_existing_known_identity_and_persistent_values_are_preserved(self):
        self.original['axes']['shoulder_pitch_joint']['identity']['1018:04']=123
        self.original['axes']['shoulder_pitch_joint']['persistent']={'607C:00':456}
        b=register(self.original,self.registration,BUS)
        self.assertEqual(b['axes']['shoulder_pitch_joint']['identity']['1018:04'],123)
        self.assertEqual(b['axes']['shoulder_pitch_joint']['persistent'],{'607C:00':456})

if __name__=='__main__':unittest.main()
