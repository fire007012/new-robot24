import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'robot_hardware')]
from register_flippers_baseline import register, FLIPPERS
from robot_hardware_runtime.policy import Policy, IDENTITY, MAINTENANCE, key, expected_pdos

spec = importlib.util.spec_from_file_location('generate_flipper_bus', ROOT / 'robot_canopen_config/scripts/generate_bus.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
load = lambda path: yaml.safe_load((ROOT / path).read_text())
BUS = generator.build_bus(load('robot_canopen_config/config/legacy_master.yaml'),
                          load('robot_canopen_config/config/legacy_joints.yaml'), '.')


class FlippersRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.baseline = load('robot_hardware/config/arm_registered_baseline.yaml')
        self.registration = load('robot_hardware/config/flippers_canopen_registration.yaml')

    def test_live_policy_accepts_registration_without_writes_or_fabricated_identity(self):
        baseline = register(self.baseline, self.registration, BUS)
        self.assertEqual(set(baseline['commissioned_groups']), {'arm', 'base', 'flippers', 'gripper'})
        self.assertFalse(baseline['commissioned'])
        for joint in FLIPPERS:
            self.assertEqual(baseline['axes'][joint]['identity'], self.baseline['axes'][joint]['identity'])
            self.assertEqual(baseline['axes'][joint]['persistent'], self.baseline['axes'][joint]['persistent'])
        class IO:
            writes = []
            def read(self, joint, index, sub):
                return {**expected_pdos(BUS, joint), **{key(i, s): 0 for i, s in MAINTENANCE},
                        **{key(i, s): 1 for i, s in IDENTITY}, '100A:00': 'test-firmware'}[key(index, sub)]
            def write(self, *args): self.writes.append(args)
        io = IO()
        with tempfile.TemporaryDirectory() as directory:
            self.assertTrue(Policy(io, BUS, baseline, Path(directory) / 'journal').verify(FLIPPERS))
        self.assertFalse(io.writes)
        self.assertEqual(baseline['arm_registration'], self.baseline['arm_registration'])

    def test_mismatched_bus_is_rejected(self):
        for field, value in (('node_id', 6), ('position_mode', 7), ('velocity_mode', 8), ('scale_pos_to_dev', 1)):
            with self.subTest(field=field):
                bus = copy.deepcopy(BUS); bus['left_front_arm_joint'][field] = value
                with self.assertRaises(ValueError): register(self.baseline, self.registration, bus)

    def test_confirmation_is_required(self):
        self.registration['registration_status'] = 'unconfirmed'
        with self.assertRaises(ValueError): register(self.baseline, self.registration, BUS)

    def test_existing_identity_is_preserved_and_still_enforced(self):
        self.baseline['axes']['left_front_arm_joint']['identity']['1018:04'] = 123
        baseline = register(self.baseline, self.registration, BUS)
        self.assertEqual(baseline['axes']['left_front_arm_joint']['identity']['1018:04'], 123)
        class WrongDevice:
            def read(self, joint, index, sub):
                return 'test-firmware' if (index, sub) == (0x100A, 0) else 1
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'identity/firmware mismatch'):
                Policy(WrongDevice(), BUS, baseline, Path(directory) / 'journal').verify(['left_front_arm_joint'])


if __name__ == '__main__':
    unittest.main()
