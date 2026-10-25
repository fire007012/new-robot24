import importlib.util
import math
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generate_bus", ROOT / "scripts" / "generate_bus.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class BusContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = yaml.safe_load((ROOT / "config" / "legacy_master.yaml").read_text())
        cls.joints = yaml.safe_load((ROOT / "config" / "legacy_joints.yaml").read_text())
        cls.bus = MODULE.build_bus(cls.source, cls.joints, Path("/opt/robot/config"))

    def test_six_axes_and_identity(self):
        self.assertEqual(set(self.bus) - {"options", "master"}, set(MODULE.AXES))
        self.assertEqual(self.bus["master"]["node_id"], 127)
        self.assertEqual(self.bus["master"]["sync_period"], 5000)
        for name, (node_id, mode, _) in MODULE.AXES.items():
            node = self.bus[name]
            self.assertEqual(node["node_id"], node_id)
            self.assertFalse(node["perform_homing_on_init"])
            self.assertTrue(node["motion_safety"])
            self.assertEqual(node["sdo"], [
                {"index": 0x6060, "sub_index": 0, "value": mode},
                {"index": 0x60C2, "sub_index": 1, "value": 5},
            ])
            self.assertEqual(node["rpdo"][3]["enabled"], mode == 9)
            self.assertEqual(node["velocity_mode"] if mode == 9 else node["position_mode"], mode)

    def test_units_and_mappings(self):
        for name, (_, _, counts_per_rev) in MODULE.AXES.items():
            node = self.bus[name]
            self.assertAlmostEqual(node["scale_pos_to_dev"] * 2 * math.pi, counts_per_rev)
            self.assertAlmostEqual(node["scale_pos_to_dev"] * node["scale_pos_from_dev"], 1)
            self.assertAlmostEqual(node["scale_vel_to_dev"] * node["scale_vel_from_dev"], 1)
            self.assertEqual([entry["index"] for entry in node["rpdo"][1]["mapping"]],
                             [0x6040, 0x607A, 0x6060])
            self.assertEqual(node["rpdo"][2]["mapping"],
                             [{"index": 0x60C1, "sub_index": 1}])
            self.assertEqual([entry["index"] for entry in node["tpdo"][2]["mapping"]],
                             [0x606C, 0x6077])

    def test_unexpected_wiring_change_fails(self):
        changed = yaml.safe_load(yaml.safe_dump(self.source))
        changed["shoulder_yaw_joint"]["node_id"] = 5
        with self.assertRaises(ValueError):
            MODULE.build_bus(changed, self.joints, Path("/opt/robot/config"))

    def test_single_axis_has_no_other_slave(self):
        for joint in MODULE.AXES:
            bus = MODULE.build_bus(self.source, self.joints, Path('.'), [joint])
            self.assertEqual(set(bus), {'options', 'master', joint})
            self.assertEqual(bus[joint], self.bus[joint])
        self.assertEqual(self.bus['options']['retry_factor'], 0)

    def test_mismatched_count_fails(self):
        changed = yaml.safe_load(yaml.safe_dump(self.joints))
        changed["joints"][0]["counts_per_rev"] = 1
        with self.assertRaises(ValueError):
            MODULE.build_bus(self.source, changed, Path("/opt/robot/config"))


if __name__ == "__main__":
    unittest.main()
