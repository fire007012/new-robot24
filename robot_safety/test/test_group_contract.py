"""Check the versioned contract against real controller resources and both URDF backends."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]

class UniqueLoader(yaml.SafeLoader):
    pass

def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f'duplicate contract key: {key}')
        result[key] = loader.construct_object(value_node)
    return result

UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)

class GroupContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = yaml.load((ROOT / 'robot_safety/config/group_policy_v1.yaml').read_text(), Loader=UniqueLoader)
        cls.controllers = yaml.safe_load((ROOT / 'robot_hardware/config/controllers.yaml').read_text())

    def test_axis_ownership_matches_both_hardware_components(self):
        actual = {}
        for filename in ('robot_hardware/urdf/private.ros2_control.xacro',
                         'robot_canopen_config/urdf/canopen.ros2_control.xacro'):
            for component in ET.parse(ROOT / filename).iter('ros2_control'):
                for joint in component.findall('joint'):
                    self.assertNotIn(joint.attrib['name'], actual)
                    actual[joint.attrib['name']] = component.attrib['name']
        axes = self.contract['axes']
        self.assertEqual({name: axis['backend'] for name, axis in axes.items()}, actual)
        self.assertEqual({g: sum(a['group'] == g for a in axes.values())
                          for g in self.contract['groups']}, {'arm': 6, 'flippers': 4, 'base': 2, 'gripper': 1})
        self.assertEqual({axes[name]['backend'] for name in axes if axes[name]['group'] == 'arm'},
                         {'CanDriverSystem', 'RobotCanopenSystem'})

    def test_controllers_only_claim_their_groups_axes(self):
        seen = set()
        for group, definition in self.contract['groups'].items():
            axes = {name for name, axis in self.contract['axes'].items() if axis['group'] == group}
            for controller in definition['controllers']:
                self.assertNotIn(controller, seen)
                seen.add(controller)
                parameters = self.controllers[controller]['ros__parameters']
                claimed = parameters.get('joints', parameters.get('left_wheel_names', []) + parameters.get('right_wheel_names', []))
                self.assertEqual(set(claimed), axes)
                self.assertEqual(len(claimed), len(axes))
        self.assertEqual(self.contract['groups']['flippers']['controller_selection'], 'exactly_one')

    def test_physical_conditions_are_unknown_and_have_no_implicit_true_default(self):
        for rule in self.contract['associations']:
            self.assertIn(rule['source_group'], self.contract['groups'])
            self.assertIn(rule['dependent_group'], self.contract['groups'])
            self.assertNotEqual(rule['source_group'], rule['dependent_group'])
            self.assertTrue(rule['source'])
            self.assertEqual(rule['unknown'], 'block')
            condition = self.contract['conditions'][rule['condition']]
            self.assertEqual(condition['initial'], 'unknown')
            self.assertEqual(condition['field_evidence'], 'pending')

    def test_unclassified_errors_never_silently_continue(self):
        events = self.contract['events']
        self.assertEqual(events['unknown_emcy']['severity'], 'group_stop')
        self.assertTrue(events['unknown_emcy']['preserve_raw_code'])
        self.assertEqual(events['known_emcy'], [])
        self.assertEqual(events['heartbeat_timeout']['additional_delay_ns'], 0)
        self.assertEqual(events['emergency_stop']['severity'], 'global_stop')
        self.assertEqual(self.contract['activation'], 'staged')

    def test_duplicate_configuration_keys_are_rejected(self):
        with self.assertRaises(ValueError):
            yaml.load('axis: arm\naxis: base\n', Loader=UniqueLoader)

if __name__ == '__main__':
    unittest.main(verbosity=2)
