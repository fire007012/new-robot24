import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from robot_hardware_runtime.selection import parse_groups, select_description, selected_controllers


class Tests(unittest.TestCase):
    def test_no_unselected_component_or_axis_survives(self):
        contract={'axes': {'wheel': {'group':'base','backend':'CanDriverSystem'},
            'finger':{'group':'gripper','backend':'CanDriverSystem'},
            'flipper':{'group':'flippers','backend':'RobotCanopenSystem'},
            'shoulder':{'group':'arm','backend':'RobotCanopenSystem'}}}
        xml='<robot><ros2_control name="CanDriverSystem"><hardware/><joint name="wheel"/><joint name="finger"/></ros2_control><ros2_control name="RobotCanopenSystem"><hardware><param name="bus_config"/><param name="master_config"/></hardware><joint name="flipper"/><joint name="shoulder"/></ros2_control></robot>'
        with tempfile.TemporaryDirectory() as tmp:
            for group in ('arm','flippers'):
                directory=Path(tmp)/('groups_'+group);directory.mkdir()
                for name in ('bus.yml','master.dcf'):(directory/name).touch()
            for group, expected, backend in [('base',{'wheel'},'CanDriverSystem'),
                    ('flippers',{'flipper'},'RobotCanopenSystem'),('arm',{'shoulder'},'RobotCanopenSystem')]:
                root=ET.fromstring(select_description(xml,contract,{group},tmp))
                self.assertEqual({j.attrib['name'] for j in root.findall('ros2_control/joint')},expected)
                self.assertEqual([c.attrib['name'] for c in root.findall('ros2_control')],[backend])
                for p in root.findall('ros2_control/hardware/param'):
                    self.assertIn('groups_'+group,p.text)
            with self.assertRaises(ValueError):select_description(xml,contract,{'unknown'},tmp)
        for invalid in ('','base,base','all'):
            with self.assertRaises(ValueError):parse_groups(invalid)

    def test_runtime_cannot_operate_unselected_groups_or_recover_absent_backend(self):
        import yaml
        from robot_hardware_runtime.groups import GroupCoordinator
        path=Path(__file__).resolve().parents[2]/'robot_safety/config/group_policy_v2.yaml'
        coordinator=GroupCoordinator(object(),object(),yaml.safe_load(path.read_text()),['base'])
        self.assertEqual(coordinator.selected(None),['base'])
        self.assertEqual(coordinator.COMPONENTS,('CanDriverSystem',))
        self.assertEqual(coordinator.resource_groups('CanDriverSystem'),['base'])
        for groups in (['arm'],['flippers'],['base','gripper']):
            with self.assertRaises(ValueError):coordinator.init(groups)
        with self.assertRaises(ValueError):coordinator.recover_resource('RobotCanopenSystem')
