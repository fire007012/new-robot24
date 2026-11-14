from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from group_selection import requirements, required_devices, selected_groups
from manager import Backend, Fault


class Tests(unittest.TestCase):
    def test_base_has_no_arm_flipper_controller_or_action_requirement(self):
        groups, controllers, backends=requirements(['base'])
        self.assertEqual(groups,{'base'})
        self.assertEqual(controllers,{'joint_state_broadcaster','wheel_controller'})
        self.assertEqual(backends,{'CanDriverSystem'})
        self.assertEqual(selected_groups(['arm']),{'arm','gripper'})
        self.assertEqual(selected_groups(['flippers','base']),{'flippers','base'})
        self.assertEqual(selected_groups(),{'arm','base','flippers','gripper'})

    def test_device_checks_only_cover_selected_groups_and_mapping_is_complete(self):
        profile=dict(mode='hardware',video=False,baseline=__file__,
                     devices=['canable0','canable1'],requested_presets=['base'],
                     group_devices=dict(base=['canable1'],gripper=['canable1'],
                                        arm=['canable0','canable1'],flippers=['canable0']))
        backend=Backend({},Path(__file__).parent,Path(__file__).parent)
        backend.validate(profile)
        self.assertEqual(required_devices(profile),['canable1'])
        profile['requested_presets']=['arm']
        self.assertEqual(required_devices(profile),['canable0','canable1'])
        del profile['group_devices']['arm']
        with self.assertRaises(Fault):backend.validate(profile)

    def test_invalid_selection_never_falls_back_to_whole_vehicle(self):
        for value in ([],['base','base'],['unknown'],'base',[1],{}):
            with self.assertRaises(ValueError):selected_groups(value)

if __name__=='__main__':unittest.main()
