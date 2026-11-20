import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manager import Backend, Fault
from can_config import effective_can


class Tests(unittest.TestCase):
    def test_selection_drives_real_launch_and_devices(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'baseline.yaml').write_text('commissioned: false\n')
            p = json.loads((Path(__file__).resolve().parents[1]/'config/hardware.json').read_text())
            p['baseline'] = 'baseline.yaml'
            b = Backend({'control_endpoint': {'port':9090}}, root, root)
            b.command = lambda args: args
            for preset, groups in [('base','base'), ('flippers','flippers')]:
                p['requested_presets'] = [preset]
                b.validate(p)
                with patch('manager.subprocess.Popen') as spawn:
                    spawn.return_value.pid = os.getpid()
                    b.start('core', p, preset)
                    command = spawn.call_args.args[0]
                    for arg in ('mock:=false', 'selected_groups:='+groups,
                                'private_interface:=can0', 'canopen_interface:=can1'):
                        self.assertIn(arg, command)
                    self.assertNotIn('enable_servo:=true', command)
            # The configured two buses support arm and mixed groups. Reject a
            # deliberately shared bus independently of the current defaults.
            for presets in (['arm'], ['base','flippers'], ['arm','base','flippers']):
                p['requested_presets'] = presets
                self.assertEqual(effective_can(p), {'canopen_interface':'can1','private_interface':'can0'})
            p['can_interfaces']['canopen_interface']='can0'
            p['devices']=['can0']
            p['group_devices']={g:['can0'] for g in ('arm','base','flippers','gripper')}
            for presets in (['arm'], ['base','flippers'], ['arm','base','flippers']):
                p['requested_presets'] = presets
                with patch('manager.subprocess.Popen') as spawn:
                    with self.assertRaises(Fault) as error: b.start('core',p,'blocked')
                    self.assertEqual(error.exception.data['code'], 'CAN_TOPOLOGY')
                    spawn.assert_not_called()
            for key in ('can_interfaces', 'group_devices'):
                broken = copy.deepcopy(p); broken.pop(key)
                with self.assertRaises(Fault): b.validate(broken)

    def test_full_explicit_topology_and_mock(self):
        p=json.loads((Path(__file__).resolve().parents[1]/'config/hardware-full.json').read_text())
        self.assertEqual(effective_can(p), p['can_interfaces'])
        p['requested_presets']=['arm']
        self.assertEqual(effective_can(p), p['can_interfaces'])
        self.assertEqual(effective_can({'mode':'mock'}), {})
        p['group_devices']['base']=['can1']
        with self.assertRaises(ValueError):effective_can(p)
