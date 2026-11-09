"""Reject mixed generated bus/DCF periods before a controller manager starts."""
import copy
import tempfile
import unittest
from pathlib import Path

import yaml
from robot_hardware_runtime.control_period import control_rate


class ControlPeriodTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bus = {'options': {}, 'master': {'sync_period': 20000},
                    'axis': {'period': 20, 'sdo': [
                        {'index': 0x60C2, 'sub_index': 1, 'value': 20}]}}

    def write(self, bus, sync=20000):
        (self.root / 'bus.yml').write_text(yaml.safe_dump(bus))
        (self.root / 'master.dcf').write_text(f'[1006]\nDefaultValue={sync}\n')

    def test_manager_follows_both_supported_profiles(self):
        for ms, rate in ((5, 200), (20, 50)):
            self.bus['master']['sync_period'] = ms * 1000
            self.bus['axis']['period'] = ms
            self.bus['axis']['sdo'][0]['value'] = ms
            self.write(self.bus, ms * 1000)
            self.assertEqual(control_rate(self.root), rate)

    def test_mixed_timing_or_invalid_values_refuse_startup(self):
        for field in ('sync', 'driver', 'interpolation', 'dcf', 'missing_sdo', 'empty'):
            with self.subTest(field=field):
                bus = copy.deepcopy(self.bus)
                if field == 'sync': bus['master']['sync_period'] = 10000
                if field == 'driver': bus['axis']['period'] = 5
                if field == 'interpolation': bus['axis']['sdo'][0]['value'] = 5
                if field == 'missing_sdo': bus['axis']['sdo'] = []
                if field == 'empty': del bus['axis']
                self.write(bus, 5000 if field == 'dcf' else 20000)
                with self.assertRaises(ValueError):
                    control_rate(self.root)
