import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stop_evidence import cold_stopped


class ColdStopTests(unittest.TestCase):
    def test_partial_startup_requires_complete_fresh_stopped_evidence(self):
        hardware = dict(RobotCanopenSystem=1, CanDriverSystem=1)
        runtime = dict(state='Stopped', diagnostics=dict(readable=True),
                       groups={g:'Stopped' for g in ('arm','base','flippers','gripper')})
        self.assertTrue(cold_stopped([], hardware, runtime, .1))
        self.assertTrue(cold_stopped(['inactive'], hardware, runtime, .1))
        self.assertFalse(cold_stopped(None, hardware, runtime, .1))
        self.assertFalse(cold_stopped([], None, runtime, .1))
        self.assertFalse(cold_stopped(['active'], hardware, runtime, .1))
        self.assertFalse(cold_stopped([], dict(hardware, CanDriverSystem=2), runtime, .1))
        self.assertFalse(cold_stopped([], {'RobotCanopenSystem':1}, runtime, .1))
        for age in (.5, 1, -1, float('nan'), float('inf')):
            self.assertFalse(cold_stopped([], hardware, runtime, age))
        for field, value in [('state','Running'), ('diagnostics',{}),
                             ('groups',{}), ('groups',dict(runtime['groups'], arm='Running'))]:
            invalid = copy.deepcopy(runtime)
            invalid[field] = value
            self.assertFalse(cold_stopped([], hardware, invalid, .1))
