import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stop_evidence import cold_stopped, controller_manager_resolution_failed


class ColdStopTests(unittest.TestCase):
    def test_only_explicit_controller_manager_resolution_failure_is_cold_launch_evidence(self):
        marker = "[ERROR] Caught exception in launch: package 'controller_manager' not found"
        self.assertTrue(controller_manager_resolution_failed(marker))
        self.assertFalse(controller_manager_resolution_failed("package 'controller_manager' not found"))
        self.assertFalse(controller_manager_resolution_failed("Caught exception in launch: hardware init failed"))

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

    def test_single_backend_requires_explicit_expected_ownership(self):
        runtime=dict(state='Stopped',diagnostics=dict(readable=True),
                     groups={g:'Stopped' for g in ('arm','base','flippers','gripper')})
        self.assertTrue(cold_stopped([],{'CanDriverSystem':1},runtime,.1,{'CanDriverSystem'}))
        self.assertFalse(cold_stopped([],{},runtime,.1,set()))
        self.assertFalse(cold_stopped([],{'CanDriverSystem':1},runtime,.1,{'RobotCanopenSystem'}))
