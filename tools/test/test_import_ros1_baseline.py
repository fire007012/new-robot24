import copy
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('import_ros1_baseline',Path(__file__).resolve().parents[1]/'import_ros1_baseline.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Tests(unittest.TestCase):
    def snapshot(self):
        draft=module.build()
        return {'axes':{name:dict(node_id=entry['node_id'],
            identity={'1000:00':1,'1018:01':2,'1018:02':3,'1018:03':4,'1018:04':5,'100A:00':'TEST-FIRMWARE'},
            persistent={'607C:00':0,'2003:00':1,'607D:01':0,'607D:02':100}) for name,entry in draft['axes'].items()}}

    def test_confirmed_parameters_do_not_invent_identity(self):
        result=module.build()
        self.assertFalse(result['commissioned'])
        self.assertEqual(result['parameter_reference']['confirmation'],'user_confirmed_working_ros1_parameters')
        self.assertTrue(all(axis['direction_verified'] for axis in result['axes'].values()))
        self.assertTrue(all(v is None for axis in result['axes'].values() for v in axis['identity'].values()))

    def test_complete_readback_can_complete_parameter_baseline(self):
        snapshot=self.snapshot();result=module.build(snapshot)
        self.assertTrue(result['commissioned'])
        self.assertEqual(result['axes']['shoulder_yaw_joint']['identity'],snapshot['axes']['shoulder_yaw_joint']['identity'])

    def test_incomplete_mismatched_and_simulated_readback_rejected(self):
        mutations=(lambda a:a.update(node_id=99),lambda a:a['identity'].pop('1018:04'),
                   lambda a:a['identity'].update({'100A:00':'MOCK'}),
                   lambda a:a['identity'].update({'1000:00':0x4d4f434b}),
                   lambda a:a['persistent'].pop('607C:00'),lambda a:a['persistent'].update({'607C:00':True}))
        for change in mutations:
            with self.subTest(change=change):
                snapshot=self.snapshot();change(next(iter(snapshot['axes'].values())))
                with self.assertRaises(ValueError):module.build(snapshot)

if __name__=='__main__':unittest.main()
