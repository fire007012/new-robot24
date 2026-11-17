import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from deploy_can0 import deploy
from manager import Manager


class Tests(unittest.TestCase):
    def test_stopped_deployment_and_active_bundle_are_idempotent(self):
        for transactions in (False,True):
            with self.subTest(transactions=transactions), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);profiles=root/'profiles';profiles.mkdir()
                (profiles/'mock.json').write_text(json.dumps(dict(profile_id='mock',config_revision='1',mode='mock',video=False,build_version='old',devices=[])))
                (root/'token').write_text('x'*32)
                config=root/'manager.json'
                config.write_text(json.dumps(dict(enable_config_transactions=transactions,robot_id='robot24',config_root=str(profiles),state_dir=str(root/'state'),token_file=str(root/'token'),control_endpoint={'port':0})))
                (root/'setup.bash').touch()
                manifest=root/'release.json';manifest.write_text(json.dumps(dict(build_version='new',management_protocol=1,control_protocol=2)))
                baseline=root/'baseline.yaml';baseline.write_text('commissioned: false\n')
                # Seed a bundle which contains only mock, as in the reported failure.
                m=Manager(config);m.closed.set();m.lock_file.close()
                a=deploy(config,root/'setup.bash',manifest,baseline)
                b=deploy(config,root/'setup.bash',manifest,baseline)
                self.assertEqual(a,b)
                m=Manager(config)
                try:
                    values=m.dispatch(dict(version=1,token='x'*32,request_id='profiles',method='profiles',params={}))
                    self.assertEqual({v['profile_id'] for v in values},{'mock','hardware'})
                    h=next(v for v in values if v['profile_id']=='hardware')
                    p=m.profile(dict(robot_id='robot24',profile_id='hardware',config_revision=h['config_revision'],presets=['base']))
                    self.assertEqual(p['mode'],'hardware')
                    self.assertEqual(p['can_interfaces']['private_interface'],'can0')
                    self.assertFalse(m.data['resources'])
                    self.assertEqual((root/'token').read_text(),'x'*32)
                    self.assertEqual(baseline.read_text(),'commissioned: false\n')
                finally:m.closed.set();m.lock_file.close()
