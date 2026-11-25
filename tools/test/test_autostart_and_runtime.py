import json
from pathlib import Path
import sys
import tempfile
import unittest

SRC_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC_ROOT))
sys.path.insert(0, str(SRC_ROOT / 'tools'))

from robot_manager.autostart import install_autostart, plan_layout, uninstall_autostart
from tools.prepare_runtime import prepare_runtime


class AutostartAndRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='robot24-autostart-test-')
        self.root = Path(self.tmp.name)
        self.ws = self.root / 'ws'
        (self.ws / 'install').mkdir(parents=True)
        (self.ws / 'install/setup.bash').write_text('# mock setup\n')
        (self.ws / 'src/robot_manager/systemd').mkdir(parents=True)
        (self.ws / 'src/robot_manager/config').mkdir(parents=True)
        (self.ws / 'src/robot_hardware/config').mkdir(parents=True)
        (self.ws / 'src/robot_bringup/scripts').mkdir(parents=True)
        for rel in (
            'src/robot_manager/manager.py',
            'src/robot_manager/release_manifest.py',
            'src/robot_manager/deployment_paths.py',
            'src/robot_manager/systemd/robot24-manager.service',
            'src/robot_manager/systemd/robot24-manager.logrotate',
            'src/robot_bringup/scripts/install_can_autosetup.sh',
            'src/robot_hardware/config/full_registered_baseline.yaml',
        ):
            src = SRC_ROOT.parent / rel
            dst = self.ws / rel
            dst.write_text(src.read_text())

    def tearDown(self):
        self.tmp.cleanup()

    def test_prepare_runtime_is_idempotent_and_preserves_token(self):
        runtime = self.root / 'runtime'
        cfg_path = prepare_runtime(self.ws, self.ws / 'install', runtime)
        cfg = json.loads(cfg_path.read_text())
        token_before = (runtime / 'manager.token').read_text()
        self.assertGreaterEqual(len(token_before.strip()), 32)
        self.assertEqual((runtime / 'manager.token').stat().st_mode & 0o777, 0o600)
        self.assertTrue((runtime / 'profiles/mock.json').is_file())
        self.assertTrue((runtime / 'profiles/hardware.json').is_file())
        self.assertEqual(cfg['setup'], str((self.ws / 'install/setup.bash').resolve()))

        # Re-running preserves token and existing state.
        (runtime / 'state/state.json').write_text('{"resources": [], "software_state": "stopped"}')
        prepare_runtime(self.ws, self.ws / 'install', runtime)
        self.assertEqual((runtime / 'manager.token').read_text(), token_before)
        self.assertIn('stopped', (runtime / 'state/state.json').read_text())

    def test_staged_autostart_plan_install_refusal_and_uninstall(self):
        runtime = self.root / 'runtime'
        prepare_runtime(self.ws, self.ws / 'install', runtime)
        prefix = self.root / 'stage'

        planned = plan_layout(self.ws, prefix)
        self.assertTrue(planned['install_exists'])
        self.assertTrue(planned['manager_exists'])

        installed = install_autostart(self.ws, root_prefix=prefix, source_runtime=runtime)
        self.assertTrue(installed['installed'])
        self.assertFalse(installed['live_system'])
        self.assertTrue((prefix / 'opt/robot24/install/release.json').is_file())
        self.assertTrue((prefix / 'opt/robot24/manager/manager.py').is_file())
        self.assertTrue((prefix / 'etc/robot24/manager.json').is_file())
        self.assertTrue((prefix / 'etc/robot24/manager.token').is_file())
        self.assertTrue((prefix / 'etc/systemd/system/robot24-manager.service').is_file())
        self.assertTrue((prefix / 'etc/logrotate.d/robot24-manager').is_file())
        etc_cfg = json.loads((prefix / 'etc/robot24/manager.json').read_text())
        self.assertEqual(etc_cfg['setup'], '/opt/robot24/install/setup.bash')
        self.assertEqual(etc_cfg['build_manifest'], '/opt/robot24/install/release.json')

        # Active resources must block re-install or uninstall.
        (prefix / 'var/lib/robot24-manager/state.json').write_text(
            json.dumps({'resources': [{'module': 'core'}], 'software_state': 'ready'})
        )
        with self.assertRaises(RuntimeError):
            uninstall_autostart(root_prefix=prefix)

        (prefix / 'var/lib/robot24-manager/state.json').write_text(
            json.dumps({'resources': [], 'software_state': 'stopped'})
        )
        uninstalled = uninstall_autostart(root_prefix=prefix, purge=False)
        self.assertTrue(uninstalled['uninstalled'])
        self.assertFalse((prefix / 'opt/robot24').exists())
        self.assertFalse((prefix / 'etc/systemd/system/robot24-manager.service').exists())
        self.assertTrue((prefix / 'etc/robot24/manager.json').exists())


if __name__ == '__main__':
    unittest.main()
