import json
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'prepare_local_link.py'


class LocalLinkTests(unittest.TestCase):
    def test_preparation_is_mock_only_and_preserves_existing_state(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); install = root / 'install'; install.mkdir()
            for name in ('robot_bringup', 'host_bridge', 'car_control', 'arm_control'):
                package = install / name / 'share' / name; package.mkdir(parents=True)
                (package / 'package.xml').write_text('<package/>')
            (install / 'setup.bash').write_text('# test fixture only\n')
            output = root / 'runtime'
            cmd = [sys.executable, '-B', str(SCRIPT), '--install', str(install), '--output', str(output)]
            result = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            config = json.loads((output / 'manager.json').read_text())
            self.assertEqual(config['bind'], '127.0.0.1')
            self.assertEqual(config['control_endpoint']['protocol_version'], 2)
            self.assertEqual(json.loads((output / 'profiles/mock.json').read_text())['mode'], 'mock')
            self.assertNotIn('mock_safe_posture',json.loads((output / 'profiles/mock.json').read_text()))
            self.assertEqual(len(list((output / 'profiles').glob('*.json'))), 1)
            token = output / 'manager.token'
            self.assertEqual(stat.S_IMODE(token.stat().st_mode), 0o600)
            original = token.read_text()
            self.assertGreaterEqual(len(original.strip()), 32)
            self.assertNotIn(original.strip(), result.stdout + result.stderr)
            second = subprocess.run(cmd, capture_output=True, text=True)
            self.assertNotEqual(second.returncode, 0)
            self.assertEqual(token.read_text(), original)

    def test_incomplete_installation_creates_no_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); output = root / 'runtime'
            result = subprocess.run([sys.executable, '-B', str(SCRIPT), '--install', str(root),
                                     '--output', str(output)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
