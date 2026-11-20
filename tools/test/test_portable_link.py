"""Offline export tests: relocation, release integrity and stopped-state guard."""
import argparse
import fcntl
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('portable_link', SOURCE / 'tools/portable_link.py')
portable = importlib.util.module_from_spec(spec)
spec.loader.exec_module(portable)


class PortableTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        install = self.root / 'original/install'
        manager = install / 'manager'
        manager.mkdir(parents=True)
        for path in (SOURCE / 'robot_manager').glob('*.py'):
            shutil.copy2(path, manager / path.name)
        (install / 'setup.bash').write_text('# fixture\n')
        external = install / 'library-data'
        external.write_text('library fixture')
        (install / 'library').symlink_to(external)
        # Fingerprint the content, then confirm an internal link is materialized on export.
        (install / 'library').unlink()
        (install / 'library').write_text(external.read_text())
        subprocess.run([sys.executable, '-B', str(manager / 'release_manifest.py'),
                        '--root', str(install), '--build-version', 'test'], check=True)
        (install / 'library').unlink()
        (install / 'library').symlink_to(external)
        runtime = self.root / 'original/runtime'
        (runtime / 'profiles').mkdir(parents=True)
        (runtime / 'state/control').mkdir(parents=True)
        (runtime / 'profiles/baseline.yaml').write_text('commissioned: false\n')
        portable.write(runtime / 'profiles/hardware.json', {'baseline': 'baseline.yaml'})
        portable.write(runtime / 'state/state.json', dict(resources=[], software_state='stopped', operations={}))
        (runtime / 'state/control/journal.json').write_text('{"emergency_latched":true}')
        (runtime / 'state/lock').touch()
        (runtime / 'manager.token').write_text('x' * 40)
        (runtime / 'manager.token').chmod(0o600)
        portable.write(runtime / 'manager.json', dict(config_root='profiles', state_dir='state',
            token_file='manager.token', setup='../install/setup.bash', build_manifest='../install/release.json',
            build_version='test', control_endpoint={'host': '127.0.0.1', 'protocol_version': 2}))
        self.args = argparse.Namespace(install=install, config=runtime / 'manager.json',
                                      output=self.root / 'export.tar.gz', advertise_host='192.168.1.20')

    def test_export_relocation_and_tamper(self):
        portable.pack(self.args)
        self.assertEqual(self.args.output.stat().st_mode & 0o777, 0o600)
        target = self.root / 'different path'
        target.mkdir()
        with tarfile.open(self.args.output) as archive:
            archive.extractall(target)
        bundle = target / 'robot24-portable'
        shutil.rmtree(self.root / 'original')
        portable.check(bundle, dependencies=False)
        self.assertFalse((bundle / 'install/library').is_symlink())
        self.assertEqual((bundle / 'runtime/state/control/journal.json').read_text(), '{"emergency_latched":true}')
        config = portable.read(bundle / 'runtime/manager.json')
        self.assertEqual(config['control_endpoint']['host'], '192.168.1.20')
        (bundle / 'install/library').write_text('tampered')
        with self.assertRaisesRegex(ValueError, 'deployment verification failed'):
            portable.check(bundle, dependencies=False)

    def test_library_paths_with_spaces_and_external_dependency(self):
        root = self.root / 'bundle with spaces'
        (root / 'install').mkdir(parents=True)
        (root / 'install/library.so').write_bytes(b'\x7fELF')
        result = subprocess.CompletedProcess([], 0,
            stdout=f'libtest.so => {root}/install/library.so (0x1234)\n', stderr='')
        with patch.object(portable.subprocess, 'run', return_value=result):
            portable.libraries(root)
        result.stdout = 'libtest.so => /home/old-workspace/library.so (0x1234)\n'
        with patch.object(portable.subprocess, 'run', return_value=result):
            with self.assertRaisesRegex(ValueError, 'external non-system dependency'):
                portable.libraries(root)

    def test_live_manager_rejected(self):
        with (self.args.config.parent / 'state/lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                portable.pack(self.args)
        self.assertFalse(self.args.output.exists())

    def test_unresolved_resources_rejected(self):
        portable.write(self.args.config.parent / 'state/state.json',
                       dict(resources=[{'pid': 1}], software_state='stopped', operations={}))
        with self.assertRaisesRegex(ValueError, 'stop/reconcile'):
            portable.pack(self.args)
        self.assertFalse(self.args.output.exists())


if __name__ == '__main__':
    unittest.main()
